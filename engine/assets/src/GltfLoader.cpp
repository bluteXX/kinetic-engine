#include "kinetic/assets/GltfLoader.h"

#include "LoaderInternal.h"

#include "kinetic/core/Log.h"

#include <cgltf.h>
#include <glm/gtx/matrix_decompose.hpp>

#include <algorithm>
#include <memory>
#include <numeric>
#include <string>
#include <type_traits>
#include <unordered_map>

namespace kinetic::assets {

namespace {

struct CgltfDeleter {
    void operator()(cgltf_data* data) const { cgltf_free(data); }
};
using CgltfPtr = std::unique_ptr<cgltf_data, CgltfDeleter>;

CgltfPtr parseFile(const std::filesystem::path& path, std::string& error)
{
    const std::string pathString = path.string();
    cgltf_options options{};
    cgltf_data* raw = nullptr;
    if (cgltf_parse_file(&options, pathString.c_str(), &raw) != cgltf_result_success) {
        error = "failed to parse glTF";
        return nullptr;
    }
    CgltfPtr data(raw);
    if (cgltf_load_buffers(&options, data.get(), pathString.c_str()) != cgltf_result_success) {
        error = "failed to load glTF buffers";
        return nullptr;
    }
    return data;
}

anim::JointTransform readLocalTransform(const cgltf_node& node)
{
    anim::JointTransform t;
    if (node.has_matrix) {
        glm::mat4 m;
        std::copy_n(node.matrix, 16, &m[0][0]);
        glm::vec3 skew;
        glm::vec4 perspective;
        glm::decompose(m, t.scale, t.rotation, t.translation, skew, perspective);
        return t;
    }
    if (node.has_translation) {
        t.translation = {node.translation[0], node.translation[1], node.translation[2]};
    }
    if (node.has_rotation) { // glTF stores x, y, z, w; glm::quat ctor takes w first
        t.rotation = glm::quat(node.rotation[3], node.rotation[0], node.rotation[1], node.rotation[2]);
    }
    if (node.has_scale) {
        t.scale = {node.scale[0], node.scale[1], node.scale[2]};
    }
    return t;
}

template <typename T>
anim::Track<T> readTrack(const cgltf_animation_sampler& sampler)
{
    anim::Track<T> track;
    const cgltf_accessor* input = sampler.input;
    const cgltf_accessor* output = sampler.output;
    const usize keyCount = input->count;
    const bool cubic = sampler.interpolation == cgltf_interpolation_type_cubic_spline;

    track.interpolation = sampler.interpolation == cgltf_interpolation_type_step ? anim::Interpolation::Step
                                                                                  : anim::Interpolation::Linear;
    track.times.resize(keyCount);
    track.values.resize(keyCount);

    for (usize k = 0; k < keyCount; ++k) {
        cgltf_accessor_read_float(input, k, &track.times[k], 1);
        // Cubic spline stores [in-tangent, value, out-tangent]; keep the value, play it linearly for now.
        const usize element = cubic ? k * 3 + 1 : k;
        float v[4] = {0.0f, 0.0f, 0.0f, 1.0f};
        cgltf_accessor_read_float(output, element, v, std::is_same_v<T, glm::quat> ? 4 : 3);
        if constexpr (std::is_same_v<T, glm::quat>) {
            track.values[k] = glm::normalize(glm::quat(v[3], v[0], v[1], v[2]));
        } else {
            track.values[k] = T(v[0], v[1], v[2]);
        }
    }
    return track;
}

// Skeleton built from one glTF skin, with the mapping between glTF nodes / skin joint order (as
// used by JOINTS_0 and inverseBindMatrices) and engine order (parents before children).
//
// With `includeAncestors`, every ancestor node of the skin's joints is added as an extra helper
// joint (identity inverse bind, not a bone). glTF joint world transforms include those ancestors
// (e.g. an "Armature" node scaled by 0.01), so this is needed to skin such files correctly.
struct SkinSkeleton {
    anim::Skeleton skeleton;
    std::unordered_map<const cgltf_node*, usize> engineIndexOfNode; // every node in the skeleton
    std::vector<usize> engineIndexOf;                               // skin joint index -> engine joint
    std::vector<u8> isBone;                                         // per engine joint
};

SkinSkeleton buildSkeleton(const cgltf_skin& skin, bool includeAncestors)
{
    // Nodes in the skeleton (glTF skin joints first, in skin order) and their parent among them.
    std::vector<const cgltf_node*> nodes(skin.joints, skin.joints + skin.joints_count);
    std::unordered_map<const cgltf_node*, usize> indexOf;
    for (usize i = 0; i < nodes.size(); ++i) {
        indexOf[nodes[i]] = i;
    }
    if (includeAncestors) {
        for (usize i = 0; i < skin.joints_count; ++i) {
            for (const cgltf_node* n = skin.joints[i]->parent; n != nullptr; n = n->parent) {
                if (indexOf.try_emplace(n, nodes.size()).second) {
                    nodes.push_back(n);
                }
            }
        }
    }
    const usize count = nodes.size();

    // Nearest ancestor that is also in the skeleton (other nodes in between are skipped).
    std::vector<i32> parentOf(count, anim::kNoParent);
    std::vector<usize> depth(count, 0);
    for (usize i = 0; i < count; ++i) {
        for (const cgltf_node* n = nodes[i]->parent; n != nullptr; n = n->parent) {
            if (auto it = indexOf.find(n); it != indexOf.end()) {
                if (parentOf[i] == anim::kNoParent) {
                    parentOf[i] = static_cast<i32>(it->second);
                }
                ++depth[i];
            }
        }
    }

    // Engine order: sorted by depth, so every parent precedes its children.
    std::vector<usize> order(count);
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](usize a, usize b) { return depth[a] < depth[b]; });
    std::vector<usize> engineOf(count);
    for (usize e = 0; e < count; ++e) {
        engineOf[order[e]] = e;
    }

    SkinSkeleton out;
    out.engineIndexOf.assign(engineOf.begin(), engineOf.begin() + static_cast<std::ptrdiff_t>(skin.joints_count));
    out.isBone.assign(count, 0);
    std::vector<std::string> names(count);
    std::vector<i32> parents(count, anim::kNoParent);
    std::vector<glm::mat4> inverseBind(count, glm::mat4(1.0f));
    anim::Pose restPose(count);

    for (usize e = 0; e < count; ++e) {
        const usize g = order[e];
        const cgltf_node& node = *nodes[g];
        out.engineIndexOfNode[&node] = e;
        names[e] = node.name ? node.name : "joint_" + std::to_string(e);
        parents[e] = parentOf[g] == anim::kNoParent ? anim::kNoParent
                                                    : static_cast<i32>(engineOf[static_cast<usize>(parentOf[g])]);
        restPose.set(e, readLocalTransform(node));
        if (g < skin.joints_count) {
            out.isBone[e] = 1;
            if (skin.inverse_bind_matrices) {
                cgltf_accessor_read_float(skin.inverse_bind_matrices, g, &inverseBind[e][0][0], 16);
            }
        }
    }
    out.skeleton = anim::Skeleton(std::move(names), std::move(parents), std::move(restPose), std::move(inverseBind));
    return out;
}

std::vector<anim::AnimationClip> readClips(const cgltf_data& data, const SkinSkeleton& skin)
{
    std::vector<anim::AnimationClip> clips;
    const usize jointCount = skin.skeleton.jointCount();

    for (usize a = 0; a < data.animations_count; ++a) {
        const cgltf_animation& animation = data.animations[a];
        std::vector<anim::JointTracks> tracks(jointCount);
        f32 duration = 0.0f;
        bool touchesSkin = false;

        for (usize c = 0; c < animation.channels_count; ++c) {
            const cgltf_animation_channel& channel = animation.channels[c];
            const auto it = skin.engineIndexOfNode.find(channel.target_node);
            if (it == skin.engineIndexOfNode.end()) {
                continue; // animates something that is not part of this skeleton
            }
            anim::JointTracks& jt = tracks[it->second];
            const cgltf_animation_sampler& sampler = *channel.sampler;
            if (sampler.input->count == 0) {
                continue;
            }
            if (sampler.interpolation == cgltf_interpolation_type_cubic_spline) {
                KN_WARN("glTF: cubic spline in '{}' is played back as linear", animation.name ? animation.name : "?");
            }
            switch (channel.target_path) {
            case cgltf_animation_path_type_translation: jt.translation = readTrack<glm::vec3>(sampler); break;
            case cgltf_animation_path_type_rotation:    jt.rotation = readTrack<glm::quat>(sampler); break;
            case cgltf_animation_path_type_scale:       jt.scale = readTrack<glm::vec3>(sampler); break;
            default: continue; // morph weights: not supported yet
            }
            touchesSkin = true;
            f32 lastKey = 0.0f;
            cgltf_accessor_read_float(sampler.input, sampler.input->count - 1, &lastKey, 1);
            duration = std::max(duration, lastKey);
        }

        if (touchesSkin) {
            std::string clipName = animation.name ? animation.name : "clip_" + std::to_string(a);
            clips.emplace_back(std::move(clipName), duration, std::move(tracks));
        }
    }
    return clips;
}

const cgltf_accessor* findAttribute(const cgltf_primitive& primitive, cgltf_attribute_type type)
{
    for (usize i = 0; i < primitive.attributes_count; ++i) {
        const cgltf_attribute& attribute = primitive.attributes[i];
        if (attribute.type == type && attribute.index == 0) {
            return attribute.data;
        }
    }
    return nullptr;
}

// Appends one triangle primitive of a skinned mesh. Returns false if it lacks skin attributes.
bool appendPrimitive(const cgltf_primitive& primitive, const SkinSkeleton& skin, anim::SkinnedMesh& out,
                     bool& missingNormals)
{
    const cgltf_accessor* positions = findAttribute(primitive, cgltf_attribute_type_position);
    const cgltf_accessor* normals = findAttribute(primitive, cgltf_attribute_type_normal);
    const cgltf_accessor* joints = findAttribute(primitive, cgltf_attribute_type_joints);
    const cgltf_accessor* weights = findAttribute(primitive, cgltf_attribute_type_weights);
    if (primitive.type != cgltf_primitive_type_triangles || !positions || !joints || !weights) {
        return false;
    }

    const auto base = static_cast<u32>(out.mesh.positions.size());
    const usize vertexCount = positions->count;
    missingNormals = missingNormals || normals == nullptr;

    for (usize v = 0; v < vertexCount; ++v) {
        glm::vec3 p(0.0f);
        cgltf_accessor_read_float(positions, v, &p.x, 3);
        glm::vec3 n(0.0f, 1.0f, 0.0f);
        if (normals) {
            cgltf_accessor_read_float(normals, v, &n.x, 3);
        }
        cgltf_uint j[4] = {0, 0, 0, 0};
        cgltf_accessor_read_uint(joints, v, j, 4);
        glm::vec4 w(0.0f);
        cgltf_accessor_read_float(weights, v, &w.x, 4);

        glm::uvec4 engineJoints(0u);
        for (glm::length_t k = 0; k < 4; ++k) {
            if (j[k] < skin.engineIndexOf.size()) {
                engineJoints[k] = static_cast<u32>(skin.engineIndexOf[j[k]]);
            } else {
                w[k] = 0.0f; // out-of-range joint: drop the influence
            }
        }
        out.mesh.positions.push_back(p);
        out.mesh.normals.push_back(n);
        out.joints.push_back(engineJoints);
        out.weights.push_back(anim::normalizeWeights(w));
    }

    if (primitive.indices) {
        for (usize i = 0; i < primitive.indices->count; ++i) {
            out.mesh.indices.push_back(base + static_cast<u32>(cgltf_accessor_read_index(primitive.indices, i)));
        }
    } else {
        for (usize i = 0; i < vertexCount; ++i) {
            out.mesh.indices.push_back(base + static_cast<u32>(i));
        }
    }
    return true;
}

} // namespace

std::optional<SkeletalAnimationAsset> loadSkeletalAnimation(const std::filesystem::path& path)
{
    std::string error;
    const CgltfPtr data = parseFile(path, error);
    if (!data) {
        KN_ERROR("glTF: '{}': {}", path.string(), error);
        return std::nullopt;
    }
    if (data->skins_count == 0) {
        KN_ERROR("glTF: '{}' has no skin", path.string());
        return std::nullopt;
    }
    SkinSkeleton skin = buildSkeleton(data->skins[0], false);
    std::vector<anim::AnimationClip> clips = readClips(*data, skin);
    KN_INFO("glTF: loaded '{}' ({} joints, {} clips)", path.filename().string(), skin.skeleton.jointCount(), clips.size());
    return SkeletalAnimationAsset{std::move(skin.skeleton), std::move(clips)};
}

namespace detail {

std::optional<anim::SkinnedModel> loadSkinnedModelGltf(const std::filesystem::path& path, std::string& error)
{
    const CgltfPtr data = parseFile(path, error);
    if (!data) {
        return std::nullopt;
    }
    if (data->skins_count == 0) {
        error = "glTF file has no skin";
        return std::nullopt;
    }
    const cgltf_skin& gltfSkin = data->skins[0];
    SkinSkeleton skin = buildSkeleton(gltfSkin, true);

    anim::SkinnedModel model;
    bool missingNormals = false;
    // Skinned vertices are in mesh space and the mesh node's own transform is ignored (glTF spec).
    for (usize n = 0; n < data->nodes_count; ++n) {
        const cgltf_node& node = data->nodes[n];
        if (!node.mesh || node.skin != &gltfSkin) {
            continue;
        }
        for (usize p = 0; p < node.mesh->primitives_count; ++p) {
            appendPrimitive(node.mesh->primitives[p], skin, model.skin, missingNormals);
        }
    }
    if (model.skin.mesh.empty()) {
        error = "no triangle mesh is bound to the skin";
        return std::nullopt;
    }
    if (missingNormals) {
        computeNormals(model.skin.mesh);
    }

    model.clips = readClips(*data, skin);
    model.isBone = std::move(skin.isBone);
    model.skeleton = std::move(skin.skeleton);
    return model;
}

} // namespace detail

} // namespace kinetic::assets
