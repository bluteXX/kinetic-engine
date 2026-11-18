#include "kinetic/assets/GltfLoader.h"

#include "kinetic/core/Log.h"

#include <cgltf.h>
#include <glm/gtx/matrix_decompose.hpp>

#include <algorithm>
#include <string>
#include <type_traits>
#include <memory>
#include <numeric>
#include <unordered_map>

namespace kinetic::assets {

namespace {

struct CgltfDeleter {
    void operator()(cgltf_data* data) const { cgltf_free(data); }
};
using CgltfPtr = std::unique_ptr<cgltf_data, CgltfDeleter>;

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

} // namespace

std::optional<SkeletalAnimationAsset> loadSkeletalAnimation(const std::filesystem::path& path)
{
    const std::string pathString = path.string();
    cgltf_options options{};
    cgltf_data* raw = nullptr;
    if (cgltf_parse_file(&options, pathString.c_str(), &raw) != cgltf_result_success) {
        KN_ERROR("glTF: failed to parse '{}'", pathString);
        return std::nullopt;
    }
    CgltfPtr data(raw);
    if (cgltf_load_buffers(&options, data.get(), pathString.c_str()) != cgltf_result_success) {
        KN_ERROR("glTF: failed to load buffers for '{}'", pathString);
        return std::nullopt;
    }
    if (data->skins_count == 0) {
        KN_ERROR("glTF: '{}' has no skin", pathString);
        return std::nullopt;
    }

    const cgltf_skin& skin = data->skins[0];
    const usize jointCount = skin.joints_count;

    // glTF joint index of each node in the skin.
    std::unordered_map<const cgltf_node*, usize> gltfIndexOf;
    for (usize i = 0; i < jointCount; ++i) {
        gltfIndexOf[skin.joints[i]] = i;
    }

    // Nearest ancestor that is also a joint (non-joint nodes in between are skipped).
    std::vector<i32> gltfParent(jointCount, anim::kNoParent);
    std::vector<usize> depth(jointCount, 0);
    for (usize i = 0; i < jointCount; ++i) {
        for (const cgltf_node* n = skin.joints[i]->parent; n != nullptr; n = n->parent) {
            if (auto it = gltfIndexOf.find(n); it != gltfIndexOf.end()) {
                if (gltfParent[i] == anim::kNoParent) {
                    gltfParent[i] = static_cast<i32>(it->second);
                }
                ++depth[i];
            }
        }
    }

    // Engine order: sorted by depth, so every parent precedes its children.
    std::vector<usize> order(jointCount);
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](usize a, usize b) { return depth[a] < depth[b]; });
    std::vector<usize> engineIndexOf(jointCount);
    for (usize e = 0; e < jointCount; ++e) {
        engineIndexOf[order[e]] = e;
    }

    std::vector<std::string> names(jointCount);
    std::vector<i32> parents(jointCount, anim::kNoParent);
    std::vector<glm::mat4> inverseBind(jointCount, glm::mat4(1.0f));
    anim::Pose restPose(jointCount);

    for (usize e = 0; e < jointCount; ++e) {
        const usize g = order[e];
        const cgltf_node& node = *skin.joints[g];
        names[e] = node.name ? node.name : "joint_" + std::to_string(e);
        parents[e] = gltfParent[g] == anim::kNoParent
                         ? anim::kNoParent
                         : static_cast<i32>(engineIndexOf[static_cast<usize>(gltfParent[g])]);
        restPose.set(e, readLocalTransform(node));
        if (skin.inverse_bind_matrices) {
            cgltf_accessor_read_float(skin.inverse_bind_matrices, g, &inverseBind[e][0][0], 16);
        }
    }

    SkeletalAnimationAsset asset{
        anim::Skeleton(std::move(names), std::move(parents), std::move(restPose), std::move(inverseBind)), {}};

    for (usize a = 0; a < data->animations_count; ++a) {
        const cgltf_animation& animation = data->animations[a];
        std::vector<anim::JointTracks> tracks(jointCount);
        f32 duration = 0.0f;
        bool touchesSkin = false;

        for (usize c = 0; c < animation.channels_count; ++c) {
            const cgltf_animation_channel& channel = animation.channels[c];
            const auto it = gltfIndexOf.find(channel.target_node);
            if (it == gltfIndexOf.end()) {
                continue; // animates something that is not part of this skeleton
            }
            touchesSkin = true;
            anim::JointTracks& jt = tracks[engineIndexOf[it->second]];
            const cgltf_animation_sampler& sampler = *channel.sampler;
            if (sampler.interpolation == cgltf_interpolation_type_cubic_spline) {
                KN_WARN("glTF: cubic spline in '{}' is played back as linear", animation.name ? animation.name : "?");
            }
            switch (channel.target_path) {
            case cgltf_animation_path_type_translation: jt.translation = readTrack<glm::vec3>(sampler); break;
            case cgltf_animation_path_type_rotation:    jt.rotation = readTrack<glm::quat>(sampler); break;
            case cgltf_animation_path_type_scale:       jt.scale = readTrack<glm::vec3>(sampler); break;
            default: continue; // morph weights: not supported yet
            }
            if (sampler.input->count > 0) {
                f32 lastKey = 0.0f;
                cgltf_accessor_read_float(sampler.input, sampler.input->count - 1, &lastKey, 1);
                duration = std::max(duration, lastKey);
            }
        }

        if (touchesSkin) {
            std::string clipName = animation.name ? animation.name : "clip_" + std::to_string(a);
            asset.clips.emplace_back(std::move(clipName), duration, std::move(tracks));
        }
    }

    KN_INFO("glTF: loaded '{}' ({} joints, {} clips)", path.filename().string(), jointCount, asset.clips.size());
    return asset;
}

} // namespace kinetic::assets
