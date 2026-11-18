#include "LoaderInternal.h"

#include "kinetic/core/Log.h"

#include <glm/gtx/matrix_decompose.hpp>
#include <ufbx.h>

#include <algorithm>
#include <array>
#include <memory>
#include <unordered_map>
#include <vector>

namespace kinetic::assets::detail {

namespace {

struct SceneDeleter {
    void operator()(ufbx_scene* scene) const { ufbx_free_scene(scene); }
};
struct BakeDeleter {
    void operator()(ufbx_baked_anim* bake) const { ufbx_free_baked_anim(bake); }
};

glm::vec3 toGlm(const ufbx_vec3& v)
{
    return {static_cast<f32>(v.x), static_cast<f32>(v.y), static_cast<f32>(v.z)};
}

glm::quat toGlm(const ufbx_quat& q)
{
    return glm::quat(static_cast<f32>(q.w), static_cast<f32>(q.x), static_cast<f32>(q.y), static_cast<f32>(q.z));
}

glm::mat4 toGlm(const ufbx_matrix& m)
{
    glm::mat4 r(1.0f);
    for (int c = 0; c < 4; ++c) {
        r[c] = glm::vec4(toGlm(m.cols[c]), c == 3 ? 1.0f : 0.0f);
    }
    return r;
}

std::string errorString(const ufbx_error& error)
{
    char buffer[512];
    ufbx_format_error(buffer, sizeof(buffer), &error);
    std::string message = buffer;
    while (!message.empty() && (message.back() == '\n' || message.back() == '\r')) {
        message.pop_back();
    }
    return message;
}

// Joints are FBX nodes. Every node that moves a vertex (skin bone, or the node of a rigid mesh)
// is added together with all of its ancestors up to and including the scene root, so the
// joint chain reproduces node_to_world exactly (including the axis/unit conversion on the root).
class JointBuilder {
public:
    usize add(const ufbx_node* node)
    {
        if (const auto it = m_index.find(node); it != m_index.end()) {
            return it->second;
        }
        std::vector<const ufbx_node*> chain;
        for (const ufbx_node* n = node; n && !m_index.contains(n); n = n->parent) {
            chain.push_back(n);
        }
        // Ancestors first: keeps the parents-before-children invariant.
        for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
            m_index[*it] = nodes.size();
            nodes.push_back(*it);
            bindWorld.push_back(toGlm((*it)->node_to_world));
            isBone.push_back((*it)->bone != nullptr ? 1 : 0);
        }
        return m_index.at(node);
    }

    [[nodiscard]] const usize* find(const ufbx_node* node) const
    {
        const auto it = m_index.find(node);
        return it != m_index.end() ? &it->second : nullptr;
    }

    std::vector<const ufbx_node*> nodes;
    // World transform of each joint in the bind pose: the skin cluster's bind matrix for bones,
    // the node's transform as loaded for everything else. The rest pose is derived from these,
    // because node transforms as loaded may already be posed by the file's current take.
    std::vector<glm::mat4> bindWorld;
    std::vector<u8> isBone;

private:
    std::unordered_map<const ufbx_node*, usize> m_index;
};

void appendMesh(const ufbx_node& node, JointBuilder& joints, anim::SkinnedMesh& out)
{
    const ufbx_mesh& mesh = *node.mesh;
    const ufbx_skin_deformer* skin = mesh.skin_deformers.count > 0 ? mesh.skin_deformers.data[0] : nullptr;

    // Joint per skin cluster; the cluster's bind matrix is that joint's bind-pose world transform.
    std::vector<usize> clusterJoint;
    glm::mat4 geometryToBindWorld = toGlm(node.geometry_to_world);
    if (skin) {
        bool haveBindWorld = false;
        for (const ufbx_skin_cluster* cluster : skin->clusters) {
            if (!cluster->bone_node) {
                clusterJoint.push_back(joints.add(&node));
                continue;
            }
            const usize joint = joints.add(cluster->bone_node);
            joints.bindWorld[joint] = toGlm(cluster->bind_to_world);
            joints.isBone[joint] = 1;
            clusterJoint.push_back(joint);
            if (!haveBindWorld) {
                // geometry -> bone at bind time, then bone -> world at bind time.
                geometryToBindWorld = toGlm(cluster->bind_to_world) * toGlm(cluster->geometry_to_bone);
                haveBindWorld = true;
            }
        }
    }
    // Rigid meshes (and unweighted vertices of skinned ones) follow the mesh's own node.
    std::optional<usize> rigidJoint;
    const auto rigid = [&]() {
        if (!rigidJoint) {
            rigidJoint = joints.add(&node);
        }
        return static_cast<u32>(*rigidJoint);
    };

    const glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(geometryToBindWorld)));
    std::vector<uint32_t> triangle(mesh.max_face_triangles * 3);

    for (const ufbx_face face : mesh.faces) {
        const u32 numTriangles = ufbx_triangulate_face(triangle.data(), triangle.size(), &mesh, face);
        for (u32 i = 0; i < numTriangles * 3; ++i) {
            const uint32_t corner = triangle[i];
            const glm::vec3 position = toGlm(ufbx_get_vertex_vec3(&mesh.vertex_position, corner));
            glm::vec3 normal(0.0f, 1.0f, 0.0f);
            if (mesh.vertex_normal.exists) {
                normal = toGlm(ufbx_get_vertex_vec3(&mesh.vertex_normal, corner));
            }
            normal = normalMatrix * normal;
            const f32 length = glm::length(normal);
            normal = length > 1e-8f ? normal / length : glm::vec3(0.0f, 1.0f, 0.0f);

            glm::uvec4 influenceJoints(0u);
            glm::vec4 influenceWeights(0.0f);
            if (skin) {
                const uint32_t vertex = mesh.vertex_indices.data[corner];
                const ufbx_skin_vertex& sv = skin->vertices.data[vertex];
                // Weights are sorted by decreasing weight: keep the strongest four.
                const u32 count = std::min<u32>(sv.num_weights, static_cast<u32>(anim::kMaxInfluences));
                for (u32 k = 0; k < count; ++k) {
                    const ufbx_skin_weight& weight = skin->weights.data[sv.weight_begin + k];
                    influenceJoints[static_cast<glm::length_t>(k)] = static_cast<u32>(clusterJoint[weight.cluster_index]);
                    influenceWeights[static_cast<glm::length_t>(k)] = static_cast<f32>(weight.weight);
                }
                influenceWeights = anim::normalizeWeights(influenceWeights);
            }
            if (influenceWeights == glm::vec4(0.0f)) {
                influenceJoints = glm::uvec4(rigid(), 0u, 0u, 0u);
                influenceWeights = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
            }

            out.mesh.indices.push_back(static_cast<u32>(out.mesh.positions.size()));
            out.mesh.positions.push_back(glm::vec3(geometryToBindWorld * glm::vec4(position, 1.0f)));
            out.mesh.normals.push_back(normal);
            out.joints.push_back(influenceJoints);
            out.weights.push_back(influenceWeights);
        }
    }
}

template <typename Key, typename T, typename Convert>
anim::Track<T> toTrack(const Key* keys, usize count, Convert convert)
{
    anim::Track<T> track;
    track.times.reserve(count);
    track.values.reserve(count);
    for (usize k = 0; k < count; ++k) {
        const auto time = static_cast<f32>(keys[k].time);
        if (!track.times.empty() && time <= track.times.back()) {
            continue; // keep times strictly increasing
        }
        track.times.push_back(time);
        track.values.push_back(convert(keys[k].value));
    }
    return track;
}

std::vector<anim::AnimationClip> bakeClips(const ufbx_scene& scene, const JointBuilder& joints)
{
    std::vector<anim::AnimationClip> clips;
    for (const ufbx_anim_stack* stack : scene.anim_stacks) {
        ufbx_bake_opts opts{};
        opts.trim_start_time = true;
        ufbx_error error{};
        const std::unique_ptr<ufbx_baked_anim, BakeDeleter> bake(ufbx_bake_anim(&scene, stack->anim, &opts, &error));
        const std::string name = stack->name.length > 0 ? std::string(stack->name.data, stack->name.length)
                                                        : "clip_" + std::to_string(clips.size());
        if (!bake) {
            KN_WARN("FBX: cannot bake animation '{}': {}", name, errorString(error));
            continue;
        }

        std::vector<anim::JointTracks> tracks(joints.nodes.size());
        f32 duration = static_cast<f32>(bake->playback_duration);
        bool animatesSkeleton = false;
        for (const ufbx_baked_node& baked : bake->nodes) {
            const usize* joint = joints.find(scene.nodes.data[baked.typed_id]);
            if (!joint) {
                continue;
            }
            animatesSkeleton = true;
            anim::JointTracks& jt = tracks[*joint];
            const auto vec3 = [](const ufbx_vec3& v) { return toGlm(v); };
            const auto quat = [](const ufbx_quat& q) { return glm::normalize(toGlm(q)); };
            jt.translation = toTrack<ufbx_baked_vec3, glm::vec3>(baked.translation_keys.data, baked.translation_keys.count, vec3);
            jt.rotation = toTrack<ufbx_baked_quat, glm::quat>(baked.rotation_keys.data, baked.rotation_keys.count, quat);
            jt.scale = toTrack<ufbx_baked_vec3, glm::vec3>(baked.scale_keys.data, baked.scale_keys.count, vec3);
            for (const auto* track : {&jt.translation.times, &jt.scale.times}) {
                if (!track->empty()) duration = std::max(duration, track->back());
            }
            if (!jt.rotation.empty()) duration = std::max(duration, jt.rotation.times.back());
        }
        if (animatesSkeleton && duration > 0.0f) {
            clips.emplace_back(name, duration, std::move(tracks));
        }
    }
    return clips;
}

} // namespace

std::optional<anim::SkinnedModel> loadSkinnedModelFbx(const std::filesystem::path& path, std::string& error)
{
    ufbx_load_opts opts{};
    opts.target_axes = ufbx_axes_right_handed_y_up;
    opts.target_unit_meters = 1.0;
    // The conversion lives in the root node's transform, which is part of the joint chain, so
    // animation keys stay in their authored (node-local) space.
    opts.space_conversion = UFBX_SPACE_CONVERSION_TRANSFORM_ROOT;
    opts.generate_missing_normals = true;

    ufbx_error ufbxError{};
    const std::unique_ptr<ufbx_scene, SceneDeleter> scene(ufbx_load_file(path.string().c_str(), &opts, &ufbxError));
    if (!scene) {
        error = errorString(ufbxError);
        return std::nullopt;
    }

    bool hasSkin = false;
    for (const ufbx_node* node : scene->nodes) {
        if (node->mesh && node->mesh->num_faces > 0 && node->mesh->skin_deformers.count > 0) {
            hasSkin = true;
        }
    }
    if (!hasSkin && scene->anim_stacks.count == 0) {
        error = "file has no skin and no animation";
        return std::nullopt;
    }

    JointBuilder joints;
    anim::SkinnedModel model;
    for (const ufbx_node* node : scene->nodes) {
        if (node->mesh && node->mesh->num_faces > 0) {
            appendMesh(*node, joints, model.skin);
        }
    }
    if (model.skin.mesh.empty()) {
        error = "file contains no mesh geometry";
        return std::nullopt;
    }

    std::vector<std::string> names;
    std::vector<i32> parents;
    std::vector<glm::mat4> inverseBind;
    anim::Pose restPose(joints.nodes.size());
    for (usize j = 0; j < joints.nodes.size(); ++j) {
        const ufbx_node* node = joints.nodes[j];
        std::string name(node->name.data, node->name.length);
        names.push_back(name.empty() ? (node->is_root ? "Root" : "node_" + std::to_string(j)) : std::move(name));
        const usize* parent = node->parent ? joints.find(node->parent) : nullptr;
        parents.push_back(parent ? static_cast<i32>(*parent) : anim::kNoParent);

        // Rest pose = bind pose, expressed relative to the parent joint.
        const glm::mat4 local = parent ? glm::inverse(joints.bindWorld[*parent]) * joints.bindWorld[j] : joints.bindWorld[j];
        anim::JointTransform t;
        glm::vec3 skew;
        glm::vec4 perspective;
        glm::decompose(local, t.scale, t.rotation, t.translation, skew, perspective);
        restPose.set(j, t);
        inverseBind.push_back(glm::inverse(joints.bindWorld[j]));
    }

    model.clips = bakeClips(*scene, joints);
    if (!hasSkin && model.clips.empty()) {
        error = "file has no skin and no animation"; // e.g. an empty default take
        return std::nullopt;
    }
    model.isBone = joints.isBone;
    model.skeleton = anim::Skeleton(std::move(names), std::move(parents), std::move(restPose), std::move(inverseBind));
    return model;
}

} // namespace kinetic::assets::detail
