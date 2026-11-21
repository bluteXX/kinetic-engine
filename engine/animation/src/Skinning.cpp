#include "kinetic/animation/Skinning.h"

#include "kinetic/animation/PoseOps.h"
#include "kinetic/core/Assert.h"

namespace kinetic::anim {

bool SkinnedMesh::isValid(usize jointCount) const
{
    const usize n = mesh.positions.size();
    if (mesh.normals.size() != n || joints.size() != n || weights.size() != n) {
        return false;
    }
    for (usize v = 0; v < n; ++v) {
        for (usize k = 0; k < kMaxInfluences; ++k) {
            if (weights[v][static_cast<glm::length_t>(k)] > 0.0f && joints[v][static_cast<glm::length_t>(k)] >= jointCount) {
                return false;
            }
        }
    }
    return true;
}

glm::vec4 normalizeWeights(const glm::vec4& weights)
{
    const f32 sum = weights.x + weights.y + weights.z + weights.w;
    return sum > 1e-8f ? weights / sum : glm::vec4(0.0f);
}

void skinVertices(const SkinnedMesh& skin, std::span<const glm::mat4> skinMatrices, std::span<glm::vec3> outPositions,
                  std::span<glm::vec3> outNormals, std::span<glm::vec4> outTangents)
{
    const usize n = skin.mesh.positions.size();
    KN_ASSERT(outPositions.size() == n && outNormals.size() == n);
    const bool tangents = !outTangents.empty() && skin.mesh.hasTangents();
    KN_ASSERT(!tangents || outTangents.size() == n);
    KN_ASSERT(skin.joints.size() == n && skin.weights.size() == n);

    for (usize v = 0; v < n; ++v) {
        const glm::uvec4& j = skin.joints[v];
        const glm::vec4& w = skin.weights[v];

        glm::mat4 m(0.0f);
        f32 total = 0.0f;
        for (glm::length_t k = 0; k < static_cast<glm::length_t>(kMaxInfluences); ++k) {
            if (w[k] > 0.0f) {
                m += skinMatrices[j[k]] * w[k];
                total += w[k];
            }
        }
        if (total <= 0.0f) {
            m = glm::mat4(1.0f); // unweighted vertex: stays in its bind position
        }

        outPositions[v] = glm::vec3(m * glm::vec4(skin.mesh.positions[v], 1.0f));
        const glm::vec3 normal = glm::mat3(m) * skin.mesh.normals[v];
        const f32 length = glm::length(normal);
        outNormals[v] = length > 1e-8f ? normal / length : skin.mesh.normals[v];
        if (tangents) {
            const glm::vec4& t = skin.mesh.tangents[v];
            const glm::vec3 tangent = glm::mat3(m) * glm::vec3(t);
            const f32 tl = glm::length(tangent);
            outTangents[v] = tl > 1e-8f ? glm::vec4(tangent / tl, t.w) : t;
        }
    }
}

MeshData skinnedRestMesh(const SkinnedModel& model)
{
    const usize joints = model.skeleton.jointCount();
    std::vector<glm::mat4> world(joints);
    std::vector<glm::mat4> skin(joints);
    localToModel(model.skeleton, model.skeleton.restPose(), world);
    computeSkinningMatrices(model.skeleton, world, skin);

    MeshData posed;
    posed.positions.resize(model.skin.mesh.positions.size());
    posed.normals.resize(posed.positions.size());
    if (model.skin.mesh.hasTangents()) {
        posed.tangents.resize(posed.positions.size());
    }
    posed.uvs = model.skin.mesh.uvs;
    posed.indices = model.skin.mesh.indices;
    posed.material = model.skin.mesh.material;
    skinVertices(model.skin, skin, posed.positions, posed.normals, posed.tangents);
    return posed;
}

void computeRestBounds(SkinnedModel& model)
{
    const MeshData posed = skinnedRestMesh(model);
    model.restBoundsMin = posed.boundsMin();
    model.restBoundsMax = posed.boundsMax();
}

} // namespace kinetic::anim
