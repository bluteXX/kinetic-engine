#include "kinetic/core/MeshData.h"

#include <cmath>

namespace kinetic {

namespace {

// Any unit vector perpendicular to n.
glm::vec3 perpendicular(const glm::vec3& n)
{
    const glm::vec3 axis = std::abs(n.x) < 0.9f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    return glm::normalize(glm::cross(axis, n));
}

} // namespace

void computeTangents(MeshData& mesh)
{
    const usize n = mesh.positions.size();
    mesh.tangents.assign(n, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    if (mesh.normals.size() != n) {
        return;
    }
    std::vector<glm::vec3> tan(n, glm::vec3(0.0f));
    std::vector<glm::vec3> bitan(n, glm::vec3(0.0f));
    if (mesh.hasUvs()) {
        for (usize t = 0; t + 2 < mesh.indices.size(); t += 3) {
            const u32 i0 = mesh.indices[t];
            const u32 i1 = mesh.indices[t + 1];
            const u32 i2 = mesh.indices[t + 2];
            if (i0 >= n || i1 >= n || i2 >= n) {
                continue;
            }
            const glm::vec3 e1 = mesh.positions[i1] - mesh.positions[i0];
            const glm::vec3 e2 = mesh.positions[i2] - mesh.positions[i0];
            const glm::vec2 d1 = mesh.uvs[i1] - mesh.uvs[i0];
            const glm::vec2 d2 = mesh.uvs[i2] - mesh.uvs[i0];
            const f32 det = d1.x * d2.y - d2.x * d1.y;
            if (std::abs(det) < 1e-12f) {
                continue;
            }
            const f32 r = 1.0f / det;
            // Weighted by triangle area implicitly (edges are not normalised).
            const glm::vec3 sdir = (e1 * d2.y - e2 * d1.y) * r;
            const glm::vec3 tdir = (e2 * d1.x - e1 * d2.x) * r;
            for (const u32 i : {i0, i1, i2}) {
                tan[i] += sdir;
                bitan[i] += tdir;
            }
        }
    }
    for (usize v = 0; v < n; ++v) {
        const glm::vec3& normal = mesh.normals[v];
        glm::vec3 t = tan[v] - normal * glm::dot(normal, tan[v]); // Gram-Schmidt
        const f32 length = glm::length(t);
        if (length < 1e-8f) {
            mesh.tangents[v] = glm::vec4(perpendicular(normal), 1.0f);
            continue;
        }
        t /= length;
        // uvs have v pointing down the image, while normal maps (OpenGL convention) store +Y as
        // "up in the image": w makes cross(normal, tangent) * w point up the image (-dP/dv).
        const f32 w = glm::dot(glm::cross(normal, t), bitan[v]) < 0.0f ? 1.0f : -1.0f;
        mesh.tangents[v] = glm::vec4(t, w);
    }
}

} // namespace kinetic
