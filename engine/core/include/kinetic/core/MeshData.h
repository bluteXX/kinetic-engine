#pragma once

#include "kinetic/core/Types.h"

#include <glm/glm.hpp>

#include <vector>

namespace kinetic {

// CPU-side triangle mesh: one position + normal per vertex, indexed triangle list (CCW = front).
// Produced by the asset loaders, uploaded by the renderer.
struct MeshData {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<u32> indices;

    [[nodiscard]] bool empty() const { return indices.empty(); }
    [[nodiscard]] usize triangleCount() const { return indices.size() / 3; }

    // Axis-aligned bounds of all positions ({0,0,0} for an empty mesh).
    [[nodiscard]] glm::vec3 boundsMin() const;
    [[nodiscard]] glm::vec3 boundsMax() const;
};

inline glm::vec3 MeshData::boundsMin() const
{
    if (positions.empty()) {
        return glm::vec3(0.0f);
    }
    glm::vec3 r = positions[0];
    for (const glm::vec3& p : positions) {
        r = glm::min(r, p);
    }
    return r;
}

inline glm::vec3 MeshData::boundsMax() const
{
    if (positions.empty()) {
        return glm::vec3(0.0f);
    }
    glm::vec3 r = positions[0];
    for (const glm::vec3& p : positions) {
        r = glm::max(r, p);
    }
    return r;
}

} // namespace kinetic
