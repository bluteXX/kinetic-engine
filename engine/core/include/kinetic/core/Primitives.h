#pragma once

#include "kinetic/core/MeshData.h"

namespace kinetic {

// Procedural meshes (CCW triangles, outward normals), e.g. to visualize physics shapes.

// Axis-aligned box centred at `center`.
[[nodiscard]] MeshData makeBoxMesh(const glm::vec3& halfExtents, const glm::vec3& center = glm::vec3(0.0f));

// Capsule along the Y axis: cylinder of half height `halfHeight` capped by hemispheres.
[[nodiscard]] MeshData makeCapsuleMesh(f32 radius, f32 halfHeight, const glm::vec3& center = glm::vec3(0.0f),
                                       u32 segments = 20, u32 rings = 6);

[[nodiscard]] inline MeshData makeSphereMesh(f32 radius, const glm::vec3& center = glm::vec3(0.0f))
{
    return makeCapsuleMesh(radius, 0.0f, center);
}

} // namespace kinetic
