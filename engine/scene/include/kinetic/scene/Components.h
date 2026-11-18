#pragma once

#include "kinetic/core/Types.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <string>

namespace kinetic::scene {

struct NameComponent {
    std::string name;
};

struct TransformComponent {
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};

    [[nodiscard]] glm::mat4 matrix() const;
};

// Perspective camera. It looks down its local -Z axis (right-handed, +Y up); position and
// orientation come from the entity's TransformComponent (scale is ignored).
struct CameraComponent {
    f32 fovY = glm::radians(60.0f); // vertical field of view, radians
    f32 nearPlane = 0.05f;
    f32 farPlane = 500.0f;

    [[nodiscard]] glm::mat4 projection(f32 aspect) const; // depth 0..1 (GLM_FORCE_DEPTH_ZERO_TO_ONE)
    [[nodiscard]] static glm::mat4 view(const TransformComponent& transform);
};

// Meshes the renderer knows how to draw. Grows as soon as meshes can be loaded from glTF.
enum class BuiltinMesh : u8 {
    Triangle,
    Count,
};

inline constexpr std::array<const char*, static_cast<usize>(BuiltinMesh::Count)> kBuiltinMeshNames = {
    "Triangle",
};

[[nodiscard]] constexpr const char* builtinMeshName(BuiltinMesh mesh)
{
    return mesh < BuiltinMesh::Count ? kBuiltinMeshNames[static_cast<usize>(mesh)] : "?";
}

struct MeshRendererComponent {
    BuiltinMesh mesh = BuiltinMesh::Triangle;
    glm::vec4 tint{1.0f}; // RGBA, multiplies the mesh colour
};

} // namespace kinetic::scene
