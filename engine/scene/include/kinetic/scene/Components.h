#pragma once

#include "kinetic/core/Types.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

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

// Index into the renderer's mesh library. Mesh 0 is always the built-in triangle; imported
// meshes (FBX / OBJ) are appended after it.
using MeshId = u32;
inline constexpr MeshId kTriangleMesh = 0;

struct MeshRendererComponent {
    MeshId mesh = kTriangleMesh;
    glm::vec4 tint{1.0f}; // RGB multiplies the lit mesh colour (alpha unused for now)
    bool visible = true;
};

// Static box collider for gameplay / physics (e.g. floor, stones). `offset` is the box centre
// relative to the entity origin; both scale with the entity's transform.
struct ColliderComponent {
    glm::vec3 halfExtents{0.5f};
    glm::vec3 offset{0.0f};
};

// A playable, physically simulated character (see the editor's Play mode). The entity also has
// an AnimatorComponent whose model is the character rig.
struct CharacterComponent {
    f32 walkSpeed = 1.6f;    // m/s
    f32 runSpeed = 3.6f;     // m/s
    f32 strength = 1.0f;     // multiplies every muscle's strength
};

} // namespace kinetic::scene
