#pragma once

#include "kinetic/core/Types.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <entt/entity/entity.hpp>

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

// Index into the renderer's texture library. The first two are built in: plain white (no colour
// texture) and a flat normal map (no normal texture).
using TextureId = u32;
inline constexpr TextureId kWhiteTexture = 0;
inline constexpr TextureId kFlatNormalTexture = 1;

struct MeshRendererComponent {
    MeshId mesh = kTriangleMesh;
    glm::vec4 tint{1.0f}; // RGB multiplies the base colour (texture x tint; alpha unused for now)
    bool visible = true;
    // Material
    TextureId baseColorTexture = kWhiteTexture;
    TextureId normalTexture = kFlatNormalTexture;
    f32 normalStrength = 1.0f; // 0 = ignore the normal map
    bool flipNormalY = false;  // normal maps authored for DirectX (green down)
};

// Follows a joint of another entity's animated skeleton (a sword in a hand, a hat on a head):
// every frame, after the animation, its transform becomes parent world * joint * offset. Without
// `inheritScale` the parent's scale is removed, so the attachment keeps its own size in metres.
struct AttachmentComponent {
    entt::entity parent = entt::null;
    i32 joint = -1;          // joint of the parent's AnimatorComponent skeleton (-1 = the parent itself)
    glm::mat4 offset{1.0f};  // in the joint's space
    bool inheritScale = false;
};

// A hand-held weapon (usually also an AttachmentComponent in a hand). Its collision box is given in
// the attachment's own space (metres; the sword mesh: grip at the origin, blade along +Y). In
// Play, the arm holding it is simulated with muscles during attacks (kinetic::physanim).
struct WeaponComponent {
    glm::vec3 halfExtents{0.024f, 0.47f, 0.006f}; // blade box
    glm::vec3 center{0.0f, 0.57f, 0.0f};          // box centre
    glm::vec3 balance{0.0f, 0.12f, 0.0f};          // centre of mass (a sword balances near the guard)
    f32 mass = 1.3f;                               // kg
    bool sharp = true;                             // cuts into soft materials
};

// What a surface is made of: how hard it is, how much it bounces and grips. Hardness decides how
// a blade reacts in physics: hard materials stop / deflect it, soft ones (below 0.65) let it cut in
// against a resistance.
enum class SurfaceMaterial : u8 { Stone, Metal, Wood, Dirt, Flesh, Count };

struct SurfaceProperties {
    const char* name;
    f32 hardness;    // 0..1
    f32 restitution; // bounce
    f32 friction;
};

[[nodiscard]] inline const SurfaceProperties& surfaceProperties(SurfaceMaterial m)
{
    static constexpr SurfaceProperties kTable[] = {
        {"Stone", 1.00f, 0.35f, 0.70f},
        {"Metal", 1.00f, 0.55f, 0.35f},
        {"Wood", 0.75f, 0.20f, 0.60f},
        {"Dirt", 0.45f, 0.05f, 0.90f},
        {"Flesh", 0.25f, 0.02f, 0.80f},
    };
    const auto i = static_cast<usize>(m);
    return kTable[i < static_cast<usize>(SurfaceMaterial::Count) ? i : 0];
}

// Static box collider for gameplay / physics (e.g. floor, stones). `offset` is the box centre
// relative to the entity origin; both scale with the entity's transform.
struct ColliderComponent {
    glm::vec3 halfExtents{0.5f};
    glm::vec3 offset{0.0f};
    SurfaceMaterial material = SurfaceMaterial::Stone;
    // "On hit" event of this object: the effect (assets/vfx/<name>.vfx) played where it is struck.
    // Empty = the material's default ("hit_" + material, e.g. hit_wood); "none" = no effect.
    std::string onHit;
};

// A playable, physically simulated character (see the editor's Play mode). The entity also has
// an AnimatorComponent whose model is the character rig.
struct CharacterComponent {
    f32 walkSpeed = 1.6f;    // m/s
    f32 runSpeed = 3.6f;     // m/s
    f32 strength = 1.0f;     // multiplies every muscle's strength
};

} // namespace kinetic::scene
