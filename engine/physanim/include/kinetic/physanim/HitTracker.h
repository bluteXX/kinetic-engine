#pragma once

#include <kinetic/physics/PhysicsWorld.h>
#include <kinetic/scene/Components.h>

#include <array>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace kinetic::physanim {

// What a blade did on a hit. Hard materials are judged kResultDelay after the impact (the
// collision is over, the muscles have not reacted yet); soft ones when the blade stops inside
// (Stuck) or leaves / keeps cutting (Cut).
enum class HitResult : u8 { Pending, Rebound, Glance, Stop, Cut, Stuck };
[[nodiscard]] const char* hitResultName(HitResult result);

inline constexpr f32 kResultDelay = 0.015f; // s

struct Hit {
    f32 time = 0.0f;
    std::string target; // name of the body hit
    u32 targetBody = ~0u; // its physics body index
    scene::SurfaceMaterial material = scene::SurfaceMaterial::Stone;
    bool soft = false;  // the blade cut into it (see physics::kSoftHardness)
    HitResult result = HitResult::Pending;
    f32 impulse = 0.0f;                   // N s
    std::array<f32, 3> peakLoad{};        // muscle load (torque / strength) of the arm's joints, shoulder first
    f32 speed = 0.0f;                     // blade speed at the contact point before the impact, m/s
    f32 speedAfter = 0.0f;                // the same point when the result was decided
    f32 angle = 0.0f;                     // degrees between the blade's motion and the surface (90 = straight in)
    f32 depth = 0.0f;                     // deepest cut, m (soft materials)
    glm::vec3 normal{0.0f};               // surface normal, out of the target
    glm::vec3 point{0.0f};
    glm::vec3 velocityBefore{0.0f};
};

// "On hit" event: something was struck (sent the step the contact starts; the result comes later
// in the hit log). Games react to it: effects (kinetic::vfx), sounds, damage.
struct HitEvent {
    f32 time = 0.0f;
    glm::vec3 point{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f}; // out of the target
    glm::vec3 velocity{0.0f};           // of the striking point, before the hit
    scene::SurfaceMaterial material = scene::SurfaceMaterial::Stone;
    bool soft = false;
    u32 targetBody = ~0u;   // physics body hit
    u32 strikerBody = ~0u;  // the blade (or body part) that hit it
};

// Physics body index -> surface material (static colliders). Bodies not in it count as flesh.
using SurfaceMap = std::unordered_map<u32, scene::SurfaceMaterial>;

// Watches one blade body and logs its hits (newest last, at most 16).
class HitTracker {
public:
    struct Events {
        bool impact = false;  // a new hit started this step
        bool judged = false;  // the last hit got its result this step
        HitResult result = HitResult::Pending;
        bool soft = false;
    };

    // Call after every physics step while the blade is simulated.
    Events update(const physics::PhysicsWorld& world, physics::BodyId blade, const SurfaceMap& surfaces, f32 time);
    // Peak muscle loads of the last hit while it is recent (0.3 s).
    void recordLoads(std::span<const f32> loads, f32 time);
    // The blade stopped being simulated: the next contact is a new hit.
    void reset() { m_inContact = false; m_primed = false; }

    [[nodiscard]] const std::vector<Hit>& hits() const { return m_hits; }
    [[nodiscard]] bool inContact() const { return m_inContact; }

private:
    std::vector<Hit> m_hits;
    bool m_inContact = false;
    bool m_primed = false; // previous-step blade motion is valid
    bool m_judged = true;  // the last hit has its result
    glm::vec3 m_linear{0.0f};
    glm::vec3 m_angular{0.0f};
    glm::vec3 m_center{0.0f};
};

} // namespace kinetic::physanim
