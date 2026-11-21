#include "kinetic/physanim/HitTracker.h"

#include <algorithm>
#include <cmath>

namespace kinetic::physanim {

const char* hitResultName(HitResult r)
{
    switch (r) {
    case HitResult::Pending: return "...";
    case HitResult::Rebound: return "REBOUND";
    case HitResult::Glance: return "GLANCE";
    case HitResult::Stop: return "STOP";
    case HitResult::Cut: return "CUT";
    case HitResult::Stuck: return "STUCK";
    }
    return "?";
}

HitTracker::Events HitTracker::update(const physics::PhysicsWorld& world, physics::BodyId bladeId, const SurfaceMap& surfaces,
                                      f32 time)
{
    Events events;
    const physics::BodyState& blade = world.body(bladeId);
    const auto pointVelocity = [&](const glm::vec3& p) {
        return blade.linearVelocity + glm::cross(blade.angularVelocity, p - blade.centerOfMass);
    };

    // Jolt resolves rigid contacts speculatively (before they touch, depth <= 0), so those count
    // for what was hit; soft ones only when the blade is really inside.
    bool touching = false;
    bool soft = false;
    f32 depth = 0.0f;
    u32 other = ~0u;
    f32 closest = -1e9f;
    glm::vec3 point(0.0f);
    glm::vec3 normal(0.0f); // out of the target, towards the blade
    u32 count = 0;
    for (const physics::ContactPoint& c : world.contacts()) {
        const bool isA = c.bodyA.index == bladeId.index;
        if (!isA && c.bodyB.index != bladeId.index) {
            continue;
        }
        if (c.soft && c.depth <= 0.0f) {
            continue;
        }
        if (c.depth > 0.0f) {
            touching = true;
            soft |= c.soft;
            depth = std::max(depth, c.depth);
        }
        if (c.depth > closest) {
            closest = c.depth;
            other = isA ? c.bodyB.index : c.bodyA.index;
        }
        point += c.position;
        normal += isA ? -c.normal : c.normal; // c.normal points from A to B
        ++count;
    }
    if (count > 0) {
        point /= static_cast<f32>(count);
        normal = glm::length(normal) > 1e-6f ? glm::normalize(normal) : glm::vec3(0.0f, 1.0f, 0.0f);
    }
    // Rigid contacts are also seen through the momentum balance.
    touching = touching || (blade.inContact && glm::length(blade.contactForce) > 1.0f);

    if (touching && !m_inContact && m_primed) {
        Hit hit;
        hit.time = time;
        hit.soft = soft;
        hit.point = count > 0 ? point : blade.contactPoint;
        hit.normal = count > 0 ? normal : glm::vec3(0.0f, 1.0f, 0.0f);
        hit.velocityBefore = m_linear + glm::cross(m_angular, hit.point - m_center);
        hit.speed = glm::length(hit.velocityBefore);
        if (hit.speed > 1e-3f) {
            const f32 into = -glm::dot(hit.velocityBefore, hit.normal) / hit.speed;
            hit.angle = glm::degrees(std::asin(std::clamp(into, -1.0f, 1.0f)));
        }
        if (const auto it = surfaces.find(other); it != surfaces.end()) {
            hit.material = it->second;
        } else if (other != ~0u) {
            hit.material = scene::SurfaceMaterial::Flesh; // a character
        }
        if (other != ~0u) {
            hit.target = world.bodyDesc(physics::BodyId{other}).name;
            hit.targetBody = other;
        }
        m_hits.push_back(hit);
        if (m_hits.size() > 16) {
            m_hits.erase(m_hits.begin());
        }
        events.impact = true;
        events.soft = soft;
        m_judged = false;
    }
    m_inContact = touching;

    if (!m_hits.empty() && time - m_hits.back().time < 0.3f) {
        Hit& hit = m_hits.back();
        if (touching) {
            hit.impulse += glm::length(blade.contactForce) * world.lastStep();
            hit.depth = std::max(hit.depth, depth);
        }
        const glm::vec3 now = pointVelocity(hit.point);
        const f32 since = time - hit.time;
        if (hit.result == HitResult::Pending && hit.soft) {
            // In the material: stuck when it stops inside, a cut when it comes out still moving.
            if (touching && glm::length(now) < 0.6f && since > 0.02f) {
                hit.result = HitResult::Stuck;
            } else if (!touching || since > 0.25f) {
                hit.result = HitResult::Cut;
            }
        } else if (hit.result == HitResult::Pending && since >= kResultDelay) {
            // Hard: compare the motion along the normal and along the surface, before and after.
            const f32 inBefore = -glm::dot(hit.velocityBefore, hit.normal);
            const f32 outAfter = glm::dot(now, hit.normal);
            const glm::vec3 slideBefore = hit.velocityBefore + hit.normal * inBefore;
            const glm::vec3 slideAfter = now - hit.normal * outAfter;
            if (hit.angle < 40.0f && glm::length(slideAfter) > 0.5f * glm::length(slideBefore) && glm::length(slideAfter) > 1.0f) {
                hit.result = HitResult::Glance;
            } else if (inBefore > 1e-3f && outAfter > 0.2f * inBefore) {
                hit.result = HitResult::Rebound;
            } else {
                hit.result = HitResult::Stop;
            }
        }
        if (hit.result != HitResult::Pending && !m_judged) {
            m_judged = true;
            hit.speedAfter = glm::length(now);
            events.judged = true;
            events.result = hit.result;
            events.soft = hit.soft;
        }
    }
    m_linear = blade.linearVelocity;
    m_angular = blade.angularVelocity;
    m_center = blade.centerOfMass;
    m_primed = true;
    return events;
}

void HitTracker::recordLoads(std::span<const f32> loads, f32 time)
{
    if (m_hits.empty() || time - m_hits.back().time >= 0.3f) {
        return;
    }
    Hit& hit = m_hits.back();
    for (usize i = 0; i < std::min(loads.size(), hit.peakLoad.size()); ++i) {
        hit.peakLoad[i] = std::max(hit.peakLoad[i], loads[i]);
    }
}

} // namespace kinetic::physanim
