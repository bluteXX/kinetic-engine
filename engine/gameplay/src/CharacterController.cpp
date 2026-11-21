#include "kinetic/gameplay/CharacterController.h"

#include <kinetic/scene/AnimatorComponent.h>
#include <kinetic/scene/SceneQueries.h>

#include <algorithm>
#include <cmath>

namespace kinetic::gameplay {

namespace {

constexpr f32 kPi = 3.14159265f;
constexpr f32 kSkin = 0.02f; // ground snap tolerance (m)

f32 wrapAngle(f32 a)
{
    while (a > kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

// Pushes a vertical cylinder (feet at p.y, `height` tall) out of every collider box it overlaps
// above step height. Returns the push applied (xz).
glm::vec3 resolveWalls(const scene::Scene& scene, entt::entity self, glm::vec3& p, const CharacterControllerComponent& c)
{
    glm::vec3 total(0.0f);
    const auto view = scene.registry().view<const scene::ColliderComponent, const scene::TransformComponent>();
    for (int iteration = 0; iteration < 3; ++iteration) {
        bool moved = false;
        for (const entt::entity e : view) {
            if (e == self) {
                continue; // a collider on the character itself moves with it: never an obstacle
            }
            const auto& [collider, t] = view.get(e);
            const glm::vec3 half = glm::max(collider.halfExtents * glm::abs(t.scale), glm::vec3(1e-4f));
            const glm::vec3 center = t.position + t.rotation * (collider.offset * t.scale);
            // Vertical extent of the (possibly rotated) box.
            const glm::mat3 r = glm::mat3_cast(t.rotation);
            const f32 extentY = std::abs(r[0].y) * half.x + std::abs(r[1].y) * half.y + std::abs(r[2].y) * half.z;
            const f32 top = center.y + extentY;
            const f32 bottom = center.y - extentY;
            if (top <= p.y + c.stepHeight || bottom >= p.y + c.height) {
                continue; // walkable (low enough to step on) or overhead
            }
            // Tilted boxes (ramps, slopes): what matters is the surface next to the character - the
            // point of the box's upper face nearest to it - not the box's highest corner.
            {
                const glm::quat toLocal = glm::conjugate(t.rotation);
                const glm::vec3 up = toLocal * glm::vec3(0.0f, 1.0f, 0.0f);
                int axis = 0;
                for (int i = 1; i < 3; ++i) {
                    if (std::abs(up[i]) > std::abs(up[axis])) axis = i;
                }
                glm::vec3 nearLocal = glm::clamp(toLocal * (glm::vec3(p.x, center.y, p.z) - center), -half, half);
                nearLocal[axis] = up[axis] > 0.0f ? half[axis] : -half[axis];
                const f32 surface = (center + t.rotation * nearLocal).y;
                if (surface <= p.y + c.stepHeight) {
                    continue; // a slope (or low edge) the character can walk onto
                }
            }
            // Circle vs box in the box's local frame, at the character's mid height.
            const glm::quat inv = glm::conjugate(t.rotation);
            const glm::vec3 probe(p.x, std::clamp(p.y + c.height * 0.5f, bottom, top), p.z);
            const glm::vec3 local = inv * (probe - center);
            const glm::vec3 closest = glm::clamp(local, -half, half);
            glm::vec3 d = local - closest;
            d.y = 0.0f;
            const f32 dist = glm::length(d);
            glm::vec3 pushLocal(0.0f);
            if (dist > 1e-6f) {
                if (dist >= c.radius) {
                    continue;
                }
                pushLocal = d / dist * (c.radius - dist);
            } else {
                // Centre inside the box: leave through the nearest side.
                const f32 px = half.x - std::abs(local.x);
                const f32 pz = half.z - std::abs(local.z);
                pushLocal = px < pz ? glm::vec3((local.x < 0 ? -1.0f : 1.0f) * (px + c.radius), 0.0f, 0.0f)
                                    : glm::vec3(0.0f, 0.0f, (local.z < 0 ? -1.0f : 1.0f) * (pz + c.radius));
            }
            glm::vec3 push = t.rotation * pushLocal;
            push.y = 0.0f;
            p += push;
            total += push;
            moved = true;
        }
        if (!moved) {
            break;
        }
    }
    return total;
}

} // namespace

f32 horizontalSpeed(const CharacterControllerComponent& c) { return glm::length(glm::vec2(c.velocity.x, c.velocity.z)); }

void updateCharacter(scene::Scene& scene, entt::entity entity, const CharacterInput& input, f32 dt)
{
    auto& registry = scene.registry();
    auto* c = registry.try_get<CharacterControllerComponent>(entity);
    auto* transform = registry.try_get<scene::TransformComponent>(entity);
    if (!c || !transform || dt < 0.0f) {
        return;
    }
    if (!c->initialized) {
        const glm::vec3 forward = transform->rotation * glm::vec3(0.0f, 0.0f, 1.0f);
        c->yaw = std::atan2(forward.x, forward.z);
        c->initialized = true;
    }

    // Wanted horizontal velocity, camera relative.
    const glm::vec3 camForward(std::sin(input.cameraYaw), 0.0f, std::cos(input.cameraYaw));
    const glm::vec3 camRight(-std::cos(input.cameraYaw), 0.0f, std::sin(input.cameraYaw));
    glm::vec2 move = input.move;
    if (glm::length(move) > 1.0f) {
        move = glm::normalize(move);
    }
    const glm::vec3 input3 = (camForward * move.y + camRight * move.x) * (input.run ? c->runSpeed : c->walkSpeed);

    // A committed action (attack) of the animation graph: no walking; only a short turn window at
    // its start, and the clip's root motion moves the character.
    auto* animator = registry.try_get<scene::AnimatorComponent>(entity);
    const anim::AnimState* state =
        animator && animator->blendTreeActive() ? animator->treeState.currentState(*animator->blendTree) : nullptr;
    const bool locked = state && state->lockMovement;
    const bool mayTurn = !locked || animator->treeState.stateTime < state->turnWindow;

    // Face the movement.
    const f32 yawBefore = c->yaw;
    f32 alignment = 1.0f;
    if (glm::length(input3) > 0.05f && mayTurn) {
        const f32 target = std::atan2(input3.x, input3.z);
        const f32 diff = wrapAngle(target - c->yaw);
        c->yaw = wrapAngle(c->yaw + std::clamp(diff, -c->turnSpeed * dt, c->turnSpeed * dt));
        alignment = std::cos(wrapAngle(target - c->yaw));
    }
    // Turn first, then go: the character moves where it faces, faster the better it faces the
    // wanted direction (standing still while turning more than 90 degrees). Without this it would
    // slide sideways while the turn is still playing.
    glm::vec3 wanted = input3;
    if (c->faceBeforeMoving) {
        const glm::vec3 facing(std::sin(c->yaw), 0.0f, std::cos(c->yaw));
        wanted = facing * (glm::length(input3) * std::max(alignment, 0.0f));
    }
    if (locked) {
        wanted = glm::vec3(0.0f);
    }

    glm::vec3 horizontal(c->velocity.x, 0.0f, c->velocity.z);
    const glm::vec3 delta = wanted - horizontal;
    const f32 maxChange = (locked ? 4.0f : 1.0f) * c->acceleration * dt; // attacks stop the run quickly
    horizontal += glm::length(delta) > maxChange ? glm::normalize(delta) * maxChange : delta;
    if (dt > 0.0f) {
        const f32 rate = wrapAngle(c->yaw - yawBefore) / dt;
        c->turnRate += (rate - c->turnRate) * (1.0f - std::exp(-12.0f * dt));
    }

    // Move horizontally (plus the animation's root motion of the last update, turned and scaled
    // like the character), then push out of walls (and drop the velocity into them).
    glm::vec3 p = transform->position + horizontal * dt;
    if (animator && animator->blendTreeActive() && c->grounded) {
        const glm::vec3 rm = animator->treeState.rootMotion;
        if (glm::dot(rm, rm) > 0.0f) {
            const f32 scale = std::abs(transform->scale.y);
            p += glm::angleAxis(c->yaw, glm::vec3(0.0f, 1.0f, 0.0f)) * (rm * scale);
            animator->treeState.rootMotion = glm::vec3(0.0f); // consumed
        }
    }
    const glm::vec3 push = resolveWalls(scene, entity, p, *c);
    if (glm::length(push) > 1e-6f) {
        const glm::vec3 n = glm::normalize(push);
        const f32 into = glm::dot(horizontal, n);
        if (into < 0.0f) {
            horizontal -= n * into;
        }
    }

    // Jump, then ground: probe from step height down; snap when close, otherwise fall.
    const anim::GroundQuery ground = scene::colliderGroundQuery(scene, entity);
    f32 vy = c->velocity.y;
    bool jumped = false;
    if (input.jump && c->grounded && c->jumpSpeed > 0.0f && !locked) {
        vy = c->jumpSpeed;
        c->grounded = false;
        jumped = true;
    }
    const f32 fall = std::max(0.0f, -vy * dt);
    const auto hit = ground(p + glm::vec3(0.0f, c->stepHeight, 0.0f), c->stepHeight + fall + (c->grounded ? c->stepHeight : kSkin));
    if (hit && vy <= 0.0f) {
        p.y = hit->point.y;
        vy = 0.0f;
        c->grounded = true;
    } else {
        vy -= c->gravity * dt;
        p.y += vy * dt;
        c->grounded = false;
        if (p.y < -100.0f) { // fell out of the world: back to the origin's floor level
            p = glm::vec3(0.0f);
            vy = 0.0f;
        }
    }

    c->velocity = glm::vec3(horizontal.x, vy, horizontal.z);
    transform->position = p;
    transform->rotation = glm::angleAxis(c->yaw, glm::vec3(0.0f, 1.0f, 0.0f));

    if (animator && animator->blendTree) {
        const anim::BlendTree& graph = *animator->blendTree;
        anim::BlendTreeInstance& params = animator->treeState;
        params.setParameter(graph, c->speedParameter, horizontalSpeed(*c));
        params.setParameter(graph, c->turnParameter, c->turnRate);
        params.setParameter(graph, "grounded", c->grounded ? 1.0f : 0.0f);
        params.setParameter(graph, "moveInput", glm::length(move));
        if (jumped) {
            params.setTrigger(graph, "jump");
        }
        if (input.attack) {
            params.setTrigger(graph, "attack"); // buffered by the graph (the trigger's buffer time)
        }
    }
}

} // namespace kinetic::gameplay
