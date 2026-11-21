#include "ComponentGizmos.h"

#include "CharacterSetup.h"

#include <kinetic/gameplay/CharacterController.h>
#include <kinetic/scene/AnimatorComponent.h>

#include <array>
#include <cmath>

namespace kinetic::editor {

namespace {

constexpr glm::vec3 kColliderColor{0.35f, 0.95f, 0.45f};
constexpr glm::vec3 kControllerColor{0.35f, 0.82f, 1.0f};
constexpr f32 kPi = 3.14159265f;
constexpr int kSegments = 32;

void line(std::vector<renderer::DebugVertex>& out, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
    out.push_back({a, c});
    out.push_back({b, c});
}

glm::vec3 transformPoint(const glm::mat4& m, const glm::vec3& p) { return glm::vec3(m * glm::vec4(p, 1.0f)); }

// Box `center` +- `half` in the actor's local space, placed with `world`.
void box(std::vector<renderer::DebugVertex>& out, const glm::mat4& world, const glm::vec3& center, const glm::vec3& half,
         const glm::vec3& color)
{
    std::array<glm::vec3, 8> c;
    for (int i = 0; i < 8; ++i) {
        const glm::vec3 s(i & 1 ? 1.0f : -1.0f, i & 2 ? 1.0f : -1.0f, i & 4 ? 1.0f : -1.0f);
        c[static_cast<usize>(i)] = transformPoint(world, center + s * half);
    }
    static constexpr int kEdges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                                          {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto& e : kEdges) {
        line(out, c[static_cast<usize>(e[0])], c[static_cast<usize>(e[1])], color);
    }
}

// Circle in the local XZ plane at height y.
void ring(std::vector<renderer::DebugVertex>& out, const glm::mat4& world, f32 y, f32 radius, const glm::vec3& color)
{
    glm::vec3 previous = transformPoint(world, {radius, y, 0.0f});
    for (int i = 1; i <= kSegments; ++i) {
        const f32 a = 2.0f * kPi * static_cast<f32>(i) / kSegments;
        const glm::vec3 p = transformPoint(world, {radius * std::cos(a), y, radius * std::sin(a)});
        line(out, previous, p, color);
        previous = p;
    }
}

// Half circle in a vertical plane (direction `axis` in XZ), bulging up (+1) or down (-1).
void arc(std::vector<renderer::DebugVertex>& out, const glm::mat4& world, f32 y, f32 radius, const glm::vec3& axis,
         f32 up, const glm::vec3& color)
{
    glm::vec3 previous = transformPoint(world, glm::vec3(0.0f, y, 0.0f) + axis * radius);
    for (int i = 1; i <= kSegments / 2; ++i) {
        const f32 a = kPi * static_cast<f32>(i) / (kSegments / 2);
        const glm::vec3 p = transformPoint(world, glm::vec3(0.0f, y + up * radius * std::sin(a), 0.0f) + axis * (radius * std::cos(a)));
        line(out, previous, p, color);
        previous = p;
    }
}

} // namespace

void appendComponentGizmos(const scene::Scene& scene, entt::entity actor, const glm::mat4& world,
                           std::vector<renderer::DebugVertex>& out, const GizmoOptions& options)
{
    const auto& registry = scene.registry();
    if (!registry.valid(actor)) {
        return;
    }
    const f32 boost = options.highlight ? 1.0f : 0.8f;

    if (options.collider) {
        if (const auto* collider = registry.try_get<scene::ColliderComponent>(actor)) {
            box(out, world, collider->offset, collider->halfExtents, kColliderColor * boost);
        }
    }

    if (options.controller) {
        if (const auto* c = registry.try_get<gameplay::CharacterControllerComponent>(actor)) {
            // The capsule is in metres around the actor's origin, upright: undo the actor's scale and
            // rotation so it stays a true, vertical capsule (only the yaw is kept for the arrow).
            const auto& t = registry.get<scene::TransformComponent>(actor);
            const f32 scale = std::abs(t.scale.y) > 1e-6f ? std::abs(t.scale.y) : 1.0f;
            const glm::vec3 origin = transformPoint(world, glm::vec3(0.0f));
            const f32 worldScale = glm::length(glm::vec3(world[1])) / scale; // 1 in the scene, 1/scale in a preview
            glm::mat4 upright(1.0f);
            upright[3] = glm::vec4(origin, 1.0f);
            for (int i = 0; i < 3; ++i) upright[i][i] = worldScale;
            const f32 r = std::min(c->radius, c->height * 0.5f);
            const f32 bottom = r;
            const f32 top = std::max(c->height - r, r);
            const glm::vec3 color = kControllerColor * boost;
            ring(out, upright, bottom, r, color);
            ring(out, upright, top, r, color);
            for (const glm::vec3& axis : {glm::vec3(1, 0, 0), glm::vec3(0, 0, 1)}) {
                arc(out, upright, top, r, axis, 1.0f, color);
                arc(out, upright, bottom, r, axis, -1.0f, color);
                line(out, transformPoint(upright, glm::vec3(0, bottom, 0) + axis * r), transformPoint(upright, glm::vec3(0, top, 0) + axis * r), color);
                line(out, transformPoint(upright, glm::vec3(0, bottom, 0) - axis * r), transformPoint(upright, glm::vec3(0, top, 0) - axis * r), color);
            }
            // Step height and the facing direction on the ground.
            ring(out, upright, c->stepHeight, r * 1.15f, color * 0.6f);
            const glm::vec3 forward = glm::normalize(glm::vec3(world * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f)));
            const glm::vec3 f(forward.x, 0.0f, forward.z);
            if (glm::length(f) > 1e-4f) {
                const glm::vec3 dir = glm::normalize(f);
                const glm::vec3 side(-dir.z, 0.0f, dir.x);
                const f32 len = (r + 0.25f) * worldScale;
                const glm::vec3 base = origin + glm::vec3(0.0f, 0.02f * worldScale, 0.0f);
                const glm::vec3 tip = base + dir * len;
                line(out, base, tip, color);
                line(out, tip, tip - dir * 0.08f * worldScale + side * 0.05f * worldScale, color);
                line(out, tip, tip - dir * 0.08f * worldScale - side * 0.05f * worldScale, color);
            }
        }
    }

    if (options.footIK) {
        if (const auto* animator = registry.try_get<scene::AnimatorComponent>(actor)) {
            // Foot IK debug is stored in world space (scene only).
            const auto& t = registry.get<scene::TransformComponent>(actor);
            if (glm::length(glm::vec3(world[3]) - t.position) < 1e-4f) {
                appendFootIKLines(*animator, out);
            }
        }
    }
}

void appendAllColliders(const scene::Scene& scene, std::vector<renderer::DebugVertex>& out)
{
    const auto view = scene.registry().view<const scene::ColliderComponent, const scene::TransformComponent>();
    for (const entt::entity e : view) {
        const auto& [collider, t] = view.get(e);
        box(out, t.matrix(), collider.offset, collider.halfExtents, kColliderColor * 0.55f);
    }
}

} // namespace kinetic::editor
