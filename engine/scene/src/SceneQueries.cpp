#include "kinetic/scene/SceneQueries.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kinetic::scene {

namespace {

// Ray (local space, box centred at the origin) against a box; entry distance and face normal.
std::optional<std::pair<f32, glm::vec3>> intersectBox(const glm::vec3& origin, const glm::vec3& dir, const glm::vec3& half)
{
    f32 tMin = -std::numeric_limits<f32>::infinity();
    f32 tMax = std::numeric_limits<f32>::infinity();
    int axisMin = -1;
    f32 signMin = 0.0f;
    for (int a = 0; a < 3; ++a) {
        if (std::abs(dir[a]) < 1e-9f) {
            if (origin[a] < -half[a] || origin[a] > half[a]) {
                return std::nullopt;
            }
            continue;
        }
        const f32 inv = 1.0f / dir[a];
        f32 t0 = (-half[a] - origin[a]) * inv;
        f32 t1 = (half[a] - origin[a]) * inv;
        f32 sign = -1.0f; // entering through the -half face
        if (t0 > t1) {
            std::swap(t0, t1);
            sign = 1.0f;
        }
        if (t0 > tMin) {
            tMin = t0;
            axisMin = a;
            signMin = sign;
        }
        tMax = std::min(tMax, t1);
        if (tMin > tMax) {
            return std::nullopt;
        }
    }
    if (tMin < 0.0f || axisMin < 0) {
        return std::nullopt; // starts inside (or behind)
    }
    glm::vec3 normal(0.0f);
    normal[axisMin] = signMin;
    return std::pair{tMin, normal};
}

} // namespace

std::optional<ColliderHit> raycastColliders(const Scene& scene, const Ray& ray, f32 maxDistance, entt::entity ignore)
{
    std::optional<ColliderHit> best;
    const auto view = scene.registry().view<const ColliderComponent, const TransformComponent>();
    for (const entt::entity e : view) {
        if (e == ignore) {
            continue;
        }
        const auto& [collider, t] = view.get(e);
        const glm::vec3 half = glm::max(collider.halfExtents * glm::abs(t.scale), glm::vec3(1e-4f));
        const glm::vec3 center = t.position + t.rotation * (collider.offset * t.scale);
        const glm::quat inv = glm::conjugate(t.rotation);
        const auto hit = intersectBox(inv * (ray.origin - center), inv * ray.direction, half);
        if (!hit || hit->first > maxDistance || (best && hit->first >= best->distance)) {
            continue;
        }
        best = ColliderHit{e, hit->first, ray.at(hit->first), glm::normalize(t.rotation * hit->second)};
    }
    return best;
}

std::optional<ColliderHit> raycastCollider(const Scene& scene, entt::entity entity, const Ray& ray, f32 maxDistance)
{
    const auto& registry = scene.registry();
    const auto* collider = registry.try_get<ColliderComponent>(entity);
    const auto* t = registry.try_get<TransformComponent>(entity);
    if (!collider || !t) {
        return std::nullopt;
    }
    const glm::vec3 half = glm::max(collider->halfExtents * glm::abs(t->scale), glm::vec3(1e-4f));
    const glm::vec3 center = t->position + t->rotation * (collider->offset * t->scale);
    const glm::quat inv = glm::conjugate(t->rotation);
    const auto hit = intersectBox(inv * (ray.origin - center), inv * ray.direction, half);
    if (!hit || hit->first > maxDistance) {
        return std::nullopt;
    }
    return ColliderHit{entity, hit->first, ray.at(hit->first), glm::normalize(t->rotation * hit->second)};
}

bool hasColliders(const Scene& scene)
{
    const auto view = scene.registry().view<const ColliderComponent>();
    return view.begin() != view.end();
}

namespace {

bool hasOtherColliders(const Scene& scene, entt::entity ignore)
{
    const auto view = scene.registry().view<const ColliderComponent>();
    return std::ranges::any_of(view, [ignore](entt::entity e) { return e != ignore; });
}

} // namespace

anim::GroundQuery colliderGroundQuery(const Scene& scene, entt::entity ignore)
{
    return [&scene, ignore](const glm::vec3& from, f32 maxDistance) -> std::optional<anim::GroundHit> {
        if (!hasOtherColliders(scene, ignore)) {
            if (from.y >= 0.0f && from.y - maxDistance <= 0.0f) {
                return anim::GroundHit{{from.x, 0.0f, from.z}, {0.0f, 1.0f, 0.0f}};
            }
            return std::nullopt;
        }
        const auto hit = raycastColliders(scene, Ray{from, {0.0f, -1.0f, 0.0f}}, maxDistance, ignore);
        if (!hit) {
            return std::nullopt;
        }
        return anim::GroundHit{hit->point, hit->normal};
    };
}

} // namespace kinetic::scene
