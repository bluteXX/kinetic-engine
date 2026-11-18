#pragma once

#include "kinetic/core/Types.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <optional>

namespace kinetic {

// Ray `origin + t * direction`, t >= 0. `direction` need not be normalized: intersection
// distances are returned in units of t, so they stay comparable after an affine transform of the
// ray (e.g. into an object's local space).
struct Ray {
    glm::vec3 origin{0.0f};
    glm::vec3 direction{0.0f, 0.0f, -1.0f};

    [[nodiscard]] glm::vec3 at(f32 t) const { return origin + direction * t; }
};

// Slab test. Returns the entry t (0 if the origin is inside the box), or nothing on a miss.
[[nodiscard]] inline std::optional<f32> intersectAabb(const Ray& ray, const glm::vec3& boxMin, const glm::vec3& boxMax)
{
    f32 tMin = 0.0f;
    f32 tMax = INFINITY;
    for (int axis = 0; axis < 3; ++axis) {
        const f32 d = ray.direction[axis];
        const f32 o = ray.origin[axis];
        if (std::abs(d) < 1e-12f) {
            if (o < boxMin[axis] || o > boxMax[axis]) {
                return std::nullopt; // parallel to this slab and outside it
            }
            continue;
        }
        f32 t0 = (boxMin[axis] - o) / d;
        f32 t1 = (boxMax[axis] - o) / d;
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        tMin = std::max(tMin, t0);
        tMax = std::min(tMax, t1);
        if (tMin > tMax) {
            return std::nullopt;
        }
    }
    return tMin;
}

// Möller–Trumbore, two-sided. Returns t of the hit, or nothing.
[[nodiscard]] inline std::optional<f32> intersectTriangle(const Ray& ray, const glm::vec3& a, const glm::vec3& b,
                                                          const glm::vec3& c)
{
    const glm::vec3 e1 = b - a;
    const glm::vec3 e2 = c - a;
    const glm::vec3 p = glm::cross(ray.direction, e2);
    const f32 det = glm::dot(e1, p);
    if (std::abs(det) < 1e-12f) {
        return std::nullopt; // parallel or degenerate
    }
    const f32 inv = 1.0f / det;
    const glm::vec3 s = ray.origin - a;
    const f32 u = glm::dot(s, p) * inv;
    if (u < 0.0f || u > 1.0f) {
        return std::nullopt;
    }
    const glm::vec3 q = glm::cross(s, e1);
    const f32 v = glm::dot(ray.direction, q) * inv;
    if (v < 0.0f || u + v > 1.0f) {
        return std::nullopt;
    }
    const f32 t = glm::dot(e2, q) * inv;
    return t >= 0.0f ? std::optional<f32>(t) : std::nullopt;
}

} // namespace kinetic
