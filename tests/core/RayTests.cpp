#include <doctest/doctest.h>

#include <kinetic/core/Ray.h>

using namespace kinetic;

TEST_CASE("intersectAabb")
{
    const glm::vec3 lo(-1.0f);
    const glm::vec3 hi(1.0f);
    const auto hit = intersectAabb({{0, 0, 5}, {0, 0, -1}}, lo, hi);
    REQUIRE(hit.has_value());
    CHECK(*hit == doctest::Approx(4.0f));

    CHECK(intersectAabb({{0, 0, 0}, {1, 0, 0}}, lo, hi).value_or(-1.0f) == doctest::Approx(0.0f)); // starts inside
    CHECK_FALSE(intersectAabb({{0, 3, 5}, {0, 0, -1}}, lo, hi).has_value());       // parallel, outside
    CHECK_FALSE(intersectAabb({{0, 0, 5}, {0, 0, 1}}, lo, hi).has_value());        // box behind the ray

    // Unnormalized direction: t is in ray units.
    const auto scaled = intersectAabb({{0, 0, 5}, {0, 0, -2}}, lo, hi);
    REQUIRE(scaled.has_value());
    CHECK(*scaled == doctest::Approx(2.0f));
}

TEST_CASE("intersectTriangle is two-sided and rejects misses")
{
    const glm::vec3 a(0, 0, 0);
    const glm::vec3 b(1, 0, 0);
    const glm::vec3 c(0, 1, 0);
    const auto front = intersectTriangle({{0.25f, 0.25f, 2}, {0, 0, -1}}, a, b, c);
    REQUIRE(front.has_value());
    CHECK(*front == doctest::Approx(2.0f));
    const auto back = intersectTriangle({{0.25f, 0.25f, -3}, {0, 0, 1}}, a, b, c);
    REQUIRE(back.has_value());
    CHECK(*back == doctest::Approx(3.0f));

    CHECK_FALSE(intersectTriangle({{0.8f, 0.8f, 2}, {0, 0, -1}}, a, b, c).has_value()); // outside the edge
    CHECK_FALSE(intersectTriangle({{0.25f, 0.25f, 2}, {0, 0, 1}}, a, b, c).has_value()); // pointing away
    CHECK_FALSE(intersectTriangle({{0, 0, 2}, {1, 0, 0}}, a, b, c).has_value());         // parallel
}
