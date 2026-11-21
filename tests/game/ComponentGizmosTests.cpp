#include "ComponentGizmos.h"

#include <kinetic/gameplay/CharacterController.h>

#include <doctest/doctest.h>

#include <algorithm>

using namespace kinetic;

namespace {

void bounds(const std::vector<renderer::DebugVertex>& lines, glm::vec3& lo, glm::vec3& hi)
{
    lo = glm::vec3(1e9f);
    hi = glm::vec3(-1e9f);
    for (const auto& v : lines) {
        lo = glm::min(lo, v.position);
        hi = glm::max(hi, v.position);
    }
}

} // namespace

TEST_CASE("Component gizmos: collider box follows the actor's transform")
{
    scene::Scene scene;
    const entt::entity e = scene.createEntity("Stone");
    auto& t = scene.registry().get<scene::TransformComponent>(e);
    t.position = {2, 0, 0};
    t.scale = glm::vec3(2.0f);
    scene.registry().emplace<scene::ColliderComponent>(e, glm::vec3(0.5f, 0.25f, 0.5f), glm::vec3(0, 0.25f, 0));

    std::vector<renderer::DebugVertex> lines;
    editor::appendComponentGizmos(scene, e, t.matrix(), lines);
    REQUIRE(lines.size() == 24); // 12 edges
    glm::vec3 lo;
    glm::vec3 hi;
    bounds(lines, lo, hi);
    CHECK(lo.x == doctest::Approx(1.0f));
    CHECK(hi.x == doctest::Approx(3.0f));
    CHECK(lo.y == doctest::Approx(0.0f));
    CHECK(hi.y == doctest::Approx(1.0f));
}

TEST_CASE("Component gizmos: controller capsule is in metres, whatever the model's scale")
{
    scene::Scene scene;
    const entt::entity e = scene.createEntity("Hero");
    auto& t = scene.registry().get<scene::TransformComponent>(e);
    t.scale = glm::vec3(1.75f);
    auto& c = scene.registry().emplace<gameplay::CharacterControllerComponent>(e);
    c.height = 1.8f;
    c.radius = 0.3f;

    std::vector<renderer::DebugVertex> lines;
    editor::GizmoOptions options;
    options.footIK = false;
    editor::appendComponentGizmos(scene, e, t.matrix(), lines, options);
    glm::vec3 lo;
    glm::vec3 hi;
    bounds(lines, lo, hi);
    CHECK(lo.y == doctest::Approx(0.0f).epsilon(0.01));
    CHECK(hi.y == doctest::Approx(1.8f).epsilon(0.01));

    // In a preview (model shown unscaled) the capsule shrinks with the model.
    lines.clear();
    editor::appendComponentGizmos(scene, e, glm::mat4(1.0f), lines, options);
    bounds(lines, lo, hi);
    CHECK(hi.y == doctest::Approx(1.8f / 1.75f).epsilon(0.01));
}
