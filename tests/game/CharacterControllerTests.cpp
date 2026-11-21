#include <kinetic/gameplay/CharacterController.h>
#include <kinetic/scene/AnimatorComponent.h>
#include <kinetic/scene/SceneQueries.h>

#include <doctest/doctest.h>

using namespace kinetic;

namespace {

entt::entity addBox(scene::Scene& scene, const glm::vec3& center, const glm::vec3& half)
{
    const entt::entity e = scene.createEntity("box");
    scene.registry().get<scene::TransformComponent>(e).position = center;
    scene.registry().emplace<scene::ColliderComponent>(e, half, glm::vec3(0.0f));
    return e;
}

entt::entity addCharacter(scene::Scene& scene, const glm::vec3& position)
{
    const entt::entity e = scene.createEntity("hero");
    scene.registry().get<scene::TransformComponent>(e).position = position;
    scene.registry().emplace<gameplay::CharacterControllerComponent>(e);
    return e;
}

void run(scene::Scene& scene, entt::entity e, const gameplay::CharacterInput& input, f32 seconds)
{
    const f32 dt = 1.0f / 60.0f;
    for (f32 t = 0.0f; t < seconds; t += dt) {
        gameplay::updateCharacter(scene, e, input, dt);
    }
}

} // namespace

TEST_CASE("Collider raycast finds the top of a box and its normal")
{
    scene::Scene scene;
    addBox(scene, {0, -0.1f, 0}, {5, 0.1f, 5});
    const auto hit = scene::raycastColliders(scene, Ray{{1, 2, 1}, {0, -1, 0}}, 10.0f);
    REQUIRE(hit.has_value());
    CHECK(hit->point.y == doctest::Approx(0.0f));
    CHECK(hit->normal.y == doctest::Approx(1.0f));
    CHECK_FALSE(scene::raycastColliders(scene, Ray{{1, 2, 1}, {0, -1, 0}}, 1.0f).has_value());
}

TEST_CASE("Character controller walks forward relative to the camera and feeds the blend tree")
{
    scene::Scene scene;
    addBox(scene, {0, -0.1f, 0}, {20, 0.1f, 20});
    const entt::entity hero = addCharacter(scene, {0, 0, 0});
    auto& animator = scene.registry().emplace<scene::AnimatorComponent>(hero);
    animator.blendTree = std::make_shared<anim::BlendTree>();
    animator.blendTree->addParameter("speed", 0.0f, 0.0f, 5.0f);

    run(scene, hero, {.move = {0, 1}, .run = false, .cameraYaw = 0.0f}, 2.0f);
    const auto& t = scene.registry().get<scene::TransformComponent>(hero);
    const auto& c = scene.registry().get<gameplay::CharacterControllerComponent>(hero);
    CHECK(t.position.z > 2.0f); // ~1.5 m/s for 2 s, minus the acceleration
    CHECK(std::abs(t.position.x) < 1e-3f);
    CHECK(t.position.y == doctest::Approx(0.0f));
    CHECK(gameplay::horizontalSpeed(c) == doctest::Approx(c.walkSpeed).epsilon(0.01));
    CHECK(animator.treeState.parameter(*animator.blendTree, "speed") == doctest::Approx(c.walkSpeed).epsilon(0.01));
    // Faces +Z (yaw 0).
    CHECK(std::abs(c.yaw) < 0.05f);

    // Camera turned 90 degrees: forward input now moves along +X.
    run(scene, hero, {.move = {0, 1}, .run = true, .cameraYaw = 1.5707963f}, 2.0f);
    CHECK(t.position.x > 3.0f);
    CHECK(c.yaw == doctest::Approx(1.5707963f).epsilon(0.02));
}

TEST_CASE("Character controller steps onto a low ledge and is blocked by a wall")
{
    scene::Scene scene;
    addBox(scene, {0, -0.1f, 0}, {20, 0.1f, 20});
    addBox(scene, {0, 0.1f, 3}, {2, 0.1f, 1});   // 20 cm ledge from z = 2 to 4
    addBox(scene, {0, 1.0f, -3}, {2, 1.0f, 0.2f}); // 2 m wall at z = -3
    const entt::entity hero = addCharacter(scene, {0, 0, 0});
    const auto& t = scene.registry().get<scene::TransformComponent>(hero);

    run(scene, hero, {.move = {0, 1}}, 2.0f); // walk onto the ledge
    CHECK(t.position.z > 2.2f);
    CHECK(t.position.y == doctest::Approx(0.2f));

    run(scene, hero, {.move = {0, -1}, .run = true}, 4.0f); // run back into the wall
    CHECK(t.position.y == doctest::Approx(0.0f)); // stepped down again
    const f32 wallFace = -3.0f + 0.2f;
    CHECK(t.position.z >= wallFace + 0.29f); // stopped by its radius in front of the wall
    CHECK(t.position.z < wallFace + 0.35f);
}

TEST_CASE("Character controller falls when there is no ground under it")
{
    scene::Scene scene;
    addBox(scene, {0, -0.1f, 0}, {1, 0.1f, 1});
    const entt::entity hero = addCharacter(scene, {0, 0, 0});
    run(scene, hero, {.move = {0, 1}, .run = true}, 1.0f); // runs off the 1 m platform
    const auto& c = scene.registry().get<gameplay::CharacterControllerComponent>(hero);
    CHECK_FALSE(c.grounded);
    CHECK(scene.registry().get<scene::TransformComponent>(hero).position.y < -0.5f);
}

TEST_CASE("Character controller ignores a collider on the character itself")
{
    // Regression: a Box Collider on the hero made the controller push it out of its own box
    // every frame - the hero shot away at ~70 m/s when Play started.
    scene::Scene scene;
    addBox(scene, {0, -0.1f, 0}, {20, 0.1f, 20});
    const entt::entity hero = addCharacter(scene, {0, 0, 0});
    scene.registry().emplace<scene::ColliderComponent>(hero, glm::vec3(0.5f), glm::vec3(0.0f));
    run(scene, hero, {}, 1.0f);
    const auto& t = scene.registry().get<scene::TransformComponent>(hero);
    CHECK(glm::length(t.position) < 1e-3f);

    // Its own box is not ground for it either.
    CHECK_FALSE(scene::raycastColliders(scene, Ray{{0, 2, 0}, {0, -1, 0}}, 1.6f, hero).has_value());
    CHECK(scene::raycastColliders(scene, Ray{{0, 2, 0}, {0, -1, 0}}, 1.6f).has_value());
}

TEST_CASE("Character controller walks up a tilted box (ramp) instead of treating it as a wall")
{
    for (const f32 degrees : {12.0f, 22.0f}) {
        scene::Scene scene;
        addBox(scene, {0, -0.1f, 0}, {20, 0.1f, 20});
        const f32 a = glm::radians(degrees);
        const f32 halfLength = 2.4f;
        const entt::entity ramp = addBox(scene, {0, halfLength * std::sin(a) - 0.1f * std::cos(a), 1.2f + halfLength * std::cos(a)},
                                         {1.0f, 0.1f, halfLength});
        scene.registry().get<scene::TransformComponent>(ramp).rotation = glm::angleAxis(-a, glm::vec3(1, 0, 0));
        const entt::entity hero = addCharacter(scene, {0, 0, 0});
        run(scene, hero, {.move = {0, 1}}, 3.5f);
        const auto& t = scene.registry().get<scene::TransformComponent>(hero);
        // Up the slope: height = distance along it * tan(angle).
        const f32 expected = (t.position.z - 1.2f) * std::tan(a);
        MESSAGE(degrees << " deg ramp: z " << t.position.z << ", y " << t.position.y << " (slope height " << expected << ")");
        CHECK(t.position.z > 3.5f);
        CHECK(t.position.y == doctest::Approx(expected).epsilon(0.05));
    }
}
