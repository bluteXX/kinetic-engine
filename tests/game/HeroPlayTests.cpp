// The whole hero pipeline on the real Tripo3D warrior (skipped when the asset is not present):
// import -> Make Playable Hero -> Play mode with WASD input -> character controller -> blend tree
// -> foot IK, all headless.

#include "CharacterSetup.h"
#include "PlayMode.h"

#include <kinetic/assets/ModelLoader.h>
#include <kinetic/gameplay/CharacterController.h>
#include <kinetic/scene/AnimatorSystem.h>

#include <doctest/doctest.h>

#include <filesystem>

using namespace kinetic;

namespace {

const std::filesystem::path kWarrior = std::filesystem::path(KINETIC_TEST_ASSETS_DIR) / "models" / "Warrior" / "Warrior.fbx";

entt::entity addBox(scene::Scene& scene, const char* name, const glm::vec3& position, const glm::vec3& half,
                    const glm::vec3& offset)
{
    const entt::entity e = scene.createEntity(name);
    scene.registry().get<scene::TransformComponent>(e).position = position;
    scene.registry().emplace<scene::ColliderComponent>(e, half, offset);
    return e;
}

f32 nodeWeight(const scene::AnimatorComponent& a, const char* name)
{
    for (usize n = 0; n < a.blendTree->nodes.size(); ++n) {
        if (a.blendTree->nodes[n].name == name) {
            return a.treeState.weight[n];
        }
    }
    return -1.0f;
}

} // namespace

TEST_CASE("Hero: Tripo warrior walks and runs in Play mode with a locomotion blend tree and foot IK")
{
    if (!std::filesystem::exists(kWarrior)) {
        MESSAGE("assets/models/Warrior/Warrior.fbx not present - skipped");
        return;
    }
    auto loaded = assets::loadSkinnedModel(kWarrior);
    REQUIRE(loaded.has_value());
    loaded->sourcePath.clear(); // no saved actor settings in tests
    const auto model = std::make_shared<const anim::SkinnedModel>(std::move(*loaded));

    scene::Scene scene;
    const entt::entity camera = scene.createCamera("Camera", {0, 1.6f, -4}, {0, 1, 0});
    addBox(scene, "Floor", {0, 0, 0}, {10, 0.1f, 10}, {0, -0.1f, 0});
    addBox(scene, "Step", {0, 0, 3.0f}, {1.5f, 0.075f, 1.0f}, {0, 0.075f, 0}); // 15 cm high, z 2..4

    const entt::entity hero = scene.createEntity("Warrior");
    auto& animator = scene.registry().emplace<scene::AnimatorComponent>(hero);
    animator.model = model;
    animator.player.play(0);

    const editor::HeroSetup setup = editor::makePlayableHero(scene, hero, 1.8f);
    MESSAGE(setup.message);
    REQUIRE(setup.ok);
    const auto& transform = scene.registry().get<scene::TransformComponent>(hero);
    CHECK(transform.scale.y * editor::restHeight(*model) == doctest::Approx(1.8f));
    REQUIRE(animator.blendTree);
    REQUIRE(animator.blendTreeActive());
    const anim::BlendTree& graph = *animator.blendTree;
    const anim::BlendNode& locomotion = graph.nodes[static_cast<usize>(graph.root)];
    REQUIRE(locomotion.type == anim::BlendNodeType::Blend2D); // turn x speed (the model has a turn clip)
    const auto child = [&](const char* name) -> const anim::BlendChild* {
        for (const anim::BlendChild& c : locomotion.children) {
            if (graph.nodes[static_cast<usize>(c.node)].name == name) return &c;
        }
        return nullptr;
    };
    REQUIRE(child("Idle"));
    REQUIRE(child("Walk"));
    REQUIRE(child("Run"));
    REQUIRE(child("Turn Left"));
    REQUIRE(child("Turn Right"));
    CHECK(child("Walk")->thresholdY > 0.6f);
    CHECK(child("Run")->thresholdY > child("Walk")->thresholdY * 1.5f);
    CHECK(child("Turn Left")->threshold == doctest::Approx(-child("Turn Right")->threshold));
    // States: Locomotion (default) and Jump.
    REQUIRE(graph.stateMachine.states.size() == 2);
    CHECK(graph.stateMachine.states[static_cast<usize>(graph.stateMachine.defaultState)].name == "Locomotion");
    CHECK(model->locomotionPrepared == false);           // the shared asset is untouched...
    CHECK(animator.model->locomotionPrepared);            // ...the hero got a prepared copy
    CHECK(animator.footIK.enabled);
    const auto& controller = scene.registry().get<gameplay::CharacterControllerComponent>(hero);
    CHECK(controller.walkSpeed == doctest::Approx(child("Walk")->thresholdY));
    CHECK(controller.runSpeed == doctest::Approx(child("Run")->thresholdY));

    editor::PlayMode play;
    play.start(scene, camera);
    REQUIRE(play.heroes().size() == 1);

    const f32 dt = 1.0f / 60.0f;
    const auto frames = [&](const editor::PlayInput& input, f32 seconds, auto&& each) {
        for (f32 t = 0.0f; t < seconds; t += dt) {
            play.update(dt, input, scene); // evaluates the animation too
            each();
        }
    };

    // Standing: idle.
    frames(editor::PlayInput{}, 0.5f, [] {});
    CHECK(nodeWeight(animator, "Idle") == doctest::Approx(1.0f));
    const glm::vec3 start = transform.position;

    // Walk forward (+Z, camera yaw 0) onto the step.
    f32 lowestFootOnStep = 10.0f;
    frames(editor::PlayInput{.move = {0, 1}, .cameraYaw = 0.0f}, 2.6f, [&] {
        if (transform.position.z > 2.4f && transform.position.z < 3.6f) {
            for (const anim::LegChain& leg : animator.legs) {
                const glm::vec3 foot = glm::vec3(transform.matrix() * animator.jointModel[leg.foot][3]);
                lowestFootOnStep = std::min(lowestFootOnStep, foot.y);
            }
        }
    });
    MESSAGE("walked to z = " << transform.position.z << ", y = " << transform.position.y);
    CHECK(transform.position.z > start.z + 2.4f);
    CHECK(transform.position.y == doctest::Approx(0.15f)); // on the step
    CHECK(gameplay::horizontalSpeed(controller) == doctest::Approx(controller.walkSpeed).epsilon(0.02));
    CHECK(nodeWeight(animator, "Walk") > 0.95f);
    // Feet on the step do not sink into it (ankles stay above its top).
    MESSAGE("lowest ankle on the step: " << lowestFootOnStep << " m");
    CHECK(lowestFootOnStep > 0.15f);

    // Run (Shift): the tree moves to the run clip.
    frames(editor::PlayInput{.move = {0, 1}, .run = true, .cameraYaw = 0.0f}, 1.0f, [] {});
    CHECK(gameplay::horizontalSpeed(controller) == doctest::Approx(controller.runSpeed).epsilon(0.02));
    CHECK(nodeWeight(animator, "Run") > 0.95f);

    // Jump (Space): the state machine goes to Jump, then back to Locomotion after landing.
    frames(editor::PlayInput{.jump = true}, 1.0f / 60.0f, [] {});
    frames(editor::PlayInput{}, 0.2f, [] {});
    CHECK(graph.stateMachine.states[static_cast<usize>(animator.treeState.state)].name == "Jump");
    frames(editor::PlayInput{}, 2.5f, [] {});
    CHECK(graph.stateMachine.states[static_cast<usize>(animator.treeState.state)].name == "Locomotion");
    CHECK(scene.registry().get<gameplay::CharacterControllerComponent>(hero).grounded);

    // Turning in place plays the turn clips: the "turn" parameter follows the yaw rate.
    f32 maxTurnWeight = 0.0f;
    frames(editor::PlayInput{.move = {0, -1}, .cameraYaw = 0.0f}, 0.6f, [&] {
        maxTurnWeight = std::max({maxTurnWeight, nodeWeight(animator, "Turn Left"), nodeWeight(animator, "Turn Right")});
    });
    MESSAGE("largest turn clip weight while turning around: " << maxTurnWeight);
    CHECK(maxTurnWeight > 0.3f);

    play.stop(scene);
    CHECK(transform.position.z == doctest::Approx(start.z).epsilon(0.001)); // scene restored
}
