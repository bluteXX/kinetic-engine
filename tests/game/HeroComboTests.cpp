// Dark-Souls-style combo on the real warrior with the Mixamo attack clips (skipped when the assets
// are not present): import + retarget -> combo graph -> Play mode with LMB presses.

#include "CharacterSetup.h"
#include "PlayMode.h"

#include <kinetic/animation/ClipTools.h>
#include <kinetic/animation/Combat.h>
#include <kinetic/animation/PoseOps.h>
#include <kinetic/assets/ModelLoader.h>
#include <kinetic/gameplay/CharacterController.h>
#include <kinetic/scene/AnimatorSystem.h>

#include <doctest/doctest.h>

#include <filesystem>
#include <string>

using namespace kinetic;

namespace {

const std::filesystem::path kWarriorDir = std::filesystem::path(KINETIC_TEST_ASSETS_DIR) / "models" / "Warrior";

std::shared_ptr<anim::SkinnedModel> loadWarriorWithAnimations()
{
    auto loaded = assets::loadSkinnedModel(kWarriorDir / "Warrior.fbx");
    REQUIRE(loaded.has_value());
    loaded->sourcePath.clear(); // no saved actor settings in tests
    auto model = std::make_shared<anim::SkinnedModel>(std::move(*loaded));
    for (const char* file : {"Attack1_CrossSlash.fbx", "Attack2_DownwardSlash.fbx", "Attack3_PowerSlash.fbx", "Idle_Combat.fbx"}) {
        std::string error;
        const auto added = assets::importAnimationsInto(*model, kWarriorDir / "animations" / file, &error);
        REQUIRE_MESSAGE(added.size() == 1, error);
    }
    for (const std::string& line : anim::prepareLocomotionClips(*model).lines) {
        MESSAGE(line);
    }
    model->locomotionPrepared = true;
    return model;
}

const char* stateName(const scene::AnimatorComponent& a)
{
    const anim::AnimState* s = a.treeState.currentState(*a.blendTree);
    return s ? s->name.c_str() : "-";
}

} // namespace

TEST_CASE("Combo: Mixamo attacks retarget onto the Tripo warrior with feet on the ground")
{
    if (!std::filesystem::exists(kWarriorDir / "animations" / "Attack1_CrossSlash.fbx")) {
        MESSAGE("Warrior animations not present - skipped");
        return;
    }
    const auto model = loadWarriorWithAnimations();
    const auto legs = anim::findLegChains(model->skeleton);
    REQUIRE(legs.size() == 2);
    const i32 hips = anim::findPelvis(model->skeleton, legs);
    std::vector<glm::mat4> rest(model->skeleton.jointCount());
    anim::localToModel(model->skeleton, model->skeleton.restPose(), rest);

    const std::vector<i32> attacks = anim::findAttackClips(model->clips);
    REQUIRE(attacks.size() == 3);
    CHECK(model->clips[static_cast<usize>(attacks[0])].name() == "Attack1_CrossSlash");
    CHECK(anim::findCombatIdle(model->clips) >= 0);

    // In every attack: the lowest foot stays near the floor, the hips at a human height.
    std::vector<glm::mat4> m(model->skeleton.jointCount());
    for (const i32 a : attacks) {
        const anim::AnimationClip& clip = model->clips[static_cast<usize>(a)];
        f32 lowestFoot = 10.0f;
        f32 highestFoot = -10.0f;
        f32 hipsMin = 10.0f;
        for (int i = 0; i <= 30; ++i) {
            anim::Pose p = model->skeleton.restPose();
            clip.sample(clip.duration() * static_cast<f32>(i) / 30.0f, false, p);
            anim::localToModel(model->skeleton, p, m);
            const f32 low = std::min(m[legs[0].foot][3].y, m[legs[1].foot][3].y);
            lowestFoot = std::min(lowestFoot, low);
            highestFoot = std::max(highestFoot, low);
            hipsMin = std::min(hipsMin, m[static_cast<usize>(hips)][3].y);
        }
        MESSAGE(clip.name() << ": planted foot height " << lowestFoot << ".." << highestFoot << ", lowest hips "
                            << hipsMin << " (rest " << rest[static_cast<usize>(hips)][3].y << "), root motion "
                            << glm::length(clip.sampleRootMotion(clip.duration())));
        CHECK(lowestFoot > -0.05f);  // never through the floor
        CHECK(lowestFoot < 0.15f);   // and standing on it at some point (ankle height ~0.08)
        CHECK(hipsMin > rest[static_cast<usize>(hips)][3].y * 0.35f); // lunges go low, not into the ground
        CHECK(clip.hasRootMotion());
    }
}

TEST_CASE("Combo: buffered presses chain the attacks, without input it returns to the combat idle")
{
    if (!std::filesystem::exists(kWarriorDir / "animations" / "Attack1_CrossSlash.fbx")) {
        return;
    }
    const auto model = loadWarriorWithAnimations();

    scene::Scene scene;
    const entt::entity camera = scene.createCamera("Camera", {0, 1.6f, -4}, {0, 1, 0});
    const entt::entity floor = scene.createEntity("Floor");
    scene.registry().emplace<scene::ColliderComponent>(floor, glm::vec3(20, 0.1f, 20), glm::vec3(0, -0.1f, 0));
    const entt::entity hero = scene.createEntity("Warrior");
    auto& animator = scene.registry().emplace<scene::AnimatorComponent>(hero);
    animator.model = model;
    animator.player.play(0);
    const editor::HeroSetup setup = editor::makePlayableHero(scene, hero, 1.8f);
    MESSAGE(setup.message);
    REQUIRE(setup.ok);
    REQUIRE(animator.blendTree->stateMachine.states.size() >= 5); // Locomotion, Jump, Combat Idle, Attack 1-3

    editor::PlayMode play;
    play.start(scene, camera);
    const auto& t = scene.registry().get<scene::TransformComponent>(hero);
    const f32 dt = 1.0f / 60.0f;
    std::vector<std::string> visited;
    const auto step = [&](editor::PlayInput input) {
        play.update(dt, input, scene); // evaluates the animation too
        const std::string s = stateName(animator);
        if (visited.empty() || visited.back() != s) visited.push_back(s);
    };
    const auto run = [&](f32 seconds, editor::PlayInput input = {}) {
        for (f32 time = 0.0f; time < seconds; time += dt) step(input);
    };

    run(0.3f);
    CHECK(std::string(stateName(animator)) == "Locomotion");

    // One press: Attack 1, played out into the combat idle.
    step(editor::PlayInput{.attack = true});
    CHECK(std::string(stateName(animator)) == "Attack 1");
    const glm::vec3 before = t.position;
    run(2.2f);
    CHECK(std::string(stateName(animator)) == "Combat Idle");
    MESSAGE("attack 1 moved the hero " << glm::length(t.position - before) << " m by root motion");

    // Mash (a press every 0.3 s, like a player): the three attacks chain in order; buffered
    // presses wait for each combo window. Then no input: back to the combat idle.
    visited.clear();
    for (int k = 0; k < 14; ++k) {
        step(editor::PlayInput{.attack = true});
        run(0.3f);
    }
    std::string path;
    for (const std::string& s : visited) path += s + " > ";
    MESSAGE("mashing: " << path);
    REQUIRE(visited.size() >= 3);
    CHECK(visited[0] == "Attack 1");
    CHECK(visited[1] == "Attack 2");
    CHECK(visited[2] == "Attack 3");
    run(4.0f);
    CHECK(std::string(stateName(animator)) == "Combat Idle");

    // A press too early (outside the buffer) is dropped: no combo.
    visited.clear();
    run(0.5f, editor::PlayInput{.move = {0, 1}}); // walk off, back to locomotion
    step(editor::PlayInput{.attack = true});
    step(editor::PlayInput{.attack = true}); // pressed right away: expires before the window opens
    run(2.5f);
    CHECK(std::ranges::find(visited, std::string("Attack 2")) == visited.end());

    // Movement is locked while attacking, and moving after the recovery point cancels into locomotion.
    visited.clear();
    run(0.5f);
    step(editor::PlayInput{.attack = true});
    const glm::vec3 start = t.position;
    run(0.3f, editor::PlayInput{.move = {0, 1}, .run = true});
    const f32 movedEarly = glm::length(t.position - start);
    MESSAGE("moved during the first 0.3 s of an attack while running: " << movedEarly << " m (root motion only)");
    CHECK(movedEarly < 0.5f);
    run(1.5f, editor::PlayInput{.move = {0, 1}});
    CHECK(std::string(stateName(animator)) == "Locomotion");
    play.stop(scene);
}

TEST_CASE("Combo: the weapon socket puts the blade away from the body")
{
    if (!std::filesystem::exists(kWarriorDir / "animations" / "Attack1_CrossSlash.fbx")) {
        return;
    }
    const auto model = loadWarriorWithAnimations();
    const i32 hand = anim::findWeaponHand(model->skeleton);
    REQUIRE(hand >= 0);
    CHECK(model->skeleton.names()[static_cast<usize>(hand)] == "mixamorig:RightHand");
    const glm::mat4 socket = anim::weaponSocket(model->skeleton, hand);
    const auto legs = anim::findLegChains(model->skeleton);
    const i32 hips = anim::findPelvis(model->skeleton, legs);
    std::vector<glm::mat4> m(model->skeleton.jointCount());
    for (const i32 c : {anim::findCombatIdle(model->clips), anim::findAttackClips(model->clips)[0]}) {
        const anim::AnimationClip& clip = model->clips[static_cast<usize>(c)];
        anim::Pose p = model->skeleton.restPose();
        clip.sample(clip.duration() * 0.5f, false, p);
        anim::localToModel(model->skeleton, p, m);
        const glm::mat4 grip = m[static_cast<usize>(hand)] * socket;
        const glm::vec3 g(grip[3]);
        const glm::vec3 blade = glm::normalize(glm::vec3(grip[1]));
        const glm::vec3 tip = g + blade * 0.55f; // model units (the warrior is ~1 m tall unscaled)
        const glm::vec3 h(m[static_cast<usize>(hips)][3]);
        const f32 gripDist = glm::length(glm::vec2(g.x - h.x, g.z - h.z));
        const f32 tipDist = glm::length(glm::vec2(tip.x - h.x, tip.z - h.z));
        MESSAGE(clip.name() << ": blade dir (" << blade.x << ", " << blade.y << ", " << blade.z << "), grip " << gripDist
                            << " / tip " << tipDist << " from the hips axis, tip height " << tip.y);
        CHECK(glm::length(tip - h) > glm::length(g - h) * 0.9f); // the blade does not go through the body
    }
}
