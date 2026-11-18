#include <doctest/doctest.h>

#include "CharacterRig.h"
#include "PlayMode.h"

#include <kinetic/scene/AnimatorComponent.h>

using namespace kinetic;
namespace ch = editor::character;

namespace {

struct World {
    scene::Scene scene;
    entt::entity hero = entt::null;
    editor::PlayMode play;

    World()
    {
        const entt::entity floor = scene.createEntity("Floor");
        scene.registry().emplace<scene::ColliderComponent>(floor, glm::vec3(10.0f, 0.1f, 10.0f), glm::vec3(0.0f, -0.1f, 0.0f));

        hero = scene.createEntity("Hero");
        auto& animator = scene.registry().emplace<scene::AnimatorComponent>(hero);
        animator.model = ch::buildModel();
        animator.player.play(ch::kIdle);
        scene.registry().emplace<scene::CharacterComponent>(hero);
    }

    void addStone(const glm::vec3& center, const glm::vec3& half)
    {
        const entt::entity stone = scene.createEntity("Stone");
        scene.registry().get<scene::TransformComponent>(stone).position = center;
        scene.registry().emplace<scene::ColliderComponent>(stone, half, glm::vec3(0.0f));
    }

    void run(f32 seconds, editor::PlayInput input = {})
    {
        constexpr f32 kFrame = 1.0f / 60.0f;
        for (f32 t = 0.0f; t < seconds; t += kFrame) {
            play.update(kFrame, input, scene);
            input.attack = false;
            input.toggleRagdoll = false;
        }
    }

    const editor::PlayCharacter& character() const { return *play.characters().front(); }
    glm::vec3 bodyPosition(u32 joint) const { return play.world().body(character().body(joint)).position; }
};

} // namespace

TEST_CASE("Character model: rigid skin, kinematic and physics clips")
{
    const auto model = ch::buildModel();
    CHECK(model->skeleton.isValid());
    CHECK(model->skeleton.jointCount() == ch::kJointCount);
    CHECK(model->skin.isValid(model->skeleton.jointCount()));
    REQUIRE(model->clips.size() == 3);
    CHECK(model->clips[ch::kIdle].kind() == anim::ClipKind::Kinematic);
    CHECK(model->clips[ch::kWalk].kind() == anim::ClipKind::Kinematic);
    CHECK(model->clips[ch::kStrike].kind() == anim::ClipKind::Physics);
    CHECK(model->restBoundsMax.y == doctest::Approx(1.74f).epsilon(0.02)); // head top
    CHECK(ch::sagittalAngle(ch::sagittal(40.0f)) == doctest::Approx(glm::radians(40.0f)));
}

TEST_CASE("Play: the character walks where it is told, on the floor")
{
    World w;
    w.play.start(w.scene, entt::null);
    REQUIRE(w.play.characters().size() == 1);
    w.run(1.5f, editor::PlayInput{.move = {0.0f, 1.0f}});
    const glm::vec3 root = w.character().rootPosition();
    MESSAGE("after 1.5 s forward: root ", root.x, " ", root.y, " ", root.z);
    CHECK(root.z > 1.5f);
    CHECK(std::abs(root.x) < 0.05f);
    CHECK(root.y == doctest::Approx(0.0f).epsilon(0.01));
    CHECK(w.bodyPosition(ch::kPelvis).y == doctest::Approx(0.95f).epsilon(0.05));
    // The entity follows the simulation.
    CHECK(w.scene.registry().get<scene::TransformComponent>(w.hero).position.z == doctest::Approx(root.z));
    w.play.stop(w.scene);
    CHECK(w.scene.registry().get<scene::TransformComponent>(w.hero).position.z == doctest::Approx(0.0f));
}

TEST_CASE("Play: a physics strike hits a stone, rebounds and returns to the regular animation")
{
    World w;
    // Waist-high stone in front of the sword arm (the character's right is -X).
    w.addStone({-0.21f, 0.45f, 1.0f}, {0.3f, 0.45f, 0.3f});
    w.play.start(w.scene, entt::null);
    w.run(0.2f);
    w.run(1.0f / 60.0f, editor::PlayInput{.attack = true});
    CHECK(w.character().mode() == editor::PlayCharacter::Mode::Attack);

    bool sawReaction = false;
    for (int i = 0; i < 150; ++i) {
        w.run(1.0f / 60.0f);
        sawReaction |= w.character().attackPhase() != editor::PlayCharacter::AttackPhase::Swing;
        for (const physics::ContactPoint& c : w.play.world().contacts()) {
            CHECK(c.depth < 0.03f);
        }
    }
    REQUIRE_FALSE(w.character().hits().empty());
    const auto& hit = w.character().hits().front();
    MESSAGE("hit impulse ", hit.impulse, " N s, loads ", hit.peakLoad[0], " ", hit.peakLoad[1], " ", hit.peakLoad[2]);
    CHECK(hit.impulse > 1.0f);
    CHECK(sawReaction);
    CHECK(w.character().mode() == editor::PlayCharacter::Mode::Locomotion); // arm back on the animation
}

TEST_CASE("Play: ragdoll collapses onto the floor and stands up again")
{
    World w;
    w.play.start(w.scene, entt::null);
    w.run(0.2f);
    w.run(1.0f / 60.0f, editor::PlayInput{.toggleRagdoll = true});
    CHECK(w.character().mode() == editor::PlayCharacter::Mode::Ragdoll);
    w.run(2.5f);
    const f32 pelvis = w.bodyPosition(ch::kPelvis).y;
    const f32 head = w.bodyPosition(ch::kHead).y;
    MESSAGE("ragdoll after 2.5 s: pelvis y ", pelvis, ", head y ", head);
    CHECK(pelvis < 0.45f);
    CHECK(pelvis > 0.0f);
    CHECK(head < 0.6f);

    w.run(1.0f / 60.0f, editor::PlayInput{.toggleRagdoll = true});
    w.run(0.2f);
    CHECK(w.character().mode() == editor::PlayCharacter::Mode::Locomotion);
    CHECK(w.bodyPosition(ch::kPelvis).y == doctest::Approx(0.95f).epsilon(0.05));
}
