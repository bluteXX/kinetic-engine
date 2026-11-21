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

    void addStone(const glm::vec3& center, const glm::vec3& half,
                  scene::SurfaceMaterial material = scene::SurfaceMaterial::Stone, const glm::quat& rotation = {1, 0, 0, 0})
    {
        const entt::entity stone = scene.createEntity("Target");
        scene.registry().get<scene::TransformComponent>(stone).position = center;
        scene.registry().get<scene::TransformComponent>(stone).rotation = rotation;
        scene.registry().emplace<scene::ColliderComponent>(stone, half, glm::vec3(0.0f), material);
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

namespace {

// One strike at a waist-high target of `material` (the Materials test scene's placement).
editor::PlayCharacter::Hit strikeAt(scene::SurfaceMaterial material, bool* sawReflexAtImpact = nullptr,
                                    const glm::quat& rotation = {1, 0, 0, 0}, glm::vec3 center = {-0.21f, 0.45f, 1.0f},
                                    glm::vec3 half = {0.3f, 0.45f, 0.3f})
{
    World w;
    w.addStone(center, half, material, rotation);
    w.play.start(w.scene, entt::null);
    w.run(0.2f);
    w.run(1.0f / 60.0f, editor::PlayInput{.attack = true});
    bool reflexAtImpact = false;
    for (int i = 0; i < 150; ++i) {
        const usize before = w.character().hits().size();
        w.run(1.0f / 60.0f);
        if (before == 0 && !w.character().hits().empty()) {
            reflexAtImpact = w.character().attackPhase() != editor::PlayCharacter::AttackPhase::Swing;
        }
    }
    if (sawReflexAtImpact) *sawReflexAtImpact = reflexAtImpact;
    REQUIRE_FALSE(w.character().hits().empty());
    CHECK(w.character().mode() == editor::PlayCharacter::Mode::Locomotion); // the arm comes back every time
    return w.character().hits().front();
}

} // namespace

TEST_CASE("Play: the same strike reacts differently to stone, metal, wood, dirt and flesh")
{
    using M = scene::SurfaceMaterial;
    using R = editor::PlayCharacter::HitResult;
    std::array<editor::PlayCharacter::Hit, 5> hits;
    const std::array<M, 5> materials = {M::Stone, M::Metal, M::Wood, M::Dirt, M::Flesh};
    for (usize i = 0; i < materials.size(); ++i) {
        hits[i] = strikeAt(materials[i]);
        const auto& h = hits[i];
        MESSAGE(std::string(scene::surfaceProperties(materials[i]).name), ": ", std::string(editor::PlayCharacter::resultName(h.result)), ", ",
                h.speed, " -> ", h.speedAfter, " m/s, angle ", h.angle, " deg, impulse ", h.impulse, " N s, cut ",
                h.depth * 100.0f, " cm, loads ", h.peakLoad[0], " ", h.peakLoad[1], " ", h.peakLoad[2]);
        CHECK(h.material == materials[i]);
        CHECK(h.result != R::Pending);
        CHECK(h.speed > 1.0f);
    }
    const auto& stone = hits[0];
    const auto& metal = hits[1];
    const auto& wood = hits[2];
    const auto& dirt = hits[3];
    const auto& flesh = hits[4];
    // Hard: rigid contact, no cut.
    for (usize i = 0; i < 3; ++i) {
        CHECK_FALSE(hits[i].soft);
        CHECK(hits[i].depth < 0.03f);
    }
    // Metal is the springiest, wood the deadest.
    CHECK(metal.speedAfter > stone.speedAfter);
    CHECK(stone.speedAfter > wood.speedAfter);
    CHECK(metal.result == R::Rebound);
    CHECK(stone.result == R::Rebound);
    CHECK(wood.result == R::Stop);
    // Soft: the blade cuts in and stays (the muscles then pull it out).
    for (const auto* h : {&dirt, &flesh}) {
        CHECK(h->soft);
        CHECK((h->result == R::Cut || h->result == R::Stuck));
        CHECK(h->depth > 0.02f);
    }
    CHECK(flesh.depth > dirt.depth);
    CHECK(stone.impulse > 0.5f);
}

TEST_CASE("Play: the impact angle changes the reaction (stone straight on rebounds, at ~30 deg it stops)")
{
    // A stone slab sloping towards the character (top facing it), where the blade comes down and back.
    const glm::quat slope = glm::angleAxis(glm::radians(-30.0f), glm::vec3(1, 0, 0));
    const auto slab = strikeAt(scene::SurfaceMaterial::Stone, nullptr, slope, {-0.21f, 0.45f, 1.05f}, {0.3f, 0.06f, 0.6f});
    const auto straight = strikeAt(scene::SurfaceMaterial::Stone);
    MESSAGE("slab: ", std::string(editor::PlayCharacter::resultName(slab.result)), ", angle ", slab.angle, " deg, ",
            slab.speed, " -> ", slab.speedAfter, " m/s; straight: angle ", straight.angle, " deg, -> ", straight.speedAfter);
    CHECK(slab.material == scene::SurfaceMaterial::Stone);
    CHECK(slab.angle < 45.0f);
    CHECK(straight.angle > 70.0f);
    CHECK(straight.result == editor::PlayCharacter::HitResult::Rebound);
    CHECK(slab.result != editor::PlayCharacter::HitResult::Rebound);
}
