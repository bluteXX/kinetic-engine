// The warrior on physics animation (kinetic::physanim): the bones a clip marks as physical run on
// muscles. Attacks with the weapon arm follow the Mixamo animation in the air and react physically
// when the blade meets stone, wood, flesh ("on hit" events, effects); with the whole body the
// warrior keeps its balance and is knocked over when something stops it. Skipped when the warrior
// assets are not present.

#include "CharacterSetup.h"
#include "PlayMode.h"

#include <kinetic/animation/ClipTools.h>
#include <kinetic/animation/Combat.h>
#include <kinetic/animation/PoseOps.h>
#include <kinetic/assets/ModelLoader.h>
#include <kinetic/scene/AnimatorSystem.h>

#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <string>

using namespace kinetic;

namespace {

const std::filesystem::path kWarriorDir = std::filesystem::path(KINETIC_TEST_ASSETS_DIR) / "models" / "Warrior";

bool haveWarrior() { return std::filesystem::exists(kWarriorDir / "animations" / "Attack1_CrossSlash.fbx"); }

std::shared_ptr<anim::SkinnedModel> warriorModel()
{
    static std::shared_ptr<anim::SkinnedModel> cached;
    if (cached) return cached;
    auto loaded = assets::loadSkinnedModel(kWarriorDir / "Warrior.fbx");
    REQUIRE(loaded.has_value());
    loaded->sourcePath.clear(); // no saved actor settings in tests
    auto model = std::make_shared<anim::SkinnedModel>(std::move(*loaded));
    for (const char* file : {"Attack1_CrossSlash.fbx", "Attack2_DownwardSlash.fbx", "Attack3_PowerSlash.fbx", "Idle_Combat.fbx"}) {
        std::string error;
        REQUIRE(assets::importAnimationsInto(*model, kWarriorDir / "animations" / file, &error).size() == 1);
    }
    (void)anim::prepareLocomotionClips(*model);
    model->locomotionPrepared = true;
    cached = model;
    return cached;
}

struct Arena {
    scene::Scene scene;
    entt::entity camera = entt::null;
    entt::entity hero = entt::null;
    entt::entity sword = entt::null;
    editor::PlayMode play;
    vfx::VfxLibrary effects;
    std::vector<physanim::HitEvent> events; // every "on hit" event so far
    static constexpr f32 kDt = 1.0f / 60.0f;

    // `armOnAttacks`: the attacks' physics bones are the weapon arm (what the test scene sets up).
    explicit Arena(bool armOnAttacks = true)
    {
        camera = scene.createCamera("Camera", {0, 1.6f, -4}, {0, 1, 0});
        const entt::entity floor = scene.createEntity("Floor");
        scene.registry().emplace<scene::ColliderComponent>(floor, glm::vec3(20, 0.1f, 20), glm::vec3(0, -0.1f, 0));
        hero = scene.createEntity("Warrior");
        auto& animator = scene.registry().emplace<scene::AnimatorComponent>(hero);
        animator.model = warriorModel();
        animator.player.play(0);
        const editor::HeroSetup setup = editor::makePlayableHero(scene, hero, 1.8f);
        REQUIRE(setup.ok);
        const i32 hand = anim::findWeaponHand(animator.model->skeleton);
        REQUIRE(hand >= 0);
        sword = scene.createEntity("Sword");
        auto& a = scene.registry().emplace<scene::AttachmentComponent>(sword);
        a.parent = hero;
        a.joint = hand;
        a.offset = anim::weaponSocket(animator.model->skeleton, hand);
        scene.registry().emplace<scene::WeaponComponent>(sword);
        if (armOnAttacks) {
            const physanim::BodyLayout layout = physanim::layoutBody(*animator.model);
            for (const i32 state : anim::attackChain(*animator.blendTree)) {
                animator.blendTree->nodes[static_cast<usize>(animator.blendTree->stateMachine.states[static_cast<usize>(state)].motion)].physicsJoints =
                    physanim::weaponArmJoints(layout, hand);
            }
        }
        const std::filesystem::path vfxDir = std::filesystem::path(KINETIC_TEST_ASSETS_DIR) / "vfx";
        effects.loadDirectory(vfxDir);
        play.setVfxLibrary(&effects);
    }

    entt::entity addTarget(const glm::vec3& center, const glm::vec3& half, scene::SurfaceMaterial material)
    {
        const entt::entity e = scene.createEntity("Target");
        scene.registry().get<scene::TransformComponent>(e).position = center;
        scene.registry().emplace<scene::ColliderComponent>(e, half, glm::vec3(0.0f), material);
        return e;
    }

    void step(editor::PlayInput input = {})
    {
        play.update(kDt, input, scene); // animation, physics, final pose, attachments
        events.insert(events.end(), play.hitEvents().begin(), play.hitEvents().end());
    }
    [[nodiscard]] const physanim::PhysicalBody* body() const { return play.heroBody(hero); }
    [[nodiscard]] i32 hand() const { return scene.registry().get<scene::AttachmentComponent>(sword).joint; }
    [[nodiscard]] usize simulatedBodies() const
    {
        usize n = 0;
        for (usize i = 0; i < body()->segmentCount(); ++i) n += body()->segmentDynamic(i) ? 1 : 0;
        return n;
    }
};

} // namespace

TEST_CASE("Physics body: the warrior's layout comes from its skeleton and skin")
{
    if (!haveWarrior()) return;
    const auto model = warriorModel();
    const physanim::BodyLayout layout = physanim::layoutBody(*model);
    MESSAGE("bodies: " << layout.segments.size());
    REQUIRE(layout.segments.size() >= 12);
    CHECK(layout.segments.size() <= 30);
    // One root (the pelvis), every other body hangs under it.
    CHECK(layout.segments.front().parent == -1);
    for (usize i = 1; i < layout.segments.size(); ++i) CHECK(layout.segments[i].parent >= 0);
    // Fingers ride on the hand; the hand is a body of its own.
    const i32 hand = anim::findWeaponHand(model->skeleton);
    const i32 handSegment = layout.segmentOf(hand);
    REQUIRE(handSegment >= 0);
    CHECK(layout.segments[static_cast<usize>(handSegment)].joint == hand);
    CHECK(layout.segments[static_cast<usize>(handSegment)].riders.size() >= 5);
    CHECK(physanim::weaponArmJoints(layout, hand).size() == 3);
    // The torso is thicker than the hand.
    CHECK(layout.segments.front().radius > 1.5f * layout.segments[static_cast<usize>(handSegment)].radius);
}

TEST_CASE("Physics body: only the leading clip's physics bones run on muscles, and the drawn sword is the simulated one")
{
    if (!haveWarrior()) return;
    Arena w;
    w.play.start(w.scene, w.camera);
    for (int i = 0; i < 20; ++i) w.step();
    REQUIRE(w.body() != nullptr);
    CHECK(w.simulatedBodies() == 0); // walking clips have no physics bones: plain animation
    w.step(editor::PlayInput{.attack = true});
    f32 sum = 0.0f;
    f32 peak = 0.0f;
    f32 worstGap = 0.0f;
    int frames = 0;
    usize most = 0;
    for (int i = 0; i < 240; ++i) {
        w.step();
        most = std::max(most, w.simulatedBodies());
        if (w.simulatedBodies() == 0) continue;
        const auto& world = w.play.world();
        const f32 e = glm::degrees(w.body()->trackingError(world, w.hand()));
        sum += e;
        peak = std::max(peak, e);
        ++frames;
        // The sword entity (attached to the hand bone, drawn) sits where the weapon body is.
        const glm::vec3 drawn = w.scene.registry().get<scene::TransformComponent>(w.sword).position;
        worstGap = std::max(worstGap, glm::length(drawn - world.body(w.body()->weapon()).position));
    }
    MESSAGE("tracking error: mean " << sum / std::max(frames, 1) << " deg, peak " << peak << " deg over " << frames
                                    << " frames; drawn sword vs body: " << worstGap * 100.0f << " cm");
    CHECK(most == 3); // hand, forearm, upper arm
    REQUIRE(frames > 30);
    CHECK(sum / static_cast<f32>(frames) < 12.0f);
    CHECK(peak < 60.0f);
    CHECK(worstGap < 0.01f); // same frame: no lag between the shown sword and the physics
    CHECK(w.body()->hits().empty());
    CHECK(w.simulatedBodies() == 0); // back on the animation
    w.play.stop(w.scene);
    CHECK(w.scene.registry().get<scene::AnimatorComponent>(w.hero).boneOverrides.empty());
}

namespace {

struct StrikeResult {
    std::vector<physanim::Hit> hits;
    // The strike: the fastest hit (the blade may brush the target slowly on the wind-up first).
    [[nodiscard]] const physanim::Hit& strike() const
    {
        return *std::max_element(hits.begin(), hits.end(), [](const auto& a, const auto& b) { return a.speed < b.speed; });
    }
    std::vector<physanim::HitEvent> events;
    std::string effect; // of the first event
    bool backToAnimation = false;
    usize vfxBursts = 0;
};

// Attack `attack` of the combo (started directly, as the number keys do in Play) into a pillar of
// `material` placed where that attack's blade strikes (like the test scene).
StrikeResult strikeInto(scene::SurfaceMaterial material, i32 attack, const std::string& onHit = {})
{
    Arena w;
    auto& registry = w.scene.registry();
    const auto& animator = registry.get<scene::AnimatorComponent>(w.hero);
    const std::vector<i32> chain = anim::attackChain(*animator.blendTree);
    REQUIRE(static_cast<usize>(attack) < chain.size());
    const i32 clip = anim::stateClip(*animator.blendTree, chain[static_cast<usize>(attack)]);
    const auto& a = registry.get<scene::AttachmentComponent>(w.sword);
    const physanim::StrikePoint sp = physanim::findStrikePoint(animator.model->skeleton, animator.model->clips[static_cast<usize>(clip)],
                                                               a.joint, a.offset, registry.get<scene::WeaponComponent>(w.sword));
    const physanim::TargetBox box = physanim::strikeTarget(sp, registry.get<scene::TransformComponent>(w.hero).matrix());
    const entt::entity target = w.addTarget(box.center, box.halfExtents, material);
    registry.get<scene::ColliderComponent>(target).onHit = onHit;
    w.play.start(w.scene, w.camera);
    for (int i = 0; i < 20; ++i) w.step();
    REQUIRE(w.body() != nullptr);
    w.step(editor::PlayInput{.playAttack = attack});
    StrikeResult r;
    for (int i = 0; i < 240; ++i) {
        w.step();
        r.vfxBursts = std::max(r.vfxBursts, w.play.vfx().bursts().size());
    }
    r.hits = w.body()->hits();
    r.events = w.events;
    if (!r.events.empty()) r.effect = w.play.hitEffect(r.events.front());
    r.backToAnimation = w.simulatedBodies() == 0;
    w.play.stop(w.scene);
    return r;
}

} // namespace

TEST_CASE("Physics body: every Mixamo attack of the combo reacts to stone, metal, wood, dirt and flesh")
{
    if (!haveWarrior()) return;
    using M = scene::SurfaceMaterial;
    using R = physanim::HitResult;
    const std::array<M, 5> materials = {M::Stone, M::Metal, M::Wood, M::Dirt, M::Flesh};
    const std::array<const char*, 5> effects = {"hit_stone", "hit_metal", "hit_wood", "hit_dirt", "hit_flesh"};
    for (i32 attack = 0; attack < 3; ++attack) {
        CAPTURE(attack);
        std::array<StrikeResult, 5> r;
        for (usize i = 0; i < materials.size(); ++i) {
            r[i] = strikeInto(materials[i], attack);
            std::string log;
            for (const auto& h : r[i].hits) {
                log += std::string(physanim::hitResultName(h.result)) + " " + std::to_string(h.speed) + " -> " +
                       std::to_string(h.speedAfter) + " m/s, cut " + std::to_string(h.depth * 100.0f) + " cm; ";
            }
            MESSAGE("attack " << attack + 1 << " " << std::string(scene::surfaceProperties(materials[i]).name) << ": " << log);
            REQUIRE_FALSE(r[i].hits.empty());
            CHECK(r[i].strike().material == materials[i]);
            CHECK(r[i].strike().speed > 10.0f); // a real strike
            CHECK(r[i].backToAnimation);
            // "On hit": an event at the hit, the material's effect played there.
            REQUIRE_FALSE(r[i].events.empty());
            CHECK(r[i].events.front().material == materials[i]);
            CHECK(r[i].effect == effects[i]);
            CHECK(r[i].vfxBursts > 0);
        }
        for (usize i = 0; i < 3; ++i) { // hard: no cut
            const auto& h = r[i].strike();
            CHECK_FALSE(h.soft);
            CHECK(h.depth < 0.03f);
            CHECK((h.result == R::Rebound || h.result == R::Stop || h.result == R::Glance));
        }
        for (usize i = 3; i < 5; ++i) { // soft: the blade cuts in and loses most of its speed
            const auto& h = r[i].strike();
            CHECK(h.soft);
            CHECK((h.result == R::Cut || h.result == R::Stuck));
            CHECK(h.depth > 0.02f);
            CHECK(h.speedAfter < 0.6f * h.speed);
        }
        CHECK(r[4].strike().depth > r[3].strike().depth); // flesh is softer than dirt
    }
}

TEST_CASE("Physics body: a collider's own On Hit effect replaces the material's")
{
    if (!haveWarrior()) return;
    const StrikeResult custom = strikeInto(scene::SurfaceMaterial::Wood, 0, "hit_metal");
    REQUIRE_FALSE(custom.events.empty());
    CHECK(custom.effect == "hit_metal");
    const StrikeResult none = strikeInto(scene::SurfaceMaterial::Wood, 0, "none");
    REQUIRE_FALSE(none.events.empty());
    CHECK(none.effect.empty());
    CHECK(none.vfxBursts == 0);
}

namespace {

// Every clip of the graph with the whole body as physics bones.
void wholeBodyEverywhere(Arena& w)
{
    auto& animator = w.scene.registry().get<scene::AnimatorComponent>(w.hero);
    const auto joints = physanim::fullBodyJoints(physanim::layoutBody(*animator.model));
    for (anim::BlendNode& n : animator.blendTree->nodes) {
        if (n.type == anim::BlendNodeType::Clip) n.physicsJoints = joints;
    }
}

} // namespace

TEST_CASE("Physics body: with the whole body on muscles the warrior stands and attacks without falling over")
{
    if (!haveWarrior()) return;
    Arena w(false);
    wholeBodyEverywhere(w);
    w.play.start(w.scene, w.camera);
    const glm::vec3 start = w.scene.registry().get<scene::TransformComponent>(w.hero).position;
    for (int i = 0; i < 120; ++i) w.step();
    REQUIRE(w.body() != nullptr);
    CHECK(w.simulatedBodies() == w.body()->segmentCount());
    w.step(editor::PlayInput{.attack = true});
    f32 worst = 0.0f;
    for (int i = 0; i < 150; ++i) {
        w.step();
        CHECK_FALSE(w.body()->down());
        const auto& animator = w.scene.registry().get<scene::AnimatorComponent>(w.hero);
        const glm::vec3 pelvisAnimated = glm::vec3(w.scene.registry().get<scene::TransformComponent>(w.hero).matrix() *
                                                   animator.animatedModel[static_cast<usize>(w.body()->layout().segments.front().joint)][3]);
        worst = std::max(worst, glm::length(w.body()->rootPosition(w.play.world()) - pelvisAnimated));
    }
    MESSAGE("pelvis off its animated place: at most " << worst * 100.0f << " cm; mass " << w.body()->totalMass() << " kg");
    CHECK(worst < 0.15f);
    CHECK(w.body()->totalMass() > 40.0f);
    CHECK(w.body()->totalMass() < 150.0f);
    (void)start;
}

TEST_CASE("Physics body: a fully physical warrior stopped by a stone wall is knocked down, then gets back up")
{
    if (!haveWarrior()) return;
    Arena w(false);
    wholeBodyEverywhere(w);
    auto& t = w.scene.registry().get<scene::TransformComponent>(w.hero);
    const glm::vec3 origin = t.position;
    const glm::vec3 forward = t.rotation * glm::vec3(0.0f, 0.0f, 1.0f);
    // A stone wall just in front (its near face 0.45 m ahead).
    w.addTarget(origin + forward * 0.55f + glm::vec3(0.0f, 1.0f, 0.0f), {1.5f, 1.0f, 0.1f}, scene::SurfaceMaterial::Stone);
    w.play.start(w.scene, w.camera);
    for (int i = 0; i < 60; ++i) w.step();
    REQUIRE(w.body() != nullptr);
    REQUIRE_FALSE(w.body()->down());
    // The animation leaps through the wall (like a leap attack's root motion would): the body
    // cannot follow.
    t.position += forward * 1.2f;
    bool wentDown = false;
    bool gotUp = false;
    for (int i = 0; i < 300; ++i) {
        w.step();
        wentDown |= w.body()->down();
        gotUp |= wentDown && w.body()->phase() == physanim::PhysicalBody::Phase::Animated;
    }
    CHECK(wentDown);
    CHECK(gotUp);
    // It fell (and got up) on its own side of the wall.
    CHECK(glm::dot(w.body()->rootPosition(w.play.world()) - origin, forward) < 0.5f);
    CHECK(glm::dot(t.position - origin, forward) < 0.5f);
}

TEST_CASE("Blade edge: aligning the Mixamo attacks makes the sword cut through the air edge first")
{
    if (!haveWarrior()) return;
    const auto model = warriorModel();
    const i32 hand = anim::findWeaponHand(model->skeleton);
    const glm::mat4 socket = anim::weaponSocket(model->skeleton, hand);
    for (const i32 c : anim::findAttackClips(model->clips)) {
        anim::AnimationClip clip = model->clips[static_cast<usize>(c)]; // a copy: the cached model stays as imported
        const f32 point = 0.55f * editor::restHeight(*model) / 1.8f; // mid-blade on a 1.8 m warrior
        const anim::EdgeAlignment r = anim::alignBladeEdge(model->skeleton, clip, hand, socket, point);
        MESSAGE(clip.name() << ": flatness " << r.before << " -> " << r.after << " deg, hand turned up to " << r.maxTwist << " deg");
        CHECK(r.before > 15.0f);
        CHECK(r.after < 0.5f * r.before);
        CHECK(r.after < 16.0f);
        CHECK(r.maxTwist < 130.0f);
        // Only the hand turns about the blade: the arm, the wrist and the swing stay the clip's.
        anim::Pose a;
        anim::Pose b;
        std::vector<glm::mat4> ma(model->skeleton.jointCount());
        std::vector<glm::mat4> mb(model->skeleton.jointCount());
        f32 wristMoved = 0.0f;
        f32 tipMoved = 0.0f;
        for (int i = 0; i <= 20; ++i) {
            const f32 t = clip.duration() * static_cast<f32>(i) / 20.0f;
            a = model->skeleton.restPose();
            b = model->skeleton.restPose();
            model->clips[static_cast<usize>(c)].sample(t, false, a);
            clip.sample(t, false, b);
            anim::localToModel(model->skeleton, a, ma);
            anim::localToModel(model->skeleton, b, mb);
            wristMoved = std::max(wristMoved, glm::length(glm::vec3(ma[static_cast<usize>(hand)][3]) - glm::vec3(mb[static_cast<usize>(hand)][3])));
            const glm::vec4 tip(0.0f, 1.0f, 0.0f, 1.0f);
            tipMoved = std::max(tipMoved, glm::length(glm::vec3(ma[static_cast<usize>(hand)] * socket * tip) - glm::vec3(mb[static_cast<usize>(hand)] * socket * tip)));
        }
        CHECK(wristMoved < 1e-4f);
        MESSAGE("  blade tip moved at most " << tipMoved << " model units");
    }
}

TEST_CASE("Blade edge: aligning an actor's attack is saved with the actor and its strikes still land")
{
    if (!haveWarrior()) return;
    Arena w;
    const auto& animator = w.scene.registry().get<scene::AnimatorComponent>(w.hero);
    const std::shared_ptr<const anim::SkinnedModel> before = animator.model;
    const i32 clip = anim::stateClip(*animator.blendTree, anim::attackChain(*animator.blendTree).front());
    const auto r = editor::alignWeaponEdge(w.scene, w.hero, clip);
    REQUIRE(r.has_value());
    CHECK(r->after < r->before);
    CHECK(animator.model != before);                                        // its own copy
    CHECK_FALSE(before->clips[static_cast<usize>(clip)].edgeAligned());     // others keep theirs
    CHECK_FALSE(editor::alignWeaponEdge(w.scene, w.hero, clip).has_value()); // once
    const Json saved = editor::actorToJson(w.scene, w.hero);
    REQUIRE(saved["edgeAlignedClips"].size() == 1);
    CHECK(saved["edgeAlignedClips"][0].asString() == animator.model->clips[static_cast<usize>(clip)].name());
}
