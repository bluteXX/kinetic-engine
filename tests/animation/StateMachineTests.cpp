#include <kinetic/animation/BlendTree.h>

#include <doctest/doctest.h>

using namespace kinetic;
using namespace kinetic::anim;

namespace {

Skeleton oneJoint() { return Skeleton({"j"}, {kNoParent}, Pose(1)); }

// Joint X moves from `base` to `base + 1` over the clip.
AnimationClip ramp(const char* name, f32 base, f32 duration)
{
    std::vector<JointTracks> tracks(1);
    tracks[0].translation.times = {0.0f, duration};
    tracks[0].translation.values = {{base, 0.0f, 0.0f}, {base + 1.0f, 0.0f, 0.0f}};
    return AnimationClip(name, duration, std::move(tracks));
}

} // namespace

TEST_CASE("Gradient band weights: exact at samples, partition of unity in between")
{
    const std::vector<glm::vec2> samples = {{0, 0}, {0, 1}, {0, 3}, {1, 0}, {-1, 0}};
    for (usize i = 0; i < samples.size(); ++i) {
        const auto w = gradientBandWeights(samples, samples[i]);
        CHECK(w[i] == doctest::Approx(1.0f));
    }
    const auto w = gradientBandWeights(samples, {0.5f, 0.5f});
    f32 sum = 0.0f;
    for (const f32 x : w) {
        CHECK(x >= 0.0f);
        sum += x;
    }
    CHECK(sum == doctest::Approx(1.0f));
    CHECK(w[2] == doctest::Approx(0.0f)); // the far sample does not contribute
    CHECK(w[4] == doctest::Approx(0.0f)); // nor the one on the other side
}

TEST_CASE("Blend2D node blends the clips around the 2D parameter point")
{
    const Skeleton skeleton = oneJoint();
    std::vector<AnimationClip> clips = {ramp("idle", 0.0f, 1.0f), ramp("walk", 10.0f, 1.0f), ramp("turn", 20.0f, 1.0f)};
    BlendTree tree;
    const i32 turn = tree.addParameter("turn", 0.0f, -2.0f, 2.0f);
    const i32 speed = tree.addParameter("speed", 0.0f, 0.0f, 2.0f);
    const i32 blend = tree.addBlend2D("Loco", turn, speed);
    tree.addChild(blend, tree.addClip("Idle", 0), 0.0f, 0.0f);
    tree.addChild(blend, tree.addClip("Walk", 1), 0.0f, 1.0f);
    tree.addChild(blend, tree.addClip("Turn", 2), 1.0f, 0.0f);
    CHECK(tree.isValid(clips.size()));

    BlendTreeInstance inst;
    inst.setParameter(tree, "speed", 1.0f);
    updateBlendTree(tree, clips, inst, 0.0f);
    Pose pose;
    evaluateBlendTree(tree, skeleton, clips, inst, pose);
    CHECK(pose.translations[0].x == doctest::Approx(10.0f)); // exactly walk

    inst.setParameter(tree, "speed", 0.0f);
    inst.setParameter(tree, "turn", 0.5f); // halfway between idle and turn
    updateBlendTree(tree, clips, inst, 0.0f);
    evaluateBlendTree(tree, skeleton, clips, inst, pose);
    CHECK(pose.translations[0].x == doctest::Approx(10.0f)); // 50% idle (0) + 50% turn (20)
}

TEST_CASE("State machine: trigger, conditions, exit time and crossfade")
{
    const Skeleton skeleton = oneJoint();
    std::vector<AnimationClip> clips = {ramp("idle", 0.0f, 1.0f), ramp("jump", 10.0f, 1.0f)};
    BlendTree tree;
    const i32 jump = tree.addParameter("jump", 0.0f, 0.0f, 1.0f, ParameterType::Trigger);
    const i32 grounded = tree.addParameter("grounded", 1.0f, 0.0f, 1.0f, ParameterType::Bool);
    const i32 idle = tree.addState("Idle", tree.addClip("Idle", 0));
    const i32 jumpNode = tree.addClip("Jump", 1);
    tree.nodes[static_cast<usize>(jumpNode)].loop = false;
    const i32 air = tree.addState("Jump", jumpNode);
    const i32 up = tree.addTransition(idle, air, 0.1f);
    tree.stateMachine.transitions[static_cast<usize>(up)].conditions.push_back({jump, ConditionOp::IsTrue, 0.0f});
    const i32 down = tree.addTransition(air, idle, 0.0f);
    auto& back = tree.stateMachine.transitions[static_cast<usize>(down)];
    back.hasExitTime = true;
    back.exitTime = 0.8f;
    back.conditions.push_back({grounded, ConditionOp::IsTrue, 0.0f});
    REQUIRE(tree.isValid(clips.size()));

    BlendTreeInstance inst;
    updateBlendTree(tree, clips, inst, 0.1f);
    CHECK(inst.state == idle);

    inst.setTrigger(tree, "jump");
    updateBlendTree(tree, clips, inst, 0.05f);
    CHECK(inst.state == air);
    CHECK(inst.inTransition());
    CHECK(inst.parameter(tree, "jump") == 0.0f); // consumed
    Pose pose;
    evaluateBlendTree(tree, skeleton, clips, inst, pose);
    // Crossfading between idle (0..1) and jump (10..11).
    CHECK(pose.translations[0].x > 0.5f);
    CHECK(pose.translations[0].x < 10.0f);

    // In the air: no way back before the exit time, nor while not grounded.
    inst.setParameter(tree, "grounded", 0.0f);
    for (int i = 0; i < 70; ++i) updateBlendTree(tree, clips, inst, 1.0f / 60.0f);
    CHECK(inst.state == air);
    CHECK_FALSE(inst.inTransition());
    evaluateBlendTree(tree, skeleton, clips, inst, pose);
    CHECK(pose.translations[0].x == doctest::Approx(11.0f)); // the one-shot holds its last frame
    inst.setParameter(tree, "grounded", 1.0f);
    updateBlendTree(tree, clips, inst, 1.0f / 60.0f);
    CHECK(inst.state == idle);

    // Removing a state drops its transitions.
    tree.removeState(air);
    CHECK(tree.stateMachine.transitions.empty());
    CHECK(tree.stateMachine.defaultState == idle);
}

TEST_CASE("Trigger input buffer: a press waits for the exit time, then expires")
{
    std::vector<AnimationClip> clips = {ramp("a", 0.0f, 1.0f), ramp("b", 10.0f, 1.0f)};
    BlendTree tree;
    const i32 attack = tree.addParameter("attack", 0.0f, 0.0f, 1.0f, ParameterType::Trigger);
    tree.parameters[static_cast<usize>(attack)].bufferTime = 0.3f;
    const i32 a = tree.addState("A", tree.addClip("A", 0));
    const i32 b = tree.addState("B", tree.addClip("B", 1));
    const i32 t = tree.addTransition(a, b, 0.0f);
    tree.stateMachine.transitions[static_cast<usize>(t)].conditions.push_back({attack, ConditionOp::IsTrue, 0.0f});
    tree.stateMachine.transitions[static_cast<usize>(t)].hasExitTime = true;
    tree.stateMachine.transitions[static_cast<usize>(t)].exitTime = 0.5f; // window opens at 0.5 s

    // Pressed at 0.3 s: buffered, fires when the window opens at 0.5 s.
    BlendTreeInstance inst;
    for (int i = 0; i < 18; ++i) updateBlendTree(tree, clips, inst, 1.0f / 60.0f);
    inst.setTrigger(tree, "attack");
    for (int i = 0; i < 6; ++i) updateBlendTree(tree, clips, inst, 1.0f / 60.0f);
    CHECK(inst.state == a);
    for (int i = 0; i < 8; ++i) updateBlendTree(tree, clips, inst, 1.0f / 60.0f);
    CHECK(inst.state == b);

    // Pressed at 0.05 s: expires (0.3 s) before the window opens - dropped.
    inst = {};
    for (int i = 0; i < 3; ++i) updateBlendTree(tree, clips, inst, 1.0f / 60.0f);
    inst.setTrigger(tree, "attack");
    for (int i = 0; i < 60; ++i) updateBlendTree(tree, clips, inst, 1.0f / 60.0f);
    CHECK(inst.state == a);
    CHECK(inst.parameter(tree, "attack") == 0.0f);
}
