#include <kinetic/animation/BlendTree.h>
#include <kinetic/animation/IK.h>
#include <kinetic/animation/PoseOps.h>

#include <doctest/doctest.h>

#include <glm/gtc/quaternion.hpp>

using namespace kinetic;
using namespace kinetic::anim;

namespace {

// One joint whose X translation encodes which clip is playing and where: clip k moves from
// k * 10 to k * 10 + 1 over its duration.
Skeleton oneJoint() { return Skeleton({"j"}, {kNoParent}, Pose(1)); }

AnimationClip ramp(const char* name, f32 base, f32 duration)
{
    std::vector<JointTracks> tracks(1);
    tracks[0].translation.times = {0.0f, duration};
    tracks[0].translation.values = {{base, 0.0f, 0.0f}, {base + 1.0f, 0.0f, 0.0f}};
    return AnimationClip(name, duration, std::move(tracks));
}

struct Locomotion {
    BlendTree tree;
    std::vector<AnimationClip> clips;
    i32 blend = -1;

    Locomotion()
    {
        clips = {ramp("idle", 0.0f, 2.0f), ramp("walk", 10.0f, 1.0f), ramp("run", 20.0f, 0.5f)};
        const i32 speed = tree.addParameter("speed", 0.0f, 0.0f, 5.0f);
        blend = tree.addBlend1D("Locomotion", speed);
        tree.addChild(blend, tree.addClip("Run", 2), 3.0f); // added out of order on purpose
        tree.addChild(blend, tree.addClip("Idle", 0), 0.0f);
        tree.addChild(blend, tree.addClip("Walk", 1), 1.0f);
    }
};

} // namespace

TEST_CASE("BlendTree: children are sorted and the tree validates")
{
    Locomotion l;
    CHECK(l.tree.root == l.blend);
    const auto& children = l.tree.nodes[static_cast<usize>(l.blend)].children;
    REQUIRE(children.size() == 3);
    CHECK(children[0].threshold == 0.0f);
    CHECK(children[1].threshold == 1.0f);
    CHECK(children[2].threshold == 3.0f);
    CHECK(l.tree.isValid(l.clips.size()));
    CHECK_FALSE(l.tree.isValid(2)); // the run clip index is out of range
}

TEST_CASE("BlendTree: 1D segment picks the neighbours of the parameter value")
{
    Locomotion l;
    BlendTreeInstance inst;
    inst.prepare(l.tree);

    inst.setParameter(l.tree, "speed", 2.0f); // halfway between walk (1) and run (3)
    Blend1DSegment s = blend1DSegment(l.tree, inst, l.blend);
    CHECK(l.tree.nodes[static_cast<usize>(s.a)].name == "Walk");
    CHECK(l.tree.nodes[static_cast<usize>(s.b)].name == "Run");
    CHECK(s.t == doctest::Approx(0.5f));

    inst.setParameter(l.tree, "speed", -1.0f);
    s = blend1DSegment(l.tree, inst, l.blend);
    CHECK(s.a == s.b);
    CHECK(l.tree.nodes[static_cast<usize>(s.a)].name == "Idle");

    inst.setParameter(l.tree, "speed", 6.0f); // twice the run threshold: scaled playback
    s = blend1DSegment(l.tree, inst, l.blend);
    CHECK(l.tree.nodes[static_cast<usize>(s.a)].name == "Run");
    CHECK(s.playbackScale == doctest::Approx(2.0f));
}

TEST_CASE("BlendTree: synced children share one clock with a blended cycle length")
{
    Locomotion l;
    const Skeleton skeleton = oneJoint();
    BlendTreeInstance inst;
    inst.setParameter(l.tree, "speed", 2.0f);

    // Cycle = mix(walk 1 s, run 0.5 s, 0.5) = 0.75 s. After 0.375 s every child is at phase 0.5.
    CHECK(blendNodeDuration(l.tree, l.clips, inst, l.blend) == doctest::Approx(0.75f));
    updateBlendTree(l.tree, l.clips, inst, 0.375f);
    for (usize n = 0; n < l.tree.nodes.size(); ++n) {
        CHECK(inst.phase[n] == doctest::Approx(0.5f));
    }
    // Weights: half walk, half run, nothing for idle.
    const auto weightOf = [&](const char* name) {
        for (usize n = 0; n < l.tree.nodes.size(); ++n) {
            if (l.tree.nodes[n].name == name) return inst.weight[n];
        }
        return -1.0f;
    };
    CHECK(weightOf("Walk") == doctest::Approx(0.5f));
    CHECK(weightOf("Run") == doctest::Approx(0.5f));
    CHECK(weightOf("Idle") == doctest::Approx(0.0f));

    Pose pose;
    evaluateBlendTree(l.tree, skeleton, l.clips, inst, pose);
    // walk at phase .5 -> 10.5, run at phase .5 -> 20.5, blended 50/50 -> 15.5
    CHECK(pose.translations[0].x == doctest::Approx(15.5f));
}

TEST_CASE("BlendTree: removing a node fixes every index")
{
    Locomotion l;
    const i32 walk = l.tree.nodes[static_cast<usize>(l.blend)].children[1].node;
    l.tree.removeNode(walk);
    CHECK(l.tree.nodes.size() == 3);
    const auto& children = l.tree.nodes[static_cast<usize>(l.tree.root)].children;
    REQUIRE(children.size() == 2);
    for (const BlendChild& c : children) {
        CHECK(l.tree.validNode(c.node));
        CHECK(l.tree.nodes[static_cast<usize>(c.node)].name != "Walk");
    }
    CHECK(l.tree.isValid(l.clips.size()));
}

TEST_CASE("Two-bone IK reaches a reachable target and keeps the bend direction")
{
    const glm::vec3 a(0.0f, 1.0f, 0.0f);
    const glm::vec3 b(0.0f, 0.5f, 0.05f); // knee slightly forward (+Z)
    const glm::vec3 c(0.0f, 0.0f, 0.0f);
    const glm::vec3 target(0.1f, 0.2f, 0.1f);
    const TwoBoneSolution s = solveTwoBone(a, b, c, target, glm::vec3(0.0f, 0.0f, 1.0f));
    CHECK(s.reached);
    const glm::vec3 newB = a + s.upper * (b - a);
    const glm::vec3 newC = newB + s.lower * (s.upper * (c - b));
    CHECK(glm::length(newC - target) < 1e-3f);
    CHECK(glm::length(newB - a) == doctest::Approx(glm::length(b - a)));
    CHECK(newB.z > 0.0f); // still bends forward
}

TEST_CASE("Two-bone IK stretches towards an unreachable target")
{
    const TwoBoneSolution s = solveTwoBone({0, 1, 0}, {0, 0.5f, 0.05f}, {0, 0, 0}, {0, -2, 0}, {0, 0, 1});
    CHECK_FALSE(s.reached);
    const glm::vec3 newB = glm::vec3(0, 1, 0) + s.upper * glm::vec3(0, -0.5f, 0.05f);
    CHECK(newB.y < 0.6f); // straightened downwards
}

TEST_CASE("Ground speed of a synthetic in-place walk")
{
    // pelvis -> thigh -> shin -> foot (x2). Feet slide back 0.5 m per 1 s while planted, then
    // swing forward lifted: ground speed 0.5 m/s.
    std::vector<std::string> names = {"Hips", "LeftUpLeg", "LeftLeg", "LeftFoot", "RightUpLeg", "RightLeg", "RightFoot"};
    std::vector<i32> parents = {kNoParent, 0, 1, 2, 0, 4, 5};
    Pose rest(7);
    rest.translations = {{0, 1, 0}, {0.1f, 0, 0}, {0, -0.45f, 0}, {0, -0.45f, 0}, {-0.1f, 0, 0}, {0, -0.45f, 0}, {0, -0.45f, 0}};
    const Skeleton skeleton(names, parents, rest);
    const auto legs = findLegChains(skeleton);
    REQUIRE(legs.size() == 2);

    std::vector<JointTracks> tracks(7);
    for (const usize foot : {3u, 6u}) {
        const f32 offset = foot == 3 ? 0.0f : 1.0f; // legs half a cycle apart
        auto& t = tracks[foot].translation;
        for (int k = 0; k <= 40; ++k) {
            const f32 time = static_cast<f32>(k) * 0.05f; // 2 s cycle
            f32 phase = std::fmod(time + offset, 2.0f);
            glm::vec3 p(0.0f, -0.45f, 0.0f);
            if (phase < 1.0f) {
                p.z = 0.25f - 0.5f * phase; // planted: slides back at 0.5 m/s
            } else {
                p.z = -0.25f + 0.5f * (phase - 1.0f);
                p.y += 0.1f; // lifted
            }
            t.times.push_back(time);
            t.values.push_back(p);
        }
    }
    const AnimationClip walk("walk", 2.0f, std::move(tracks));
    CHECK(estimateGroundSpeed(skeleton, walk, legs) == doctest::Approx(0.5f).epsilon(0.1));
}
