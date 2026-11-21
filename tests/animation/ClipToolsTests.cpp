#include <kinetic/animation/ClipTools.h>
#include <kinetic/animation/PoseOps.h>

#include <doctest/doctest.h>

#include <glm/gtc/quaternion.hpp>

using namespace kinetic;
using namespace kinetic::anim;

namespace {

// Root -> Hips -> LeftLeg / RightLeg (a mirror pair at x = +-0.1).
Skeleton smallRig()
{
    Pose rest(4);
    rest.translations = {{0, 0, 0}, {0, 1, 0}, {0.1f, -0.5f, 0}, {-0.1f, -0.5f, 0}};
    return Skeleton({"Root", "Hips", "LeftLeg", "RightLeg"}, {kNoParent, 0, 1, 1}, rest);
}

glm::vec3 jointAt(const Skeleton& s, const AnimationClip& clip, f32 t, usize joint)
{
    Pose p = s.restPose();
    clip.sample(t, false, p);
    std::vector<glm::mat4> m(s.jointCount());
    localToModel(s, p, m);
    return glm::vec3(m[joint][3]);
}

} // namespace

TEST_CASE("Clip tools: centre root, loop pose, sub clip")
{
    const Skeleton s = smallRig();
    std::vector<JointTracks> tracks(4);
    tracks[1].translation.times = {0.0f, 0.5f, 1.0f};
    tracks[1].translation.values = {{0.5f, 1, 0}, {0.7f, 1, 0}, {0.6f, 1, 0}}; // off-centre, loop gap 0.1
    AnimationClip clip("run", 1.0f, tracks);

    const f32 shift = centerRootXZ(s, clip, 1);
    CHECK(shift > 0.5f);
    f32 mean = 0.0f;
    for (int i = 0; i <= 60; ++i) {
        mean += jointAt(s, clip, static_cast<f32>(i) / 60.0f, 1).x / 61.0f;
    }
    CHECK(std::abs(mean) < 0.01f);

    const f32 gap = makeLoopPose(s, clip);
    CHECK(gap == doctest::Approx(0.1f).epsilon(0.01));
    CHECK(jointAt(s, clip, 0.0f, 1).x == doctest::Approx(jointAt(s, clip, 1.0f, 1).x));

    const AnimationClip sub = makeSubClip(clip, 0.25f, 0.75f, "mid");
    CHECK(sub.duration() == doctest::Approx(0.5f));
    CHECK(jointAt(s, sub, 0.0f, 1).x == doctest::Approx(jointAt(s, clip, 0.25f, 1).x).epsilon(0.001));
}

TEST_CASE("Clip tools: turn segment, mirroring and root yaw extraction")
{
    const Skeleton s = smallRig();
    // Hips turn 90 degrees left over 1 s while the left leg lifts.
    std::vector<JointTracks> tracks(4);
    for (int k = 0; k <= 10; ++k) {
        const f32 t = static_cast<f32>(k) / 10.0f;
        tracks[1].rotation.times.push_back(t);
        tracks[1].rotation.values.push_back(glm::angleAxis(glm::radians(90.0f) * t, glm::vec3(0, 1, 0)));
        tracks[2].translation.times.push_back(t);
        tracks[2].translation.values.push_back({0.1f, -0.5f + 0.2f * t, 0.0f});
    }
    AnimationClip turn("turn", 1.0f, tracks);
    const auto segment = findTurnSegment(s, turn, 1);
    REQUIRE(segment.has_value());
    CHECK(segment->first < 0.1f);
    CHECK(segment->second > 0.9f);

    const std::vector<usize> mirror = findMirrorJoints(s);
    CHECK(mirror[2] == 3);
    CHECK(mirror[3] == 2);
    const AnimationClip mirrored = mirrorClip(s, turn, "turn (mirror)");
    // In the mirror the RIGHT leg lifts, and the hips turn the other way.
    CHECK(jointAt(s, mirrored, 1.0f, 3).y == doctest::Approx(0.7f).epsilon(0.01));
    CHECK(jointAt(s, mirrored, 1.0f, 2).y == doctest::Approx(0.5f).epsilon(0.01));
    const std::vector<f32> yaw = rootYawCurve(s, mirrored, 1, 10);
    CHECK(yaw.back() == doctest::Approx(glm::radians(-90.0f)).epsilon(0.01));

    const f32 rate = extractRootYaw(s, turn, 1);
    CHECK(rate == doctest::Approx(glm::radians(90.0f)).epsilon(0.01));
    CHECK(turn.rootYawRate() == doctest::Approx(rate));
    const std::vector<f32> after = rootYawCurve(s, turn, 1, 10);
    CHECK(std::abs(after.back()) < 0.01f); // the pose no longer turns
}
