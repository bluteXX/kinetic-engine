#include <doctest/doctest.h>

#include <kinetic/animation/AnimationClip.h>

using namespace kinetic::anim;

namespace {

AnimationClip makeSlideClip(Interpolation interpolation)
{
    JointTracks tracks;
    tracks.translation.times = {0.0f, 1.0f, 2.0f};
    tracks.translation.values = {{0, 0, 0}, {10, 0, 0}, {20, 0, 0}};
    tracks.translation.interpolation = interpolation;
    return AnimationClip("slide", 2.0f, {tracks});
}

} // namespace

TEST_CASE("linear sampling interpolates between keys")
{
    const AnimationClip clip = makeSlideClip(Interpolation::Linear);
    Pose pose(1);
    clip.sample(0.5f, false, pose);
    CHECK(pose.translations[0].x == doctest::Approx(5.0f));
    clip.sample(1.5f, false, pose);
    CHECK(pose.translations[0].x == doctest::Approx(15.0f));
}

TEST_CASE("step sampling holds the previous key")
{
    const AnimationClip clip = makeSlideClip(Interpolation::Step);
    Pose pose(1);
    clip.sample(0.99f, false, pose);
    CHECK(pose.translations[0].x == doctest::Approx(0.0f));
}

TEST_CASE("looping wraps time, clamping holds the ends")
{
    const AnimationClip clip = makeSlideClip(Interpolation::Linear);
    Pose pose(1);
    clip.sample(2.5f, true, pose);
    CHECK(pose.translations[0].x == doctest::Approx(5.0f));
    clip.sample(5.0f, false, pose);
    CHECK(pose.translations[0].x == doctest::Approx(20.0f));
    clip.sample(-1.0f, false, pose);
    CHECK(pose.translations[0].x == doctest::Approx(0.0f));
}

TEST_CASE("channels without tracks keep the input pose")
{
    const AnimationClip clip = makeSlideClip(Interpolation::Linear);
    Pose pose(1);
    pose.scales[0] = {2.0f, 2.0f, 2.0f};
    clip.sample(1.0f, false, pose);
    CHECK(pose.scales[0].x == doctest::Approx(2.0f));
}
