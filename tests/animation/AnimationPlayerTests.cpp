#include <doctest/doctest.h>

#include <kinetic/animation/AnimationPlayer.h>

using namespace kinetic;

namespace {

// One joint whose X translation goes 0 -> 10 over `duration` seconds.
anim::AnimationClip rampClip(std::string name, f32 duration, f32 endValue = 10.0f)
{
    anim::JointTracks tracks;
    tracks.translation.times = {0.0f, duration};
    tracks.translation.values = {glm::vec3(0.0f), glm::vec3(endValue, 0.0f, 0.0f)};
    return anim::AnimationClip(std::move(name), duration, {tracks});
}

anim::Skeleton oneJoint()
{
    return anim::Skeleton({"root"}, {anim::kNoParent}, anim::Pose(1));
}

f32 sampleX(const anim::AnimationPlayer& player, const anim::Skeleton& skeleton,
            const std::vector<anim::AnimationClip>& clips)
{
    anim::Pose out;
    anim::Pose scratch;
    player.evaluate(skeleton, clips, out, scratch);
    return out.translations[0].x;
}

} // namespace

TEST_CASE("AnimationPlayer loops and stops non-looping clips at the end")
{
    const anim::Skeleton skeleton = oneJoint();
    const std::vector<anim::AnimationClip> clips = {rampClip("ramp", 2.0f)};

    anim::AnimationPlayer player;
    CHECK(sampleX(player, skeleton, clips) == doctest::Approx(0.0f)); // no clip: rest pose
    player.play(0);
    player.update(0.5f, clips);
    CHECK(player.time() == doctest::Approx(0.5f));
    CHECK(sampleX(player, skeleton, clips) == doctest::Approx(2.5f));

    player.update(2.0f, clips); // 2.5 s wraps to 0.5 s
    CHECK(player.time() == doctest::Approx(0.5f));
    CHECK(player.playing);

    player.loop = false;
    player.update(5.0f, clips);
    CHECK(player.time() == doctest::Approx(2.0f));
    CHECK_FALSE(player.playing);

    player.setTime(1.0f, clips);
    CHECK(sampleX(player, skeleton, clips) == doctest::Approx(5.0f));
    player.stop();
    CHECK(player.time() == doctest::Approx(0.0f));
    CHECK_FALSE(player.playing);
}

TEST_CASE("AnimationPlayer plays backwards with negative speed")
{
    const std::vector<anim::AnimationClip> clips = {rampClip("ramp", 2.0f)};
    anim::AnimationPlayer player;
    player.play(0);
    player.speed = -1.0f;
    player.update(0.5f, clips);
    CHECK(player.time() == doctest::Approx(1.5f)); // wrapped from -0.5
}

TEST_CASE("AnimationPlayer crossfades between clips")
{
    const anim::Skeleton skeleton = oneJoint();
    // Clip A holds x = 0, clip B holds x = 10.
    std::vector<anim::AnimationClip> clips = {rampClip("a", 1.0f, 0.0f), rampClip("b", 1.0f, 10.0f)};
    clips[1] = [] {
        anim::JointTracks t;
        t.translation.times = {0.0f, 1.0f};
        t.translation.values = {glm::vec3(10.0f, 0, 0), glm::vec3(10.0f, 0, 0)};
        return anim::AnimationClip("b", 1.0f, {t});
    }();

    anim::AnimationPlayer player;
    player.play(0);
    player.play(1, 0.4f);
    CHECK(player.isFading());
    CHECK(player.previousClip() == 0);
    CHECK(sampleX(player, skeleton, clips) == doctest::Approx(0.0f));

    player.update(0.2f, clips);
    CHECK(player.fadeWeight() == doctest::Approx(0.5f));
    CHECK(sampleX(player, skeleton, clips) == doctest::Approx(5.0f));

    player.update(0.3f, clips);
    CHECK_FALSE(player.isFading());
    CHECK(sampleX(player, skeleton, clips) == doctest::Approx(10.0f));
}
