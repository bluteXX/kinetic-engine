#include "kinetic/animation/AnimationPlayer.h"

#include "kinetic/animation/PoseOps.h"

#include <algorithm>
#include <cmath>

namespace kinetic::anim {

namespace {

const AnimationClip* findClip(std::span<const AnimationClip> clips, i32 index)
{
    return index >= 0 && static_cast<usize>(index) < clips.size() ? &clips[static_cast<usize>(index)] : nullptr;
}

// Advances `time` within a clip of `duration`. Returns false when a non-looping clip ran off an end.
bool advance(f32& time, f32 delta, f32 duration, bool loop)
{
    time += delta;
    if (duration <= 0.0f) {
        time = 0.0f;
        return true;
    }
    if (loop) {
        time = std::fmod(time, duration);
        if (time < 0.0f) {
            time += duration;
        }
        return true;
    }
    if (time >= duration || time <= 0.0f) {
        const bool ranOff = (delta > 0.0f && time >= duration) || (delta < 0.0f && time <= 0.0f);
        time = std::clamp(time, 0.0f, duration);
        return !ranOff;
    }
    return true;
}

} // namespace

void AnimationPlayer::play(i32 clip, f32 fadeSeconds)
{
    if (fadeSeconds > 0.0f && m_clip != kNoClip && clip != m_clip) {
        m_previousClip = m_clip;
        m_previousTime = m_time;
        m_fadeDuration = fadeSeconds;
        m_fadeElapsed = 0.0f;
    } else {
        m_previousClip = kNoClip;
        m_fadeDuration = 0.0f;
    }
    m_clip = clip;
    m_time = 0.0f;
    playing = true;
}

void AnimationPlayer::stop()
{
    playing = false;
    m_time = 0.0f;
    m_previousClip = kNoClip;
    m_fadeDuration = 0.0f;
}

void AnimationPlayer::setTime(f32 time, std::span<const AnimationClip> clips)
{
    const AnimationClip* clip = findClip(clips, m_clip);
    m_time = clip ? std::clamp(time, 0.0f, clip->duration()) : 0.0f;
    m_previousClip = kNoClip;
    m_fadeDuration = 0.0f;
}

void AnimationPlayer::update(f32 dt, std::span<const AnimationClip> clips)
{
    if (!playing || dt <= 0.0f) {
        return;
    }
    const f32 delta = dt * speed;

    if (const AnimationClip* clip = findClip(clips, m_clip)) {
        if (!advance(m_time, delta, clip->duration(), loop)) {
            playing = false; // non-looping clip reached its end
        }
    }

    if (isFading()) {
        if (const AnimationClip* previous = findClip(clips, m_previousClip)) {
            advance(m_previousTime, delta, previous->duration(), loop);
        }
        m_fadeElapsed += dt;
        if (m_fadeElapsed >= m_fadeDuration) {
            m_previousClip = kNoClip;
            m_fadeDuration = 0.0f;
        }
    }
}

f32 AnimationPlayer::fadeWeight() const
{
    return isFading() ? std::clamp(m_fadeElapsed / m_fadeDuration, 0.0f, 1.0f) : 1.0f;
}

void AnimationPlayer::evaluate(const Skeleton& skeleton, std::span<const AnimationClip> clips, Pose& out,
                               Pose& scratch) const
{
    out = skeleton.restPose();
    const auto usable = [&](const AnimationClip* clip) {
        return clip && clip->jointCount() == skeleton.jointCount();
    };

    const AnimationClip* current = findClip(clips, m_clip);
    if (usable(current)) {
        current->sample(m_time, loop, out);
    }

    const AnimationClip* previous = isFading() ? findClip(clips, m_previousClip) : nullptr;
    if (usable(previous)) {
        scratch = skeleton.restPose();
        previous->sample(m_previousTime, loop, scratch);
        blendPoses(scratch, out, fadeWeight(), out);
    }
}

} // namespace kinetic::anim
