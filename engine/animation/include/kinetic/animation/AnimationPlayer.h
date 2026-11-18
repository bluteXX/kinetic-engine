#pragma once

#include "kinetic/animation/AnimationClip.h"
#include "kinetic/animation/Skeleton.h"

#include <span>

namespace kinetic::anim {

// Playback state for one animated object: current clip, time, speed, looping, and a crossfade
// from the previous clip when switching. Clips are passed in by the caller (the player only
// stores indices), so one SkinnedModel can drive many players.
class AnimationPlayer {
public:
    static constexpr i32 kNoClip = -1;

    f32 speed = 1.0f;    // playback rate; negative plays backwards
    bool loop = true;
    bool playing = true;

    // Switches to `clip` from time 0 and starts playing. With `fadeSeconds` > 0 the previous
    // clip keeps running and is blended out over that time.
    void play(i32 clip, f32 fadeSeconds = 0.0f);
    // Pauses and rewinds to the start of the current clip.
    void stop();
    // Jumps to `time` (seconds) in the current clip; cancels any crossfade.
    void setTime(f32 time, std::span<const AnimationClip> clips);

    // Advances time by dt * speed (when playing) and progresses the crossfade.
    void update(f32 dt, std::span<const AnimationClip> clips);

    // Local pose at the current time: rest pose with the clip applied (blended with the previous
    // clip during a crossfade). `scratch` is working memory, reused between calls.
    void evaluate(const Skeleton& skeleton, std::span<const AnimationClip> clips, Pose& out, Pose& scratch) const;

    [[nodiscard]] i32 clip() const { return m_clip; }
    [[nodiscard]] f32 time() const { return m_time; }
    [[nodiscard]] bool isFading() const { return m_previousClip != kNoClip && m_fadeDuration > 0.0f; }
    // Weight of the current clip during a crossfade (1 when not fading).
    [[nodiscard]] f32 fadeWeight() const;
    [[nodiscard]] i32 previousClip() const { return m_previousClip; }

private:
    i32 m_clip = kNoClip;
    f32 m_time = 0.0f;
    i32 m_previousClip = kNoClip;
    f32 m_previousTime = 0.0f;
    f32 m_fadeDuration = 0.0f;
    f32 m_fadeElapsed = 0.0f;
};

} // namespace kinetic::anim
