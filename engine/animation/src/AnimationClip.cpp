#include "kinetic/animation/AnimationClip.h"

#include "kinetic/core/Assert.h"

#include <algorithm>
#include <cmath>

namespace kinetic::anim {

namespace {

glm::vec3 interpolate(const glm::vec3& a, const glm::vec3& b, f32 t) { return glm::mix(a, b, t); }

glm::quat interpolate(const glm::quat& a, const glm::quat& b, f32 t) { return glm::slerp(a, b, t); }

template <typename T>
T sampleTrack(const Track<T>& track, f32 time)
{
    const auto& times = track.times;
    if (times.size() == 1 || time <= times.front()) {
        return track.values.front();
    }
    if (time >= times.back()) {
        return track.values.back();
    }
    // First key strictly after `time`; the segment is [next - 1, next].
    const auto it = std::upper_bound(times.begin(), times.end(), time);
    const auto next = static_cast<usize>(it - times.begin());
    const usize prev = next - 1;
    if (track.interpolation == Interpolation::Step) {
        return track.values[prev];
    }
    const f32 span = times[next] - times[prev];
    const f32 t = span > 0.0f ? (time - times[prev]) / span : 0.0f;
    return interpolate(track.values[prev], track.values[next], t);
}

} // namespace

AnimationClip::AnimationClip(std::string name, f32 duration, std::vector<JointTracks> tracks)
    : m_name(std::move(name))
    , m_duration(duration)
    , m_tracks(std::move(tracks))
{
}

void AnimationClip::sample(f32 time, bool loop, Pose& out) const
{
    KN_ASSERT(out.jointCount() == m_tracks.size());
    if (loop && m_duration > 0.0f) {
        time = std::fmod(time, m_duration);
        if (time < 0.0f) {
            time += m_duration;
        }
    } else {
        time = std::clamp(time, 0.0f, m_duration);
    }

    for (usize joint = 0; joint < m_tracks.size(); ++joint) {
        const JointTracks& jt = m_tracks[joint];
        if (!jt.translation.empty()) {
            out.translations[joint] = sampleTrack(jt.translation, time);
        }
        if (!jt.rotation.empty()) {
            out.rotations[joint] = glm::normalize(sampleTrack(jt.rotation, time));
        }
        if (!jt.scale.empty()) {
            out.scales[joint] = sampleTrack(jt.scale, time);
        }
    }
}

} // namespace kinetic::anim
