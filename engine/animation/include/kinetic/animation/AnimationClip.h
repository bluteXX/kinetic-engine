#pragma once

#include "kinetic/animation/Pose.h"

#include <string>
#include <vector>

namespace kinetic::anim {

enum class Interpolation : u8 { Step, Linear };

template <typename T>
struct Track {
    std::vector<f32> times; // strictly increasing, seconds
    std::vector<T> values;
    Interpolation interpolation = Interpolation::Linear;

    [[nodiscard]] bool empty() const { return times.empty(); }
};

// Animated channels of one joint. Empty tracks leave the corresponding channel untouched.
struct JointTracks {
    Track<glm::vec3> translation;
    Track<glm::quat> rotation;
    Track<glm::vec3> scale;
};

class AnimationClip {
public:
    AnimationClip() = default;
    AnimationClip(std::string name, f32 duration, std::vector<JointTracks> tracks);

    [[nodiscard]] const std::string& name() const { return m_name; }
    [[nodiscard]] f32 duration() const { return m_duration; }
    [[nodiscard]] usize jointCount() const { return m_tracks.size(); }
    [[nodiscard]] const std::vector<JointTracks>& tracks() const { return m_tracks; }

    // Writes animated channels at `time` into `out` (which should start as the rest pose).
    void sample(f32 time, bool loop, Pose& out) const;

private:
    std::string m_name;
    f32 m_duration = 0.0f;
    std::vector<JointTracks> m_tracks; // indexed by skeleton joint
};

} // namespace kinetic::anim
