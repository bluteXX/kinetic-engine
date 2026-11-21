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

// How a clip is played.
//   Kinematic  the pose is shown as sampled (regular animation)
//   Physics    the pose is a target for muscles (motors) of a physics body; what you see is the
//              simulation trying to follow it (hits, rebounds, muscles giving way)
enum class ClipKind : u8 { Kinematic, Physics };

class AnimationClip {
public:
    AnimationClip() = default;
    AnimationClip(std::string name, f32 duration, std::vector<JointTracks> tracks);

    [[nodiscard]] const std::string& name() const { return m_name; }
    [[nodiscard]] f32 duration() const { return m_duration; }
    [[nodiscard]] usize jointCount() const { return m_tracks.size(); }
    [[nodiscard]] const std::vector<JointTracks>& tracks() const { return m_tracks; }
    // Editable keys (clip processing, see ClipTools.h).
    [[nodiscard]] std::vector<JointTracks>& tracks() { return m_tracks; }
    // Root motion taken out of the pose (ClipTools::extractRootYaw): how fast the character
    // turns while this clip plays, rad/s (positive = counter-clockwise from above).
    [[nodiscard]] f32 rootYawRate() const { return m_rootYawRate; }
    void setRootYawRate(f32 rate) { m_rootYawRate = rate; }
    // Root motion taken out of the pose (ClipTools::extractRootMotion): the character's
    // displacement since the clip's start, model space (XZ), keyed like the clip. The game moves
    // the character by it (e.g. the step of an attack) instead of the hips sliding off.
    [[nodiscard]] bool hasRootMotion() const { return !m_rootMotion.empty(); }
    [[nodiscard]] const Track<glm::vec3>& rootMotion() const { return m_rootMotion; }
    void setRootMotion(Track<glm::vec3> track) { m_rootMotion = std::move(track); }
    [[nodiscard]] glm::vec3 sampleRootMotion(f32 time) const;
    [[nodiscard]] ClipKind kind() const { return m_kind; }
    void setKind(ClipKind kind) { m_kind = kind; }
    // The weapon hand was turned so the blade cuts edge first (anim::alignBladeEdge).
    [[nodiscard]] bool edgeAligned() const { return m_edgeAligned; }
    void setEdgeAligned(bool aligned) { m_edgeAligned = aligned; }

    // Writes animated channels at `time` into `out` (which should start as the rest pose).
    void sample(f32 time, bool loop, Pose& out) const;

private:
    std::string m_name;
    f32 m_duration = 0.0f;
    std::vector<JointTracks> m_tracks; // indexed by skeleton joint
    ClipKind m_kind = ClipKind::Kinematic;
    bool m_edgeAligned = false;
    f32 m_rootYawRate = 0.0f;
    Track<glm::vec3> m_rootMotion;
};

} // namespace kinetic::anim
