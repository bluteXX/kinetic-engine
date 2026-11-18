#pragma once

#include <kinetic/scene/AnimatorComponent.h>
#include <kinetic/scene/Scene.h>

#include <unordered_map>
#include <vector>

namespace kinetic::editor {

// Animation controls for an animated actor, in two layouts:
//   drawInline  - compact, inside the Inspector's "Animation" section of the selected actor
//   drawWindow  - the full "Animation" window (clip list on the left), opened on demand
//
//   Clips      list of the model's clips; click one to switch (crossfading from the current one)
//   Transport  start / previous frame / play-pause / next frame / end, loop, speed, crossfade time
//   Timeline   ruler, keyframe markers, playhead; click or drag to scrub
//   Display    skeleton overlay, mesh visibility, bind pose
//   Joints     hierarchy of the skeleton; the selected joint is highlighted in the Viewport and
//              its current local transform is shown
class AnimationPanel {
public:
    // Full window; `open` is cleared when the user closes it.
    void drawWindow(scene::Scene& scene, entt::entity selected, bool* open);
    // Compact controls for one entity (call inside another window).
    void drawInline(scene::Scene& scene, entt::entity entity);

    // Read by the editor when building the skeleton overlay.
    bool showSkeleton = true;
    [[nodiscard]] i32 selectedJoint() const { return m_joint; }

private:
    void syncSelection(entt::entity entity);
    void drawClipCombo(scene::AnimatorComponent& animator);
    void drawDisplayOptions(scene::Scene& scene, entt::entity entity, scene::AnimatorComponent& animator);
    void drawTransport(scene::AnimatorComponent& animator, bool compact);
    void drawTimeline(scene::AnimatorComponent& animator);
    void drawJoints(const scene::AnimatorComponent& animator);
    void drawJointNode(const scene::AnimatorComponent& animator, usize joint);
    const std::vector<f32>& keyTimes(const anim::AnimationClip& clip);

    static constexpr f32 kFrameRate = 30.0f; // frame stepping and the frame counter

    f32 m_crossfade = 0.25f;
    i32 m_joint = -1;
    entt::entity m_jointOwner = entt::null; // entity m_joint belongs to
    bool m_scrubbing = false;
    bool m_playingBeforeScrub = false;
    std::vector<std::vector<usize>> m_children; // per joint, rebuilt every draw
    std::unordered_map<const anim::AnimationClip*, std::vector<f32>> m_keyTimes;
};

} // namespace kinetic::editor
