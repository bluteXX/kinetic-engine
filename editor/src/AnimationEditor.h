#pragma once

#include <kinetic/gfx/Device.h>
#include <kinetic/renderer/Renderer.h>
#include <kinetic/scene/AnimatorComponent.h>
#include <kinetic/scene/Scene.h>

#include <unordered_map>
#include <vector>

namespace kinetic::editor {

// The Animation Editor: a separate window for working with one animated actor.
//
//   left    Clips (click to switch, crossfading)   +   Skeleton (joint hierarchy; the selected
//           joint is highlighted and its local transform shown)
//   centre  Preview: the actor's model on its own in a small scene with an orbit camera
//           (LMB orbit, MMB pan, wheel zoom), toggles for skeleton / mesh / grid / bind pose
//   bottom  Transport (start, frame step, play/pause, end, stop, loop, speed, crossfade) and the
//           timeline (ruler, keyframes, playhead; click or drag to scrub)
//
// It drives the actor's own AnimatorComponent, so the main Viewport shows the same pose. The
// preview renders the actor's (already skinned) instance mesh into its own render target.
class AnimationEditor {
public:
    static constexpr u32 kOverlayLayer = 1; // Renderer overlay layer used by the preview

    void open(entt::entity actor);
    [[nodiscard]] bool isOpen() const { return m_open; }
    [[nodiscard]] entt::entity actor() const { return m_actor; }

    // Window (call in onImGui).
    void draw(scene::Scene& scene, renderer::Renderer& renderer);
    // After the animation system ran: preview overlay lines (skeleton).
    void updateOverlay(const scene::Scene& scene, renderer::Renderer& renderer);
    // Offscreen pass into the preview target (call from Application::onRenderOffscreen).
    void renderPreview(gfx::Frame& frame, renderer::Renderer& renderer);
    void shutdown(renderer::Renderer& renderer);

private:
    void syncPreview(const scene::Scene& scene, const scene::AnimatorComponent& animator);
    void frameModel(const anim::SkinnedModel& model);
    void updateOrbitCamera();
    void drawPreview(renderer::Renderer& renderer, scene::AnimatorComponent& animator);
    void drawClipList(scene::AnimatorComponent& animator);
    void drawTransport(scene::AnimatorComponent& animator);
    void drawTimeline(scene::AnimatorComponent& animator);
    void drawSkeleton(const scene::AnimatorComponent& animator);
    void drawJointNode(const scene::AnimatorComponent& animator, usize joint);
    const std::vector<f32>& keyTimes(const anim::AnimationClip& clip);

    static constexpr f32 kFrameRate = 30.0f; // frame stepping and the frame counter

    bool m_open = false;
    entt::entity m_actor = entt::null;
    const anim::SkinnedModel* m_framedModel = nullptr; // model the camera was last framed on

    // Preview scene: a camera and one actor that shows the edited actor's instance mesh.
    scene::Scene m_preview;
    entt::entity m_previewCamera = entt::null;
    entt::entity m_previewActor = entt::null;
    renderer::RenderTarget m_target;
    bool m_targetVisible = false; // the preview image was drawn this frame

    // Orbit camera.
    glm::vec3 m_orbitTarget{0.0f, 1.0f, 0.0f};
    f32 m_yaw = 0.6f;
    f32 m_pitch = 0.25f;
    f32 m_distance = 3.0f;

    // Display options.
    bool m_showSkeleton = true;
    bool m_showMesh = true;
    bool m_showGrid = true;

    f32 m_crossfade = 0.25f;
    i32 m_joint = -1;
    bool m_scrubbing = false;
    bool m_playingBeforeScrub = false;
    std::vector<std::vector<usize>> m_children; // per joint, rebuilt every draw
    std::unordered_map<const anim::AnimationClip*, std::vector<f32>> m_keyTimes;
};

} // namespace kinetic::editor
