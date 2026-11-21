#pragma once

#include <kinetic/gfx/Device.h>
#include <kinetic/physanim/PhysicalBody.h>
#include <kinetic/renderer/Renderer.h>
#include <kinetic/scene/AnimatorComponent.h>
#include <kinetic/scene/Scene.h>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace kinetic::editor {

// Components an actor can have, as listed in the Actor Editor.
enum class ComponentKind : u8 { MeshRenderer, Animator, CharacterController, Collider, PhysicsCharacter };

// What the Actor Editor needs from the application to show / edit components (the asset drawer,
// GPU resources, ...). The editor app implements it.
class ActorEditorHost {
public:
    virtual ~ActorEditorHost() = default;
    // Settings of one component (no header; the Actor Editor draws the frame).
    virtual void drawComponentDetails(entt::entity actor, ComponentKind kind) = 0;
    virtual void removeComponent(entt::entity actor, ComponentKind kind) = 0;
    // Items of the "+ Add Component" popup.
    virtual void drawAddComponentMenu(entt::entity actor) = 0;
};

// The Actor Editor: a separate window for one actor (double-click it in the Viewport or the
// Hierarchy). Layout:
//
//   left    Preview (the actor alone, orbit camera)     right   tabs: Animation Graph |
//           Components (list + the selected one's               Clips & Timeline | Skeleton
//           settings, e.g. Animator: foot IK)
//
// Animation Graph: Unity-Animator-like state machine canvas (states, transitions, Entry,
// Any State; pan / zoom, drag states, right-click to create, link and delete), the parameter list
// on its left (live values), and the selected state's motion (clip, 1D or 2D blend with its plot)
// or transition (conditions, exit time, crossfade) underneath.
//
// It drives the actor's own AnimatorComponent, so the main Viewport shows the same pose. The
// preview renders the actor's (already skinned) instance mesh into its own render target.
class AnimationEditor {
public:
    static constexpr u32 kOverlayLayer = 1; // Renderer overlay layer used by the preview

    enum class Tab : u8 { Graph, Clips, Skeleton };

    void open(entt::entity actor, std::optional<Tab> tab = std::nullopt);
    [[nodiscard]] bool isOpen() const { return m_open; }
    [[nodiscard]] entt::entity actor() const { return m_actor; }
    // The window (or one of its children) has keyboard focus: the app's own shortcuts (Del, F,
    // Esc) must not act on the scene then.
    [[nodiscard]] bool focused() const { return m_open && m_focused; }

    // Window (call in onImGui).
    void draw(scene::Scene& scene, renderer::Renderer& renderer, ActorEditorHost& host);
    // After the animation system ran: preview overlay lines (skeleton).
    void updateOverlay(const scene::Scene& scene, renderer::Renderer& renderer);
    // Offscreen pass into the preview target (call from Application::onRenderOffscreen).
    void renderPreview(gfx::Frame& frame, renderer::Renderer& renderer);
    void shutdown(renderer::Renderer& renderer);

private:
    // Selection inside the graph.
    enum class GraphItem : u8 { None, State, Transition, Entry, AnyState };

    // --- AnimationEditor.cpp: window, preview, components, clips, skeleton ---
    void syncPreview(const scene::Scene& scene, const renderer::Renderer& renderer);
    void frame(const glm::vec3& boundsMin, const glm::vec3& boundsMax);
    void updateOrbitCamera();
    void drawPreview(renderer::Renderer& renderer, scene::AnimatorComponent* animator);
    void drawComponents(scene::Scene& scene, ActorEditorHost& host);
    void drawClipsTab(scene::Scene& scene, scene::AnimatorComponent& animator);
    // Blade edge alignment of the selected clip (actors holding a weapon).
    void drawBladeEdge(scene::Scene& scene, scene::AnimatorComponent& animator);
    void drawClipList(scene::AnimatorComponent& animator);
    void drawTransport(scene::AnimatorComponent& animator);
    void drawTimeline(scene::AnimatorComponent& animator);
    void drawSkeleton(const scene::AnimatorComponent& animator);
    void drawJointNode(const scene::AnimatorComponent& animator, usize joint);
    const std::vector<f32>& keyTimes(const anim::AnimationClip& clip);

    // --- AnimationGraphEditor.cpp: state machine canvas, parameters, details ---
    void drawGraphTab(scene::Scene& scene, scene::AnimatorComponent& animator);
    void drawParameters(scene::AnimatorComponent& animator);
    void drawCanvas(scene::AnimatorComponent& animator);
    void drawCanvasMenus(scene::AnimatorComponent& animator, const glm::vec2& mouseGraph);
    void drawGraphDetails(const scene::Scene& scene, scene::AnimatorComponent& animator);
    void drawStateDetails(const scene::Scene& scene, scene::AnimatorComponent& animator, i32 state);
    void drawTransitionDetails(scene::AnimatorComponent& animator, i32 transition);
    void drawMotionEditor(const scene::Scene& scene, scene::AnimatorComponent& animator, i32 node);
    // Physics bones of a clip motion: which bones run on muscles while it plays (physanim).
    void drawPhysicsBones(const scene::Scene& scene, scene::AnimatorComponent& animator, anim::BlendNode& node);
    void drawBlend1DPlot(scene::AnimatorComponent& animator, i32 node);
    void drawBlend2DPlot(scene::AnimatorComponent& animator, i32 node);
    void deleteGraphSelection(scene::AnimatorComponent& animator);

    static constexpr f32 kFrameRate = 30.0f; // frame stepping and the frame counter

    bool m_open = false;
    bool m_focused = false;
    entt::entity m_actor = entt::null;
    Tab m_tab = Tab::Graph;
    bool m_selectTab = false;
    const void* m_framedFor = nullptr; // model / mesh the camera was last framed on

    // Preview scene: a camera and one actor that shows the edited actor's mesh.
    scene::Scene m_preview;
    entt::entity m_previewCamera = entt::null;
    entt::entity m_previewActor = entt::null;
    renderer::RenderTarget m_target;
    bool m_targetVisible = false; // the preview image was drawn this frame
    glm::vec3 m_boundsMin{0.0f};
    glm::vec3 m_boundsMax{0.0f};

    // Orbit camera.
    glm::vec3 m_orbitTarget{0.0f, 1.0f, 0.0f};
    f32 m_yaw = 0.6f;
    f32 m_pitch = 0.25f;
    f32 m_distance = 3.0f;

    // Display options.
    bool m_showSkeleton = false;
    bool m_showMesh = true;
    bool m_showGrid = true;
    bool m_showGizmos = true; // the actor's components (collider, controller capsule)

    // Components panel.
    ComponentKind m_component = ComponentKind::Animator;

    // Clips & timeline.
    f32 m_crossfade = 0.25f;
    i32 m_joint = -1;
    bool m_scrubbing = false;
    bool m_playingBeforeScrub = false;
    std::vector<std::vector<usize>> m_children; // per joint, rebuilt every draw
    std::unordered_map<const anim::AnimationClip*, std::vector<f32>> m_keyTimes;

    // Graph canvas.
    glm::vec2 m_pan{0.0f};
    f32 m_zoom = 1.0f;
    GraphItem m_item = GraphItem::None;
    i32 m_itemIndex = -1;
    i32 m_dragState = -2;      // state being dragged (-1 entry, -3 any state, -2 none)
    i32 m_linkFrom = -2;       // making a transition from this state (-1 = any state, -2 = not linking)
    i32 m_selectedChild = -1;  // selected child in the 1D / 2D plot
    std::string m_edgeMessage; // result of the last blade edge alignment
    f32 m_detailsHeight = 340.0f; // height of the details panel under the graph (splitter)
    // Body layouts (physics bodies) per model, for the physics bones list.
    std::unordered_map<const anim::SkinnedModel*, physanim::BodyLayout> m_bodyLayouts;
};

} // namespace kinetic::editor
