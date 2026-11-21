#include "AnimationEditor.h"

#include "CharacterSetup.h"
#include "ComponentGizmos.h"

#include <kinetic/gameplay/CharacterController.h>
#include <kinetic/renderer/AnimationSystem.h>

#include <imgui.h>

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace kinetic::editor {

namespace {

constexpr ImU32 kTimelineBg = IM_COL32(28, 30, 36, 255);
constexpr ImU32 kTickMinor = IM_COL32(70, 74, 84, 255);
constexpr ImU32 kTickMajor = IM_COL32(120, 125, 138, 255);
constexpr ImU32 kKeyColor = IM_COL32(235, 200, 90, 255);
constexpr ImU32 kPlayhead = IM_COL32(240, 80, 70, 255);

void tooltip(const char* text)
{
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip("%s", text);
    }
}

const char* componentName(ComponentKind kind)
{
    switch (kind) {
    case ComponentKind::MeshRenderer: return "Mesh Renderer";
    case ComponentKind::Animator: return "Animator";
    case ComponentKind::CharacterController: return "Character Controller";
    case ComponentKind::Collider: return "Box Collider";
    case ComponentKind::PhysicsCharacter: return "Character (physics)";
    }
    return "?";
}

} // namespace

void AnimationEditor::open(entt::entity actor, std::optional<Tab> tab)
{
    if (actor != m_actor) {
        m_joint = -1;
        m_item = GraphItem::None;
        m_itemIndex = -1;
        m_linkFrom = -2;
        m_framedFor = nullptr; // re-frame the camera on the new actor
        m_pan = glm::vec2(0.0f);
        m_zoom = 1.0f;
    }
    if (tab) {
        m_tab = *tab;
        m_selectTab = true;
    }
    m_actor = actor;
    m_open = true;
    ImGui::SetWindowFocus("###ActorEditor");
}

void AnimationEditor::draw(scene::Scene& scene, renderer::Renderer& renderer, ActorEditorHost& host)
{
    m_targetVisible = false;
    if (!m_open) {
        return;
    }
    auto& registry = scene.registry();
    const bool valid = registry.valid(m_actor);
    const char* actorName = valid ? registry.get<scene::NameComponent>(m_actor).name.c_str() : "";

    char title[160];
    std::snprintf(title, sizeof(title), "Actor Editor - %s###ActorEditor", actorName);
    // Never larger than the editor's window, and kept inside it (a saved layout from a bigger
    // screen would otherwise hide the bottom of the window - and its resize corner - off screen).
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const ImVec2 work = viewport->WorkSize;
    ImGui::SetNextWindowSizeConstraints(ImVec2(std::min(640.0f, work.x), std::min(420.0f, work.y)), work);
    ImGui::SetNextWindowSize(ImVec2(std::min(1280.0f, work.x * 0.92f), std::min(780.0f, work.y * 0.9f)), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + work.x * 0.5f, viewport->WorkPos.y + work.y * 0.5f), ImGuiCond_FirstUseEver,
                            ImVec2(0.5f, 0.5f));
    m_focused = false;
    if (!ImGui::Begin(title, &m_open, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
        ImGui::End();
        return;
    }
    if (!ImGui::IsWindowDocked()) {
        const ImVec2 pos = ImGui::GetWindowPos();
        const ImVec2 size = ImGui::GetWindowSize();
        ImVec2 fitted = pos;
        fitted.x = std::clamp(pos.x, viewport->WorkPos.x, std::max(viewport->WorkPos.x, viewport->WorkPos.x + work.x - size.x));
        fitted.y = std::clamp(pos.y, viewport->WorkPos.y, std::max(viewport->WorkPos.y, viewport->WorkPos.y + work.y - size.y));
        if (fitted.x != pos.x || fitted.y != pos.y) {
            ImGui::SetWindowPos(fitted);
        }
    }
    m_focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
    if (!valid) {
        ImGui::Spacing();
        ImGui::TextDisabled("This actor no longer exists. Double-click an actor in the Viewport or the Hierarchy.");
        ImGui::End();
        return;
    }
    auto* animator = registry.try_get<scene::AnimatorComponent>(m_actor);
    if (animator && !animator->model) {
        animator = nullptr;
    }
    syncPreview(scene, renderer);

    if (ImGui::BeginTable("##layout", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
        ImGui::TableSetupColumn("left", ImGuiTableColumnFlags_WidthStretch, 0.38f);
        ImGui::TableSetupColumn("right", ImGuiTableColumnFlags_WidthStretch, 0.62f);
        ImGui::TableNextRow();

        // Left: preview over the components.
        ImGui::TableSetColumnIndex(0);
        const f32 leftHeight = ImGui::GetContentRegionAvail().y;
        ImGui::BeginChild("##preview", ImVec2(0, leftHeight * 0.52f), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        drawPreview(renderer, animator);
        ImGui::EndChild();
        ImGui::BeginChild("##components", ImVec2(0, 0), ImGuiChildFlags_None);
        drawComponents(scene, host);
        ImGui::EndChild();

        // Right: graph / clips / skeleton.
        ImGui::TableSetColumnIndex(1);
        ImGui::BeginChild("##right", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar);
        animator = registry.try_get<scene::AnimatorComponent>(m_actor); // the components panel may have changed it
        if (!animator || !animator->model) {
            ImGui::Spacing();
            ImGui::TextDisabled("No Animator: add one in Components (the actor's mesh must be an animated asset).");
        } else if (ImGui::BeginTabBar("##rightTabs")) {
            const auto flags = [&](Tab t) {
                return m_selectTab && m_tab == t ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None;
            };
            if (ImGui::BeginTabItem("Animation Graph", nullptr, flags(Tab::Graph))) {
                if (!m_selectTab) m_tab = Tab::Graph;
                drawGraphTab(scene, *animator);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Clips & Timeline", nullptr, flags(Tab::Clips))) {
                if (!m_selectTab) m_tab = Tab::Clips;
                drawClipsTab(scene, *animator);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Skeleton", nullptr, flags(Tab::Skeleton))) {
                if (!m_selectTab) m_tab = Tab::Skeleton;
                drawSkeleton(*animator);
                ImGui::EndTabItem();
            }
            m_selectTab = false;
            ImGui::EndTabBar();
        }
        ImGui::EndChild();
        ImGui::EndTable();
    }
    ImGui::End();
}

// ---------------------------------------------------------------------------------------------- //
// Preview

void AnimationEditor::syncPreview(const scene::Scene& scene, const renderer::Renderer& renderer)
{
    if (m_previewCamera == entt::null) {
        m_previewCamera = m_preview.createCamera("Preview Camera", {0.0f, 1.0f, 3.0f}, {0.0f, 1.0f, 0.0f});
        m_previewActor = m_preview.createMesh("Preview", scene::kTriangleMesh, glm::vec4(1.0f));
    }
    const auto& registry = scene.registry();
    auto& mesh = m_preview.registry().get<scene::MeshRendererComponent>(m_previewActor);
    const auto* source = registry.try_get<scene::MeshRendererComponent>(m_actor);
    if (!source) {
        mesh.visible = false;
        return;
    }
    // Bounds: the animated model's rest pose, or the static mesh (both in model units: the preview
    // shows the model unscaled).
    const void* framedFor = nullptr;
    if (const auto* animator = registry.try_get<scene::AnimatorComponent>(m_actor); animator && animator->model) {
        m_boundsMin = animator->model->restBoundsMin;
        m_boundsMax = animator->model->restBoundsMax;
        framedFor = animator->model.get();
    } else if (const renderer::GpuMesh* gpu = renderer.mesh(source->mesh)) {
        m_boundsMin = gpu->boundsMin;
        m_boundsMax = gpu->boundsMax;
        framedFor = gpu;
    }
    if (framedFor && m_framedFor != framedFor) {
        m_framedFor = framedFor;
        m_keyTimes.clear(); // clip addresses may have changed
        frame(m_boundsMin, m_boundsMax);
    }

    // The preview actor shows the edited actor's mesh (animated: its skinned instance), with its
    // material, standing on the preview's floor at the origin.
    mesh = *source;
    mesh.visible = m_showMesh;
    auto& transform = m_preview.registry().get<scene::TransformComponent>(m_previewActor);
    transform.scale = glm::vec3(1.0f);
    const glm::vec3 center = (m_boundsMin + m_boundsMax) * 0.5f;
    transform.position = -glm::vec3(center.x, m_boundsMin.y, center.z);
}

void AnimationEditor::frame(const glm::vec3& boundsMin, const glm::vec3& boundsMax)
{
    const glm::vec3 size = boundsMax - boundsMin;
    const f32 radius = std::max(glm::length(size) * 0.5f, 0.1f);
    m_orbitTarget = glm::vec3(0.0f, size.y * 0.5f, 0.0f);
    m_distance = radius / std::sin(glm::radians(60.0f) * 0.5f) * 1.05f;
    m_yaw = 0.6f;
    m_pitch = 0.2f;
}

void AnimationEditor::updateOrbitCamera()
{
    auto& transform = m_preview.registry().get<scene::TransformComponent>(m_previewCamera);
    const glm::vec3 offset(std::cos(m_pitch) * std::sin(m_yaw), std::sin(m_pitch), std::cos(m_pitch) * std::cos(m_yaw));
    transform.position = m_orbitTarget + offset * m_distance;
    transform.rotation = scene::lookAtRotation(transform.position, m_orbitTarget);
    auto& camera = m_preview.registry().get<scene::CameraComponent>(m_previewCamera);
    camera.nearPlane = std::max(m_distance * 0.01f, 0.005f);
    camera.farPlane = std::max(m_distance * 50.0f, 50.0f);
}

void AnimationEditor::drawPreview(renderer::Renderer& renderer, scene::AnimatorComponent* animator)
{
    const ImVec2 size = ImGui::GetContentRegionAvail();
    if (size.x < 1.0f || size.y < 1.0f) {
        return;
    }
    const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
    renderer.resizeTarget(m_target, static_cast<u32>(size.x * scale.x), static_cast<u32>(size.y * scale.y));

    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);
    // Allow overlap: the toolbar submitted on top of this button must still get the mouse.
    ImGui::SetNextItemAllowOverlap();
    ImGui::InvisibleButton("##previewInput", size,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonMiddle |
                               ImGuiButtonFlags_MouseButtonRight);
    const bool hovered = ImGui::IsItemHovered();
    const ImGuiIO& io = ImGui::GetIO();
    if (ImGui::IsItemActive()) {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) || ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
            m_yaw -= io.MouseDelta.x * 0.008f;
            m_pitch = std::clamp(m_pitch + io.MouseDelta.y * 0.008f, -1.5f, 1.5f);
        } else if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
            const auto& cam = m_preview.registry().get<scene::TransformComponent>(m_previewCamera);
            const f32 panScale = m_distance * 0.0018f;
            m_orbitTarget += cam.rotation * glm::vec3(-io.MouseDelta.x * panScale, io.MouseDelta.y * panScale, 0.0f);
        }
    }
    if (hovered && io.MouseWheel != 0.0f) {
        m_distance = std::max(m_distance * std::pow(0.88f, io.MouseWheel), 0.05f);
    }
    updateOrbitCamera();

    ImDrawList* draw = ImGui::GetWindowDrawList();
    if (m_target.valid()) {
        draw->AddImage(static_cast<ImTextureID>(reinterpret_cast<intptr_t>(m_target.color)), p0, p1);
        m_targetVisible = true;
    }

    // Toolbar over the top-left corner of the image.
    ImGui::SetCursorScreenPos(ImVec2(p0.x + 8.0f, p0.y + 6.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 2.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, IM_COL32(20, 22, 26, 200));
    ImGui::Checkbox("Mesh", &m_showMesh);
    ImGui::SameLine();
    ImGui::Checkbox("Grid", &m_showGrid);
    ImGui::SameLine();
    ImGui::Checkbox("Gizmos", &m_showGizmos);
    tooltip("Show the components: collider box, character controller capsule");
    if (animator) {
        ImGui::SameLine();
        ImGui::Checkbox("Skeleton", &m_showSkeleton);
        ImGui::SameLine();
        ImGui::Checkbox("Bind pose", &animator->bindPose);
        tooltip("Show the skeleton's rest pose instead of the animation");
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Frame")) {
        frame(m_boundsMin, m_boundsMax);
    }
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();

    const char* help = "LMB / RMB drag: orbit   MMB drag: pan   wheel: zoom";
    draw->AddText(ImVec2(p0.x + 8.0f, p1.y - ImGui::GetTextLineHeight() - 6.0f), IM_COL32(150, 155, 165, 200), help);
}

void AnimationEditor::updateOverlay(const scene::Scene& scene, renderer::Renderer& renderer)
{
    std::vector<renderer::DebugVertex> lines;
    const auto& registry = scene.registry();
    if (m_open && m_previewActor != entt::null && registry.valid(m_actor)) {
        const glm::mat4 world = m_preview.registry().get<scene::TransformComponent>(m_previewActor).matrix();
        if (const auto* animator = registry.try_get<scene::AnimatorComponent>(m_actor); animator && m_showSkeleton) {
            renderer::appendSkeletonLines(*animator, world, lines, m_joint);
        }
        if (m_showGizmos) {
            GizmoOptions options;
            options.footIK = false; // foot IK targets live in the scene, not in the preview
            options.highlight = true;
            appendComponentGizmos(scene, m_actor, world, lines, options);
        }
    }
    renderer.setOverlayLines(std::move(lines), kOverlayLayer);
}

void AnimationEditor::renderPreview(gfx::Frame& frame, renderer::Renderer& renderer)
{
    if (!m_open || !m_targetVisible || !m_target.valid()) {
        return;
    }
    SDL_GPURenderPass* pass = renderer.beginTargetPass(frame.cmd, m_target, glm::vec4(0.11f, 0.12f, 0.14f, 1.0f));
    renderer::SceneDrawOptions options;
    options.grid = m_showGrid;
    options.overlayLayer = kOverlayLayer;
    options.aspect = m_target.aspect();
    renderer.drawScene(frame.cmd, pass, m_preview, options);
    renderer.endPass(pass);
}

void AnimationEditor::shutdown(renderer::Renderer& renderer) { renderer.releaseTarget(m_target); }

// ---------------------------------------------------------------------------------------------- //
// Components

void AnimationEditor::drawComponents(scene::Scene& scene, ActorEditorHost& host)
{
    const auto& registry = scene.registry();
    std::vector<ComponentKind> present;
    if (registry.all_of<scene::MeshRendererComponent>(m_actor)) present.push_back(ComponentKind::MeshRenderer);
    if (registry.all_of<scene::AnimatorComponent>(m_actor)) present.push_back(ComponentKind::Animator);
    if (registry.all_of<scene::CharacterComponent>(m_actor)) present.push_back(ComponentKind::PhysicsCharacter);
    if (registry.all_of<scene::ColliderComponent>(m_actor)) present.push_back(ComponentKind::Collider);
    if (registry.all_of<gameplay::CharacterControllerComponent>(m_actor)) present.push_back(ComponentKind::CharacterController);
    std::ranges::sort(present);
    if (!present.empty() && std::ranges::find(present, m_component) == present.end()) {
        m_component = present.front();
    }

    ImGui::SeparatorText("Components");
    ImGui::BeginChild("##componentList", ImVec2(0, ImGui::GetFrameHeightWithSpacing() * (static_cast<f32>(present.size()) + 0.4f)),
                      ImGuiChildFlags_None);
    std::optional<ComponentKind> remove;
    for (const ComponentKind kind : present) {
        ImGui::PushID(static_cast<int>(kind));
        if (ImGui::Selectable(componentName(kind), m_component == kind, ImGuiSelectableFlags_AllowOverlap)) {
            m_component = kind;
        }
        ImGui::SameLine(ImGui::GetContentRegionAvail().x + ImGui::GetCursorPosX() - 18.0f);
        if (ImGui::SmallButton("x")) {
            remove = kind;
        }
        tooltip("Remove component");
        ImGui::PopID();
    }
    ImGui::EndChild();
    if (remove) {
        host.removeComponent(m_actor, *remove);
    }
    if (ImGui::Button("+ Add Component", ImVec2(-FLT_MIN, 0))) {
        ImGui::OpenPopup("##addComponent");
    }
    if (ImGui::BeginPopup("##addComponent")) {
        host.drawAddComponentMenu(m_actor);
        ImGui::EndPopup();
    }

    if (!present.empty() && !remove) {
        ImGui::SeparatorText(componentName(m_component));
        host.drawComponentDetails(m_actor, m_component);
    }
}

// ---------------------------------------------------------------------------------------------- //
// Clips & timeline

void AnimationEditor::drawClipsTab(scene::Scene& scene, scene::AnimatorComponent& animator)
{
    if (animator.blendTreeActive()) {
        ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.4f, 1.0f), "The animation graph is playing.");
        ImGui::SameLine();
        ImGui::TextDisabled("Clicking a clip plays it alone (graph off).");
    }
    if (ImGui::BeginTable("##clipsLayout", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
        ImGui::TableSetupColumn("list", ImGuiTableColumnFlags_WidthFixed, 230.0f);
        ImGui::TableSetupColumn("timeline", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        drawClipList(animator);
        ImGui::TableSetColumnIndex(1);
        drawTransport(animator);
        drawBladeEdge(scene, animator);
        drawTimeline(animator);
        ImGui::EndTable();
    }
}

void AnimationEditor::drawBladeEdge(scene::Scene& scene, scene::AnimatorComponent& animator)
{
    const i32 clip = animator.player.clip();
    const auto clips = animator.clips();
    if (clip < 0 || static_cast<usize>(clip) >= clips.size()) return;
    bool armed = false;
    for (const entt::entity e : scene.registry().view<scene::AttachmentComponent, scene::WeaponComponent>()) {
        armed |= scene.registry().get<scene::AttachmentComponent>(e).parent == m_actor;
    }
    if (!armed) return;
    if (clips[static_cast<usize>(clip)].edgeAligned()) {
        ImGui::TextColored(ImVec4(0.55f, 0.85f, 0.55f, 1.0f), "Blade edge aligned");
        tooltip("The hand turns about the blade so the sword cuts through the air edge first (saved with the actor)");
    } else if (ImGui::SmallButton("Align blade edge")) {
        if (const auto r = alignWeaponEdge(scene, m_actor, clip)) {
            char text[160];
            std::snprintf(text, sizeof(text), "blade off its edge %.0f -> %.0f deg (hand turned up to %.0f deg)",
                          static_cast<double>(r->before), static_cast<double>(r->after), static_cast<double>(r->maxTwist));
            m_edgeMessage = text;
        }
    } else {
        tooltip("Attacks animated without a weapon often swing the blade flat. This turns the hand about the blade\n"
                "(only while it moves fast) so an edge leads, like a real cut. Changes this clip for this actor.");
    }
    if (!m_edgeMessage.empty()) {
        ImGui::SameLine();
        ImGui::TextDisabled("%s", m_edgeMessage.c_str());
    }
}

void AnimationEditor::drawClipList(scene::AnimatorComponent& animator)
{
    const anim::SkinnedModel& model = *animator.model;
    if (model.clips.empty()) {
        ImGui::TextDisabled("(no clips in this file)");
    }
    for (usize i = 0; i < model.clips.size(); ++i) {
        const anim::AnimationClip& clip = model.clips[i];
        const bool current = animator.player.clip() == static_cast<i32>(i) && !animator.blendTreeActive();
        const bool physics = clip.kind() == anim::ClipKind::Physics;
        ImGui::PushID(static_cast<int>(i));
        if (physics) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.65f, 0.3f, 1.0f));
        }
        if (ImGui::Selectable(clip.name().c_str(), current, ImGuiSelectableFlags_AllowOverlap)) {
            animator.bindPose = false;
            animator.useBlendTree = false;
            animator.player.play(static_cast<i32>(i), current ? 0.0f : m_crossfade);
        }
        if (physics) {
            ImGui::PopStyleColor();
        }
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            ImGui::SetTooltip("%s%s", physics ? "Physics animation: in Play the muscles follow it" : "Regular (kinematic) animation",
                              std::abs(clip.rootYawRate()) > 0.0f ? "\nRoot yaw extracted: the character turns, not the pose"
                              : clip.hasRootMotion()               ? "\nRoot motion: the clip's step moves the character"
                                                                   : "");
        }
        char duration[32];
        std::snprintf(duration, sizeof(duration), "%s %.2fs", physics ? "PHYS" : "KIN", static_cast<double>(clip.duration()));
        ImGui::SameLine(ImGui::GetContentRegionAvail().x + ImGui::GetCursorPosX() - ImGui::CalcTextSize(duration).x);
        ImGui::TextDisabled("%s", duration);
        ImGui::PopID();
    }
}

void AnimationEditor::drawTransport(scene::AnimatorComponent& animator)
{
    anim::AnimationPlayer& player = animator.player;
    const auto clips = animator.clips();
    const anim::AnimationClip* clip =
        player.clip() >= 0 && static_cast<usize>(player.clip()) < clips.size() ? &clips[static_cast<usize>(player.clip())] : nullptr;
    const f32 duration = clip ? clip->duration() : 0.0f;
    const f32 frameTime = 1.0f / kFrameRate;

    ImGui::BeginDisabled(clip == nullptr);
    const ImVec2 button(34.0f, 0.0f);
    if (ImGui::Button("|<", button)) {
        player.setTime(0.0f, clips);
    }
    tooltip("Go to start");
    ImGui::SameLine();
    if (ImGui::Button("<", button)) {
        player.playing = false;
        player.setTime(player.time() - frameTime, clips);
    }
    tooltip("Previous frame");
    ImGui::SameLine();
    if (ImGui::Button(player.playing ? "Pause" : "Play", ImVec2(64.0f, 0.0f))) {
        if (!player.playing && !player.loop && player.time() >= duration && player.speed > 0.0f) {
            player.setTime(0.0f, clips); // replay a finished one-shot clip
        }
        player.playing = !player.playing;
        animator.bindPose = false;
    }
    ImGui::SameLine();
    if (ImGui::Button(">", button)) {
        player.playing = false;
        player.setTime(player.time() + frameTime, clips);
    }
    tooltip("Next frame");
    ImGui::SameLine();
    if (ImGui::Button(">|", button)) {
        player.playing = false;
        player.setTime(duration, clips);
    }
    tooltip("Go to end");
    ImGui::SameLine();
    if (ImGui::Button("Stop")) {
        player.stop();
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::Checkbox("Loop", &player.loop);
    ImGui::SetNextItemWidth(160.0f);
    ImGui::SliderFloat("Speed", &player.speed, -2.0f, 3.0f, "x%.2f");
    tooltip("Playback rate; negative plays backwards. Ctrl+click to type a value.");
    ImGui::SameLine();
    if (ImGui::SmallButton("1x")) {
        player.speed = 1.0f;
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::SliderFloat("Crossfade", &m_crossfade, 0.0f, 1.0f, "%.2f s");
    tooltip("Blend time when switching clips");
}

void AnimationEditor::drawTimeline(scene::AnimatorComponent& animator)
{
    anim::AnimationPlayer& player = animator.player;
    const auto clips = animator.clips();
    if (player.clip() < 0 || static_cast<usize>(player.clip()) >= clips.size()) {
        ImGui::TextDisabled("No clip selected.");
        return;
    }
    const anim::AnimationClip& clip = clips[static_cast<usize>(player.clip())];
    const f32 duration = std::max(clip.duration(), 1e-3f);

    ImGui::Spacing();
    constexpr f32 kHeight = 52.0f;
    constexpr f32 kPad = 8.0f; // keeps the 0 s and end markers inside the widget
    const f32 width = std::max(ImGui::GetContentRegionAvail().x, 100.0f);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1(p0.x + width, p0.y + kHeight);
    const f32 x0 = p0.x + kPad;
    const f32 x1 = p1.x - kPad;
    const auto timeToX = [&](f32 t) { return x0 + (x1 - x0) * (t / duration); };

    ImGui::InvisibleButton("##timeline", ImVec2(width, kHeight));
    if (ImGui::IsItemActivated()) {
        m_scrubbing = true;
        m_playingBeforeScrub = player.playing;
        animator.bindPose = false;
        animator.useBlendTree = false;
    }
    if (m_scrubbing && ImGui::IsItemActive()) {
        player.playing = false;
        const f32 t = (ImGui::GetIO().MousePos.x - x0) / (x1 - x0) * duration;
        player.setTime(std::clamp(t, 0.0f, duration), clips);
    }
    if (m_scrubbing && ImGui::IsItemDeactivated()) {
        m_scrubbing = false;
        player.playing = m_playingBeforeScrub;
    }

    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(p0, p1, kTimelineBg, 4.0f);

    // Ruler: pick the smallest "nice" step that leaves room between ticks.
    const f32 pixelsPerSecond = (x1 - x0) / duration;
    constexpr f32 kSteps[] = {1.0f / 30.0f, 0.1f, 0.25f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f, 30.0f, 60.0f};
    f32 minor = kSteps[std::size(kSteps) - 1];
    for (const f32 step : kSteps) {
        if (step * pixelsPerSecond >= 7.0f) {
            minor = step;
            break;
        }
    }
    f32 major = minor;
    for (const f32 step : kSteps) {
        if (step >= minor && step * pixelsPerSecond >= 60.0f) {
            major = step;
            break;
        }
    }
    const f32 rulerY = p0.y + 16.0f;
    const auto tickCount = static_cast<int>(std::floor(duration / minor + 1e-3f));
    for (int i = 0; i <= tickCount; ++i) {
        const f32 t = static_cast<f32>(i) * minor;
        const f32 x = timeToX(t);
        const f32 ratio = t / major;
        const bool isMajor = std::abs(ratio - std::round(ratio)) < 1e-3f;
        draw->AddLine(ImVec2(x, rulerY - (isMajor ? 7.0f : 3.0f)), ImVec2(x, rulerY), isMajor ? kTickMajor : kTickMinor);
        if (isMajor) {
            char label[16];
            std::snprintf(label, sizeof(label), major < 1.0f ? "%.2f" : "%.0fs", static_cast<double>(t));
            draw->AddText(ImVec2(x + 3.0f, p0.y + 1.0f), kTickMajor, label);
        }
    }
    draw->AddLine(ImVec2(x0, rulerY), ImVec2(x1, rulerY), kTickMinor);

    // Keyframes (union over every joint and channel); orange for physics clips.
    const bool physicsClip = clip.kind() == anim::ClipKind::Physics;
    const f32 keyY = p0.y + 34.0f;
    for (const f32 t : keyTimes(clip)) {
        const f32 x = timeToX(t);
        draw->AddQuadFilled(ImVec2(x, keyY - 4.0f), ImVec2(x + 4.0f, keyY), ImVec2(x, keyY + 4.0f), ImVec2(x - 4.0f, keyY),
                            physicsClip ? IM_COL32(255, 150, 60, 255) : kKeyColor);
    }

    // Playhead.
    const f32 px = timeToX(std::clamp(player.time(), 0.0f, duration));
    draw->AddLine(ImVec2(px, p0.y), ImVec2(px, p1.y), kPlayhead, 2.0f);
    draw->AddTriangleFilled(ImVec2(px - 5.0f, p0.y), ImVec2(px + 5.0f, p0.y), ImVec2(px, p0.y + 7.0f), kPlayhead);

    if (ImGui::IsItemHovered() && !m_scrubbing) {
        ImGui::SetTooltip("Click or drag to scrub");
    }

    const int frameNumber = static_cast<int>(std::round(player.time() * kFrameRate));
    const int frameCount = static_cast<int>(std::round(duration * kFrameRate));
    ImGui::Text("%.2f / %.2f s", static_cast<double>(player.time()), static_cast<double>(clip.duration()));
    ImGui::SameLine();
    ImGui::TextDisabled("frame %d / %d @ %.0f fps  |  %zu keys", frameNumber, frameCount,
                        static_cast<double>(kFrameRate), keyTimes(clip).size());
    if (player.isFading()) {
        ImGui::SameLine();
        ImGui::TextDisabled("|  crossfade %.0f%%", static_cast<double>(player.fadeWeight() * 100.0f));
    }
    if (physicsClip) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.3f, 1.0f), "|  PHYSICS clip: muscle targets (simulated in Play)");
    }
}

const std::vector<f32>& AnimationEditor::keyTimes(const anim::AnimationClip& clip)
{
    auto [it, inserted] = m_keyTimes.try_emplace(&clip);
    if (inserted) {
        std::vector<f32>& times = it->second;
        for (const anim::JointTracks& jt : clip.tracks()) {
            times.insert(times.end(), jt.translation.times.begin(), jt.translation.times.end());
            times.insert(times.end(), jt.rotation.times.begin(), jt.rotation.times.end());
            times.insert(times.end(), jt.scale.times.begin(), jt.scale.times.end());
        }
        std::ranges::sort(times);
        // Merge keys closer than half a frame: they would draw on top of each other anyway.
        const auto last = std::unique(times.begin(), times.end(),
                                      [](f32 a, f32 b) { return std::abs(a - b) < 0.5f / kFrameRate; });
        times.erase(last, times.end());
    }
    return it->second;
}

// ---------------------------------------------------------------------------------------------- //
// Skeleton

void AnimationEditor::drawSkeleton(const scene::AnimatorComponent& animator)
{
    const anim::Skeleton& skeleton = animator.model->skeleton;
    const usize n = skeleton.jointCount();

    m_children.assign(n, {});
    for (usize j = 0; j < n; ++j) {
        if (const i32 p = skeleton.parents()[j]; p != anim::kNoParent) {
            m_children[static_cast<usize>(p)].push_back(j);
        }
    }

    // Selected joint: its current local transform.
    if (m_joint >= 0 && static_cast<usize>(m_joint) < n && animator.localPose.jointCount() == n) {
        const auto j = static_cast<usize>(m_joint);
        const anim::JointTransform t = animator.localPose.get(j);
        const glm::vec3 euler = glm::degrees(glm::eulerAngles(t.rotation));
        ImGui::Text("%s", skeleton.names()[j].c_str());
        ImGui::TextDisabled("T %.3f %.3f %.3f   R %.1f %.1f %.1f   S %.2f %.2f %.2f", static_cast<double>(t.translation.x),
                            static_cast<double>(t.translation.y), static_cast<double>(t.translation.z),
                            static_cast<double>(euler.x), static_cast<double>(euler.y), static_cast<double>(euler.z),
                            static_cast<double>(t.scale.x), static_cast<double>(t.scale.y), static_cast<double>(t.scale.z));
    } else {
        ImGui::TextDisabled("Click a joint to inspect it (it is highlighted in the preview with Skeleton on).");
    }

    ImGui::BeginChild("##joints", ImVec2(0, 0), ImGuiChildFlags_Borders);
    for (usize j = 0; j < n; ++j) {
        if (skeleton.parents()[j] == anim::kNoParent) {
            drawJointNode(animator, j);
        }
    }
    ImGui::EndChild();
}

void AnimationEditor::drawJointNode(const scene::AnimatorComponent& animator, usize joint)
{
    const anim::SkinnedModel& model = *animator.model;
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_DefaultOpen |
                               ImGuiTreeNodeFlags_SpanAvailWidth;
    if (m_children[joint].empty()) {
        flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
    }
    if (static_cast<i32>(joint) == m_joint) {
        flags |= ImGuiTreeNodeFlags_Selected;
    }
    if (!model.jointIsBone(joint)) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    }
    const bool open = ImGui::TreeNodeEx(reinterpret_cast<void*>(joint), flags, "%s", model.skeleton.names()[joint].c_str());
    if (!model.jointIsBone(joint)) {
        ImGui::PopStyleColor();
    }
    if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
        m_joint = m_joint == static_cast<i32>(joint) ? -1 : static_cast<i32>(joint);
        m_showSkeleton = true;
    }
    if (open && !m_children[joint].empty()) {
        for (const usize child : m_children[joint]) {
            drawJointNode(animator, child);
        }
        ImGui::TreePop();
    }
}

} // namespace kinetic::editor
