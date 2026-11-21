// Editor: dockable tool UI. The 3D scene is rendered offscreen and shown in the "Viewport" panel.
//
// Viewport controls:
//   click an object                  — select it (click empty space to deselect)
//   hold left mouse button + move   — look around
//   W A S D (while holding LMB)      — move, Q / E — down / up, Shift — faster
//   mouse wheel                      — move forward / back
//   click an axis in the corner gizmo — view the scene from that axis
//
// Keys (outside text fields): Delete — delete the selection, F — frame it, Esc — deselect.
//
// Actors: everything in the scene is an actor (name + transform). "Add Actor" creates an empty
// one; "+ Add Component" in the Inspector adds a Mesh Renderer (which asset it shows) and, for
// assets with a skeleton (ANIM), an Animator (clip, loop, speed). The Inspector only holds these
// settings; "Edit Animation..." (or double-clicking an animated actor) opens the Animation Editor
// with its own preview, the clip list, the timeline and the skeleton.
//
// Assets: the drawer (bottom) shows assets/meshes/ and assets/models/. Drag a tile onto the Viewport
// to place a new actor on the floor, or onto an actor's Mesh field to switch its asset. Dropping
// .fbx / .obj / .glb files on the window copies them into the asset folders.

#include "AnimationEditor.h"
#include "AssetDrawer.h"
#include "CharacterRig.h"
#include "CharacterSetup.h"
#include "ComponentGizmos.h"
#include "PlayMode.h"
#include "SwordStrikeTest.h"

#include <kinetic/animation/Combat.h>
#include <kinetic/physanim/PhysicalBody.h>
#include <kinetic/animation/PoseOps.h>
#include <kinetic/app/Application.h>
#include <kinetic/core/Primitives.h>
#include <kinetic/core/Log.h>
#include <kinetic/renderer/AnimationSystem.h>
#include <kinetic/renderer/Picking.h>
#include <kinetic/renderer/VfxRender.h>
#include <kinetic/scene/AnimatorComponent.h>
#include <kinetic/scene/Scene.h>

#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_main.h>
#include <imgui.h>
#include <imgui_internal.h> // DockBuilder API

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

using namespace kinetic;

namespace {

// Fly camera that edits the camera entity's TransformComponent directly.
struct FlyCameraController {
    f32 moveSpeed = 3.0f;         // m/s
    f32 lookSensitivity = 0.003f; // radians per pixel

    // `looking`: the viewport is being dragged with LMB. `hovered`: mouse is over the viewport.
    void update(scene::TransformComponent& transform, f32 dt, bool looking, bool hovered) const
    {
        const ImGuiIO& io = ImGui::GetIO();
        if (looking) {
            // Yaw around world up, pitch around the camera's own X axis.
            const f32 yaw = -io.MouseDelta.x * lookSensitivity;
            const f32 pitch = -io.MouseDelta.y * lookSensitivity;
            transform.rotation = glm::normalize(glm::angleAxis(yaw, glm::vec3(0, 1, 0)) * transform.rotation *
                                                glm::angleAxis(pitch, glm::vec3(1, 0, 0)));

            glm::vec3 move(0.0f);
            if (ImGui::IsKeyDown(ImGuiKey_W)) move.z -= 1.0f;
            if (ImGui::IsKeyDown(ImGuiKey_S)) move.z += 1.0f;
            if (ImGui::IsKeyDown(ImGuiKey_A)) move.x -= 1.0f;
            if (ImGui::IsKeyDown(ImGuiKey_D)) move.x += 1.0f;
            if (ImGui::IsKeyDown(ImGuiKey_E)) move.y += 1.0f;
            if (ImGui::IsKeyDown(ImGuiKey_Q)) move.y -= 1.0f;
            if (glm::dot(move, move) > 0.0f) {
                const f32 speed = moveSpeed * (io.KeyShift ? 4.0f : 1.0f);
                transform.position += transform.rotation * glm::normalize(move) * speed * dt;
            }
        }
        if (hovered && io.MouseWheel != 0.0f) {
            transform.position += transform.rotation * glm::vec3(0, 0, -io.MouseWheel * 0.5f);
        }
    }
};

// Tints given to new objects, in turn.
constexpr std::array<glm::vec3, 6> kPalette = {{
    {0.95f, 0.55f, 0.25f},
    {0.30f, 0.70f, 1.00f},
    {0.45f, 0.85f, 0.45f},
    {0.90f, 0.40f, 0.60f},
    {0.95f, 0.85f, 0.35f},
    {0.80f, 0.80f, 0.82f},
}};

// Root of the folders the Assets drawer shows: the project's assets/ (dev builds), else next to the exe.
std::filesystem::path assetsRoot()
{
#ifdef KINETIC_PROJECT_ASSETS_DIR
    const std::filesystem::path project = std::filesystem::path(KINETIC_PROJECT_ASSETS_DIR);
    if (std::filesystem::is_directory(project / "meshes")) {
        return project;
    }
#endif
    return std::filesystem::path(SDL_GetBasePath()) / "assets";
}

constexpr glm::vec3 kSelectionColor{1.00f, 0.62f, 0.20f};

ImU32 toImColor(const glm::vec3& c, f32 alpha = 1.0f)
{
    return ImGui::ColorConvertFloat4ToU32(ImVec4(c.r, c.g, c.b, alpha));
}

} // namespace

class EditorApp final : public Application, public editor::ActorEditorHost {
public:
    using Application::Application;

protected:
    bool onInit() override
    {
        m_camera = m_scene.createCamera("Camera", {0.0f, 1.6f, 4.0f}, {0.0f, 0.8f, 0.0f});
        m_drawer.init(renderer(), assetsRoot());
        m_vfxLibrary.loadDirectory(assetsRoot() / "vfx");
        m_play.setVfxLibrary(&m_vfxLibrary);
        addPresets();
        // Start with a floor to walk on.
        for (usize i = 0; i < m_drawer.assets().size(); ++i) {
            if (m_drawer.assets()[i].preset == "floor") {
                spawnAsset(i, glm::vec3(0.0f));
            }
        }
        m_selected = entt::null;
        return true;
    }

    void onEvent(const SDL_Event& event) override
    {
        if (event.type == SDL_EVENT_DROP_FILE && event.drop.data) {
            m_droppedFiles.emplace_back(event.drop.data);
        }
    }

    void onUpdate(f32 /*dt*/) override
    {
        for (const std::string& path : std::exchange(m_droppedFiles, {})) {
            m_drawer.importFile(path);
        }
    }

    void onImGui() override
    {
        const ImGuiID dockspace = ImGui::GetID("EditorLayout_v5"); // bump the name to reset saved layouts
        if (!m_layoutInitialized) {
            buildDefaultLayout(dockspace);
            m_layoutInitialized = true;
        }
        ImGui::DockSpaceOverViewport(dockspace, nullptr);

        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Quit")) {
                    requestQuit();
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Scene")) {
                if (ImGui::MenuItem("Add Actor")) {
                    addActor();
                }
                if (ImGui::MenuItem("Delete Selected", "Del", false, canDeleteSelected())) {
                    deleteSelected();
                }
                if (ImGui::MenuItem("Frame Selected", "F", false, m_scene.registry().valid(m_selected))) {
                    frameSelected();
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Give Selected a Sword", nullptr, false, isAnimated(m_selected))) {
                    giveSword(m_selected);
                }
                if (ImGui::MenuItem("Test Scene: Uneven Ground")) {
                    buildIkTestScene();
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Replaces the scene: stairs, ramps, rocks, uneven tiles, the warrior with a sword, the tentacle");
                }
                if (ImGui::MenuItem("Test Scene: Warrior vs Materials")) {
                    buildWarriorMaterialScene();
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Replaces the scene: the warrior with a sword and a row of stone, metal, wood, dirt and "
                                      "flesh pillars. Play (F5): A / D along the row, W to face a pillar, LMB attack - the sword "
                                      "arm runs on muscles and the blade bounces, stops or cuts in; the HUD lists each hit.");
                }
                if (ImGui::MenuItem("Test Scene: Materials (physics strikes)")) {
                    buildMaterialTestScene();
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Replaces the scene: six physics swordsmen in front of stone, metal, wood, dirt, flesh "
                                      "and a slanted stone slab. Play (F5), LMB: all strike at once; the HUD lists each hit.");
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Physics")) {
                if (ImGui::MenuItem(m_swordTest.active() ? "Sword Strike Test (running)" : "Sword Strike Test")) {
                    startSwordTest();
                }
                if (ImGui::MenuItem("Remove Sword Strike Test", nullptr, false, m_swordTest.active())) {
                    m_swordTest.destroy(m_scene, renderer());
                    m_selected = entt::null;
                }
                ImGui::Separator();
                ImGui::MenuItem("Physics Window", nullptr, &m_showPhysicsWindow);
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("View")) {
                if (ImGui::MenuItem("Actor Editor", nullptr, false, canOpenActorEditor(m_selected))) {
                    m_animationEditor.open(m_selected);
                }
                ImGui::Separator();
                ImGui::MenuItem("Skeleton of Selection", nullptr, &m_showSkeleton);
                ImGui::MenuItem("Selection Outline", nullptr, &m_showSelectionBox);
                ImGui::MenuItem("Component Gizmos", nullptr, &m_showGizmos);
                ImGui::MenuItem("All Colliders", nullptr, &m_showAllColliders);
                ImGui::MenuItem("Grid", nullptr, &m_showGrid);
                ImGui::MenuItem("Play: Physics Vectors", nullptr, &m_play.showDebug);
                ImGui::MenuItem("Play: Body Centres", nullptr, &m_play.showShapes);
                ImGui::EndMenu();
            }
            // Play / Stop, centred in the menu bar.
            const char* playLabel = m_play.active() ? "  Stop (Esc)  " : "  Play (F5)  ";
            ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX() + 20.0f,
                                          (ImGui::GetWindowWidth() - ImGui::CalcTextSize(playLabel).x) * 0.5f));
            ImGui::PushStyleColor(ImGuiCol_Button, m_play.active() ? ImVec4(0.75f, 0.25f, 0.2f, 1.0f) : ImVec4(0.2f, 0.55f, 0.3f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, m_play.active() ? ImVec4(0.9f, 0.3f, 0.25f, 1.0f) : ImVec4(0.25f, 0.7f, 0.38f, 1.0f));
            if (ImGui::Button(playLabel)) {
                togglePlay();
            }
            ImGui::PopStyleColor(2);
            ImGui::EndMainMenuBar();
        }

        drawViewport();
        drawHierarchy();
        drawInspector();
        drawStats();
        m_drawer.draw();
        m_animationEditor.draw(m_scene, renderer(), *this);
        if (m_showPhysicsWindow) {
            m_swordTest.drawWindow(&m_showPhysicsWindow);
        }
        if (const auto index = m_drawer.takeSpawnRequest()) {
            spawnAsset(*index, std::nullopt);
        }
        handleShortcuts();

        // After the UI, so scrubbing / clip changes / physics controls show up this frame.
        m_swordTest.update(ImGui::GetIO().DeltaTime, m_scene);
        if (m_play.active()) {
            m_playInput.cameraYaw = m_play.cameraYaw();
            m_play.update(ImGui::GetIO().DeltaTime, m_playInput, m_scene);
            m_play.updateCamera(m_scene, m_playOrbit, m_playWheel);
        }
        m_playInput = {};
        m_playOrbit = glm::vec2(0.0f);
        m_playWheel = 0.0f;
        // In Play the poses were evaluated (and the physics applied) by PlayMode::update already.
        renderer::updateAnimations(m_scene, renderer(), ImGui::GetIO().DeltaTime, !m_play.active());
        renderer::syncVfx(m_scene, renderer(), m_play.vfx(), m_vfxRender);
        autosaveActors(ImGui::GetIO().DeltaTime);
        std::vector<renderer::DebugVertex> overlay;
        if (m_showSkeleton && isAnimated(m_selected)) {
            renderer::appendSkeletonLines(m_scene.registry().get<scene::AnimatorComponent>(m_selected),
                                          m_scene.registry().get<scene::TransformComponent>(m_selected).matrix(), overlay);
        }
        // Components of the selected actor (collider box, controller capsule, foot IK targets).
        if (m_showGizmos && !m_play.active() && m_scene.registry().valid(m_selected) && m_selected != m_camera) {
            editor::GizmoOptions gizmos;
            gizmos.highlight = m_animationEditor.isOpen() && m_animationEditor.actor() == m_selected;
            editor::appendComponentGizmos(m_scene, m_selected, m_scene.registry().get<scene::TransformComponent>(m_selected).matrix(),
                                          overlay, gizmos);
        }
        if (m_showAllColliders) {
            editor::appendAllColliders(m_scene, overlay);
        }
        glm::vec3 boxMin;
        glm::vec3 boxMax;
        if (m_showSelectionBox && renderer::worldBounds(m_scene, renderer(), m_selected, boxMin, boxMax)) {
            const glm::vec3 pad = glm::max((boxMax - boxMin) * 0.02f, glm::vec3(0.005f));
            renderer::appendBoxLines(overlay, boxMin - pad, boxMax + pad, kSelectionColor);
        }
        m_swordTest.appendDebugLines(overlay);
        m_play.appendDebugLines(overlay);
        renderer().setOverlayLines(std::move(overlay));
        m_animationEditor.updateOverlay(m_scene, renderer());
    }

    void onRenderOffscreen(gfx::Frame& frame, renderer::Renderer& renderer) override
    {
        m_animationEditor.renderPreview(frame, renderer);
    }

    void onShutdown() override { m_animationEditor.shutdown(renderer()); }

    void onRender(RenderContext& ctx) override
    {
        renderer::SceneDrawOptions options;
        options.grid = m_showGrid;
        ctx.renderer.drawScene(ctx.frame.cmd, ctx.pass, m_scene, options);
    }

private:
    // Hierarchy + Stats on the left, Inspector on the right, Viewport in the centre, Assets below it.
    // The Animation window is not docked by default: it opens on demand (Inspector / View menu).
    // Only applied when there is no saved layout in imgui.ini yet.
    static void buildDefaultLayout(ImGuiID dockspace)
    {
        const ImGuiDockNode* existing = ImGui::DockBuilderGetNode(dockspace);
        if (existing && existing->IsSplitNode()) {
            return;
        }
        ImGui::DockBuilderRemoveNode(dockspace);
        ImGui::DockBuilderAddNode(dockspace, ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockspace, ImGui::GetMainViewport()->WorkSize);

        ImGuiID center = dockspace;
        ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.2f, nullptr, &center);
        const ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.25f, nullptr, &center);
        const ImGuiID leftBottom = ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.4f, nullptr, &left);
        const ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.26f, nullptr, &center);

        ImGui::DockBuilderDockWindow("Viewport", center);
        ImGui::DockBuilderDockWindow("Assets", bottom);
        ImGui::DockBuilderDockWindow("Hierarchy", left);
        ImGui::DockBuilderDockWindow("Stats", leftBottom);
        ImGui::DockBuilderDockWindow("Inspector", right);
        ImGui::DockBuilderFinish(dockspace);
    }

    // --- Viewport --------------------------------------------------------------------------------

    void drawViewport()
    {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::Begin("Viewport", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::PopStyleVar();

        const ImVec2 size = ImGui::GetContentRegionAvail();
        if (size.x < 1.0f || size.y < 1.0f) {
            ImGui::End();
            return;
        }

        // Render at the panel's size in physical pixels (Retina: 2x the ImGui size).
        const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
        renderer().resizeSceneTarget(static_cast<u32>(size.x * scale.x), static_cast<u32>(size.y * scale.y));

        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const ImVec2 p1(p0.x + size.x, p0.y + size.y);

        // One invisible button over the whole image: it owns mouse input, so dragging inside the
        // viewport never moves or undocks the window.
        ImGui::InvisibleButton("##viewport", size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        const bool hovered = ImGui::IsItemHovered();
        const bool active = ImGui::IsItemActive();

        // A click (press + release without dragging) selects; a drag looks around.
        constexpr f32 kClickSlop = 4.0f; // pixels
        const bool clicked = ImGui::IsItemDeactivated() &&
                             ImGui::GetIO().MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] < kClickSlop * kClickSlop;

        // Asset dragged from the drawer: place a new actor on the floor under the cursor.
        if (ImGui::BeginDragDropTarget()) {
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(editor::AssetDrawer::kAssetPayload)) {
                usize index = 0;
                std::memcpy(&index, payload->Data, sizeof(index));
                spawnAsset(index, groundPointUnderMouse(p0, size));
            }
            ImGui::EndDragDropTarget();
        }

        ImDrawList* drawList = ImGui::GetWindowDrawList();
        if (SDL_GPUTexture* texture = renderer().sceneColorTexture()) {
            drawList->AddImage(static_cast<ImTextureID>(reinterpret_cast<intptr_t>(texture)), p0, p1);
        }

        if (const entt::entity cam = m_scene.activeCamera(); cam != entt::null) {
            const auto& registry = m_scene.registry();
            const glm::mat4 viewProjection = registry.get<scene::CameraComponent>(cam).projection(size.x / size.y) *
                                             scene::CameraComponent::view(registry.get<scene::TransformComponent>(cam));
            m_swordTest.drawViewportLabels(drawList, viewProjection, p0, size);
        }

        if (m_play.active()) {
            // Play: LMB attacks, RMB drag orbits the camera, wheel zooms; the editor camera is off.
            const ImGuiIO& io = ImGui::GetIO();
            if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                m_playInput.attack = true;
            }
            if ((hovered || active) && ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
                m_playOrbit += glm::vec2(io.MouseDelta.x, io.MouseDelta.y);
            }
            if (hovered) {
                m_playWheel += io.MouseWheel;
            }
            drawList->AddRect(p0, p1, IM_COL32(80, 200, 110, 255), 0.0f, 0, 3.0f);
            m_play.drawHud(drawList, p0, size);
            ImGui::End();
            return;
        }

        const bool gizmoClicked = drawAxisGizmo(drawList, p0, p1, hovered);
        if (gizmoClicked) {
            m_lookBlocked = true; // this drag started on the gizmo, not a camera look
        }
        if (clicked && !m_lookBlocked) {
            selectUnderMouse(p0, size);
        }
        if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && !m_lookBlocked) {
            selectUnderMouse(p0, size);
            if (canOpenActorEditor(m_selected)) {
                m_animationEditor.open(m_selected);
            }
        }
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            m_lookBlocked = false;
        }

        const entt::entity camera = m_scene.activeCamera();
        if (camera != entt::null) {
            auto& transform = m_scene.registry().get<scene::TransformComponent>(camera);
            m_cameraController.update(transform, ImGui::GetIO().DeltaTime, active && !m_lookBlocked, hovered);
        }

        ImGui::End();
    }

    // Orientation gizmo in the top-right corner: X (red), Y (green), Z (blue) as seen by the camera.
    // Clicking an axis end moves the camera onto that axis, looking at the pivot (selected object or
    // origin). Returns true if an axis was clicked this frame.
    bool drawAxisGizmo(ImDrawList* drawList, const ImVec2& p0, const ImVec2& p1, bool viewportHovered)
    {
        const entt::entity camera = m_scene.activeCamera();
        if (camera == entt::null) {
            return false;
        }
        auto& camTransform = m_scene.registry().get<scene::TransformComponent>(camera);
        const glm::mat3 viewRotation = glm::mat3_cast(glm::conjugate(camTransform.rotation));

        constexpr f32 kRadius = 38.0f;
        constexpr f32 kHandle = 9.0f;
        const ImVec2 center(p1.x - kRadius - 18.0f, p0.y + kRadius + 18.0f);
        if (center.x - kRadius < p0.x || center.y + kRadius > p1.y) {
            return false; // viewport too small
        }

        struct Axis {
            glm::vec3 dir;
            glm::vec3 color;
            const char* label;
            glm::vec3 view; // direction in view space
        };
        std::array<Axis, 6> axes = {{
            {{1, 0, 0}, {0.90f, 0.30f, 0.30f}, "X", {}},
            {{0, 1, 0}, {0.40f, 0.80f, 0.35f}, "Y", {}},
            {{0, 0, 1}, {0.30f, 0.50f, 0.95f}, "Z", {}},
            {{-1, 0, 0}, {0.90f, 0.30f, 0.30f}, nullptr, {}},
            {{0, -1, 0}, {0.40f, 0.80f, 0.35f}, nullptr, {}},
            {{0, 0, -1}, {0.30f, 0.50f, 0.95f}, nullptr, {}},
        }};
        for (Axis& axis : axes) {
            axis.view = viewRotation * axis.dir;
        }
        // Painter's order: farthest (most negative view z) first.
        std::ranges::sort(axes, {}, [](const Axis& a) { return a.view.z; });

        drawList->AddCircleFilled(center, kRadius + kHandle + 2.0f, IM_COL32(20, 22, 26, 110));

        const ImVec2 mouse = ImGui::GetIO().MousePos;
        const Axis* clicked = nullptr;
        for (const Axis& axis : axes) {
            const ImVec2 tip(center.x + axis.view.x * kRadius, center.y - axis.view.y * kRadius);
            const f32 dx = mouse.x - tip.x;
            const f32 dy = mouse.y - tip.y;
            const bool hot = viewportHovered && dx * dx + dy * dy <= kHandle * kHandle;
            const f32 alpha = axis.view.z > 0.0f ? 0.55f : 1.0f; // pointing away from the viewer: dimmer

            if (axis.label) {
                drawList->AddLine(center, tip, toImColor(axis.color, alpha), 2.0f);
                drawList->AddCircleFilled(tip, kHandle, toImColor(hot ? glm::vec3(1.0f) : axis.color, alpha));
                const ImVec2 textSize = ImGui::CalcTextSize(axis.label);
                drawList->AddText(ImVec2(tip.x - textSize.x * 0.5f, tip.y - textSize.y * 0.5f), IM_COL32(15, 15, 18, 255),
                                  axis.label);
            } else {
                drawList->AddCircleFilled(tip, kHandle * 0.75f, toImColor(axis.color, alpha * 0.45f));
                drawList->AddCircle(tip, kHandle * 0.75f, toImColor(hot ? glm::vec3(1.0f) : axis.color, alpha), 0, 1.5f);
            }
            if (hot && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                clicked = &axis;
            }
        }

        if (clicked) {
            glm::vec3 pivot(0.0f);
            auto& registry = m_scene.registry();
            if (registry.valid(m_selected) && m_selected != camera) {
                pivot = registry.get<scene::TransformComponent>(m_selected).position;
            }
            const f32 distance = std::max(glm::length(camTransform.position - pivot), 1.0f);
            camTransform.position = pivot + clicked->dir * distance;
            camTransform.rotation = scene::lookAtRotation(camTransform.position, pivot);
            return true;
        }
        return false;
    }

    // --- Placing objects -------------------------------------------------------------------------

    // Ray from the camera through the mouse cursor (viewport at p0 with `size`).
    std::optional<Ray> mouseRay(const ImVec2& p0, const ImVec2& size) const
    {
        const entt::entity cameraEntity = m_scene.activeCamera();
        if (cameraEntity == entt::null || size.x <= 0.0f || size.y <= 0.0f) {
            return std::nullopt;
        }
        const auto& registry = m_scene.registry();
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        const glm::vec2 ndc((mouse.x - p0.x) / size.x * 2.0f - 1.0f, 1.0f - (mouse.y - p0.y) / size.y * 2.0f);
        return renderer::cameraRay(registry.get<scene::CameraComponent>(cameraEntity),
                                   registry.get<scene::TransformComponent>(cameraEntity), size.x / size.y, ndc);
    }

    // Where the ray from the camera through the mouse cursor hits the floor (y = 0), if it does.
    std::optional<glm::vec3> groundPointUnderMouse(const ImVec2& p0, const ImVec2& size) const
    {
        const auto ray = mouseRay(p0, size);
        if (!ray || ray->direction.y > -1e-4f) {
            return std::nullopt; // no camera, or looking up / parallel to the floor
        }
        const f32 t = -ray->origin.y / ray->direction.y;
        if (t < 0.0f || t > 200.0f) {
            return std::nullopt;
        }
        return ray->at(t);
    }

    // Selects the closest mesh under the cursor; clicking empty space clears the selection.
    void selectUnderMouse(const ImVec2& p0, const ImVec2& size)
    {
        const auto ray = mouseRay(p0, size);
        if (!ray) {
            return;
        }
        const auto hit = renderer::pickEntity(m_scene, renderer(), *ray);
        m_selected = hit ? hit->entity : entt::null;
    }

    // Adds an actor showing the drawer asset `index` (animated if the asset has animation). With a
    // ground point it stands on the floor there; otherwise it is placed in front of the camera.
    void spawnAsset(usize index, std::optional<glm::vec3> groundPoint)
    {
        const editor::MeshAsset* asset = m_drawer.asset(index);
        const renderer::GpuMesh* mesh = asset ? renderer().mesh(asset->id) : nullptr;
        if (!mesh) {
            return;
        }
        // Copy the bounds now: enableAnimation adds a mesh, which may reallocate the mesh library.
        const glm::vec3 boundsMin = mesh->boundsMin;
        const glm::vec3 boundsMax = mesh->boundsMax;
        if (asset->isPreset()) {
            spawnPreset(*asset, groundPoint);
            return;
        }
        // Textured assets show their texture as authored; others get a palette colour.
        const glm::vec3 tint = asset->textured() ? glm::vec3(1.0f) : kPalette[m_objectCounter % kPalette.size()];
        const std::string name = asset->id == scene::kTriangleMesh ? "Actor " + std::to_string(m_objectCounter) : asset->name;
        const entt::entity entity = m_scene.createMesh(name, asset->id, glm::vec4(tint, 1.0f));
        applyMaterial(entity, *asset);
        ++m_objectCounter;
        if (asset->animated()) {
            renderer::enableAnimation(m_scene, renderer(), entity, asset->model);
        }
        placeEntity(entity, boundsMin, boundsMax, groundPoint);
    }

    void applyMaterial(entt::entity entity, const editor::MeshAsset& asset)
    {
        auto& mesh = m_scene.registry().get<scene::MeshRendererComponent>(entity);
        mesh.baseColorTexture = asset.baseColorTexture;
        mesh.normalTexture = asset.normalTexture;
    }

    // --- Test scene: uneven ground, a hero with a sword, the tentacle ------------------------------

    // A long sword (metres, grip at the origin, blade along +Y, flat side facing +Z).
    static MeshData makeSwordMesh()
    {
        MeshData sword;
        const auto add = [&](const glm::vec3& half, const glm::vec3& center) {
            const MeshData part = makeBoxMesh(half, center);
            const auto base = static_cast<u32>(sword.positions.size());
            sword.positions.insert(sword.positions.end(), part.positions.begin(), part.positions.end());
            sword.normals.insert(sword.normals.end(), part.normals.begin(), part.normals.end());
            for (const u32 i : part.indices) sword.indices.push_back(base + i);
        };
        add({0.016f, 0.10f, 0.016f}, {0.0f, 0.0f, 0.0f});     // grip
        add({0.025f, 0.022f, 0.025f}, {0.0f, -0.12f, 0.0f});  // pommel
        add({0.11f, 0.015f, 0.025f}, {0.0f, 0.115f, 0.0f});   // cross guard
        add({0.024f, 0.42f, 0.005f}, {0.0f, 0.55f, 0.0f});    // blade
        add({0.012f, 0.03f, 0.004f}, {0.0f, 0.995f, 0.0f});   // tip
        return sword;
    }

    // Attaches a sword to the selected animated actor's weapon hand (anim::findWeaponHand).
    entt::entity giveSword(entt::entity actor)
    {
        auto& registry = m_scene.registry();
        const auto* animator = registry.try_get<scene::AnimatorComponent>(actor);
        if (!animator || !animator->model) {
            return entt::null;
        }
        const i32 hand = anim::findWeaponHand(animator->model->skeleton);
        if (hand < 0) {
            return entt::null;
        }
        if (m_swordMesh == renderer::Renderer::kInvalidMesh) {
            m_swordMesh = renderer().addMesh("Sword", makeSwordMesh());
        }
        const entt::entity sword = m_scene.createMesh("Sword", m_swordMesh, glm::vec4(0.78f, 0.80f, 0.84f, 1.0f));
        auto& attachment = registry.emplace<scene::AttachmentComponent>(sword);
        attachment.parent = actor;
        attachment.joint = hand;
        attachment.offset = anim::weaponSocket(animator->model->skeleton, hand);
        // Collision box matching makeSwordMesh (guard to tip); in Play the bones holding it can run
        // on muscles (a clip's physics bones).
        registry.emplace<scene::WeaponComponent>(sword);
        editor::loadActorWeapons(m_scene, actor); // a socket saved for this model
        return sword;
    }

    entt::entity addBlock(const char* name, const glm::vec3& position, const glm::vec3& half, const glm::quat& rotation,
                          const glm::vec3& color)
    {
        const scene::MeshId mesh = renderer().addMesh(name, makeBoxMesh(half));
        const entt::entity e = m_scene.createMesh(name, mesh, glm::vec4(color, 1.0f));
        auto& t = m_scene.registry().get<scene::TransformComponent>(e);
        t.position = position;
        t.rotation = rotation;
        m_scene.registry().emplace<scene::ColliderComponent>(e, half, glm::vec3(0.0f));
        return e;
    }

    // Replaces the scene with ground to test foot IK and the controller on (stairs, ramps, a
    // slope, rocks, uneven tiles), the warrior as a playable hero with a sword, and the tentacle.
    // The warrior asset as a playable hero with a sword (its arm runs on muscles when attacking
    // in Play). Returns null when the asset is not in the drawer.
    entt::entity spawnWarriorHero(const glm::vec3& position)
    {
        for (usize i = 0; i < m_drawer.assets().size(); ++i) {
            const editor::MeshAsset& asset = *m_drawer.asset(i);
            if (asset.name != "Warrior") continue;
            const entt::entity hero = m_scene.createMesh("Warrior", asset.id, glm::vec4(1.0f));
            applyMaterial(hero, asset);
            if (asset.animated()) {
                renderer::enableAnimation(m_scene, renderer(), hero, asset.model);
                m_heroMessage = editor::makePlayableHero(m_scene, hero, 1.8f).message;
                m_heroMessageFor = hero;
                giveSword(hero);
            }
            m_scene.registry().get<scene::TransformComponent>(hero).position = position;
            m_selected = hero;
            return hero;
        }
        return entt::null;
    }

    // The warrior (muscle-driven sword arm during its Mixamo attacks) and, for every attack of its
    // combo, a row of tall pillars (stone, metal, wood, dirt, flesh) placed automatically where
    // that attack's blade strikes (physanim::findStrikePoint). Stand on a pad (facing +Z, the way
    // the hero starts) and press the attack's number key; the HUD lists what the blade did.
    void buildWarriorMaterialScene()
    {
        clearScene();
        const glm::quat none(1.0f, 0.0f, 0.0f, 0.0f);
        constexpr f32 kColumn = 3.0f; // between materials
        constexpr f32 kRow = 6.0f;    // between attacks (rows go back along -Z)
        const entt::entity hero = spawnWarriorHero({0.0f, 0.0f, 0.0f});
        if (hero == entt::null) {
            return;
        }
        auto& registry = m_scene.registry();
        // The Mixamo attacks swing the blade flat in places: turn the hand so it cuts edge first.
        if (const auto* a = registry.try_get<scene::AnimatorComponent>(hero); a && a->blendTree) {
            const std::shared_ptr<anim::BlendTree> tree = a->blendTree;
            for (const i32 state : anim::attackChain(*tree)) {
                (void)editor::alignWeaponEdge(m_scene, hero, anim::stateClip(*tree, state));
            }
        }
        const auto* animator = registry.try_get<scene::AnimatorComponent>(hero);
        std::vector<physanim::StrikePoint> strikes;
        entt::entity sword = entt::null;
        for (const entt::entity e : registry.view<scene::AttachmentComponent, scene::WeaponComponent>()) {
            if (registry.get<scene::AttachmentComponent>(e).parent == hero) sword = e;
        }
        if (animator && animator->blendTree && animator->model && sword != entt::null) {
            const auto& a = registry.get<scene::AttachmentComponent>(sword);
            const physanim::BodyLayout layout = physanim::layoutBody(*animator->model);
            for (const i32 state : anim::attackChain(*animator->blendTree)) {
                const i32 clip = anim::stateClip(*animator->blendTree, state);
                if (clip < 0) continue;
                // Attacks without physics bones yet: the weapon arm (a saved setup is kept).
                anim::BlendNode& node = animator->blendTree->nodes[static_cast<usize>(animator->blendTree->stateMachine.states[static_cast<usize>(state)].motion)];
                if (node.physicsJoints.empty()) node.physicsJoints = physanim::weaponArmJoints(layout, a.joint);
                strikes.push_back(physanim::findStrikePoint(animator->model->skeleton, animator->model->clips[static_cast<usize>(clip)],
                                                            a.joint, a.offset, registry.get<scene::WeaponComponent>(sword)));
            }
        }
        const f32 rows = static_cast<f32>(std::max<usize>(strikes.size(), 1));
        addBlock("Floor", {4.5f, -0.1f, -(rows - 1.0f) * kRow * 0.5f}, {12.0f, 0.1f, rows * kRow * 0.5f + 5.0f}, none,
                 {0.30f, 0.32f, 0.36f});
        struct Pillar {
            const char* name;
            scene::SurfaceMaterial material;
            glm::vec3 color;
        };
        const std::array<Pillar, 5> pillars = {{
            {"Stone", scene::SurfaceMaterial::Stone, {0.55f, 0.53f, 0.49f}},
            {"Metal", scene::SurfaceMaterial::Metal, {0.62f, 0.68f, 0.76f}},
            {"Wood", scene::SurfaceMaterial::Wood, {0.55f, 0.36f, 0.20f}},
            {"Dirt", scene::SurfaceMaterial::Dirt, {0.36f, 0.27f, 0.18f}},
            {"Flesh", scene::SurfaceMaterial::Flesh, {0.85f, 0.60f, 0.55f}},
        }};
        const glm::mat4 heroMatrix = registry.get<scene::TransformComponent>(hero).matrix(); // at the origin, facing +Z
        for (usize k = 0; k < strikes.size(); ++k) {
            // In the blade's path at the strike, relative to where the hero stands.
            const physanim::TargetBox box = physanim::strikeTarget(strikes[k], heroMatrix);
            for (usize i = 0; i < pillars.size(); ++i) {
                const glm::vec3 station(static_cast<f32>(i) * kColumn, 0.0f, -static_cast<f32>(k) * kRow);
                char name[64];
                std::snprintf(name, sizeof(name), "%s (attack %zu)", pillars[i].name, k + 1);
                const entt::entity e = addBlock(name, station + box.center, box.halfExtents, none, pillars[i].color);
                registry.get<scene::ColliderComponent>(e).material = pillars[i].material;
                // Where to stand: a flat pad (no collider), brighter for higher attack numbers.
                std::snprintf(name, sizeof(name), "Stand here: key %zu", k + 1);
                const f32 shade = 0.35f + 0.2f * static_cast<f32>(k);
                const entt::entity pad = addBlock(name, station + glm::vec3(0.0f, 0.005f, 0.0f), {0.35f, 0.005f, 0.35f}, none,
                                                  {shade, 0.75f, 0.35f});
                registry.remove<scene::ColliderComponent>(pad);
            }
        }
        m_selected = hero;
        if (m_camera != entt::null) {
            auto& cam = registry.get<scene::TransformComponent>(m_camera);
            cam.position = {3.5f, 2.6f, -4.0f};
            cam.rotation = scene::lookAtRotation(cam.position, {0.0f, 1.0f, 1.2f});
        }
    }

    // Stops Play and removes everything but the camera.
    void clearScene()
    {
        if (m_play.active()) {
            m_play.stop(m_scene);
        }
        std::vector<entt::entity> old;
        for (const entt::entity e : m_scene.registry().view<scene::NameComponent>()) {
            if (e != m_camera) old.push_back(e);
        }
        for (const entt::entity e : old) {
            renderer::destroyEntity(m_scene, renderer(), e);
        }
        m_eulerCache.clear();
        m_selected = entt::null;
    }

    // Material test: one physics character per target (stone, metal, wood, dirt, flesh and a
    // slanted stone slab). They all take the same input, so one click = six strikes side by side;
    // the HUD lists what each blade did (rebound / glance / stop / cut / stuck).
    void buildMaterialTestScene()
    {
        clearScene();
        const editor::MeshAsset* character = nullptr;
        for (const editor::MeshAsset& asset : m_drawer.assets()) {
            if (asset.preset == "character") character = &asset;
        }
        if (!character) {
            return;
        }
        const glm::quat none(1.0f, 0.0f, 0.0f, 0.0f);
        const entt::entity floor = addBlock("Floor", {0.0f, -0.1f, 0.0f}, {12.0f, 0.1f, 12.0f}, none, {0.30f, 0.32f, 0.36f});
        m_scene.registry().get<scene::ColliderComponent>(floor).material = scene::SurfaceMaterial::Stone;

        struct Target {
            const char* name;
            scene::SurfaceMaterial material;
            glm::vec3 color;
            glm::vec3 half;
            f32 slope; // degrees about X (0 = upright pillar)
        };
        const std::array<Target, 6> targets = {{
            {"Stone pillar", scene::SurfaceMaterial::Stone, {0.55f, 0.53f, 0.49f}, {0.3f, 0.45f, 0.3f}, 0.0f},
            {"Metal pillar", scene::SurfaceMaterial::Metal, {0.62f, 0.68f, 0.76f}, {0.3f, 0.45f, 0.3f}, 0.0f},
            {"Wood pillar", scene::SurfaceMaterial::Wood, {0.55f, 0.36f, 0.20f}, {0.3f, 0.45f, 0.3f}, 0.0f},
            {"Dirt mound", scene::SurfaceMaterial::Dirt, {0.36f, 0.27f, 0.18f}, {0.3f, 0.45f, 0.3f}, 0.0f},
            {"Flesh dummy", scene::SurfaceMaterial::Flesh, {0.85f, 0.60f, 0.55f}, {0.3f, 0.45f, 0.3f}, 0.0f},
            {"Stone slab 30", scene::SurfaceMaterial::Stone, {0.48f, 0.47f, 0.44f}, {0.3f, 0.06f, 0.6f}, -30.0f},
        }};
        constexpr f32 kSpacing = 1.8f;
        entt::entity middle = entt::null;
        for (usize i = 0; i < targets.size(); ++i) {
            const Target& t = targets[i];
            const f32 x = (static_cast<f32>(i) - 2.5f) * kSpacing;
            // The sword arm is on the character's right (-X): the target sits in front of it, where
            // the strike comes down (same placement as the Play strike test).
            glm::vec3 center(x - 0.21f, t.half.y, 1.0f);
            glm::quat rotation = none;
            if (t.slope != 0.0f) {
                rotation = glm::angleAxis(glm::radians(t.slope), glm::vec3(1.0f, 0.0f, 0.0f)); // top tilted towards the character: the blade (coming down and back) meets it at ~30 deg
                center = {x - 0.21f, 0.45f, 1.05f};
            }
            const entt::entity e = addBlock(t.name, center, t.half, rotation, t.color);
            m_scene.registry().get<scene::ColliderComponent>(e).material = t.material;

            const entt::entity c = m_scene.createMesh(std::string("Swordsman (") + scene::surfaceProperties(t.material).name +
                                                          (t.slope != 0.0f ? " slab)" : ")"),
                                                      character->id, glm::vec4(0.78f, 0.64f, 0.52f, 1.0f));
            renderer::enableAnimation(m_scene, renderer(), c, character->model);
            m_scene.registry().emplace<scene::CharacterComponent>(c);
            m_scene.registry().get<scene::TransformComponent>(c).position = {x, 0.0f, 0.0f};
            if (i == 2) middle = c;
        }
        m_selected = middle;
        if (m_camera != entt::null) {
            auto& cam = m_scene.registry().get<scene::TransformComponent>(m_camera);
            cam.position = {-2.0f, 2.4f, -5.5f};
            cam.rotation = scene::lookAtRotation(cam.position, {-0.5f, 0.6f, 0.8f});
        }
    }

    void buildIkTestScene()
    {
        clearScene();

        const glm::quat none(1.0f, 0.0f, 0.0f, 0.0f);
        const glm::vec3 ground(0.30f, 0.32f, 0.36f);
        const glm::vec3 stone(0.52f, 0.50f, 0.46f);
        addBlock("Floor", {0.0f, -0.1f, 0.0f}, {14.0f, 0.1f, 14.0f}, none, ground);

        // Stairs ahead (+Z): 4 steps of 12 cm, then a landing.
        for (int k = 0; k < 4; ++k) {
            const f32 h = 0.12f * static_cast<f32>(k + 1);
            addBlock("Step", {0.0f, h * 0.5f, 2.3f + 0.6f * static_cast<f32>(k)}, {1.0f, h * 0.5f, 0.3f}, none,
                     glm::mix(ground, stone, 0.3f + 0.15f * static_cast<f32>(k)));
        }
        addBlock("Landing", {0.0f, 0.24f, 5.6f}, {1.0f, 0.24f, 0.6f}, none, glm::mix(ground, stone, 0.9f));

        // Ramps: 12 degrees (right) and 22 degrees (left), rising towards +Z.
        for (const auto& [x, degrees] : {std::pair{3.4f, 12.0f}, std::pair{-3.4f, 22.0f}}) {
            const f32 a = glm::radians(degrees);
            const f32 halfLength = 2.4f;
            const glm::vec3 half(1.0f, 0.1f, halfLength);
            const glm::quat r = glm::angleAxis(-a, glm::vec3(1.0f, 0.0f, 0.0f));
            const f32 topCenter = halfLength * std::sin(a);
            addBlock(degrees < 15.0f ? "Ramp 12" : "Ramp 22",
                     {x, topCenter - half.y * std::cos(a), 1.2f + halfLength * std::cos(a)}, half, r, glm::mix(ground, stone, 0.5f));
        }

        // Rocks: low tilted blocks (behind the start, to the right).
        const std::array<glm::vec4, 9> rocks = {{{4.5f, -2.0f, 0.06f, 10.0f}, {5.3f, -2.6f, 0.10f, 35.0f},
                                                 {4.8f, -3.4f, 0.14f, 60.0f}, {5.9f, -1.6f, 0.08f, 80.0f},
                                                 {6.4f, -3.0f, 0.18f, 15.0f}, {3.9f, -2.9f, 0.05f, 45.0f},
                                                 {5.6f, -4.1f, 0.12f, 25.0f}, {6.8f, -2.2f, 0.07f, 70.0f},
                                                 {4.4f, -4.4f, 0.09f, 5.0f}}};
        for (const glm::vec4& rk : rocks) {
            const glm::quat r = glm::angleAxis(glm::radians(rk.w), glm::vec3(0, 1, 0)) *
                                glm::angleAxis(glm::radians(6.0f + rk.w * 0.1f), glm::vec3(1, 0, 0));
            addBlock("Rock", {rk.x, rk.z * 0.5f, rk.y}, {0.28f, rk.z * 0.5f + 0.02f, 0.22f}, r, stone);
        }

        // Uneven tiles: 6 x 6 tiles of 50 cm, 0 - 15 cm high (behind the start, to the left).
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                const f32 h = 0.02f + 0.13f * static_cast<f32>((i * 7 + j * 13 + i * j * 5) % 11) / 10.0f;
                addBlock("Tile", {-3.0f - 0.5f * static_cast<f32>(i), h * 0.5f, -1.8f - 0.5f * static_cast<f32>(j)},
                         {0.25f, h * 0.5f, 0.25f}, none, glm::mix(ground, stone, h * 5.0f));
            }
        }

        // The hero: the warrior, playable, with a sword in its hand.
        const auto findAsset = [&](const std::string& name) -> std::optional<usize> {
            for (usize i = 0; i < m_drawer.assets().size(); ++i) {
                if (m_drawer.assets()[i].name == name) return i;
            }
            return std::nullopt;
        };
        spawnWarriorHero(glm::vec3(0.0f));
        // The tentacle, to swing at (an animated mesh: no physics reaction yet).
        if (const auto tentacle = findAsset("TestTentacle_Skinned")) {
            const editor::MeshAsset& asset = *m_drawer.asset(*tentacle);
            const entt::entity e = m_scene.createMesh("Tentacle", asset.id, glm::vec4(0.75f, 0.45f, 0.55f, 1.0f));
            applyMaterial(e, asset);
            if (asset.animated()) {
                renderer::enableAnimation(m_scene, renderer(), e, asset.model);
            }
            const renderer::GpuMesh* mesh = renderer().mesh(asset.id);
            auto& t = m_scene.registry().get<scene::TransformComponent>(e);
            t.position = {1.8f, 0.0f, 1.2f};
            if (mesh) {
                const f32 height = std::max(mesh->boundsMax.y - mesh->boundsMin.y, 1e-3f);
                t.scale = glm::vec3(1.6f / height); // about 1.6 m tall
                t.position.y = -mesh->boundsMin.y * t.scale.y;
            }
        }

        if (m_camera != entt::null) {
            auto& cam = m_scene.registry().get<scene::TransformComponent>(m_camera);
            cam.position = {-2.5f, 2.6f, -5.0f};
            cam.rotation = scene::lookAtRotation(cam.position, {0.5f, 0.6f, 1.5f});
        }
    }

    // Built-in presets: the physics character, a floor and a stone (both static box colliders).
    void addPresets()
    {
        // Character: its static mesh shows the idle pose (the rest pose has the sword in the floor).
        auto model = editor::character::buildModel();
        {
            anim::Pose pose = model->skeleton.restPose();
            model->clips[editor::character::kIdle].sample(0.0f, true, pose);
            std::vector<glm::mat4> world(model->skeleton.jointCount());
            std::vector<glm::mat4> skin(model->skeleton.jointCount());
            anim::localToModel(model->skeleton, pose, world);
            anim::computeSkinningMatrices(model->skeleton, world, skin);
            MeshData mesh;
            mesh.indices = model->skin.mesh.indices;
            mesh.positions.resize(model->skin.mesh.positions.size());
            mesh.normals.resize(mesh.positions.size());
            anim::skinVertices(model->skin, skin, mesh.positions, mesh.normals);
            const scene::MeshId id = renderer().addMesh("Character", mesh);
            m_drawer.addPreset({"Character", "PRESET", {}, id, static_cast<u32>(mesh.triangleCount()), model, "character"});
        }
        {
            const MeshData mesh = makeBoxMesh(kFloorHalf, {0.0f, -kFloorHalf.y - 0.002f, 0.0f});
            const scene::MeshId id = renderer().addMesh("Floor", mesh);
            m_drawer.addPreset({"Floor", "PRESET", {}, id, static_cast<u32>(mesh.triangleCount()), nullptr, "floor"});
        }
        {
            const MeshData mesh = makeBoxMesh(kStoneHalf, {0.0f, kStoneHalf.y, 0.0f});
            const scene::MeshId id = renderer().addMesh("Stone", mesh);
            m_drawer.addPreset({"Stone", "PRESET", {}, id, static_cast<u32>(mesh.triangleCount()), nullptr, "stone"});
        }
    }

    void spawnPreset(const editor::MeshAsset& asset, std::optional<glm::vec3> groundPoint)
    {
        auto& registry = m_scene.registry();
        // On the floor under the cursor, or on the floor in front of the camera.
        glm::vec3 position = groundPoint.value_or(glm::vec3(0.0f));
        if (!groundPoint) {
            if (const entt::entity camera = m_scene.activeCamera(); camera != entt::null) {
                const auto& cam = registry.get<scene::TransformComponent>(camera);
                glm::vec3 ahead = cam.rotation * glm::vec3(0.0f, 0.0f, -3.0f);
                position = cam.position + ahead;
                position.y = 0.0f;
            }
        }
        entt::entity entity = entt::null;
        if (asset.preset == "character") {
            entity = m_scene.createMesh("Character", asset.id, glm::vec4(0.78f, 0.64f, 0.52f, 1.0f));
            renderer::enableAnimation(m_scene, renderer(), entity, asset.model);
            registry.emplace<scene::CharacterComponent>(entity);
        } else if (asset.preset == "floor") {
            entity = m_scene.createMesh("Floor", asset.id, glm::vec4(0.30f, 0.32f, 0.36f, 1.0f));
            registry.emplace<scene::ColliderComponent>(entity, kFloorHalf, glm::vec3(0.0f, -kFloorHalf.y, 0.0f));
            if (!groundPoint) {
                position = glm::vec3(0.0f);
            }
        } else if (asset.preset == "stone") {
            entity = m_scene.createMesh("Stone", asset.id, glm::vec4(0.56f, 0.53f, 0.49f, 1.0f));
            registry.emplace<scene::ColliderComponent>(entity, kStoneHalf, glm::vec3(0.0f, kStoneHalf.y, 0.0f));
        }
        if (entity == entt::null) {
            return;
        }
        position.y = std::max(position.y, 0.0f);
        registry.get<scene::TransformComponent>(entity).position = position;
        m_selected = entity;
    }

    void togglePlay()
    {
        if (m_play.active()) {
            m_play.stop(m_scene);
        } else {
            m_play.start(m_scene, m_camera);
        }
    }

    // Empty actor (name + transform) in front of the camera, ready for components.
    void addActor()
    {
        const entt::entity entity = m_scene.createEntity("Actor " + std::to_string(m_objectCounter++));
        placeEntity(entity, glm::vec3(0.0f), glm::vec3(0.0f), std::nullopt);
    }

    void placeEntity(entt::entity entity, const glm::vec3& boundsMin, const glm::vec3& boundsMax,
                     std::optional<glm::vec3> groundPoint)
    {
        const glm::vec3 center = (boundsMin + boundsMax) * 0.5f;
        const f32 radius = std::max(glm::length(boundsMax - boundsMin) * 0.5f, 0.05f);
        auto& position = m_scene.registry().get<scene::TransformComponent>(entity).position;
        if (groundPoint) {
            position = *groundPoint - glm::vec3(center.x, boundsMin.y, center.z);
        } else if (const entt::entity camera = m_scene.activeCamera(); camera != entt::null) {
            const auto& cam = m_scene.registry().get<scene::TransformComponent>(camera);
            position = cam.position + cam.rotation * glm::vec3(0.0f, 0.0f, -std::max(2.5f, radius * 2.5f)) - center;
        }
        m_selected = entity;
    }

    // Builds the sword-vs-stone physics prototype, frames the camera on it, opens the Physics window.
    void startSwordTest()
    {
        if (!m_swordTest.active()) {
            m_swordTest.create(m_scene, renderer());
            if (const entt::entity camera = m_scene.activeCamera(); camera != entt::null) {
                auto& cam = m_scene.registry().get<scene::TransformComponent>(camera);
                cam.position = {0.7f, 1.35f, 3.1f};
                cam.rotation = scene::lookAtRotation(cam.position, {0.55f, 1.0f, 0.0f});
            }
        }
        m_showPhysicsWindow = true;
        ImGui::SetWindowFocus("Physics");
    }

    [[nodiscard]] bool canDeleteSelected() const { return m_scene.registry().valid(m_selected) && m_selected != m_camera; }

    void deleteSelected()
    {
        if (!canDeleteSelected()) {
            return;
        }
        m_eulerCache.erase(m_selected);
        // Things attached to it (a sword in its hand) go with it.
        std::vector<entt::entity> attached;
        for (const entt::entity e : m_scene.registry().view<scene::AttachmentComponent>()) {
            if (m_scene.registry().get<scene::AttachmentComponent>(e).parent == m_selected) attached.push_back(e);
        }
        for (const entt::entity e : attached) {
            renderer::destroyEntity(m_scene, renderer(), e);
        }
        renderer::destroyEntity(m_scene, renderer(), m_selected);
        m_selected = entt::null;
    }

    // Moves the camera back along its view direction until the selection fills the view.
    void frameSelected()
    {
        const entt::entity camera = m_scene.activeCamera();
        auto& registry = m_scene.registry();
        if (camera == entt::null || !registry.valid(m_selected) || m_selected == camera) {
            return;
        }
        glm::vec3 boxMin;
        glm::vec3 boxMax;
        if (!renderer::worldBounds(m_scene, renderer(), m_selected, boxMin, boxMax)) {
            boxMin = boxMax = registry.get<scene::TransformComponent>(m_selected).position;
        }
        const glm::vec3 center = (boxMin + boxMax) * 0.5f;
        const f32 radius = std::max(glm::length(boxMax - boxMin) * 0.5f, 0.25f);
        auto& cam = registry.get<scene::TransformComponent>(camera);
        const f32 fov = registry.get<scene::CameraComponent>(camera).fovY;
        const f32 distance = radius / std::sin(fov * 0.5f) * 1.1f;
        cam.position = center - cam.rotation * glm::vec3(0.0f, 0.0f, -1.0f) * distance;
    }

    // Global keys, ignored while typing into a text field.
    void handleShortcuts()
    {
        if (ImGui::GetIO().WantTextInput) {
            return;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_F5, false)) {
            togglePlay();
            return;
        }
        if (m_play.active()) {
            if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
                togglePlay();
                return;
            }
            m_playInput.move = glm::vec2((ImGui::IsKeyDown(ImGuiKey_D) ? 1.0f : 0.0f) - (ImGui::IsKeyDown(ImGuiKey_A) ? 1.0f : 0.0f),
                                         (ImGui::IsKeyDown(ImGuiKey_W) ? 1.0f : 0.0f) - (ImGui::IsKeyDown(ImGuiKey_S) ? 1.0f : 0.0f));
            if (glm::length(m_playInput.move) > 1.0f) {
                m_playInput.move = glm::normalize(m_playInput.move);
            }
            m_playInput.run = ImGui::GetIO().KeyShift;
            m_playInput.toggleRagdoll = ImGui::IsKeyPressed(ImGuiKey_R, false);
            m_playInput.jump = ImGui::IsKeyPressed(ImGuiKey_Space, false);
            m_playInput.playAttack = -1;
            for (int k = 0; k < 9; ++k) {
                if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_1 + k), false)) m_playInput.playAttack = k;
            }
            return;
        }
        if (m_animationEditor.focused()) {
            return; // Del / F / Esc belong to the Actor Editor (graph) while it has focus
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Delete, false)) {
            deleteSelected();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_F, false)) {
            frameSelected();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
            m_selected = entt::null;
        }
    }

    // --- Panels ------------------------------------------------------------------------------------

    void drawStats()
    {
        ImGui::Begin("Stats");
        ImGui::Text("Backend: %s", device().backendName());
        ImGui::Text("%.1f FPS", static_cast<double>(ImGui::GetIO().Framerate));
        ImGui::Text("Viewport: %u x %u", renderer().sceneWidth(), renderer().sceneHeight());
        ImGui::Separator();
        ImGui::Checkbox("Show grid", &m_showGrid);
        ImGui::SliderFloat("Camera speed", &m_cameraController.moveSpeed, 0.5f, 20.0f);
        ImGui::ColorEdit3("Background", &clearColor.r);
        ImGui::Separator();
        ImGui::TextDisabled("Click: select (F: frame, Del: delete)");
        ImGui::TextDisabled("LMB + mouse: look");
        ImGui::TextDisabled("LMB + WASD/QE: move (Shift: fast)");
        ImGui::TextDisabled("Wheel: forward / back");
        ImGui::TextDisabled("Corner gizmo: click an axis");
        ImGui::TextDisabled("Assets: drag a tile onto the Viewport");
        ImGui::TextDisabled("Double-click animated actor: Animation Editor");
        ImGui::End();
    }

    void drawHierarchy()
    {
        ImGui::Begin("Hierarchy");

        // EnTT iterates newest-first; show entities in creation order instead.
        std::vector<entt::entity> entities;
        for (const entt::entity entity : m_scene.registry().view<scene::NameComponent>()) {
            entities.push_back(entity);
        }
        std::ranges::sort(entities, {}, [](entt::entity e) { return entt::to_integral(e); });

        for (const entt::entity entity : entities) {
            const auto& name = m_scene.registry().get<scene::NameComponent>(entity).name;
            const char* icon = m_scene.registry().all_of<scene::CameraComponent>(entity)     ? "[cam] "
                               : m_scene.registry().all_of<scene::CharacterComponent>(entity) ? "[char] "
                               : m_scene.registry().all_of<gameplay::CharacterControllerComponent>(entity) ? "[hero] "
                               : m_scene.registry().all_of<scene::AnimatorComponent>(entity) ? "[anim] "
                                                                                             : "";
            ImGui::PushID(static_cast<int>(entt::to_integral(entity)));
            if (ImGui::Selectable((std::string(icon) + name).c_str(), m_selected == entity,
                                  ImGuiSelectableFlags_AllowDoubleClick)) {
                m_selected = entity;
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && canOpenActorEditor(entity)) {
                    m_animationEditor.open(entity);
                }
            }
            if (canOpenActorEditor(entity) && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) {
                ImGui::SetTooltip("Double-click: Actor Editor");
            }
            ImGui::PopID();
        }

        ImGui::Spacing();
        if (ImGui::Button("+ Add Actor", ImVec2(-FLT_MIN, 0))) {
            addActor();
        }
        ImGui::End();
    }

    // Inspector: name, transform and quick options only. Everything else about an actor (its
    // components, animation graph, clips) lives in the Actor Editor (double-click the actor).
    void drawInspector()
    {
        ImGui::Begin("Inspector");
        auto& registry = m_scene.registry();
        if (!registry.valid(m_selected)) {
            ImGui::TextDisabled("Nothing selected");
            ImGui::End();
            return;
        }

        auto& name = registry.get<scene::NameComponent>(m_selected);
        std::array<char, 128> buffer{};
        std::snprintf(buffer.data(), buffer.size(), "%s", name.name.c_str());
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::InputText("##name", buffer.data(), buffer.size())) {
            name.name = buffer.data();
        }
        ImGui::Spacing();

        // Attached to a bone (a sword in a hand): its transform is recomputed every frame from the
        // socket, so the socket is what you edit.
        if (registry.all_of<scene::AttachmentComponent>(m_selected)) {
            drawSocketSection();
            drawQuickOptions();
            ImGui::Spacing();
            ImGui::Separator();
            if (ImGui::Button("Delete Actor", ImVec2(-FLT_MIN, 0))) {
                deleteSelected();
            }
            ImGui::End();
            return;
        }

        if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen) && beginProperties("##transform")) {
            auto& transform = registry.get<scene::TransformComponent>(m_selected);
            property("Position");
            ImGui::DragFloat3("##position", glm::value_ptr(transform.position), 0.05f);
            // Keep the angles the user typed; re-derive them only when the rotation was changed
            // elsewhere (e.g. by the camera controller), picking the solution nearest the old values.
            EulerCache& euler = m_eulerCache[m_selected];
            if (!euler.valid || euler.rotation != transform.rotation) {
                euler.degrees =
                    scene::eulerDegreesNear(transform.rotation, euler.valid ? euler.degrees : glm::vec3(0.0f));
                euler.rotation = transform.rotation;
                euler.valid = true;
            }
            property("Rotation");
            if (ImGui::DragFloat3("##rotation", glm::value_ptr(euler.degrees), 0.5f)) {
                transform.rotation = glm::quat(glm::radians(euler.degrees));
                euler.rotation = transform.rotation;
            }
            if (!registry.all_of<scene::CameraComponent>(m_selected)) {
                property("Scale");
                ImGui::DragFloat3("##scale", glm::value_ptr(transform.scale), 0.05f);
            }
            ImGui::EndTable();
        }

        if (auto* camera = registry.try_get<scene::CameraComponent>(m_selected)) {
            if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen) && beginProperties("##camera")) {
                f32 fovDeg = glm::degrees(camera->fovY);
                property("FOV");
                if (ImGui::SliderFloat("##fov", &fovDeg, 20.0f, 120.0f, "%.0f deg")) {
                    camera->fovY = glm::radians(fovDeg);
                }
                property("Near / Far");
                ImGui::DragFloatRange2("##clip", &camera->nearPlane, &camera->farPlane, 0.05f, 0.01f, 10000.0f);
                ImGui::EndTable();
            }
            ImGui::TextDisabled("The scene camera cannot be deleted.");
            ImGui::End();
            return;
        }

        drawQuickOptions();

        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.40f, 0.62f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.28f, 0.50f, 0.76f, 1.0f));
        if (ImGui::Button("Open Actor Editor", ImVec2(-FLT_MIN, 30.0f))) {
            m_animationEditor.open(m_selected);
        }
        ImGui::PopStyleColor(2);
        tooltip("Components, animation graph, clips and skeleton (or double-click the actor)");
        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Delete Actor", ImVec2(-FLT_MIN, 0))) {
            deleteSelected();
        }
        ImGui::End();
    }

    // Socket of an attached actor: which bone of which parent, and the offset in that bone's space
    // (position in cm, rotation in degrees). Edits apply live, also during Play.
    void drawSocketSection()
    {
        auto& registry = m_scene.registry();
        auto& a = registry.get<scene::AttachmentComponent>(m_selected);
        if (!ImGui::CollapsingHeader("Socket", ImGuiTreeNodeFlags_DefaultOpen) || !beginProperties("##socket")) {
            return;
        }
        const auto* animator = registry.valid(a.parent) ? registry.try_get<scene::AnimatorComponent>(a.parent) : nullptr;
        const anim::Skeleton* skeleton = animator && animator->model ? &animator->model->skeleton : nullptr;

        property("Parent");
        const auto* parentName = registry.valid(a.parent) ? registry.try_get<scene::NameComponent>(a.parent) : nullptr;
        ImGui::TextDisabled("%s", parentName ? parentName->name.c_str() : "<none>");

        property("Bone");
        const bool validJoint = skeleton && a.joint >= 0 && static_cast<usize>(a.joint) < skeleton->jointCount();
        if (ImGui::BeginCombo("##bone", validJoint ? skeleton->names()[static_cast<usize>(a.joint)].c_str() : "(actor origin)",
                              ImGuiComboFlags_HeightLarge)) {
            if (ImGui::Selectable("(actor origin)", a.joint < 0)) a.joint = -1;
            for (usize j = 0; skeleton && j < skeleton->jointCount(); ++j) {
                if (ImGui::Selectable(skeleton->names()[j].c_str(), a.joint == static_cast<i32>(j))) {
                    a.joint = static_cast<i32>(j);
                }
            }
            ImGui::EndCombo();
        }
        tooltip("The bone the actor follows");

        // Offset = T * R (no scale); edit as cm + degrees, keeping the typed angles stable.
        // The offset may carry the inverse of a bone scale (e.g. FBX rigs in cm): kept as is.
        glm::mat3 basis(a.offset);
        const glm::vec3 offsetScale(glm::length(basis[0]), glm::length(basis[1]), glm::length(basis[2]));
        for (int i = 0; i < 3; ++i) {
            if (offsetScale[i] > 1e-8f) basis[i] /= offsetScale[i];
        }
        // Position shown in the bone's axes, converted to metres of the attached actor.
        const f32 unit = offsetScale.x > 1e-8f ? 1.0f / offsetScale.x : 1.0f;
        glm::vec3 positionCm = glm::vec3(a.offset[3]) * unit * 100.0f;
        const glm::quat rotation = glm::normalize(glm::quat_cast(basis));
        EulerCache& euler = m_eulerCache[m_selected];
        if (!euler.valid || euler.rotation != rotation) {
            euler.degrees = scene::eulerDegreesNear(rotation, euler.valid ? euler.degrees : glm::vec3(0.0f));
            euler.rotation = rotation;
            euler.valid = true;
        }
        bool changed = false;
        property("Position");
        changed |= ImGui::DragFloat3("##socketPos", glm::value_ptr(positionCm), 0.1f, 0.0f, 0.0f, "%.1f cm");
        tooltip("Offset from the bone, in the bone's own axes (cm). Ctrl+click to type a value.");
        property("Rotation");
        if (ImGui::DragFloat3("##socketRot", glm::value_ptr(euler.degrees), 0.5f, 0.0f, 0.0f, "%.1f deg")) {
            euler.rotation = glm::quat(glm::radians(euler.degrees));
            changed = true;
        }
        tooltip("Rotation relative to the bone (X, Y, Z degrees). The sword's blade runs along its +Y.");
        if (changed) {
            a.offset = glm::translate(glm::mat4(1.0f), positionCm / (100.0f * unit)) * glm::mat4_cast(euler.rotation) *
                       glm::scale(glm::mat4(1.0f), offsetScale);
        }
        property("Keep scale");
        ImGui::Checkbox("##inheritScale", &a.inheritScale);
        tooltip("Off: keeps its own size in metres even if the parent is scaled");
        ImGui::EndTable();

        const f32 half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (ImGui::Button("Flip blade", ImVec2(half, 0))) {
            a.offset = a.offset * glm::mat4_cast(glm::angleAxis(glm::pi<f32>(), glm::vec3(1.0f, 0.0f, 0.0f)));
        }
        tooltip("Turn 180 degrees about X (blade up <-> down)");
        ImGui::SameLine();
        const i32 hand = skeleton ? anim::findWeaponHand(*skeleton) : -1;
        ImGui::BeginDisabled(hand < 0);
        if (ImGui::Button("Reset to hand", ImVec2(-FLT_MIN, 0))) {
            a.joint = hand;
            a.offset = anim::weaponSocket(*skeleton, hand);
        }
        ImGui::EndDisabled();
        tooltip("Automatic grip in the weapon hand (palm, between the fingers and the thumb)");
    }

    // The few things you toggle all the time, as one-line switches.
    void drawQuickOptions()
    {
        auto& registry = m_scene.registry();
        if (!ImGui::CollapsingHeader("Quick Options", ImGuiTreeNodeFlags_DefaultOpen) || !beginProperties("##quick")) {
            return;
        }
        if (auto* mesh = registry.try_get<scene::MeshRendererComponent>(m_selected)) {
            property("Visible");
            ImGui::Checkbox("##visible", &mesh->visible);
            property("Asset");
            const editor::MeshAsset* asset = selectedAsset();
            ImGui::TextDisabled("%s", asset ? asset->name.c_str() : "-");
        }
        if (auto* animator = registry.try_get<scene::AnimatorComponent>(m_selected)) {
            if (animator->blendTree && !animator->blendTree->stateMachine.empty()) {
                // Graph: which state (forcing one is a quick way to preview it).
                anim::BlendTree& graph = *animator->blendTree;
                property("State");
                const i32 state = animator->treeState.state;
                const char* current = graph.stateMachine.validState(state) ? graph.stateMachine.states[static_cast<usize>(state)].name.c_str() : "-";
                if (ImGui::BeginCombo("##state", animator->useBlendTree ? current : "(graph off)")) {
                    for (usize s = 0; s < graph.stateMachine.states.size(); ++s) {
                        if (ImGui::Selectable(graph.stateMachine.states[s].name.c_str(), state == static_cast<i32>(s))) {
                            animator->treeState.forceState(graph, static_cast<i32>(s));
                            animator->useBlendTree = true;
                            animator->bindPose = false;
                        }
                    }
                    ImGui::EndCombo();
                }
            } else {
                property("Clip");
                const auto clips = animator->clips();
                const i32 clip = animator->player.clip();
                const bool hasClip = clip >= 0 && static_cast<usize>(clip) < clips.size();
                if (ImGui::BeginCombo("##clip", hasClip ? clips[static_cast<usize>(clip)].name().c_str() : "<none>")) {
                    for (usize i = 0; i < clips.size(); ++i) {
                        if (ImGui::Selectable(clips[i].name().c_str(), static_cast<i32>(i) == clip)) {
                            animator->player.play(static_cast<i32>(i), 0.25f);
                            animator->bindPose = false;
                        }
                    }
                    ImGui::EndCombo();
                }
            }
            property("Foot IK");
            if (ImGui::Checkbox("##footIK", &animator->footIK.enabled)) {
                animator->footIKState = {};
            }
            tooltip("Feet follow the colliders under them (settings: Actor Editor > Animator)");

            property("Playable");
            bool playable = registry.all_of<gameplay::CharacterControllerComponent>(m_selected);
            if (ImGui::Checkbox("##playable", &playable)) {
                if (playable) {
                    // A complete hero (graph, foot IK, controller) unless it already has a graph.
                    if (!animator->blendTree) {
                        m_heroMessage = editor::makePlayableHero(m_scene, m_selected, m_heroHeight).message;
                        m_heroMessageFor = m_selected;
                    } else {
                        registry.emplace<gameplay::CharacterControllerComponent>(m_selected);
                    }
                } else {
                    registry.remove<gameplay::CharacterControllerComponent>(m_selected);
                }
            }
            tooltip("Controlled with WASD / Shift / Space in Play (F5). First time: scales the model to a human height, "
                    "builds the locomotion graph and turns on foot IK.");
        }
        property("Collider");
        bool collider = registry.all_of<scene::ColliderComponent>(m_selected);
        if (registry.all_of<gameplay::CharacterControllerComponent>(m_selected) && !collider) {
            ImGui::TextDisabled("controller capsule");
            tooltip("A playable actor collides with its Character Controller's capsule (blue). A Box Collider is for "
                    "static things: floors, walls, stones.");
        } else if (ImGui::Checkbox("##collider", &collider)) {
            if (collider) {
                registry.emplace<scene::ColliderComponent>(m_selected);
            } else {
                registry.remove<scene::ColliderComponent>(m_selected);
            }
        }
        tooltip("Static box: characters stand on it, swords hit it");
        ImGui::EndTable();
        if (!m_heroMessage.empty() && m_heroMessageFor == m_selected) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextDisabled("%s", m_heroMessage.c_str());
            ImGui::PopTextWrapPos();
        }
    }

    // Two-column property grid: label on the left, full-width widget on the right.
    static bool beginProperties(const char* id)
    {
        if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_PadOuterX)) {
            return false;
        }
        ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 84.0f);
        ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
        return true;
    }

    static void property(const char* label)
    {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(label);
        ImGui::TableSetColumnIndex(1);
        ImGui::SetNextItemWidth(-FLT_MIN);
    }

    [[nodiscard]] bool canOpenActorEditor(entt::entity entity) const
    {
        return m_scene.registry().valid(entity) && entity != m_camera;
    }

    [[nodiscard]] bool isAnimated(entt::entity entity) const
    {
        const auto& registry = m_scene.registry();
        return registry.valid(entity) && registry.all_of<scene::AnimatorComponent>(entity);
    }

    // Asset shown by an actor's Mesh Renderer.
    [[nodiscard]] const editor::MeshAsset* assetOf(entt::entity entity) const
    {
        const auto& registry = m_scene.registry();
        if (const auto* animator = registry.try_get<scene::AnimatorComponent>(entity)) {
            if (const editor::MeshAsset* asset = m_drawer.findByModel(animator->model.get())) {
                return asset;
            }
        }
        const auto* mesh = registry.try_get<scene::MeshRendererComponent>(entity);
        if (const auto* animator = registry.try_get<scene::AnimatorComponent>(entity); animator && mesh) {
            return m_drawer.findByMesh(animator->restMesh);
        }
        return mesh ? m_drawer.findByMesh(mesh->mesh) : nullptr;
    }
    [[nodiscard]] const editor::MeshAsset* selectedAsset() const { return assetOf(m_selected); }

    // --- Component settings (Actor Editor > Components) ----------------------------------------

    void drawComponentDetails(entt::entity actor, editor::ComponentKind kind) override
    {
        switch (kind) {
        case editor::ComponentKind::MeshRenderer: drawMeshDetails(actor); break;
        case editor::ComponentKind::Animator: drawAnimatorDetails(actor); break;
        case editor::ComponentKind::CharacterController: editor::drawCharacterControllerDetails(m_scene, actor); break;
        case editor::ComponentKind::Collider: drawColliderDetails(actor); break;
        case editor::ComponentKind::PhysicsCharacter: drawPhysicsCharacterDetails(actor); break;
        }
    }

    void removeComponent(entt::entity actor, editor::ComponentKind kind) override
    {
        auto& registry = m_scene.registry();
        switch (kind) {
        case editor::ComponentKind::MeshRenderer: renderer::removeMesh(m_scene, renderer(), actor); break;
        case editor::ComponentKind::Animator: renderer::disableAnimation(m_scene, renderer(), actor); break;
        case editor::ComponentKind::CharacterController: registry.remove<gameplay::CharacterControllerComponent>(actor); break;
        case editor::ComponentKind::Collider: registry.remove<scene::ColliderComponent>(actor); break;
        case editor::ComponentKind::PhysicsCharacter: registry.remove<scene::CharacterComponent>(actor); break;
        }
    }

    void drawAddComponentMenu(entt::entity actor) override
    {
        auto& registry = m_scene.registry();
        const bool hasMesh = registry.all_of<scene::MeshRendererComponent>(actor);
        if (ImGui::MenuItem("Mesh Renderer", nullptr, false, !hasMesh)) {
            registry.emplace<scene::MeshRendererComponent>(actor, scene::kTriangleMesh,
                                                           glm::vec4(kPalette[m_objectCounter++ % kPalette.size()], 1.0f));
        }
        const editor::MeshAsset* asset = assetOf(actor);
        const bool canAnimate = hasMesh && !isAnimated(actor) && asset && asset->animated();
        if (ImGui::MenuItem("Animator", nullptr, false, canAnimate)) {
            renderer::enableAnimation(m_scene, renderer(), actor, asset->model);
        }
        if (!canAnimate && !isAnimated(actor) && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            ImGui::SetTooltip("Needs a Mesh Renderer showing an animated (ANIM) asset");
        }
        if (ImGui::MenuItem("Character Controller", nullptr, false, !registry.all_of<gameplay::CharacterControllerComponent>(actor))) {
            registry.emplace<gameplay::CharacterControllerComponent>(actor);
        }
        if (ImGui::MenuItem("Box Collider", nullptr, false, !registry.all_of<scene::ColliderComponent>(actor))) {
            registry.emplace<scene::ColliderComponent>(actor);
        }
    }

    void drawMeshDetails(entt::entity actor)
    {
        auto& registry = m_scene.registry();
        auto& mesh = registry.get<scene::MeshRendererComponent>(actor);
        if (beginProperties("##mesh")) {
            const editor::MeshAsset* current = assetOf(actor);
            property("Asset");
            if (const auto picked = m_drawer.assetCombo("##asset", current)) {
                setMeshAsset(actor, *picked, isAnimated(actor));
            }
            tooltip("Pick an asset, or drag a tile from the Assets drawer onto this field");
            property("Tint");
            ImGui::ColorEdit3("##tint", glm::value_ptr(mesh.tint));
            property("Visible");
            ImGui::Checkbox("##visible", &mesh.visible);
            ImGui::EndTable();
        }
        editor::drawMaterialSection(mesh, renderer());
    }

    void drawAnimatorDetails(entt::entity actor)
    {
        auto& animator = m_scene.registry().get<scene::AnimatorComponent>(actor);
        if (beginProperties("##animator")) {
            property("Mode");
            int mode = animator.useBlendTree && animator.blendTree ? 1 : 0;
            if (ImGui::Combo("##mode", &mode, animator.blendTree ? "Single clip\0Animation graph\0" : "Single clip\0")) {
                animator.useBlendTree = mode == 1;
                animator.bindPose = false;
            }
            tooltip("Animation graph: the state machine in the Animation Graph tab. Single clip: Clips & Timeline.");
            ImGui::EndTable();
        }
        if (!animator.blendTree) {
            if (ImGui::Button("Create Locomotion Graph", ImVec2(-FLT_MIN, 0))) {
                editor::rebuildLocomotionTree(m_scene, actor);
            }
        }
        editor::drawFootIKSettings(m_scene, actor);

        if (!m_scene.registry().all_of<gameplay::CharacterControllerComponent>(actor) &&
            !m_scene.registry().all_of<scene::CharacterComponent>(actor)) {
            ImGui::SeparatorText("Playable Hero");
            ImGui::SetNextItemWidth(90.0f);
            ImGui::DragFloat("Height (m)", &m_heroHeight, 0.01f, 0.3f, 4.0f, "%.2f");
            if (ImGui::Button("Make Playable Hero", ImVec2(-FLT_MIN, 0))) {
                m_heroMessage = editor::makePlayableHero(m_scene, actor, m_heroHeight).message;
                m_heroMessageFor = actor;
            }
            tooltip("Scale to this height, locomotion graph (idle / walk / run / turns / jump), foot IK and a Character "
                    "Controller - then Play (F5): WASD, Shift, Space");
        }
        if (!m_heroMessage.empty() && m_heroMessageFor == actor) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextDisabled("%s", m_heroMessage.c_str());
            ImGui::PopTextWrapPos();
        }
    }

    void drawPhysicsCharacterDetails(entt::entity actor)
    {
        auto& c = m_scene.registry().get<scene::CharacterComponent>(actor);
        if (beginProperties("##character")) {
            property("Walk speed");
            ImGui::SliderFloat("##walk", &c.walkSpeed, 0.5f, 3.0f, "%.1f m/s");
            property("Run speed");
            ImGui::SliderFloat("##run", &c.runSpeed, 1.0f, 7.0f, "%.1f m/s");
            property("Strength");
            ImGui::SliderFloat("##strength", &c.strength, 0.2f, 3.0f, "x%.2f");
            tooltip("Scales every muscle's strength (how hard the sword arm can push / resist)");
            ImGui::EndTable();
        }
        ImGui::TextDisabled("Play (F5): WASD move, Shift run, LMB attack, R ragdoll.");
    }

    void drawColliderDetails(entt::entity actor)
    {
        auto& c = m_scene.registry().get<scene::ColliderComponent>(actor);
        if (beginProperties("##collider")) {
            property("Half extents");
            ImGui::DragFloat3("##half", glm::value_ptr(c.halfExtents), 0.01f, 0.01f, 100.0f);
            property("Offset");
            ImGui::DragFloat3("##offset", glm::value_ptr(c.offset), 0.01f);
            property("Material");
            const scene::SurfaceProperties& current = scene::surfaceProperties(c.material);
            if (ImGui::BeginCombo("##material", current.name)) {
                for (u8 m = 0; m < static_cast<u8>(scene::SurfaceMaterial::Count); ++m) {
                    const auto material = static_cast<scene::SurfaceMaterial>(m);
                    const scene::SurfaceProperties& p = scene::surfaceProperties(material);
                    if (ImGui::Selectable(p.name, material == c.material)) c.material = material;
                    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
                        ImGui::SetTooltip("hardness %.2f, bounce %.2f, friction %.2f%s", static_cast<double>(p.hardness),
                                          static_cast<double>(p.restitution), static_cast<double>(p.friction),
                                          p.hardness < 0.65f ? "\nsoft: a blade cuts in" : "\nhard: a blade stops / deflects");
                    }
                }
                ImGui::EndCombo();
            }
            // "On hit" event: the effect played where this is struck.
            property("On hit");
            const std::string fallback = "hit_" + lowerCase(scene::surfaceProperties(c.material).name);
            const std::string label = c.onHit.empty() ? fallback + " (material)" : c.onHit;
            if (ImGui::BeginCombo("##onHit", label.c_str())) {
                if (ImGui::Selectable((fallback + " (material)").c_str(), c.onHit.empty())) c.onHit.clear();
                if (ImGui::Selectable("none", c.onHit == "none")) c.onHit = "none";
                for (const std::string& name : m_vfxLibrary.names()) {
                    if (ImGui::Selectable(name.c_str(), c.onHit == name)) c.onHit = name;
                }
                ImGui::EndCombo();
            }
            tooltip("Effect played where a weapon strikes this (assets/vfx/*.vfx). By default the material's: hit_stone,\n"
                    "hit_metal, hit_wood, hit_dirt, hit_flesh.");
            ImGui::EndTable();
        }
        ImGui::TextDisabled("Static: characters stand on it and swords hit it in Play.");
    }

    static std::string lowerCase(std::string text)
    {
        std::ranges::transform(text, text.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        return text;
    }

    // Saves an actor's settings file (<model>.actor: graph, physics bones, weapon sockets) when
    // they change. Checked once a second, not while playing; the first look only notes the state.
    void autosaveActors(f32 dt)
    {
        m_autosaveTimer += dt;
        if (m_autosaveTimer < 1.0f || m_play.active()) {
            return;
        }
        m_autosaveTimer = 0.0f;
        auto& registry = m_scene.registry();
        for (auto it = m_actorSaved.begin(); it != m_actorSaved.end();) {
            it = registry.valid(it->first) ? std::next(it) : m_actorSaved.erase(it);
        }
        for (const entt::entity e : registry.view<scene::AnimatorComponent>()) {
            const auto& animator = registry.get<scene::AnimatorComponent>(e);
            if (!animator.model || !animator.blendTree || editor::actorFilePath(*animator.model).empty()) continue;
            std::string text = editor::actorToJson(m_scene, e).dump();
            auto [entry, first] = m_actorSaved.try_emplace(e, text);
            if (first || entry->second == text) continue;
            std::string message;
            if (editor::saveActor(m_scene, e, &message)) {
                KN_INFO("{}", message);
                entry->second = std::move(text);
            }
        }
    }

    static void tooltip(const char* text)
    {
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            ImGui::SetTooltip("%s", text);
        }
    }

    // Switches the selected actor's mesh to drawer asset `index`, staying animated if possible.
    void setMeshAsset(entt::entity actor, usize index, bool wasAnimated)
    {
        const editor::MeshAsset* asset = m_drawer.asset(index);
        if (!asset) {
            return;
        }
        renderer::disableAnimation(m_scene, renderer(), actor);
        m_scene.registry().get<scene::MeshRendererComponent>(actor).mesh = asset->id;
        applyMaterial(actor, *asset);
        if (asset->animated() && wasAnimated) {
            renderer::enableAnimation(m_scene, renderer(), actor, asset->model);
        }
    }

    scene::Scene m_scene;
    entt::entity m_camera = entt::null;
    entt::entity m_selected = entt::null;
    FlyCameraController m_cameraController;
    bool m_layoutInitialized = false;
    bool m_showGrid = true;
    bool m_showSelectionBox = true;
    bool m_showGizmos = true;        // the selected actor's components (collider, controller, foot IK)
    bool m_showAllColliders = false; // every collider in the scene
    bool m_showSkeleton = true; // skeleton of the selected animated actor in the Viewport
    bool m_lookBlocked = false;
    editor::AssetDrawer m_drawer;
    editor::AnimationEditor m_animationEditor;
    editor::SwordStrikeTest m_swordTest;
    editor::PlayMode m_play;
    vfx::VfxLibrary m_vfxLibrary;
    renderer::VfxRenderState m_vfxRender;
    std::unordered_map<entt::entity, std::string> m_actorSaved; // last saved / seen actor file text
    f32 m_autosaveTimer = 0.0f;
    editor::PlayInput m_playInput;
    glm::vec2 m_playOrbit{0.0f};
    f32 m_playWheel = 0.0f;
    static constexpr glm::vec3 kFloorHalf{10.0f, 0.1f, 10.0f};
    static constexpr glm::vec3 kStoneHalf{0.35f, 0.45f, 0.35f};
    bool m_showPhysicsWindow = false;
    std::vector<std::string> m_droppedFiles;

    // Euler angles shown in the inspector, per entity (see drawInspector).
    struct EulerCache {
        glm::vec3 degrees{0.0f};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        bool valid = false;
    };
    std::unordered_map<entt::entity, EulerCache> m_eulerCache;

    usize m_objectCounter = 1;
    f32 m_heroHeight = 1.8f;
    scene::MeshId m_swordMesh = renderer::Renderer::kInvalidMesh;
    std::string m_heroMessage;
    entt::entity m_heroMessageFor = entt::null;
};

int main(int argc, char** argv)
{
    AppDesc desc;
    desc.window.title = "Kinetic Editor";
    desc.presentScene = false; // the scene lives in the Viewport panel
    Application::applyCommandLine(argc, argv, desc);
    EditorApp app(desc);
    return app.run();
}
