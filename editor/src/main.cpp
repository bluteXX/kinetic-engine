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
#include "PlayMode.h"
#include "SwordStrikeTest.h"

#include <kinetic/animation/PoseOps.h>
#include <kinetic/app/Application.h>
#include <kinetic/core/Primitives.h>
#include <kinetic/core/Log.h>
#include <kinetic/renderer/AnimationSystem.h>
#include <kinetic/renderer/Picking.h>
#include <kinetic/scene/AnimatorComponent.h>
#include <kinetic/scene/Scene.h>

#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_main.h>
#include <imgui.h>
#include <imgui_internal.h> // DockBuilder API

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
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

class EditorApp final : public Application {
public:
    using Application::Application;

protected:
    bool onInit() override
    {
        m_camera = m_scene.createCamera("Camera", {0.0f, 1.6f, 4.0f}, {0.0f, 0.8f, 0.0f});
        m_drawer.init(renderer(), {assetsRoot() / "meshes", assetsRoot() / "models"});
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
                if (ImGui::MenuItem("Animation Editor", nullptr, false, isAnimated(m_selected))) {
                    m_animationEditor.open(m_selected);
                }
                ImGui::Separator();
                ImGui::MenuItem("Skeleton of Selection", nullptr, &m_showSkeleton);
                ImGui::MenuItem("Selection Outline", nullptr, &m_showSelectionBox);
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
        m_animationEditor.draw(m_scene, renderer());
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
        renderer::updateAnimations(m_scene, renderer(), ImGui::GetIO().DeltaTime);
        std::vector<renderer::DebugVertex> overlay;
        if (m_showSkeleton && isAnimated(m_selected)) {
            renderer::appendSkeletonLines(m_scene.registry().get<scene::AnimatorComponent>(m_selected),
                                          m_scene.registry().get<scene::TransformComponent>(m_selected).matrix(), overlay);
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
            if (isAnimated(m_selected)) {
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
        const glm::vec3 tint = kPalette[m_objectCounter % kPalette.size()];
        const std::string name = asset->id == scene::kTriangleMesh ? "Actor " + std::to_string(m_objectCounter) : asset->name;
        const entt::entity entity = m_scene.createMesh(name, asset->id, glm::vec4(tint, 1.0f));
        ++m_objectCounter;
        if (asset->animated()) {
            renderer::enableAnimation(m_scene, renderer(), entity, asset->model);
        }
        placeEntity(entity, boundsMin, boundsMax, groundPoint);
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
            return;
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
                               : m_scene.registry().all_of<scene::AnimatorComponent>(entity) ? "[anim] "
                                                                                             : "";
            ImGui::PushID(static_cast<int>(entt::to_integral(entity)));
            if (ImGui::Selectable((std::string(icon) + name).c_str(), m_selected == entity,
                                  ImGuiSelectableFlags_AllowDoubleClick)) {
                m_selected = entity;
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && isAnimated(entity)) {
                    m_animationEditor.open(entity);
                }
            }
            if (isAnimated(entity) && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) {
                ImGui::SetTooltip("Double-click: Animation Editor");
            }
            ImGui::PopID();
        }

        ImGui::Spacing();
        if (ImGui::Button("+ Add Actor", ImVec2(-FLT_MIN, 0))) {
            addActor();
        }
        ImGui::End();
    }

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
        }

        if (!registry.all_of<scene::CameraComponent>(m_selected)) {
            if (registry.all_of<scene::MeshRendererComponent>(m_selected)) {
                drawMeshComponent();
            }
            if (registry.all_of<scene::AnimatorComponent>(m_selected)) {
                drawAnimatorComponent();
            }
            if (registry.all_of<scene::CharacterComponent>(m_selected)) {
                drawCharacterComponent();
            }
            if (registry.all_of<scene::ColliderComponent>(m_selected)) {
                drawColliderComponent();
            }
            drawAddComponent();
        }

        ImGui::Spacing();
        ImGui::Separator();
        if (m_selected == m_camera) {
            ImGui::TextDisabled("The scene camera cannot be deleted.");
        } else if (ImGui::Button("Delete Actor", ImVec2(-FLT_MIN, 0))) {
            deleteSelected();
        }
        ImGui::End();
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

    [[nodiscard]] bool isAnimated(entt::entity entity) const
    {
        const auto& registry = m_scene.registry();
        return registry.valid(entity) && registry.all_of<scene::AnimatorComponent>(entity);
    }

    // Asset shown by the selected actor's Mesh Renderer.
    [[nodiscard]] const editor::MeshAsset* selectedAsset() const
    {
        const auto& registry = m_scene.registry();
        if (const auto* animator = registry.try_get<scene::AnimatorComponent>(m_selected)) {
            return m_drawer.findByModel(animator->model.get());
        }
        const auto* mesh = registry.try_get<scene::MeshRendererComponent>(m_selected);
        return mesh ? m_drawer.findByMesh(mesh->mesh) : nullptr;
    }

    // Mesh Renderer: which asset the actor shows, its tint and visibility.
    void drawMeshComponent()
    {
        auto& registry = m_scene.registry();
        bool keep = true;
        if (ImGui::CollapsingHeader("Mesh Renderer", &keep, ImGuiTreeNodeFlags_DefaultOpen) && beginProperties("##mesh")) {
            const editor::MeshAsset* current = selectedAsset();
            property("Asset");
            if (const auto picked = m_drawer.assetCombo("##asset", current)) {
                setMeshAsset(*picked, isAnimated(m_selected));
                current = m_drawer.asset(*picked);
            }
            tooltip("Pick an asset, or drag a tile from the Assets drawer onto this field");

            auto& mesh = registry.get<scene::MeshRendererComponent>(m_selected);
            property("Tint");
            ImGui::ColorEdit3("##tint", glm::value_ptr(mesh.tint));
            property("Visible");
            ImGui::Checkbox("##visible", &mesh.visible);
            ImGui::EndTable();

            if (current && current->animated() && !isAnimated(m_selected)) {
                ImGui::TextDisabled("This asset has %zu animation clip(s).", current->model->clips.size());
                ImGui::SameLine();
                if (ImGui::SmallButton("Add Animator")) {
                    renderer::enableAnimation(m_scene, renderer(), m_selected, current->model);
                }
            }
        }
        if (!keep) {
            renderer::removeMesh(m_scene, renderer(), m_selected);
        }
    }

    // Animator: playback settings only. Working with clips happens in the Animation Editor.
    void drawAnimatorComponent()
    {
        bool keep = true;
        if (ImGui::CollapsingHeader("Animator", &keep, ImGuiTreeNodeFlags_DefaultOpen) && beginProperties("##animator")) {
            auto& animator = m_scene.registry().get<scene::AnimatorComponent>(m_selected);
            anim::AnimationPlayer& player = animator.player;
            const auto clips = animator.clips();
            const i32 clip = player.clip();
            const bool hasClip = clip >= 0 && static_cast<usize>(clip) < clips.size();

            property("Clip");
            if (ImGui::BeginCombo("##clip", hasClip ? clips[static_cast<usize>(clip)].name().c_str() : "<none>")) {
                for (usize i = 0; i < clips.size(); ++i) {
                    ImGui::PushID(static_cast<int>(i));
                    if (ImGui::Selectable(clips[i].name().c_str(), static_cast<i32>(i) == clip)) {
                        player.play(static_cast<i32>(i), 0.25f);
                        animator.bindPose = false;
                    }
                    ImGui::SameLine();
                    clipKindBadge(clips[i]);
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            if (hasClip) {
                property("Kind");
                clipKindBadge(clips[static_cast<usize>(clip)]);
            }
            property("Loop");
            ImGui::Checkbox("##loop", &player.loop);
            property("Speed");
            ImGui::SliderFloat("##speed", &player.speed, -2.0f, 3.0f, "x%.2f");
            property("Playback");
            ImGui::BeginDisabled(!hasClip);
            if (ImGui::SmallButton(player.playing ? "Pause" : "Play")) {
                player.playing = !player.playing;
                animator.bindPose = false;
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (hasClip) {
                ImGui::TextDisabled("%.2f / %.2f s", static_cast<double>(player.time()),
                                    static_cast<double>(clips[static_cast<usize>(clip)].duration()));
            } else {
                ImGui::TextDisabled("no clip");
            }
            ImGui::EndTable();

            if (ImGui::Button("Edit Animation...", ImVec2(-FLT_MIN, 0))) {
                m_animationEditor.open(m_selected);
            }
            tooltip("Animation Editor: preview, clips, timeline and skeleton (or double-click the actor)");
        }
        if (!keep) {
            renderer::disableAnimation(m_scene, renderer(), m_selected);
        }
    }

    void drawCharacterComponent()
    {
        bool keep = true;
        if (ImGui::CollapsingHeader("Character (physics)", &keep, ImGuiTreeNodeFlags_DefaultOpen) && beginProperties("##character")) {
            auto& c = m_scene.registry().get<scene::CharacterComponent>(m_selected);
            property("Walk speed");
            ImGui::SliderFloat("##walk", &c.walkSpeed, 0.5f, 3.0f, "%.1f m/s");
            property("Run speed");
            ImGui::SliderFloat("##run", &c.runSpeed, 1.0f, 7.0f, "%.1f m/s");
            property("Strength");
            ImGui::SliderFloat("##strength", &c.strength, 0.2f, 3.0f, "x%.2f");
            tooltip("Scales every muscle's strength (how hard the sword arm can push / resist)");
            ImGui::EndTable();
            ImGui::TextDisabled("Play (F5): WASD move, Shift run, LMB attack, R ragdoll.");
            ImGui::TextDisabled("Walk / Idle are regular animations; Strike is played by the muscles.");
        }
        if (!keep) {
            m_scene.registry().remove<scene::CharacterComponent>(m_selected);
        }
    }

    void drawColliderComponent()
    {
        bool keep = true;
        if (ImGui::CollapsingHeader("Box Collider", &keep, ImGuiTreeNodeFlags_DefaultOpen) && beginProperties("##collider")) {
            auto& c = m_scene.registry().get<scene::ColliderComponent>(m_selected);
            property("Half extents");
            ImGui::DragFloat3("##half", glm::value_ptr(c.halfExtents), 0.01f, 0.01f, 100.0f);
            property("Offset");
            ImGui::DragFloat3("##offset", glm::value_ptr(c.offset), 0.01f);
            ImGui::EndTable();
            ImGui::TextDisabled("Static: characters stand on it and swords hit it in Play.");
        }
        if (!keep) {
            m_scene.registry().remove<scene::ColliderComponent>(m_selected);
        }
    }

    void drawAddComponent()
    {
        ImGui::Spacing();
        if (ImGui::Button("+ Add Component", ImVec2(-FLT_MIN, 0))) {
            ImGui::OpenPopup("##addComponent");
        }
        if (ImGui::BeginPopup("##addComponent")) {
            auto& registry = m_scene.registry();
            const bool hasMesh = registry.all_of<scene::MeshRendererComponent>(m_selected);
            if (ImGui::MenuItem("Box Collider", nullptr, false, !registry.all_of<scene::ColliderComponent>(m_selected))) {
                registry.emplace<scene::ColliderComponent>(m_selected);
            }
            if (ImGui::MenuItem("Mesh Renderer", nullptr, false, !hasMesh)) {
                registry.emplace<scene::MeshRendererComponent>(
                    m_selected, scene::kTriangleMesh, glm::vec4(kPalette[m_objectCounter++ % kPalette.size()], 1.0f));
            }
            const editor::MeshAsset* asset = selectedAsset();
            const bool canAnimate = hasMesh && !isAnimated(m_selected) && asset && asset->animated();
            if (ImGui::MenuItem("Animator", nullptr, false, canAnimate)) {
                renderer::enableAnimation(m_scene, renderer(), m_selected, asset->model);
            }
            if (!canAnimate && !isAnimated(m_selected) &&
                ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                ImGui::SetTooltip("Needs a Mesh Renderer showing an animated (ANIM) asset");
            }
            ImGui::EndPopup();
        }
    }

    // [KIN] regular animation / [PHYS] played by muscles of a physics body.
    static void clipKindBadge(const anim::AnimationClip& clip)
    {
        if (clip.kind() == anim::ClipKind::Physics) {
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.25f, 1.0f), "[PHYS]");
            tooltip("Physics animation: in Play the muscles follow this clip; hits and rebounds are simulated");
        } else {
            ImGui::TextColored(ImVec4(0.45f, 0.7f, 1.0f, 1.0f), "[KIN]");
            tooltip("Regular (kinematic) animation: the pose is shown as it is in the clip");
        }
    }

    static void tooltip(const char* text)
    {
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            ImGui::SetTooltip("%s", text);
        }
    }

    // Switches the selected actor's mesh to drawer asset `index`, staying animated if possible.
    void setMeshAsset(usize index, bool wasAnimated)
    {
        const editor::MeshAsset* asset = m_drawer.asset(index);
        if (!asset) {
            return;
        }
        renderer::disableAnimation(m_scene, renderer(), m_selected);
        m_scene.registry().get<scene::MeshRendererComponent>(m_selected).mesh = asset->id;
        if (asset->animated() && wasAnimated) {
            renderer::enableAnimation(m_scene, renderer(), m_selected, asset->model);
        }
    }

    scene::Scene m_scene;
    entt::entity m_camera = entt::null;
    entt::entity m_selected = entt::null;
    FlyCameraController m_cameraController;
    bool m_layoutInitialized = false;
    bool m_showGrid = true;
    bool m_showSelectionBox = true;
    bool m_showSkeleton = true; // skeleton of the selected animated actor in the Viewport
    bool m_lookBlocked = false;
    editor::AssetDrawer m_drawer;
    editor::AnimationEditor m_animationEditor;
    editor::SwordStrikeTest m_swordTest;
    editor::PlayMode m_play;
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
