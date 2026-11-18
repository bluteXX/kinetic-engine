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
// one; "+ Add Component" in the Inspector gives it a Mesh. The Mesh component picks any asset from
// the drawer; assets with a skeleton (ANIM) can be ticked "Animated", which adds an Animation
// section with the playback controls. "Open in window" pops those out into the Animation window.
//
// Assets: the drawer (bottom) shows assets/meshes/ and assets/models/. Drag a tile onto the Viewport
// to place a new actor on the floor, or onto an actor's Mesh field to switch its asset. Dropping
// .fbx / .obj / .glb files on the window copies them into the asset folders.

#include "AnimationPanel.h"
#include "AssetDrawer.h"

#include <kinetic/app/Application.h>
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
        m_camera = m_scene.createCamera("Camera", {0.0f, 1.2f, 3.5f}, {0.0f, 0.3f, 0.0f});
        m_selected = m_scene.createMesh("Triangle", scene::kTriangleMesh, glm::vec4(kPalette[0], 1.0f));
        m_scene.registry().get<scene::TransformComponent>(m_selected).position = {0.0f, 0.4f, 0.0f};
        m_drawer.init(renderer(), {assetsRoot() / "meshes", assetsRoot() / "models"});
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
            if (ImGui::BeginMenu("View")) {
                ImGui::MenuItem("Animation Window", nullptr, &m_showAnimationWindow);
                ImGui::MenuItem("Skeletons", nullptr, &m_animationPanel.showSkeleton);
                ImGui::MenuItem("Selection Outline", nullptr, &m_showSelectionBox);
                ImGui::MenuItem("Grid", nullptr, &m_showGrid);
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        drawViewport();
        drawHierarchy();
        drawInspector();
        drawStats();
        m_drawer.draw();
        if (m_showAnimationWindow) {
            m_animationPanel.drawWindow(m_scene, m_selected, &m_showAnimationWindow);
        }
        if (const auto index = m_drawer.takeSpawnRequest()) {
            spawnAsset(*index, std::nullopt);
        }
        handleShortcuts();

        // After the UI, so scrubbing / clip changes show up this frame.
        renderer::updateAnimations(m_scene, renderer(), ImGui::GetIO().DeltaTime);
        std::vector<renderer::DebugVertex> overlay;
        if (m_animationPanel.showSkeleton) {
            renderer::appendSkeletonLines(m_scene, overlay, m_selected, m_animationPanel.selectedJoint());
        }
        glm::vec3 boxMin;
        glm::vec3 boxMax;
        if (m_showSelectionBox && renderer::worldBounds(m_scene, renderer(), m_selected, boxMin, boxMax)) {
            const glm::vec3 pad = glm::max((boxMax - boxMin) * 0.02f, glm::vec3(0.005f));
            renderer::appendBoxLines(overlay, boxMin - pad, boxMax + pad, kSelectionColor);
        }
        renderer().setOverlayLines(std::move(overlay));
    }

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
        ImGui::InvisibleButton("##viewport", size, ImGuiButtonFlags_MouseButtonLeft);
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

        const bool gizmoClicked = drawAxisGizmo(drawList, p0, p1, hovered);
        if (gizmoClicked) {
            m_lookBlocked = true; // this drag started on the gizmo, not a camera look
        }
        if (clicked && !m_lookBlocked) {
            selectUnderMouse(p0, size);
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
        const glm::vec3 tint = kPalette[m_objectCounter % kPalette.size()];
        const std::string name = asset->id == scene::kTriangleMesh ? "Actor " + std::to_string(m_objectCounter) : asset->name;
        const entt::entity entity = m_scene.createMesh(name, asset->id, glm::vec4(tint, 1.0f));
        ++m_objectCounter;
        if (asset->animated()) {
            renderer::enableAnimation(m_scene, renderer(), entity, asset->model);
        }
        placeEntity(entity, boundsMin, boundsMax, groundPoint);
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
        ImGui::TextDisabled("Animation: Inspector > Mesh > Animated");
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
                               : m_scene.registry().all_of<scene::AnimatorComponent>(entity) ? "[anim] "
                                                                                             : "";
            ImGui::PushID(static_cast<int>(entt::to_integral(entity)));
            if (ImGui::Selectable((std::string(icon) + name).c_str(), m_selected == entity)) {
                m_selected = entity;
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
        if (ImGui::InputText("Name", buffer.data(), buffer.size())) {
            name.name = buffer.data();
        }

        if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen)) {
            auto& transform = registry.get<scene::TransformComponent>(m_selected);
            ImGui::DragFloat3("Position", glm::value_ptr(transform.position), 0.05f);
            // Keep the angles the user typed; re-derive them only when the rotation was changed
            // elsewhere (e.g. by the camera controller), picking the solution nearest the old values.
            EulerCache& euler = m_eulerCache[m_selected];
            if (!euler.valid || euler.rotation != transform.rotation) {
                euler.degrees =
                    scene::eulerDegreesNear(transform.rotation, euler.valid ? euler.degrees : glm::vec3(0.0f));
                euler.rotation = transform.rotation;
                euler.valid = true;
            }
            if (ImGui::DragFloat3("Rotation", glm::value_ptr(euler.degrees), 0.5f)) {
                transform.rotation = glm::quat(glm::radians(euler.degrees));
                euler.rotation = transform.rotation;
            }
            if (!registry.all_of<scene::CameraComponent>(m_selected)) {
                ImGui::DragFloat3("Scale", glm::value_ptr(transform.scale), 0.05f);
            }
        }

        if (auto* camera = registry.try_get<scene::CameraComponent>(m_selected)) {
            if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
                f32 fovDeg = glm::degrees(camera->fovY);
                if (ImGui::SliderFloat("FOV", &fovDeg, 20.0f, 120.0f, "%.0f deg")) {
                    camera->fovY = glm::radians(fovDeg);
                }
                ImGui::DragFloatRange2("Near / Far", &camera->nearPlane, &camera->farPlane, 0.05f, 0.01f, 10000.0f);
            }
        }

        if (!registry.all_of<scene::CameraComponent>(m_selected)) {
            if (registry.all_of<scene::MeshRendererComponent>(m_selected)) {
                drawMeshComponent();
            }
            if (registry.all_of<scene::AnimatorComponent>(m_selected)) {
                drawAnimationComponent();
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

    // Mesh component: which asset the actor shows and whether it plays the asset's animation.
    void drawMeshComponent()
    {
        auto& registry = m_scene.registry();
        bool keep = true;
        if (ImGui::CollapsingHeader("Mesh", &keep, ImGuiTreeNodeFlags_DefaultOpen)) {
            auto& mesh = registry.get<scene::MeshRendererComponent>(m_selected);
            const auto* animator = registry.try_get<scene::AnimatorComponent>(m_selected);
            const editor::MeshAsset* current =
                animator ? m_drawer.findByModel(animator->model.get()) : m_drawer.findByMesh(mesh.mesh);

            if (const auto picked = m_drawer.assetCombo("Asset", current)) {
                setMeshAsset(*picked, animator != nullptr);
                current = m_drawer.asset(*picked);
            }
            tooltip("Pick an asset, or drag a tile from the Assets drawer onto this field");

            // Animated: only for assets with a skeleton.
            bool animated = registry.all_of<scene::AnimatorComponent>(m_selected);
            const bool canAnimate = current && current->animated();
            ImGui::BeginDisabled(!canAnimate);
            if (ImGui::Checkbox("Animated", &animated)) {
                if (animated) {
                    renderer::enableAnimation(m_scene, renderer(), m_selected, current->model);
                } else {
                    renderer::disableAnimation(m_scene, renderer(), m_selected);
                }
            }
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled | ImGuiHoveredFlags_DelayShort)) {
                ImGui::SetTooltip(canAnimate ? "Play the asset's animation clips"
                                             : "This asset has no skeleton or animation (ANIM assets only)");
            }
            if (canAnimate) {
                ImGui::SameLine();
                ImGui::TextDisabled("%zu clip(s), %zu joints", current->model->clips.size(),
                                    current->model->skeleton.jointCount());
            }

            auto& meshNow = registry.get<scene::MeshRendererComponent>(m_selected); // may have changed
            ImGui::ColorEdit3("Tint", glm::value_ptr(meshNow.tint));
            ImGui::Checkbox("Visible", &meshNow.visible);
        }
        if (!keep) {
            renderer::removeMesh(m_scene, renderer(), m_selected);
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

    // Animation component: playback controls right here, or popped out into the Animation window.
    void drawAnimationComponent()
    {
        if (!ImGui::CollapsingHeader("Animation", ImGuiTreeNodeFlags_DefaultOpen)) {
            return;
        }
        if (ImGui::SmallButton(m_showAnimationWindow ? "Focus window" : "Open in window")) {
            m_showAnimationWindow = true;
            ImGui::SetWindowFocus("Animation");
        }
        tooltip("Bigger timeline, clip list and joint hierarchy in a separate (dockable) window");
        m_animationPanel.drawInline(m_scene, m_selected);
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
            if (ImGui::MenuItem("Mesh", nullptr, false, !hasMesh)) {
                registry.emplace<scene::MeshRendererComponent>(
                    m_selected, scene::kTriangleMesh, glm::vec4(kPalette[m_objectCounter++ % kPalette.size()], 1.0f));
            }
            ImGui::TextDisabled("Animation: tick \"Animated\" on a Mesh with an ANIM asset");
            ImGui::EndPopup();
        }
    }

    static void tooltip(const char* text)
    {
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            ImGui::SetTooltip("%s", text);
        }
    }

    scene::Scene m_scene;
    entt::entity m_camera = entt::null;
    entt::entity m_selected = entt::null;
    FlyCameraController m_cameraController;
    bool m_layoutInitialized = false;
    bool m_showGrid = true;
    bool m_showSelectionBox = true;
    bool m_showAnimationWindow = false;
    bool m_lookBlocked = false;
    editor::AssetDrawer m_drawer;
    editor::AnimationPanel m_animationPanel;
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
