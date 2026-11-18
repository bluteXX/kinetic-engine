// Editor: dockable tool UI. The 3D scene is rendered offscreen and shown in the "Viewport" panel.
//
// Viewport controls:
//   hold left mouse button + move   — look around
//   W A S D (while holding LMB)      — move, Q / E — down / up, Shift — faster
//   mouse wheel                      — move forward / back
//   click an axis in the corner gizmo — view the scene from that axis
//
// Meshes: the Assets drawer (bottom) shows assets/meshes/. Drag a tile onto the Viewport to place
// it on the floor, or onto an object's Mesh field to swap its mesh. Dropping .fbx / .obj files on
// the window copies them into that folder.

#include "AssetDrawer.h"

#include <kinetic/app/Application.h>
#include <kinetic/core/Log.h>
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

// Folder the Assets drawer shows: the project's assets/meshes (dev builds), else next to the exe.
std::filesystem::path meshesFolder()
{
#ifdef KINETIC_PROJECT_ASSETS_DIR
    const std::filesystem::path project = std::filesystem::path(KINETIC_PROJECT_ASSETS_DIR) / "meshes";
    if (std::filesystem::is_directory(project)) {
        return project;
    }
#endif
    return std::filesystem::path(SDL_GetBasePath()) / "assets" / "meshes";
}

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
        m_drawer.init(renderer(), meshesFolder());
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
        const ImGuiID dockspace = ImGui::GetID("EditorLayout_v3"); // bump the name to reset saved layouts
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
                if (ImGui::MenuItem("Add Object")) {
                    addObject();
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        drawViewport();
        drawHierarchy();
        drawInspector();
        drawStats();
        m_drawer.draw();
        if (const auto mesh = m_drawer.takeSpawnRequest()) {
            spawnMesh(*mesh, std::nullopt);
        }
    }

    void onRender(RenderContext& ctx) override
    {
        renderer::SceneDrawOptions options;
        options.grid = m_showGrid;
        ctx.renderer.drawScene(ctx.frame.cmd, ctx.pass, m_scene, options);
    }

private:
    // Hierarchy + Stats on the left, Inspector on the right, Viewport in the centre, Assets below it.
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

        // Mesh dragged from the Assets drawer: place it on the floor under the cursor.
        if (ImGui::BeginDragDropTarget()) {
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(editor::AssetDrawer::kMeshPayload)) {
                scene::MeshId id = 0;
                std::memcpy(&id, payload->Data, sizeof(id));
                spawnMesh(id, groundPointUnderMouse(p0, size));
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

    // Where the ray from the camera through the mouse cursor hits the floor (y = 0), if it does.
    std::optional<glm::vec3> groundPointUnderMouse(const ImVec2& p0, const ImVec2& size) const
    {
        const entt::entity cameraEntity = m_scene.activeCamera();
        if (cameraEntity == entt::null) {
            return std::nullopt;
        }
        const auto& registry = m_scene.registry();
        const auto& camera = registry.get<scene::CameraComponent>(cameraEntity);
        const auto& transform = registry.get<scene::TransformComponent>(cameraEntity);
        const glm::mat4 inverseViewProjection =
            glm::inverse(camera.projection(size.x / size.y) * scene::CameraComponent::view(transform));

        const ImVec2 mouse = ImGui::GetIO().MousePos;
        const f32 ndcX = (mouse.x - p0.x) / size.x * 2.0f - 1.0f;
        const f32 ndcY = 1.0f - (mouse.y - p0.y) / size.y * 2.0f;
        glm::vec4 nearPoint = inverseViewProjection * glm::vec4(ndcX, ndcY, 0.0f, 1.0f);
        glm::vec4 farPoint = inverseViewProjection * glm::vec4(ndcX, ndcY, 1.0f, 1.0f);
        const glm::vec3 origin = glm::vec3(nearPoint) / nearPoint.w;
        const glm::vec3 direction = glm::normalize(glm::vec3(farPoint) / farPoint.w - origin);

        if (direction.y > -1e-4f) {
            return std::nullopt; // looking up or parallel to the floor
        }
        const f32 t = -origin.y / direction.y;
        if (t < 0.0f || t > 200.0f) {
            return std::nullopt;
        }
        return origin + direction * t;
    }

    // Adds an object with `mesh`. With a ground point it stands on the floor there; otherwise it
    // is placed in front of the camera at a distance that fits its size.
    void spawnMesh(scene::MeshId meshId, std::optional<glm::vec3> groundPoint)
    {
        const renderer::GpuMesh* mesh = renderer().mesh(meshId);
        if (!mesh) {
            return;
        }
        const glm::vec3 center = (mesh->boundsMin + mesh->boundsMax) * 0.5f;
        const f32 radius = std::max(glm::length(mesh->boundsMax - mesh->boundsMin) * 0.5f, 0.05f);

        const glm::vec3 tint = kPalette[m_objectCounter % kPalette.size()];
        const std::string name = meshId == scene::kTriangleMesh ? "Object " + std::to_string(m_objectCounter) : mesh->name;
        const entt::entity entity = m_scene.createMesh(name, meshId, glm::vec4(tint, 1.0f));
        ++m_objectCounter;

        auto& position = m_scene.registry().get<scene::TransformComponent>(entity).position;
        if (groundPoint) {
            position = *groundPoint - glm::vec3(center.x, mesh->boundsMin.y, center.z);
        } else if (const entt::entity camera = m_scene.activeCamera(); camera != entt::null) {
            const auto& cam = m_scene.registry().get<scene::TransformComponent>(camera);
            position = cam.position + cam.rotation * glm::vec3(0.0f, 0.0f, -std::max(2.5f, radius * 2.5f)) - center;
        }
        m_selected = entity;
    }

    void addObject() { spawnMesh(scene::kTriangleMesh, std::nullopt); }

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
        ImGui::TextDisabled("LMB + mouse: look");
        ImGui::TextDisabled("LMB + WASD/QE: move (Shift: fast)");
        ImGui::TextDisabled("Wheel: forward / back");
        ImGui::TextDisabled("Corner gizmo: click an axis");
        ImGui::TextDisabled("Assets: drag a mesh onto the Viewport");
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
            const char* icon = m_scene.registry().all_of<scene::CameraComponent>(entity) ? "[cam] " : "";
            ImGui::PushID(static_cast<int>(entt::to_integral(entity)));
            if (ImGui::Selectable((std::string(icon) + name).c_str(), m_selected == entity)) {
                m_selected = entity;
            }
            ImGui::PopID();
        }

        ImGui::Spacing();
        if (ImGui::Button("+ Add Object", ImVec2(-FLT_MIN, 0))) {
            addObject();
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
            if (auto* mesh = registry.try_get<scene::MeshRendererComponent>(m_selected)) {
                bool keep = true;
                if (ImGui::CollapsingHeader("Mesh", &keep, ImGuiTreeNodeFlags_DefaultOpen)) {
                    meshCombo("Mesh", mesh->mesh);
                    ImGui::ColorEdit3("Tint", glm::value_ptr(mesh->tint));
                }
                if (!keep) {
                    registry.remove<scene::MeshRendererComponent>(m_selected);
                }
            } else if (ImGui::Button("+ Add Mesh", ImVec2(-FLT_MIN, 0))) {
                registry.emplace<scene::MeshRendererComponent>(
                    m_selected, scene::kTriangleMesh, glm::vec4(kPalette[m_objectCounter++ % kPalette.size()], 1.0f));
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        if (m_selected == m_camera) {
            ImGui::TextDisabled("The scene camera cannot be deleted.");
        } else if (ImGui::Button("Delete", ImVec2(-FLT_MIN, 0))) {
            m_eulerCache.erase(m_selected);
            m_scene.destroyEntity(m_selected);
            m_selected = entt::null;
        }
        ImGui::End();
    }

    void meshCombo(const char* label, scene::MeshId& meshId)
    {
        const auto& meshes = renderer().meshes();
        const char* preview = meshId < meshes.size() ? meshes[meshId].name.c_str() : "<missing>";
        if (ImGui::BeginCombo(label, preview)) {
            for (scene::MeshId id = 0; id < meshes.size(); ++id) {
                ImGui::PushID(static_cast<int>(id));
                if (ImGui::Selectable(meshes[id].name.c_str(), id == meshId)) {
                    meshId = id;
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(%u tris)", meshes[id].indexCount / 3);
                ImGui::PopID();
            }
            ImGui::EndCombo();
        }
        // Accept a mesh dragged from the Assets drawer.
        if (ImGui::BeginDragDropTarget()) {
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(editor::AssetDrawer::kMeshPayload)) {
                std::memcpy(&meshId, payload->Data, sizeof(meshId));
            }
            ImGui::EndDragDropTarget();
        }
    }

    scene::Scene m_scene;
    entt::entity m_camera = entt::null;
    entt::entity m_selected = entt::null;
    FlyCameraController m_cameraController;
    bool m_layoutInitialized = false;
    bool m_showGrid = true;
    bool m_lookBlocked = false;
    editor::AssetDrawer m_drawer;
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
