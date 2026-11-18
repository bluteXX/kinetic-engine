// Editor: dockable tool UI over a 3D scene.
// The scene is rendered full-window behind the panels; it moves into its own viewport panel once
// the renderer draws into an offscreen texture.
//
// Camera controls (when the mouse is not over a panel):
//   hold left mouse button + move   — look around
//   W A S D                          — move (while holding LMB), Q / E — down / up, Shift — faster
//   mouse wheel                      — move forward / back

#include <kinetic/app/Application.h>
#include <kinetic/scene/Scene.h>

#include <SDL3/SDL_main.h>
#include <imgui.h>
#include <imgui_internal.h> // DockBuilder API

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <cstdio>
#include <unordered_map>
#include <vector>

using namespace kinetic;

namespace {

// Hover-free fly camera that edits the camera entity's TransformComponent directly.
struct FlyCameraController {
    f32 moveSpeed = 3.0f;        // m/s
    f32 lookSensitivity = 0.003f; // radians per pixel
    bool looking = false;

    void update(scene::TransformComponent& transform, f32 dt)
    {
        const ImGuiIO& io = ImGui::GetIO();

        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse) {
            looking = true;
        }
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            looking = false;
        }

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

        if (!io.WantCaptureMouse && io.MouseWheel != 0.0f) {
            transform.position += transform.rotation * glm::vec3(0, 0, -io.MouseWheel * 0.5f);
        }
    }
};

} // namespace

class EditorApp final : public Application {
public:
    using Application::Application;

protected:
    bool onInit() override
    {
        m_camera = m_scene.createCamera("Camera", {0.0f, 0.4f, 2.5f}, {0.0f, 0.0f, 0.0f});
        m_selected = m_scene.createMesh("Triangle", scene::BuiltinMesh::Triangle, {0.95f, 0.55f, 0.25f, 1.0f});
        return true;
    }

    void onUpdate(f32 dt) override
    {
        auto& registry = m_scene.registry();
        const entt::entity camera = m_scene.activeCamera();
        if (camera != entt::null) {
            m_cameraController.update(registry.get<scene::TransformComponent>(camera), dt);
        }
    }

    void onImGui() override
    {
        const ImGuiID dockspace = ImGui::GetID("EditorDockSpace");
        if (!m_layoutInitialized) {
            buildDefaultLayout(dockspace);
            m_layoutInitialized = true;
        }
        ImGui::DockSpaceOverViewport(dockspace, nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Quit")) {
                    requestQuit();
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Scene")) {
                if (ImGui::MenuItem("Add Object...")) {
                    m_openAddObject = true;
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        drawHierarchy();
        drawInspector();
        drawAddObjectPopup();

        ImGui::Begin("Stats");
        ImGui::Text("Backend: %s", device().backendName());
        ImGui::Text("%.1f FPS", static_cast<double>(ImGui::GetIO().Framerate));
        ImGui::Separator();
        ImGui::TextDisabled("LMB + mouse: look");
        ImGui::TextDisabled("LMB + WASD/QE: move (Shift: fast)");
        ImGui::TextDisabled("Wheel: forward / back");
        ImGui::SliderFloat("Camera speed", &m_cameraController.moveSpeed, 0.5f, 20.0f);
        ImGui::End();
    }

    void onRender(RenderContext& ctx) override { ctx.renderer.drawScene(ctx.frame, ctx.pass, m_scene); }

private:
    // Hierarchy + Stats on the left, Inspector on the right, the centre stays free for the viewport.
    // Only applied when there is no saved layout in imgui.ini yet.
    static void buildDefaultLayout(ImGuiID dockspace)
    {
        const ImGuiDockNode* existing = ImGui::DockBuilderGetNode(dockspace);
        if (existing && existing->IsSplitNode()) {
            return;
        }
        ImGui::DockBuilderRemoveNode(dockspace);
        ImGui::DockBuilderAddNode(dockspace, static_cast<ImGuiDockNodeFlags>(ImGuiDockNodeFlags_DockSpace) |
                                                 ImGuiDockNodeFlags_PassthruCentralNode);
        ImGui::DockBuilderSetNodeSize(dockspace, ImGui::GetMainViewport()->WorkSize);

        ImGuiID center = dockspace;
        ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.2f, nullptr, &center);
        const ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.25f, nullptr, &center);
        const ImGuiID leftBottom = ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.3f, nullptr, &left);

        ImGui::DockBuilderDockWindow("Hierarchy", left);
        ImGui::DockBuilderDockWindow("Stats", leftBottom);
        ImGui::DockBuilderDockWindow("Inspector", right);
        ImGui::DockBuilderFinish(dockspace);
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
            m_openAddObject = true;
        }
        ImGui::End();
    }

    void drawAddObjectPopup()
    {
        if (m_openAddObject) {
            ImGui::OpenPopup("Add Object");
            std::snprintf(m_newName.data(), m_newName.size(), "Object %d", m_objectCounter);
            m_openAddObject = false;
        }

        ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        if (ImGui::BeginPopupModal("Add Object", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::InputText("Name", m_newName.data(), m_newName.size());
            meshCombo("Mesh", m_newMesh);
            ImGui::ColorEdit3("Tint", glm::value_ptr(m_newTint));

            if (ImGui::Button("Create", ImVec2(120, 0))) {
                const entt::entity entity = m_scene.createMesh(m_newName.data(), m_newMesh, m_newTint);
                // Place it a little in front of the camera so it is visible right away.
                const entt::entity camera = m_scene.activeCamera();
                if (camera != entt::null) {
                    const auto& cam = m_scene.registry().get<scene::TransformComponent>(camera);
                    m_scene.registry().get<scene::TransformComponent>(entity).position =
                        cam.position + cam.rotation * glm::vec3(0.0f, 0.0f, -2.5f);
                }
                m_selected = entity;
                ++m_objectCounter;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
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
                euler.degrees = scene::eulerDegreesNear(transform.rotation, euler.valid ? euler.degrees : glm::vec3(0.0f));
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

        if (auto* mesh = registry.try_get<scene::MeshRendererComponent>(m_selected)) {
            if (ImGui::CollapsingHeader("Mesh Renderer", ImGuiTreeNodeFlags_DefaultOpen)) {
                meshCombo("Mesh", mesh->mesh);
                ImGui::ColorEdit3("Tint", glm::value_ptr(mesh->tint));
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

    static void meshCombo(const char* label, scene::BuiltinMesh& mesh)
    {
        if (ImGui::BeginCombo(label, scene::builtinMeshName(mesh))) {
            for (usize i = 0; i < scene::kBuiltinMeshNames.size(); ++i) {
                const auto candidate = static_cast<scene::BuiltinMesh>(i);
                if (ImGui::Selectable(scene::kBuiltinMeshNames[i], candidate == mesh)) {
                    mesh = candidate;
                }
            }
            ImGui::EndCombo();
        }
    }

    scene::Scene m_scene;
    entt::entity m_camera = entt::null;
    entt::entity m_selected = entt::null;
    FlyCameraController m_cameraController;

    // Euler angles shown in the inspector, per entity (see drawInspector).
    struct EulerCache {
        glm::vec3 degrees{0.0f};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        bool valid = false;
    };
    std::unordered_map<entt::entity, EulerCache> m_eulerCache;
    bool m_layoutInitialized = false;

    // "Add Object" dialog state
    bool m_openAddObject = false;
    std::array<char, 64> m_newName{};
    scene::BuiltinMesh m_newMesh = scene::BuiltinMesh::Triangle;
    glm::vec4 m_newTint{0.3f, 0.7f, 1.0f, 1.0f};
    int m_objectCounter = 1;
};

int main(int argc, char** argv)
{
    AppDesc desc;
    desc.window.title = "Kinetic Editor";
    Application::applyCommandLine(argc, argv, desc);
    EditorApp app(desc);
    return app.run();
}
