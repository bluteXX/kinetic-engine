// Editor: dockable tool UI. The 3D viewport moves into a panel once the renderer
// draws into an offscreen texture (milestone 2).

#include <kinetic/app/Application.h>
#include <kinetic/scene/Scene.h>

#include <SDL3/SDL_main.h>
#include <imgui.h>
#include <imgui_internal.h> // DockBuilder API

#include <glm/gtc/type_ptr.hpp>

using namespace kinetic;

class EditorApp final : public Application {
public:
    using Application::Application;

protected:
    bool onInit() override
    {
        m_scene.createEntity("Camera");
        m_scene.createEntity("Character");
        m_scene.createEntity("Ground");
        return true;
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
            ImGui::EndMainMenuBar();
        }

        drawHierarchy();
        drawInspector();

        ImGui::Begin("Stats");
        ImGui::Text("Backend: %s", device().backendName());
        ImGui::Text("%.1f FPS", static_cast<double>(ImGui::GetIO().Framerate));
        ImGui::End();
    }

    void onRender(RenderContext& ctx) override { ctx.renderer.drawTestTriangle(ctx.pass); }

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
        auto view = m_scene.registry().view<scene::NameComponent>();
        for (const entt::entity entity : view) {
            const auto& name = view.get<scene::NameComponent>(entity).name;
            if (ImGui::Selectable(name.c_str(), m_selected == entity)) {
                m_selected = entity;
            }
        }
        if (ImGui::Button("+ Entity")) {
            m_selected = m_scene.createEntity("Entity");
        }
        ImGui::End();
    }

    void drawInspector()
    {
        ImGui::Begin("Inspector");
        auto& registry = m_scene.registry();
        if (registry.valid(m_selected)) {
            auto& name = registry.get<scene::NameComponent>(m_selected);
            ImGui::Text("%s", name.name.c_str());
            auto& transform = registry.get<scene::TransformComponent>(m_selected);
            ImGui::DragFloat3("Position", glm::value_ptr(transform.position), 0.05f);
            ImGui::DragFloat3("Scale", glm::value_ptr(transform.scale), 0.05f);
        } else {
            ImGui::TextDisabled("Nothing selected");
        }
        ImGui::End();
    }

    scene::Scene m_scene;
    entt::entity m_selected = entt::null;
    bool m_layoutInitialized = false;
};

int main(int argc, char** argv)
{
    AppDesc desc;
    desc.window.title = "Kinetic Editor";
    Application::applyCommandLine(argc, argv, desc);
    EditorApp app(desc);
    return app.run();
}
