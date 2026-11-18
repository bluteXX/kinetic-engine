// Sandbox: playground for engine features.
// Right now: test triangle + ImGui + a glTF skeleton whose animation is evaluated on the CPU.

#include <kinetic/animation/PoseOps.h>
#include <kinetic/app/Application.h>
#include <kinetic/assets/GltfLoader.h>
#include <kinetic/core/Log.h>

#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_main.h>
#include <imgui.h>

#include <optional>
#include <vector>

using namespace kinetic;

class SandboxApp final : public Application {
public:
    using Application::Application;

protected:
    bool onInit() override
    {
        const std::string path = std::string(SDL_GetBasePath()) + "assets/models/RiggedFigure.glb";
        m_asset = assets::loadSkeletalAnimation(path);
        if (m_asset) {
            const usize n = m_asset->skeleton.jointCount();
            m_pose = m_asset->skeleton.restPose();
            m_model.resize(n);
        }
        return true;
    }

    void onFixedUpdate(f32 dt) override
    {
        if (!m_asset || m_asset->clips.empty()) {
            return;
        }
        if (m_playing) {
            m_time += dt * m_speed;
        }
        // Animation runs on the fixed tick: sample -> local pose -> model-space matrices.
        m_pose = m_asset->skeleton.restPose();
        m_asset->clips[m_clip].sample(m_time, true, m_pose);
        anim::localToModel(m_asset->skeleton, m_pose, m_model);
    }

    void onImGui() override
    {
        ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
        ImGui::Begin("Kinetic", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Backend: %s", device().backendName());
        ImGui::Text("%.1f FPS (%.2f ms)", ImGui::GetIO().Framerate, 1000.0f / ImGui::GetIO().Framerate);
        ImGui::Text("Frame %llu | fixed ticks %llu", static_cast<unsigned long long>(frameIndex()),
                    static_cast<unsigned long long>(fixedTickCount()));
        ImGui::ColorEdit3("Clear color", &clearColor.r);
        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(20, 160), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(420, 420), ImGuiCond_FirstUseEver);
        ImGui::Begin("Animation");
        if (!m_asset) {
            ImGui::TextDisabled("assets/models/RiggedFigure.glb not loaded");
        } else {
            const auto& clips = m_asset->clips;
            if (!clips.empty()) {
                const anim::AnimationClip& clip = clips[m_clip];
                ImGui::Text("Clip: %s (%.2fs)", clip.name().c_str(), static_cast<double>(clip.duration()));
                ImGui::Checkbox("Play", &m_playing);
                ImGui::SliderFloat("Speed", &m_speed, 0.0f, 2.0f);
            }
            ImGui::Separator();
            ImGui::Text("Joints: %zu (model-space positions)", m_asset->skeleton.jointCount());
            if (ImGui::BeginTable("joints", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY, ImVec2(0, 300))) {
                ImGui::TableSetupColumn("Joint");
                ImGui::TableSetupColumn("x");
                ImGui::TableSetupColumn("y");
                ImGui::TableSetupColumn("z");
                ImGui::TableHeadersRow();
                for (usize i = 0; i < m_model.size(); ++i) {
                    const glm::vec3 p = m_model[i][3];
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(m_asset->skeleton.names()[i].c_str());
                    ImGui::TableNextColumn();
                    ImGui::Text("%.3f", static_cast<double>(p.x));
                    ImGui::TableNextColumn();
                    ImGui::Text("%.3f", static_cast<double>(p.y));
                    ImGui::TableNextColumn();
                    ImGui::Text("%.3f", static_cast<double>(p.z));
                }
                ImGui::EndTable();
            }
        }
        ImGui::End();
    }

    void onRender(RenderContext& ctx) override { ctx.renderer.drawTestTriangle(ctx.pass); }

private:
    std::optional<assets::SkeletalAnimationAsset> m_asset;
    anim::Pose m_pose;
    std::vector<glm::mat4> m_model;
    usize m_clip = 0;
    f32 m_time = 0.0f;
    f32 m_speed = 1.0f;
    bool m_playing = true;
};

int main(int argc, char** argv)
{
    AppDesc desc;
    desc.window.title = "Kinetic Sandbox";
    Application::applyCommandLine(argc, argv, desc);
    SandboxApp app(desc);
    return app.run();
}
