#include "kinetic/ui/ImGuiLayer.h"

#include "kinetic/gfx/Device.h"
#include "kinetic/platform/Window.h"

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlgpu3.h>

namespace kinetic::ui {

bool ImGuiLayer::init(Window& window, gfx::Device& device)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_DockingEnable;
    ImGui::StyleColorsDark();

    if (!ImGui_ImplSDL3_InitForSDLGPU(window.handle())) {
        return false;
    }
    ImGui_ImplSDLGPU3_InitInfo info{};
    info.Device = device.handle();
    info.ColorTargetFormat = device.swapchainFormat();
    info.MSAASamples = SDL_GPU_SAMPLECOUNT_1;
    if (!ImGui_ImplSDLGPU3_Init(&info)) {
        return false;
    }
    m_initialized = true;
    return true;
}

void ImGuiLayer::shutdown()
{
    if (!m_initialized) {
        return;
    }
    ImGui_ImplSDLGPU3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    m_initialized = false;
}

void ImGuiLayer::processEvent(const SDL_Event& event)
{
    ImGui_ImplSDL3_ProcessEvent(&event);
}

void ImGuiLayer::beginFrame()
{
    ImGui_ImplSDLGPU3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
}

void ImGuiLayer::endFrame()
{
    ImGui::Render();
}

void ImGuiLayer::prepare(SDL_GPUCommandBuffer* cmd)
{
    ImGui_ImplSDLGPU3_PrepareDrawData(ImGui::GetDrawData(), cmd);
}

void ImGuiLayer::render(SDL_GPUCommandBuffer* cmd, SDL_GPURenderPass* pass)
{
    ImGui_ImplSDLGPU3_RenderDrawData(ImGui::GetDrawData(), cmd, pass);
}

} // namespace kinetic::ui
