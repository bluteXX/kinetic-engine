#pragma once

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_gpu.h>

namespace kinetic {
class Window;
}
namespace kinetic::gfx {
class Device;
}

namespace kinetic::ui {

// Dear ImGui integration on top of SDL3 + SDL_GPU. Docking is enabled.
class ImGuiLayer {
public:
    bool init(Window& window, gfx::Device& device);
    void shutdown();

    void processEvent(const SDL_Event& event);
    void beginFrame();
    void endFrame(); // ImGui::Render()

    // Uploads vertex/index data — must run before the render pass begins.
    void prepare(SDL_GPUCommandBuffer* cmd);
    void render(SDL_GPUCommandBuffer* cmd, SDL_GPURenderPass* pass);

private:
    bool m_initialized = false;
};

} // namespace kinetic::ui
