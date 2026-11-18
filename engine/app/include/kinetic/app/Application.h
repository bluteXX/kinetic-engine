#pragma once

#include "kinetic/gfx/Device.h"
#include "kinetic/platform/Window.h"
#include "kinetic/renderer/Renderer.h"
#include "kinetic/ui/ImGuiLayer.h"

namespace kinetic {

struct AppDesc {
    WindowDesc window;
    f32 fixedTimestep = 1.0f / 60.0f; // animation + physics tick
    bool gpuDebug =
#ifdef KINETIC_DEBUG
        true;
#else
        false;
#endif
    u64 maxFrames = 0; // > 0: quit after N frames (smoke tests / CI)
};

struct RenderContext {
    gfx::Frame& frame;
    SDL_GPURenderPass* pass; // scene pass: colour + depth; UI is drawn afterwards in its own pass
    renderer::Renderer& renderer;
    f32 interpolationAlpha; // 0..1 between the previous and current fixed tick
};

// Owns the window, GPU device, renderer and UI; runs the main loop:
//   events -> fixed updates (0..n per frame) -> variable update -> ImGui -> render
class Application {
public:
    explicit Application(AppDesc desc);
    virtual ~Application();
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    // Parses common CLI flags (e.g. --frames N) into the description.
    static void applyCommandLine(int argc, char** argv, AppDesc& desc);

    int run();
    void requestQuit() { m_running = false; }

protected:
    virtual bool onInit() { return true; }
    virtual void onShutdown() {}
    virtual void onFixedUpdate(f32 /*dt*/) {}
    virtual void onUpdate(f32 /*dt*/) {}
    virtual void onImGui() {}
    virtual void onRender(RenderContext& /*ctx*/) {}

    [[nodiscard]] Window& window() { return m_window; }
    [[nodiscard]] gfx::Device& device() { return m_device; }
    [[nodiscard]] renderer::Renderer& renderer() { return m_renderer; }
    [[nodiscard]] u64 frameIndex() const { return m_frameIndex; }
    [[nodiscard]] u64 fixedTickCount() const { return m_fixedTicks; }

    glm::vec4 clearColor{0.08f, 0.09f, 0.11f, 1.0f};

private:
    bool initCore();
    void shutdownCore();
    void pumpEvents();
    void renderFrame(f32 alpha);

    AppDesc m_desc;
    Window m_window;
    gfx::Device m_device;
    renderer::Renderer m_renderer;
    ui::ImGuiLayer m_imgui;
    bool m_running = false;
    u64 m_frameIndex = 0;
    u64 m_fixedTicks = 0;
};

} // namespace kinetic
