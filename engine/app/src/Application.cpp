#include "kinetic/app/Application.h"

#include "kinetic/core/Clock.h"
#include "kinetic/core/Log.h"

#include <SDL3/SDL.h>

#include <algorithm>
#include <cstdlib>
#include <string_view>

namespace kinetic {

Application::Application(AppDesc desc)
    : m_desc(std::move(desc))
{
}

Application::~Application() = default;

void Application::applyCommandLine(int argc, char** argv, AppDesc& desc)
{
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--frames" && i + 1 < argc) {
            desc.maxFrames = std::strtoull(argv[++i], nullptr, 10);
        } else if (arg == "--gpu-debug") {
            desc.gpuDebug = true;
        }
    }
}

bool Application::initCore()
{
    log::init();
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
        KN_ERROR("SDL_Init failed: {}", SDL_GetError());
        return false;
    }
    return m_window.create(m_desc.window) && m_device.init(m_window, m_desc.gpuDebug) &&
           m_renderer.init(m_device) && m_imgui.init(m_window, m_device);
}

void Application::shutdownCore()
{
    m_device.waitIdle();
    m_imgui.shutdown();
    m_renderer.shutdown();
    m_device.shutdown();
    m_window.destroy();
    SDL_Quit();
}

int Application::run()
{
    if (!initCore() || !onInit()) {
        shutdownCore();
        return EXIT_FAILURE;
    }

    // Fixed-timestep loop ("Fix Your Timestep"): simulation advances in constant steps,
    // rendering happens once per frame and gets the leftover fraction for interpolation.
    constexpr f64 kMaxFrameTime = 0.25; // avoid the spiral of death after a stall / breakpoint
    const f64 step = m_desc.fixedTimestep;
    Clock clock;
    f64 previous = clock.elapsedSeconds();
    f64 accumulator = 0.0;

    m_running = true;
    while (m_running) {
        const f64 now = clock.elapsedSeconds();
        const f64 frameTime = std::min(now - previous, kMaxFrameTime);
        previous = now;

        pumpEvents();
        if (!m_running) {
            break;
        }
        if (m_window.isMinimized()) {
            SDL_Delay(10);
            continue;
        }

        accumulator += frameTime;
        while (accumulator >= step) {
            onFixedUpdate(static_cast<f32>(step));
            accumulator -= step;
            ++m_fixedTicks;
        }
        onUpdate(static_cast<f32>(frameTime));

        m_imgui.beginFrame();
        onImGui();
        m_imgui.endFrame();

        renderFrame(static_cast<f32>(accumulator / step));

        ++m_frameIndex;
        if (m_desc.maxFrames > 0 && m_frameIndex >= m_desc.maxFrames) {
            m_running = false;
        }
    }

    onShutdown();
    shutdownCore();
    return EXIT_SUCCESS;
}

void Application::pumpEvents()
{
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        m_imgui.processEvent(event);
        onEvent(event);
        if (event.type == SDL_EVENT_QUIT ||
            (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event.window.windowID == SDL_GetWindowID(m_window.handle()))) {
            m_running = false;
        }
    }
}

void Application::renderFrame(f32 alpha)
{
    gfx::Frame frame = m_device.beginFrame();
    if (frame.cmd && frame.swapchain) {
        m_imgui.prepare(frame.cmd);              // copy pass, must precede the render passes
        m_renderer.uploadPendingData(frame.cmd); // skinned meshes, overlay lines
        onRenderOffscreen(frame, m_renderer);

        if (m_desc.presentScene) {
            m_renderer.resizeSceneTarget(frame.width, frame.height);
        }
        // Scene -> offscreen target (the editor samples it in its Viewport panel).
        if (SDL_GPURenderPass* scenePass = m_renderer.beginScenePass(frame.cmd, clearColor)) {
            RenderContext ctx{frame, scenePass, m_renderer, alpha};
            onRender(ctx);
            m_renderer.endPass(scenePass);
        }
        if (m_desc.presentScene) {
            m_renderer.blitSceneToSwapchain(frame);
        }

        // UI -> swapchain.
        SDL_GPURenderPass* overlayPass = m_renderer.beginOverlayPass(frame, !m_desc.presentScene, clearColor);
        m_imgui.render(frame.cmd, overlayPass);
        m_renderer.endPass(overlayPass);
    }
    m_device.endFrame(frame);
}

} // namespace kinetic
