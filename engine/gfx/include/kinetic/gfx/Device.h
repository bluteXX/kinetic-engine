#pragma once

#include "kinetic/core/Types.h"

#include <SDL3/SDL_gpu.h>

#include <string>

namespace kinetic {
class Window;
}

namespace kinetic::gfx {

struct ShaderDesc {
    std::string name;            // file name without extension, e.g. "triangle.vert"
    SDL_GPUShaderStage stage = SDL_GPU_SHADERSTAGE_VERTEX;
    u32 samplers = 0;
    u32 uniformBuffers = 0;
    u32 storageBuffers = 0;
    u32 storageTextures = 0;
};

// Everything recorded for one frame. `swapchain` is null when there is nothing to draw into
// (minimized window, swapchain being recreated) — skip rendering but still submit.
struct Frame {
    SDL_GPUCommandBuffer* cmd = nullptr;
    SDL_GPUTexture* swapchain = nullptr;
    u32 width = 0;
    u32 height = 0;
};

// Owns the SDL_GPU device and the window's swapchain.
// Backend: Metal on macOS, Vulkan on Windows/Linux (D3D12 needs DXIL shaders - TODO via SDL_shadercross).
class Device {
public:
    Device() = default;
    ~Device();
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    bool init(Window& window, bool debug);
    void shutdown();

    [[nodiscard]] SDL_GPUDevice* handle() const { return m_device; }
    [[nodiscard]] SDL_GPUTextureFormat swapchainFormat() const;
    [[nodiscard]] const char* backendName() const;

    // Loads shaders/<name>.spv or shaders/<name>.msl (next to the executable), depending on backend.
    [[nodiscard]] SDL_GPUShader* loadShader(const ShaderDesc& desc) const;

    [[nodiscard]] Frame beginFrame();
    void endFrame(Frame& frame);

    void waitIdle() const;

private:
    SDL_GPUDevice* m_device = nullptr;
    Window* m_window = nullptr;
};

} // namespace kinetic::gfx
