#include "kinetic/gfx/Device.h"

#include "kinetic/core/Log.h"
#include "kinetic/platform/Window.h"

#include <SDL3/SDL.h>

namespace kinetic::gfx {

namespace {

constexpr SDL_GPUShaderFormat kSupportedFormats = SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_MSL;

#if defined(__APPLE__)
constexpr const char* kPreferredDriver = "metal"; // don't pick Vulkan/MoltenVK even if the Vulkan SDK is installed
#else
constexpr const char* kPreferredDriver = nullptr; // SDL decides (Vulkan, since we ship no DXIL yet)
#endif

} // namespace

Device::~Device()
{
    shutdown();
}

bool Device::init(Window& window, bool debug)
{
    m_device = SDL_CreateGPUDevice(kSupportedFormats, debug, kPreferredDriver);
    if (!m_device) {
        KN_ERROR("SDL_CreateGPUDevice failed: {}", SDL_GetError());
        return false;
    }
    if (!SDL_ClaimWindowForGPUDevice(m_device, window.handle())) {
        KN_ERROR("SDL_ClaimWindowForGPUDevice failed: {}", SDL_GetError());
        SDL_DestroyGPUDevice(m_device);
        m_device = nullptr;
        return false;
    }
    SDL_SetGPUSwapchainParameters(m_device, window.handle(), SDL_GPU_SWAPCHAINCOMPOSITION_SDR,
                                  SDL_GPU_PRESENTMODE_VSYNC);
    m_window = &window;
    KN_INFO("GPU device created (backend: {})", backendName());
    return true;
}

void Device::shutdown()
{
    if (!m_device) {
        return;
    }
    SDL_WaitForGPUIdle(m_device);
    if (m_window) {
        SDL_ReleaseWindowFromGPUDevice(m_device, m_window->handle());
        m_window = nullptr;
    }
    SDL_DestroyGPUDevice(m_device);
    m_device = nullptr;
}

SDL_GPUTextureFormat Device::swapchainFormat() const
{
    return SDL_GetGPUSwapchainTextureFormat(m_device, m_window->handle());
}

const char* Device::backendName() const
{
    return m_device ? SDL_GetGPUDeviceDriver(m_device) : "none";
}

SDL_GPUShader* Device::loadShader(const ShaderDesc& desc) const
{
    const SDL_GPUShaderFormat available = SDL_GetGPUShaderFormats(m_device);
    SDL_GPUShaderFormat format = SDL_GPU_SHADERFORMAT_INVALID;
    const char* extension = nullptr;
    const char* entrypoint = nullptr;
    if (available & SDL_GPU_SHADERFORMAT_SPIRV) {
        format = SDL_GPU_SHADERFORMAT_SPIRV;
        extension = ".spv";
        entrypoint = "main";
    } else if (available & SDL_GPU_SHADERFORMAT_MSL) {
        format = SDL_GPU_SHADERFORMAT_MSL;
        extension = ".msl";
        entrypoint = "main0"; // SPIRV-Cross renames main -> main0 (main is reserved in MSL)
    } else {
        KN_ERROR("No supported shader format for backend {}", backendName());
        return nullptr;
    }

    const std::string path = std::string(SDL_GetBasePath()) + "shaders/" + desc.name + extension;
    size_t size = 0;
    void* code = SDL_LoadFile(path.c_str(), &size);
    if (!code) {
        KN_ERROR("Failed to read shader '{}': {}", path, SDL_GetError());
        return nullptr;
    }

    SDL_GPUShaderCreateInfo info{};
    info.code = static_cast<const Uint8*>(code);
    info.code_size = size;
    info.entrypoint = entrypoint;
    info.format = format;
    info.stage = desc.stage;
    info.num_samplers = desc.samplers;
    info.num_uniform_buffers = desc.uniformBuffers;
    info.num_storage_buffers = desc.storageBuffers;
    info.num_storage_textures = desc.storageTextures;

    SDL_GPUShader* shader = SDL_CreateGPUShader(m_device, &info);
    SDL_free(code);
    if (!shader) {
        KN_ERROR("SDL_CreateGPUShader failed for '{}': {}", path, SDL_GetError());
    }
    return shader;
}

Frame Device::beginFrame()
{
    Frame frame;
    frame.cmd = SDL_AcquireGPUCommandBuffer(m_device);
    if (!frame.cmd) {
        KN_ERROR("SDL_AcquireGPUCommandBuffer failed: {}", SDL_GetError());
        return frame;
    }
    Uint32 w = 0;
    Uint32 h = 0;
    if (!SDL_WaitAndAcquireGPUSwapchainTexture(frame.cmd, m_window->handle(), &frame.swapchain, &w, &h)) {
        KN_ERROR("SDL_WaitAndAcquireGPUSwapchainTexture failed: {}", SDL_GetError());
    }
    frame.width = w;
    frame.height = h;
    return frame;
}

void Device::endFrame(Frame& frame)
{
    if (frame.cmd) {
        SDL_SubmitGPUCommandBuffer(frame.cmd);
    }
    frame = {};
}

void Device::waitIdle() const
{
    if (m_device) {
        SDL_WaitForGPUIdle(m_device);
    }
}

} // namespace kinetic::gfx
