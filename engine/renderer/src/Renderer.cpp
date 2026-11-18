#include "kinetic/renderer/Renderer.h"

#include "kinetic/core/Log.h"

namespace kinetic::renderer {

bool Renderer::init(gfx::Device& device)
{
    m_device = &device;

    SDL_GPUShader* vs = device.loadShader({.name = "triangle.vert", .stage = SDL_GPU_SHADERSTAGE_VERTEX});
    SDL_GPUShader* fs = device.loadShader({.name = "triangle.frag", .stage = SDL_GPU_SHADERSTAGE_FRAGMENT});
    if (!vs || !fs) {
        return false;
    }

    SDL_GPUColorTargetDescription colorTarget{};
    colorTarget.format = device.swapchainFormat();

    SDL_GPUGraphicsPipelineCreateInfo info{};
    info.vertex_shader = vs;
    info.fragment_shader = fs;
    info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    info.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;
    info.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE;
    info.target_info.num_color_targets = 1;
    info.target_info.color_target_descriptions = &colorTarget;

    m_trianglePipeline = SDL_CreateGPUGraphicsPipeline(device.handle(), &info);
    SDL_ReleaseGPUShader(device.handle(), vs);
    SDL_ReleaseGPUShader(device.handle(), fs);
    if (!m_trianglePipeline) {
        KN_ERROR("Failed to create triangle pipeline: {}", SDL_GetError());
        return false;
    }
    return true;
}

void Renderer::shutdown()
{
    if (m_device && m_trianglePipeline) {
        SDL_ReleaseGPUGraphicsPipeline(m_device->handle(), m_trianglePipeline);
        m_trianglePipeline = nullptr;
    }
    m_device = nullptr;
}

SDL_GPURenderPass* Renderer::beginMainPass(const gfx::Frame& frame, const glm::vec4& clearColor)
{
    SDL_GPUColorTargetInfo target{};
    target.texture = frame.swapchain;
    target.clear_color = SDL_FColor{clearColor.r, clearColor.g, clearColor.b, clearColor.a};
    target.load_op = SDL_GPU_LOADOP_CLEAR;
    target.store_op = SDL_GPU_STOREOP_STORE;
    return SDL_BeginGPURenderPass(frame.cmd, &target, 1, nullptr);
}

void Renderer::endMainPass(SDL_GPURenderPass* pass)
{
    SDL_EndGPURenderPass(pass);
}

void Renderer::drawTestTriangle(SDL_GPURenderPass* pass)
{
    SDL_BindGPUGraphicsPipeline(pass, m_trianglePipeline);
    SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0);
}

} // namespace kinetic::renderer
