#pragma once

#include "kinetic/gfx/Device.h"

#include <glm/glm.hpp>

namespace kinetic::renderer {

// High-level renderer. For now: clears the screen and draws a test triangle.
// Next milestones: camera + depth buffer, meshes from glTF, debug lines (skeletons), skinning.
class Renderer {
public:
    bool init(gfx::Device& device);
    void shutdown();

    // Opens the main render pass on the swapchain (call between beginFrame / endFrame).
    [[nodiscard]] SDL_GPURenderPass* beginMainPass(const gfx::Frame& frame, const glm::vec4& clearColor);
    void endMainPass(SDL_GPURenderPass* pass);

    void drawTestTriangle(SDL_GPURenderPass* pass);

private:
    gfx::Device* m_device = nullptr;
    SDL_GPUGraphicsPipeline* m_trianglePipeline = nullptr;
};

} // namespace kinetic::renderer
