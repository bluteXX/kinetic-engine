#pragma once

#include "kinetic/gfx/Device.h"
#include "kinetic/scene/Scene.h"

#include <glm/glm.hpp>

#include <array>

namespace kinetic::renderer {

// GPU buffers of one built-in mesh (non-indexed triangle list).
struct GpuMesh {
    SDL_GPUBuffer* vertexBuffer = nullptr;
    u32 vertexCount = 0;
};

// High-level renderer.
//
// A frame has two render passes on the swapchain:
//   scene pass  — clears colour + depth, draws 3D content (depth tested)
//   overlay pass — loads the colour, draws UI on top (no depth target)
// ImGui's pipeline is created without a depth format, and Metal requires pipeline and pass
// attachments to match, so UI cannot share the depth-tested pass.
class Renderer {
public:
    bool init(gfx::Device& device);
    void shutdown();

    [[nodiscard]] SDL_GPURenderPass* beginScenePass(const gfx::Frame& frame, const glm::vec4& clearColor);
    [[nodiscard]] SDL_GPURenderPass* beginOverlayPass(const gfx::Frame& frame);
    void endPass(SDL_GPURenderPass* pass);

    // Draws every entity with a MeshRendererComponent, seen from the scene's active camera.
    // Does nothing if the scene has no camera.
    void drawScene(const gfx::Frame& frame, SDL_GPURenderPass* pass, const scene::Scene& scene);

    // Same, but with an explicit view-projection matrix.
    void drawScene(const gfx::Frame& frame, SDL_GPURenderPass* pass, const scene::Scene& scene,
                   const glm::mat4& viewProjection);

    // Hard-coded clip-space triangle (no buffers, no camera) — pipeline sanity check.
    void drawTestTriangle(SDL_GPURenderPass* pass);

private:
    bool createPipelines();
    bool createBuiltinMeshes();
    bool ensureDepthTarget(u32 width, u32 height);

    gfx::Device* m_device = nullptr;
    SDL_GPUGraphicsPipeline* m_trianglePipeline = nullptr;
    SDL_GPUGraphicsPipeline* m_meshPipeline = nullptr;
    std::array<GpuMesh, static_cast<usize>(scene::BuiltinMesh::Count)> m_meshes{};

    SDL_GPUTextureFormat m_depthFormat = SDL_GPU_TEXTUREFORMAT_INVALID;
    SDL_GPUTexture* m_depthTexture = nullptr;
    u32 m_depthWidth = 0;
    u32 m_depthHeight = 0;
};

} // namespace kinetic::renderer
