#pragma once

#include "kinetic/core/MeshData.h"
#include "kinetic/gfx/Device.h"
#include "kinetic/scene/Scene.h"

#include <glm/glm.hpp>

#include <span>
#include <string>
#include <vector>

namespace kinetic::renderer {

// One uploaded mesh in the renderer's library. Its index is the scene::MeshId.
struct GpuMesh {
    std::string name;
    SDL_GPUBuffer* vertexBuffer = nullptr;
    SDL_GPUBuffer* indexBuffer = nullptr;
    u32 indexCount = 0;
    u32 vertexCount = 0;
    glm::vec3 boundsMin{0.0f};
    glm::vec3 boundsMax{0.0f};
    // Dynamic meshes belong to one object (e.g. a skinned character) and get new vertices every
    // frame; they are hidden from mesh pickers. `staging` is their persistent upload buffer.
    bool dynamic = false;
    SDL_GPUTransferBuffer* staging = nullptr;
    bool uploadPending = false;
    // CPU copy of the geometry as uploaded (picking). Dynamic meshes keep their initial vertices;
    // their current shape lives in the AnimatorComponent.
    std::vector<glm::vec3> positions;
    std::vector<u32> indices;

    [[nodiscard]] bool alive() const { return vertexBuffer != nullptr; }
};

enum class MeshUsage : u8 {
    Static,  // uploaded once
    Dynamic, // vertices replaced with updateMeshVertices()
};

// Coloured line vertex for debug overlays (two per segment).
struct DebugVertex {
    glm::vec3 position;
    glm::vec3 color;
};

struct SceneDrawOptions {
    bool grid = true;     // floor grid on XZ with the X (red) and Z (blue) axes
    u32 overlayLayer = 0; // which setOverlayLines() layer to draw on top
    f32 aspect = 0.0f;    // camera aspect; 0 = the main scene target's
};

// Offscreen colour + depth target. The main scene target is owned by the Renderer; apps can
// create more (e.g. an editor preview viewport) with Renderer::resizeTarget / releaseTarget.
struct RenderTarget {
    SDL_GPUTexture* color = nullptr;
    SDL_GPUTexture* depth = nullptr;
    u32 width = 0;
    u32 height = 0;

    [[nodiscard]] bool valid() const { return color != nullptr; }
    [[nodiscard]] f32 aspect() const { return height > 0 ? static_cast<f32>(width) / static_cast<f32>(height) : 1.0f; }
};

// High-level renderer.
//
// 3D content is rendered into an offscreen *scene target* (colour + depth) whose size the app
// chooses: the editor shows it in a "Viewport" panel, the sandbox blits it to the window.
// The swapchain itself only gets the UI (overlay pass).
class Renderer {
public:
    // Format of the scene colour texture (sampled by ImGui, blitted to the swapchain).
    static constexpr SDL_GPUTextureFormat kSceneColorFormat = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;

    bool init(gfx::Device& device);
    void shutdown();

    // --- Mesh library -----------------------------------------------------------------------
    // Uploads a mesh and returns its id (kInvalidMesh on failure). Mesh 0 is the built-in triangle.
    // Adding a mesh may reallocate the library: don't hold GpuMesh pointers across this call.
    static constexpr scene::MeshId kInvalidMesh = ~scene::MeshId{0};
    scene::MeshId addMesh(std::string name, const MeshData& data, MeshUsage usage = MeshUsage::Static);
    // Replaces the vertices of a dynamic mesh (same count as when it was added). The data is
    // staged now and copied to the GPU at the start of the next frame (uploadPendingData).
    bool updateMeshVertices(scene::MeshId id, std::span<const glm::vec3> positions, std::span<const glm::vec3> normals);
    // Releases a mesh's GPU buffers. The id stays reserved (mesh(id) returns null afterwards).
    void removeMesh(scene::MeshId id);
    [[nodiscard]] const std::vector<GpuMesh>& meshes() const { return m_meshes; }
    [[nodiscard]] const GpuMesh* mesh(scene::MeshId id) const
    {
        return id < m_meshes.size() && m_meshes[id].alive() ? &m_meshes[id] : nullptr;
    }

    // --- Debug overlay ------------------------------------------------------------------------
    // World-space lines drawn on top of the scene (no depth test) by drawScene calls whose
    // options.overlayLayer is `layer`, e.g. a skeleton. Replaces the layer's previous set.
    void setOverlayLines(std::vector<DebugVertex> lines, u32 layer = 0);

    // Copies staged data (dynamic meshes, overlay lines) to the GPU. Must be called once per
    // frame on the frame's command buffer, before the scene pass (Application does this).
    void uploadPendingData(SDL_GPUCommandBuffer* cmd);

    // --- Scene target -----------------------------------------------------------------------
    // (Re)creates the scene target if the size changed. Safe to call mid-frame: SDL_GPU defers
    // releasing the old textures until the GPU is done with them.
    bool resizeSceneTarget(u32 width, u32 height) { return resizeTarget(m_sceneTarget, width, height); }
    [[nodiscard]] SDL_GPUTexture* sceneColorTexture() const { return m_sceneTarget.color; }
    [[nodiscard]] u32 sceneWidth() const { return m_sceneTarget.width; }
    [[nodiscard]] u32 sceneHeight() const { return m_sceneTarget.height; }
    [[nodiscard]] f32 sceneAspect() const { return m_sceneTarget.aspect(); }

    // Any offscreen target: (re)create at a size (no-op if unchanged), release, render into.
    bool resizeTarget(RenderTarget& target, u32 width, u32 height);
    void releaseTarget(RenderTarget& target);
    [[nodiscard]] SDL_GPURenderPass* beginTargetPass(SDL_GPUCommandBuffer* cmd, const RenderTarget& target,
                                                     const glm::vec4& clearColor);

    // --- Passes -----------------------------------------------------------------------------
    // Scene pass: clears and renders into the scene target. Returns null if there is no target.
    [[nodiscard]] SDL_GPURenderPass* beginScenePass(SDL_GPUCommandBuffer* cmd, const glm::vec4& clearColor);
    // Copies the scene target onto the whole swapchain (call outside any render pass).
    void blitSceneToSwapchain(const gfx::Frame& frame);
    // Overlay pass on the swapchain for UI. `clear` = true when nothing was blitted before.
    [[nodiscard]] SDL_GPURenderPass* beginOverlayPass(const gfx::Frame& frame, bool clear, const glm::vec4& clearColor);
    void endPass(SDL_GPURenderPass* pass);

    // --- Drawing (inside the scene pass) ------------------------------------------------------
    // Draws every entity with a MeshRendererComponent from the scene's active camera.
    void drawScene(SDL_GPUCommandBuffer* cmd, SDL_GPURenderPass* pass, const scene::Scene& scene,
                   const SceneDrawOptions& options = {});
    void drawGrid(SDL_GPUCommandBuffer* cmd, SDL_GPURenderPass* pass, const glm::mat4& viewProjection);

    // Hard-coded clip-space triangle (no buffers, no camera) — pipeline sanity check.
    void drawTestTriangle(SDL_GPURenderPass* pass);

private:
    bool createPipelines();
    bool createGrid();

    gfx::Device* m_device = nullptr;
    SDL_GPUGraphicsPipeline* m_trianglePipeline = nullptr;
    SDL_GPUGraphicsPipeline* m_meshPipeline = nullptr;
    SDL_GPUGraphicsPipeline* m_linePipeline = nullptr;
    SDL_GPUGraphicsPipeline* m_overlayLinePipeline = nullptr; // lines without depth test
    std::vector<GpuMesh> m_meshes;

    struct OverlayLayer {
        std::vector<DebugVertex> lines;
        bool dirty = false;
        SDL_GPUBuffer* buffer = nullptr;
        SDL_GPUTransferBuffer* staging = nullptr;
        u32 capacity = 0;    // vertices
        u32 vertexCount = 0; // vertices uploaded and drawn
        bool uploadPending = false;
    };
    std::vector<OverlayLayer> m_overlays;

    SDL_GPUBuffer* m_gridBuffer = nullptr;
    u32 m_gridVertexCount = 0;

    SDL_GPUTextureFormat m_depthFormat = SDL_GPU_TEXTUREFORMAT_INVALID;
    RenderTarget m_sceneTarget;
};

} // namespace kinetic::renderer
