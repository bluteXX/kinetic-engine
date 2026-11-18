#include "kinetic/renderer/Renderer.h"

#include "kinetic/core/Log.h"

#include <cstring>
#include <span>

namespace kinetic::renderer {

namespace {

struct MeshVertex {
    glm::vec3 position;
    glm::vec3 color;
};

// Built-in triangle in the XY plane, facing +Z, 1 unit tall. White so the tint shows unchanged.
constexpr std::array<MeshVertex, 3> kTriangleVertices = {{
    {{0.0f, 0.6f, 0.0f}, {1.0f, 1.0f, 1.0f}},
    {{-0.6f, -0.4f, 0.0f}, {1.0f, 1.0f, 1.0f}},
    {{0.6f, -0.4f, 0.0f}, {1.0f, 1.0f, 1.0f}},
}};

SDL_GPUTextureFormat pickDepthFormat(SDL_GPUDevice* device)
{
    for (const SDL_GPUTextureFormat format : {SDL_GPU_TEXTUREFORMAT_D32_FLOAT, SDL_GPU_TEXTUREFORMAT_D24_UNORM,
                                              SDL_GPU_TEXTUREFORMAT_D16_UNORM}) {
        if (SDL_GPUTextureSupportsFormat(device, format, SDL_GPU_TEXTURETYPE_2D,
                                         SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET)) {
            return format;
        }
    }
    return SDL_GPU_TEXTUREFORMAT_INVALID;
}

// Creates a GPU vertex buffer and uploads `bytes` into it (blocking, init-time only).
SDL_GPUBuffer* uploadVertexBuffer(SDL_GPUDevice* device, std::span<const std::byte> bytes)
{
    const auto size = static_cast<Uint32>(bytes.size());

    SDL_GPUBufferCreateInfo bufferInfo{};
    bufferInfo.usage = SDL_GPU_BUFFERUSAGE_VERTEX;
    bufferInfo.size = size;
    SDL_GPUBuffer* buffer = SDL_CreateGPUBuffer(device, &bufferInfo);
    if (!buffer) {
        KN_ERROR("SDL_CreateGPUBuffer failed: {}", SDL_GetError());
        return nullptr;
    }

    SDL_GPUTransferBufferCreateInfo transferInfo{};
    transferInfo.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    transferInfo.size = size;
    SDL_GPUTransferBuffer* transfer = SDL_CreateGPUTransferBuffer(device, &transferInfo);
    if (!transfer) {
        KN_ERROR("SDL_CreateGPUTransferBuffer failed: {}", SDL_GetError());
        SDL_ReleaseGPUBuffer(device, buffer);
        return nullptr;
    }
    void* mapped = SDL_MapGPUTransferBuffer(device, transfer, false);
    std::memcpy(mapped, bytes.data(), bytes.size());
    SDL_UnmapGPUTransferBuffer(device, transfer);

    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(device);
    SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(cmd);
    const SDL_GPUTransferBufferLocation src{transfer, 0};
    const SDL_GPUBufferRegion dst{buffer, 0, size};
    SDL_UploadToGPUBuffer(copy, &src, &dst, false);
    SDL_EndGPUCopyPass(copy);
    SDL_SubmitGPUCommandBuffer(cmd);
    SDL_ReleaseGPUTransferBuffer(device, transfer); // released once the upload has finished
    return buffer;
}

} // namespace

bool Renderer::init(gfx::Device& device)
{
    m_device = &device;
    m_depthFormat = pickDepthFormat(device.handle());
    if (m_depthFormat == SDL_GPU_TEXTUREFORMAT_INVALID) {
        KN_ERROR("No supported depth format");
        return false;
    }
    return createPipelines() && createBuiltinMeshes();
}

bool Renderer::createPipelines()
{
    SDL_GPUDevice* gpu = m_device->handle();

    SDL_GPUColorTargetDescription colorTarget{};
    colorTarget.format = m_device->swapchainFormat();

    // Both pipelines render in the scene pass, so both declare its depth target.
    SDL_GPUGraphicsPipelineCreateInfo info{};
    info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    info.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;
    info.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE; // triangles are visible from both sides
    info.target_info.num_color_targets = 1;
    info.target_info.color_target_descriptions = &colorTarget;
    info.target_info.has_depth_stencil_target = true;
    info.target_info.depth_stencil_format = m_depthFormat;

    // Test triangle: no vertex input, no depth test.
    {
        SDL_GPUShader* vs = m_device->loadShader({.name = "triangle.vert", .stage = SDL_GPU_SHADERSTAGE_VERTEX});
        SDL_GPUShader* fs = m_device->loadShader({.name = "triangle.frag", .stage = SDL_GPU_SHADERSTAGE_FRAGMENT});
        if (!vs || !fs) {
            return false;
        }
        SDL_GPUGraphicsPipelineCreateInfo triangle = info;
        triangle.vertex_shader = vs;
        triangle.fragment_shader = fs;
        m_trianglePipeline = SDL_CreateGPUGraphicsPipeline(gpu, &triangle);
        SDL_ReleaseGPUShader(gpu, vs);
        SDL_ReleaseGPUShader(gpu, fs);
        if (!m_trianglePipeline) {
            KN_ERROR("Failed to create triangle pipeline: {}", SDL_GetError());
            return false;
        }
    }

    // Mesh pipeline: position + colour vertices, MVP (vertex) and tint (fragment) uniforms, depth tested.
    {
        SDL_GPUShader* vs = m_device->loadShader(
            {.name = "mesh.vert", .stage = SDL_GPU_SHADERSTAGE_VERTEX, .uniformBuffers = 1});
        SDL_GPUShader* fs = m_device->loadShader(
            {.name = "mesh.frag", .stage = SDL_GPU_SHADERSTAGE_FRAGMENT, .uniformBuffers = 1});
        if (!vs || !fs) {
            return false;
        }

        SDL_GPUVertexBufferDescription vertexBuffer{};
        vertexBuffer.slot = 0;
        vertexBuffer.pitch = sizeof(MeshVertex);
        vertexBuffer.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;

        const std::array<SDL_GPUVertexAttribute, 2> attributes = {{
            {0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, static_cast<Uint32>(offsetof(MeshVertex, position))},
            {1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, static_cast<Uint32>(offsetof(MeshVertex, color))},
        }};

        SDL_GPUGraphicsPipelineCreateInfo mesh = info;
        mesh.vertex_shader = vs;
        mesh.fragment_shader = fs;
        mesh.vertex_input_state.vertex_buffer_descriptions = &vertexBuffer;
        mesh.vertex_input_state.num_vertex_buffers = 1;
        mesh.vertex_input_state.vertex_attributes = attributes.data();
        mesh.vertex_input_state.num_vertex_attributes = static_cast<Uint32>(attributes.size());
        mesh.depth_stencil_state.enable_depth_test = true;
        mesh.depth_stencil_state.enable_depth_write = true;
        mesh.depth_stencil_state.compare_op = SDL_GPU_COMPAREOP_LESS;

        m_meshPipeline = SDL_CreateGPUGraphicsPipeline(gpu, &mesh);
        SDL_ReleaseGPUShader(gpu, vs);
        SDL_ReleaseGPUShader(gpu, fs);
        if (!m_meshPipeline) {
            KN_ERROR("Failed to create mesh pipeline: {}", SDL_GetError());
            return false;
        }
    }
    return true;
}

bool Renderer::createBuiltinMeshes()
{
    GpuMesh& triangle = m_meshes[static_cast<usize>(scene::BuiltinMesh::Triangle)];
    triangle.vertexBuffer = uploadVertexBuffer(m_device->handle(), std::as_bytes(std::span(kTriangleVertices)));
    triangle.vertexCount = static_cast<u32>(kTriangleVertices.size());
    return triangle.vertexBuffer != nullptr;
}

bool Renderer::ensureDepthTarget(u32 width, u32 height)
{
    if (m_depthTexture && m_depthWidth == width && m_depthHeight == height) {
        return true;
    }
    if (m_depthTexture) {
        SDL_ReleaseGPUTexture(m_device->handle(), m_depthTexture);
        m_depthTexture = nullptr;
    }

    SDL_GPUTextureCreateInfo info{};
    info.type = SDL_GPU_TEXTURETYPE_2D;
    info.format = m_depthFormat;
    info.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;
    info.width = width;
    info.height = height;
    info.layer_count_or_depth = 1;
    info.num_levels = 1;
    m_depthTexture = SDL_CreateGPUTexture(m_device->handle(), &info);
    if (!m_depthTexture) {
        KN_ERROR("Failed to create depth texture {}x{}: {}", width, height, SDL_GetError());
        return false;
    }
    m_depthWidth = width;
    m_depthHeight = height;
    return true;
}

void Renderer::shutdown()
{
    if (!m_device) {
        return;
    }
    SDL_GPUDevice* gpu = m_device->handle();
    for (GpuMesh& mesh : m_meshes) {
        if (mesh.vertexBuffer) {
            SDL_ReleaseGPUBuffer(gpu, mesh.vertexBuffer);
        }
        mesh = {};
    }
    if (m_depthTexture) {
        SDL_ReleaseGPUTexture(gpu, m_depthTexture);
        m_depthTexture = nullptr;
    }
    if (m_meshPipeline) {
        SDL_ReleaseGPUGraphicsPipeline(gpu, m_meshPipeline);
        m_meshPipeline = nullptr;
    }
    if (m_trianglePipeline) {
        SDL_ReleaseGPUGraphicsPipeline(gpu, m_trianglePipeline);
        m_trianglePipeline = nullptr;
    }
    m_device = nullptr;
}

SDL_GPURenderPass* Renderer::beginScenePass(const gfx::Frame& frame, const glm::vec4& clearColor)
{
    SDL_GPUColorTargetInfo color{};
    color.texture = frame.swapchain;
    color.clear_color = SDL_FColor{clearColor.r, clearColor.g, clearColor.b, clearColor.a};
    color.load_op = SDL_GPU_LOADOP_CLEAR;
    color.store_op = SDL_GPU_STOREOP_STORE;

    if (!ensureDepthTarget(frame.width, frame.height)) {
        return nullptr;
    }
    SDL_GPUDepthStencilTargetInfo depth{};
    depth.texture = m_depthTexture;
    depth.clear_depth = 1.0f;
    depth.load_op = SDL_GPU_LOADOP_CLEAR;
    depth.store_op = SDL_GPU_STOREOP_DONT_CARE;
    depth.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
    depth.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;

    return SDL_BeginGPURenderPass(frame.cmd, &color, 1, &depth);
}

SDL_GPURenderPass* Renderer::beginOverlayPass(const gfx::Frame& frame)
{
    SDL_GPUColorTargetInfo color{};
    color.texture = frame.swapchain;
    color.load_op = SDL_GPU_LOADOP_LOAD;
    color.store_op = SDL_GPU_STOREOP_STORE;
    return SDL_BeginGPURenderPass(frame.cmd, &color, 1, nullptr);
}

void Renderer::endPass(SDL_GPURenderPass* pass)
{
    if (pass) {
        SDL_EndGPURenderPass(pass);
    }
}

void Renderer::drawScene(const gfx::Frame& frame, SDL_GPURenderPass* pass, const scene::Scene& scene)
{
    const entt::entity cameraEntity = scene.activeCamera();
    if (cameraEntity == entt::null || frame.height == 0) {
        return;
    }
    const auto& registry = scene.registry();
    const auto& camera = registry.get<scene::CameraComponent>(cameraEntity);
    const auto& transform = registry.get<scene::TransformComponent>(cameraEntity);
    const f32 aspect = static_cast<f32>(frame.width) / static_cast<f32>(frame.height);
    drawScene(frame, pass, scene, camera.projection(aspect) * scene::CameraComponent::view(transform));
}

void Renderer::drawScene(const gfx::Frame& frame, SDL_GPURenderPass* pass, const scene::Scene& scene,
                         const glm::mat4& viewProjection)
{
    if (!pass) {
        return;
    }
    SDL_BindGPUGraphicsPipeline(pass, m_meshPipeline);

    auto view = scene.registry().view<const scene::TransformComponent, const scene::MeshRendererComponent>();
    for (const entt::entity entity : view) {
        const auto& [transform, renderer] = view.get(entity);
        if (renderer.mesh >= scene::BuiltinMesh::Count) {
            continue;
        }
        const GpuMesh& mesh = m_meshes[static_cast<usize>(renderer.mesh)];

        const glm::mat4 mvp = viewProjection * transform.matrix();
        SDL_PushGPUVertexUniformData(frame.cmd, 0, &mvp, sizeof(mvp));
        SDL_PushGPUFragmentUniformData(frame.cmd, 0, &renderer.tint, sizeof(renderer.tint));

        const SDL_GPUBufferBinding binding{mesh.vertexBuffer, 0};
        SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
        SDL_DrawGPUPrimitives(pass, mesh.vertexCount, 1, 0, 0);
    }
}

void Renderer::drawTestTriangle(SDL_GPURenderPass* pass)
{
    if (!pass) {
        return;
    }
    SDL_BindGPUGraphicsPipeline(pass, m_trianglePipeline);
    SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0);
}

} // namespace kinetic::renderer
