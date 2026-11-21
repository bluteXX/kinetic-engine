#include "kinetic/renderer/Renderer.h"

#include "kinetic/core/Log.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <span>

namespace kinetic::renderer {

namespace {

// Stream 0 (stream 1 holds the uvs).
struct MeshVertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec4 tangent;
};

using LineVertex = DebugVertex;

struct MeshDrawUniforms {
    glm::mat4 mvp;
    glm::mat4 normalMatrix;
    glm::mat4 model;
};

struct MaterialUniforms {
    glm::vec4 tint;
    glm::vec4 keyLightDir;
    glm::vec4 cameraDir;
    glm::vec4 params; // x = normal strength, y = normal map green sign (+1 OpenGL, -1 DirectX)
};

constexpr glm::vec4 kDefaultTangent{1.0f, 0.0f, 0.0f, 1.0f};

const glm::vec3 kKeyLightDir = glm::normalize(glm::vec3(0.4f, 0.8f, 0.45f));

// Floor grid: 1 m cells over ±kGridHalfExtent, a brighter line every 5 m, X axis red, Z axis blue.
constexpr int kGridHalfExtent = 20;
constexpr glm::vec3 kGridMinor{0.22f, 0.23f, 0.26f};
constexpr glm::vec3 kGridMajor{0.34f, 0.35f, 0.39f};
constexpr glm::vec3 kAxisX{0.85f, 0.26f, 0.26f};
constexpr glm::vec3 kAxisZ{0.26f, 0.45f, 0.90f};

MeshData makeTriangle()
{
    MeshData mesh;
    mesh.positions = {{0.0f, 0.6f, 0.0f}, {-0.6f, -0.4f, 0.0f}, {0.6f, -0.4f, 0.0f}};
    mesh.normals = {{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}};
    mesh.indices = {0, 1, 2};
    return mesh;
}

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

// Creates a GPU buffer and uploads `bytes` into it. Uses its own command buffer, submitted
// immediately, so it is ordered before any frame command buffer submitted afterwards.
SDL_GPUBuffer* uploadBuffer(SDL_GPUDevice* device, SDL_GPUBufferUsageFlags usage, std::span<const std::byte> bytes)
{
    const auto size = static_cast<Uint32>(bytes.size());

    SDL_GPUBufferCreateInfo bufferInfo{};
    bufferInfo.usage = usage;
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
    SDL_ReleaseGPUTransferBuffer(device, transfer); // freed once the upload has finished
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
    if (!createPipelines() || !createGrid() || !createTextureDefaults()) {
        return false;
    }
    return addMesh("Triangle", makeTriangle()) == scene::kTriangleMesh;
}

bool Renderer::createPipelines()
{
    SDL_GPUDevice* gpu = m_device->handle();

    SDL_GPUColorTargetDescription colorTarget{};
    colorTarget.format = kSceneColorFormat;

    // All three pipelines render into the scene target (colour + depth).
    SDL_GPUGraphicsPipelineCreateInfo base{};
    base.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    base.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;
    base.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE; // two-sided: imported meshes are often not closed
    base.rasterizer_state.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;
    base.target_info.num_color_targets = 1;
    base.target_info.color_target_descriptions = &colorTarget;
    base.target_info.has_depth_stencil_target = true;
    base.target_info.depth_stencil_format = m_depthFormat;

    const auto build = [&](const char* name, SDL_GPUGraphicsPipelineCreateInfo info, u32 vsUniforms,
                           u32 fsUniforms, u32 fsSamplers) -> SDL_GPUGraphicsPipeline* {
        const std::string base_ = name;
        SDL_GPUShader* vs = m_device->loadShader(
            {.name = base_ + ".vert", .stage = SDL_GPU_SHADERSTAGE_VERTEX, .uniformBuffers = vsUniforms});
        SDL_GPUShader* fs = m_device->loadShader(
            {.name = base_ + ".frag", .stage = SDL_GPU_SHADERSTAGE_FRAGMENT, .samplers = fsSamplers,
             .uniformBuffers = fsUniforms});
        SDL_GPUGraphicsPipeline* pipeline = nullptr;
        if (vs && fs) {
            info.vertex_shader = vs;
            info.fragment_shader = fs;
            pipeline = SDL_CreateGPUGraphicsPipeline(gpu, &info);
            if (!pipeline) {
                KN_ERROR("Failed to create '{}' pipeline: {}", name, SDL_GetError());
            }
        }
        if (vs) SDL_ReleaseGPUShader(gpu, vs);
        if (fs) SDL_ReleaseGPUShader(gpu, fs);
        return pipeline;
    };

    // Test triangle: no vertex input, no depth test.
    m_trianglePipeline = build("triangle", base, 0, 0, 0);

    // Lines: two float3 attributes in one buffer.
    const auto twoFloat3 = [](Uint32 pitch, Uint32 offset0, Uint32 offset1,
                              SDL_GPUVertexBufferDescription& buffer,
                              std::array<SDL_GPUVertexAttribute, 2>& attributes) {
        buffer = {};
        buffer.slot = 0;
        buffer.pitch = pitch;
        buffer.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
        attributes = {{{0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, offset0},
                       {1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, offset1}}};
    };

    // Meshes: stream 0 = position, normal, tangent; stream 1 = uv.
    std::array<SDL_GPUVertexBufferDescription, 2> meshBuffers{};
    meshBuffers[0].slot = 0;
    meshBuffers[0].pitch = sizeof(MeshVertex);
    meshBuffers[0].input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    meshBuffers[1].slot = 1;
    meshBuffers[1].pitch = sizeof(glm::vec2);
    meshBuffers[1].input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    const std::array<SDL_GPUVertexAttribute, 4> meshAttributes = {{
        {0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, offsetof(MeshVertex, position)},
        {1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, offsetof(MeshVertex, normal)},
        {2, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(MeshVertex, tangent)},
        {3, 1, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2, 0},
    }};
    SDL_GPUGraphicsPipelineCreateInfo mesh = base;
    mesh.vertex_input_state.vertex_buffer_descriptions = meshBuffers.data();
    mesh.vertex_input_state.num_vertex_buffers = static_cast<Uint32>(meshBuffers.size());
    mesh.vertex_input_state.vertex_attributes = meshAttributes.data();
    mesh.vertex_input_state.num_vertex_attributes = static_cast<Uint32>(meshAttributes.size());
    mesh.depth_stencil_state.enable_depth_test = true;
    mesh.depth_stencil_state.enable_depth_write = true;
    mesh.depth_stencil_state.compare_op = SDL_GPU_COMPAREOP_LESS;
    m_meshPipeline = build("mesh", mesh, 1, 1, 2);

    SDL_GPUVertexBufferDescription lineBuffer{};
    std::array<SDL_GPUVertexAttribute, 2> lineAttributes{};
    twoFloat3(sizeof(LineVertex), offsetof(LineVertex, position), offsetof(LineVertex, color), lineBuffer,
              lineAttributes);
    SDL_GPUGraphicsPipelineCreateInfo line = mesh;
    line.primitive_type = SDL_GPU_PRIMITIVETYPE_LINELIST;
    line.vertex_input_state.vertex_buffer_descriptions = &lineBuffer;
    line.vertex_input_state.num_vertex_buffers = 1;
    line.vertex_input_state.vertex_attributes = lineAttributes.data();
    line.vertex_input_state.num_vertex_attributes = static_cast<Uint32>(lineAttributes.size());
    line.depth_stencil_state.compare_op = SDL_GPU_COMPAREOP_LESS_OR_EQUAL;
    m_linePipeline = build("line", line, 1, 0, 0);

    SDL_GPUGraphicsPipelineCreateInfo overlay = line;
    overlay.depth_stencil_state.enable_depth_test = false;
    overlay.depth_stencil_state.enable_depth_write = false;
    m_overlayLinePipeline = build("line", overlay, 1, 0, 0);

    return m_trianglePipeline && m_meshPipeline && m_linePipeline && m_overlayLinePipeline;
}

bool Renderer::createGrid()
{
    std::vector<LineVertex> lines;
    const auto extent = static_cast<f32>(kGridHalfExtent);
    for (int i = -kGridHalfExtent; i <= kGridHalfExtent; ++i) {
        const auto k = static_cast<f32>(i);
        const glm::vec3 color = (i % 5 == 0) ? kGridMajor : kGridMinor;
        // Line parallel to X at z = k (the one through the origin is the X axis) ...
        const glm::vec3 alongX = i == 0 ? kAxisX : color;
        lines.push_back({{-extent, 0.0f, k}, alongX});
        lines.push_back({{extent, 0.0f, k}, alongX});
        // ... and parallel to Z at x = k (the one through the origin is the Z axis).
        const glm::vec3 alongZ = i == 0 ? kAxisZ : color;
        lines.push_back({{k, 0.0f, -extent}, alongZ});
        lines.push_back({{k, 0.0f, extent}, alongZ});
    }
    m_gridBuffer = uploadBuffer(m_device->handle(), SDL_GPU_BUFFERUSAGE_VERTEX, std::as_bytes(std::span(lines)));
    m_gridVertexCount = static_cast<u32>(lines.size());
    return m_gridBuffer != nullptr;
}

scene::MeshId Renderer::addMesh(std::string name, const MeshData& data, MeshUsage usage)
{
    if (data.empty() || data.positions.size() != data.normals.size()) {
        KN_ERROR("addMesh('{}'): empty mesh or mismatched positions/normals", name);
        return kInvalidMesh;
    }
    std::vector<MeshVertex> vertices(data.positions.size());
    const bool tangents = data.hasTangents();
    for (usize i = 0; i < vertices.size(); ++i) {
        vertices[i] = {data.positions[i], data.normals[i], tangents ? data.tangents[i] : kDefaultTangent};
    }
    const std::vector<glm::vec2> uvs =
        data.hasUvs() ? data.uvs : std::vector<glm::vec2>(vertices.size(), glm::vec2(0.0f));

    GpuMesh mesh;
    mesh.name = std::move(name);
    mesh.vertexBuffer =
        uploadBuffer(m_device->handle(), SDL_GPU_BUFFERUSAGE_VERTEX, std::as_bytes(std::span(vertices)));
    mesh.uvBuffer = uploadBuffer(m_device->handle(), SDL_GPU_BUFFERUSAGE_VERTEX, std::as_bytes(std::span(uvs)));
    mesh.indexBuffer =
        uploadBuffer(m_device->handle(), SDL_GPU_BUFFERUSAGE_INDEX, std::as_bytes(std::span(data.indices)));
    mesh.indexCount = static_cast<u32>(data.indices.size());
    mesh.vertexCount = static_cast<u32>(vertices.size());
    mesh.positions = data.positions;
    mesh.indices = data.indices;
    mesh.boundsMin = data.boundsMin();
    mesh.boundsMax = data.boundsMax();
    if (usage == MeshUsage::Dynamic) {
        mesh.dynamic = true;
        SDL_GPUTransferBufferCreateInfo stagingInfo{};
        stagingInfo.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
        stagingInfo.size = static_cast<Uint32>(vertices.size() * sizeof(MeshVertex));
        mesh.staging = SDL_CreateGPUTransferBuffer(m_device->handle(), &stagingInfo);
    }
    if (!mesh.vertexBuffer || !mesh.uvBuffer || !mesh.indexBuffer || (mesh.dynamic && !mesh.staging)) {
        if (mesh.vertexBuffer) SDL_ReleaseGPUBuffer(m_device->handle(), mesh.vertexBuffer);
        if (mesh.uvBuffer) SDL_ReleaseGPUBuffer(m_device->handle(), mesh.uvBuffer);
        if (mesh.indexBuffer) SDL_ReleaseGPUBuffer(m_device->handle(), mesh.indexBuffer);
        if (mesh.staging) SDL_ReleaseGPUTransferBuffer(m_device->handle(), mesh.staging);
        return kInvalidMesh;
    }
    m_meshes.push_back(std::move(mesh));
    return static_cast<scene::MeshId>(m_meshes.size() - 1);
}

bool Renderer::updateMeshVertices(scene::MeshId id, std::span<const glm::vec3> positions,
                                  std::span<const glm::vec3> normals, std::span<const glm::vec4> tangents)
{
    if (id >= m_meshes.size()) {
        return false;
    }
    GpuMesh& mesh = m_meshes[id];
    if (!mesh.alive() || !mesh.dynamic || positions.size() != mesh.vertexCount || normals.size() != mesh.vertexCount) {
        return false;
    }
    // cycle = true: if the GPU is still reading the previous data, SDL hands out a fresh buffer.
    auto* mapped = static_cast<MeshVertex*>(SDL_MapGPUTransferBuffer(m_device->handle(), mesh.staging, true));
    if (!mapped) {
        return false;
    }
    const bool hasTangents = tangents.size() == positions.size();
    for (usize i = 0; i < positions.size(); ++i) {
        mapped[i] = {positions[i], normals[i], hasTangents ? tangents[i] : kDefaultTangent};
    }
    SDL_UnmapGPUTransferBuffer(m_device->handle(), mesh.staging);
    mesh.uploadPending = true;
    return true;
}

void Renderer::removeMesh(scene::MeshId id)
{
    if (id >= m_meshes.size() || id == scene::kTriangleMesh) {
        return;
    }
    GpuMesh& mesh = m_meshes[id];
    SDL_GPUDevice* gpu = m_device->handle();
    // SDL_GPU defers the actual release until command buffers using them have finished.
    if (mesh.vertexBuffer) SDL_ReleaseGPUBuffer(gpu, mesh.vertexBuffer);
    if (mesh.uvBuffer) SDL_ReleaseGPUBuffer(gpu, mesh.uvBuffer);
    if (mesh.indexBuffer) SDL_ReleaseGPUBuffer(gpu, mesh.indexBuffer);
    if (mesh.staging) SDL_ReleaseGPUTransferBuffer(gpu, mesh.staging);
    mesh.vertexBuffer = nullptr;
    mesh.uvBuffer = nullptr;
    mesh.indexBuffer = nullptr;
    mesh.staging = nullptr;
    mesh.uploadPending = false;
    mesh.positions = {};
    mesh.indices = {};
}

bool Renderer::createTextureDefaults()
{
    SDL_GPUSamplerCreateInfo info{};
    info.min_filter = SDL_GPU_FILTER_LINEAR;
    info.mag_filter = SDL_GPU_FILTER_LINEAR;
    info.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR;
    info.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    info.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    info.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    info.max_lod = 1000.0f;
    info.enable_anisotropy = true;
    info.max_anisotropy = 8.0f;
    m_sampler = SDL_CreateGPUSampler(m_device->handle(), &info);
    if (!m_sampler) {
        KN_ERROR("SDL_CreateGPUSampler failed: {}", SDL_GetError());
        return false;
    }
    const auto solid = [](u8 r, u8 g, u8 b) {
        ImageData image;
        image.width = 4;
        image.height = 4;
        image.rgba.resize(4 * 4 * 4);
        for (usize i = 0; i < 16; ++i) {
            image.rgba[i * 4 + 0] = r;
            image.rgba[i * 4 + 1] = g;
            image.rgba[i * 4 + 2] = b;
            image.rgba[i * 4 + 3] = 255;
        }
        return image;
    };
    return addTexture("<white>", solid(255, 255, 255)) == scene::kWhiteTexture &&
           addTexture("<flat normal>", solid(128, 128, 255)) == scene::kFlatNormalTexture;
}

scene::TextureId Renderer::addTexture(std::string name, const ImageData& image)
{
    if (image.empty()) {
        KN_ERROR("addTexture('{}'): empty image", name);
        return kInvalidTexture;
    }
    SDL_GPUDevice* gpu = m_device->handle();
    u32 levels = 1;
    for (u32 size = std::max(image.width, image.height); size > 1; size >>= 1) {
        ++levels;
    }
    SDL_GPUTextureCreateInfo info{};
    info.type = SDL_GPU_TEXTURETYPE_2D;
    info.format = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;
    // Colour target usage is required to generate the mip chain on the GPU.
    info.usage = SDL_GPU_TEXTUREUSAGE_SAMPLER | SDL_GPU_TEXTUREUSAGE_COLOR_TARGET;
    info.width = image.width;
    info.height = image.height;
    info.layer_count_or_depth = 1;
    info.num_levels = levels;
    SDL_GPUTexture* texture = SDL_CreateGPUTexture(gpu, &info);
    if (!texture) {
        KN_ERROR("addTexture('{}'): {}", name, SDL_GetError());
        return kInvalidTexture;
    }

    const auto bytes = static_cast<Uint32>(usize{image.width} * image.height * 4);
    SDL_GPUTransferBufferCreateInfo transferInfo{};
    transferInfo.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    transferInfo.size = bytes;
    SDL_GPUTransferBuffer* transfer = SDL_CreateGPUTransferBuffer(gpu, &transferInfo);
    if (!transfer) {
        KN_ERROR("addTexture('{}'): {}", name, SDL_GetError());
        SDL_ReleaseGPUTexture(gpu, texture);
        return kInvalidTexture;
    }
    void* mapped = SDL_MapGPUTransferBuffer(gpu, transfer, false);
    std::memcpy(mapped, image.rgba.data(), bytes);
    SDL_UnmapGPUTransferBuffer(gpu, transfer);

    // Own command buffer, submitted now: ordered before any frame that samples the texture.
    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(gpu);
    SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(cmd);
    SDL_GPUTextureTransferInfo src{};
    src.transfer_buffer = transfer;
    src.pixels_per_row = image.width;
    src.rows_per_layer = image.height;
    SDL_GPUTextureRegion dst{};
    dst.texture = texture;
    dst.w = image.width;
    dst.h = image.height;
    dst.d = 1;
    SDL_UploadToGPUTexture(copy, &src, &dst, false);
    SDL_EndGPUCopyPass(copy);
    if (levels > 1) {
        SDL_GenerateMipmapsForGPUTexture(cmd, texture);
    }
    SDL_SubmitGPUCommandBuffer(cmd);
    SDL_ReleaseGPUTransferBuffer(gpu, transfer);

    m_textures.push_back({std::move(name), texture, image.width, image.height});
    return static_cast<scene::TextureId>(m_textures.size() - 1);
}

scene::TextureId Renderer::findTexture(std::string_view name) const
{
    for (usize i = 0; i < m_textures.size(); ++i) {
        if (m_textures[i].alive() && m_textures[i].name == name) {
            return static_cast<scene::TextureId>(i);
        }
    }
    return kInvalidTexture;
}

void Renderer::setOverlayLines(std::vector<DebugVertex> lines, u32 layer)
{
    if (layer >= m_overlays.size()) {
        m_overlays.resize(layer + 1);
    }
    m_overlays[layer].lines = std::move(lines);
    m_overlays[layer].dirty = true;
}

void Renderer::uploadPendingData(SDL_GPUCommandBuffer* cmd)
{
    SDL_GPUDevice* gpu = m_device->handle();

    // Grow each layer's buffers if needed and stage its lines.
    bool uploadOverlay = false;
    for (OverlayLayer& layer : m_overlays) {
        if (!layer.dirty) {
            continue;
        }
        layer.dirty = false;
        layer.vertexCount = 0;
        const auto count = static_cast<u32>(layer.lines.size());
        if (count > layer.capacity) {
            if (layer.buffer) SDL_ReleaseGPUBuffer(gpu, layer.buffer);
            if (layer.staging) SDL_ReleaseGPUTransferBuffer(gpu, layer.staging);
            layer.capacity = std::max(count, layer.capacity * 2);
            const auto bytes = static_cast<Uint32>(layer.capacity * sizeof(DebugVertex));
            SDL_GPUBufferCreateInfo bufferInfo{};
            bufferInfo.usage = SDL_GPU_BUFFERUSAGE_VERTEX;
            bufferInfo.size = bytes;
            layer.buffer = SDL_CreateGPUBuffer(gpu, &bufferInfo);
            SDL_GPUTransferBufferCreateInfo stagingInfo{};
            stagingInfo.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
            stagingInfo.size = bytes;
            layer.staging = SDL_CreateGPUTransferBuffer(gpu, &stagingInfo);
            if (!layer.buffer || !layer.staging) {
                KN_ERROR("Overlay line buffers: {}", SDL_GetError());
                layer.capacity = 0;
            }
        }
        if (count > 0 && count <= layer.capacity) {
            if (void* mapped = SDL_MapGPUTransferBuffer(gpu, layer.staging, true)) {
                std::memcpy(mapped, layer.lines.data(), count * sizeof(DebugVertex));
                SDL_UnmapGPUTransferBuffer(gpu, layer.staging);
                layer.vertexCount = count;
                layer.uploadPending = true;
                uploadOverlay = true;
            }
        }
    }

    const bool anyMesh = std::ranges::any_of(m_meshes, [](const GpuMesh& m) { return m.uploadPending; });
    if (!anyMesh && !uploadOverlay) {
        return;
    }

    SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(cmd);
    for (GpuMesh& mesh : m_meshes) {
        if (!mesh.uploadPending) {
            continue;
        }
        mesh.uploadPending = false;
        const SDL_GPUTransferBufferLocation src{mesh.staging, 0};
        const SDL_GPUBufferRegion dst{mesh.vertexBuffer, 0, static_cast<Uint32>(mesh.vertexCount * sizeof(MeshVertex))};
        SDL_UploadToGPUBuffer(copy, &src, &dst, true);
    }
    if (uploadOverlay) {
        for (OverlayLayer& layer : m_overlays) {
            if (layer.uploadPending) {
                layer.uploadPending = false;
                const SDL_GPUTransferBufferLocation src{layer.staging, 0};
                const SDL_GPUBufferRegion dst{layer.buffer, 0, static_cast<Uint32>(layer.vertexCount * sizeof(DebugVertex))};
                SDL_UploadToGPUBuffer(copy, &src, &dst, true);
            }
        }
    }
    SDL_EndGPUCopyPass(copy);
}

bool Renderer::resizeTarget(RenderTarget& target, u32 width, u32 height)
{
    if (width == 0 || height == 0) {
        return target.valid();
    }
    if (target.valid() && target.width == width && target.height == height) {
        return true;
    }
    releaseTarget(target);

    SDL_GPUDevice* gpu = m_device->handle();
    SDL_GPUTextureCreateInfo info{};
    info.type = SDL_GPU_TEXTURETYPE_2D;
    info.width = width;
    info.height = height;
    info.layer_count_or_depth = 1;
    info.num_levels = 1;

    info.format = kSceneColorFormat;
    info.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;
    target.color = SDL_CreateGPUTexture(gpu, &info);

    info.format = m_depthFormat;
    info.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;
    target.depth = SDL_CreateGPUTexture(gpu, &info);

    if (!target.color || !target.depth) {
        KN_ERROR("Failed to create render target {}x{}: {}", width, height, SDL_GetError());
        releaseTarget(target);
        return false;
    }
    target.width = width;
    target.height = height;
    return true;
}

void Renderer::releaseTarget(RenderTarget& target)
{
    // Safe mid-frame: SDL_GPU defers the release until the GPU is done with the textures.
    if (m_device) {
        if (target.color) SDL_ReleaseGPUTexture(m_device->handle(), target.color);
        if (target.depth) SDL_ReleaseGPUTexture(m_device->handle(), target.depth);
    }
    target = {};
}

void Renderer::shutdown()
{
    if (!m_device) {
        return;
    }
    SDL_GPUDevice* gpu = m_device->handle();
    for (GpuMesh& mesh : m_meshes) {
        if (mesh.vertexBuffer) SDL_ReleaseGPUBuffer(gpu, mesh.vertexBuffer);
        if (mesh.uvBuffer) SDL_ReleaseGPUBuffer(gpu, mesh.uvBuffer);
        if (mesh.indexBuffer) SDL_ReleaseGPUBuffer(gpu, mesh.indexBuffer);
        if (mesh.staging) SDL_ReleaseGPUTransferBuffer(gpu, mesh.staging);
    }
    m_meshes.clear();
    for (GpuTexture& texture : m_textures) {
        if (texture.texture) SDL_ReleaseGPUTexture(gpu, texture.texture);
    }
    m_textures.clear();
    if (m_sampler) SDL_ReleaseGPUSampler(gpu, m_sampler);
    m_sampler = nullptr;
    for (OverlayLayer& layer : m_overlays) {
        if (layer.buffer) SDL_ReleaseGPUBuffer(gpu, layer.buffer);
        if (layer.staging) SDL_ReleaseGPUTransferBuffer(gpu, layer.staging);
    }
    m_overlays.clear();
    if (m_gridBuffer) SDL_ReleaseGPUBuffer(gpu, m_gridBuffer);
    m_gridBuffer = nullptr;
    releaseTarget(m_sceneTarget);
    for (SDL_GPUGraphicsPipeline** p : {&m_trianglePipeline, &m_meshPipeline, &m_linePipeline, &m_overlayLinePipeline}) {
        if (*p) SDL_ReleaseGPUGraphicsPipeline(gpu, *p);
        *p = nullptr;
    }
    m_device = nullptr;
}

SDL_GPURenderPass* Renderer::beginScenePass(SDL_GPUCommandBuffer* cmd, const glm::vec4& clearColor)
{
    return beginTargetPass(cmd, m_sceneTarget, clearColor);
}

SDL_GPURenderPass* Renderer::beginTargetPass(SDL_GPUCommandBuffer* cmd, const RenderTarget& target,
                                             const glm::vec4& clearColor)
{
    if (!target.valid()) {
        return nullptr;
    }
    SDL_GPUColorTargetInfo color{};
    color.texture = target.color;
    color.clear_color = SDL_FColor{clearColor.r, clearColor.g, clearColor.b, 1.0f};
    color.load_op = SDL_GPU_LOADOP_CLEAR;
    color.store_op = SDL_GPU_STOREOP_STORE;

    SDL_GPUDepthStencilTargetInfo depth{};
    depth.texture = target.depth;
    depth.clear_depth = 1.0f;
    depth.load_op = SDL_GPU_LOADOP_CLEAR;
    depth.store_op = SDL_GPU_STOREOP_DONT_CARE;
    depth.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
    depth.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;

    return SDL_BeginGPURenderPass(cmd, &color, 1, &depth);
}

void Renderer::blitSceneToSwapchain(const gfx::Frame& frame)
{
    if (!m_sceneTarget.valid() || !frame.swapchain) {
        return;
    }
    SDL_GPUBlitInfo blit{};
    blit.source.texture = m_sceneTarget.color;
    blit.source.w = m_sceneTarget.width;
    blit.source.h = m_sceneTarget.height;
    blit.destination.texture = frame.swapchain;
    blit.destination.w = frame.width;
    blit.destination.h = frame.height;
    blit.load_op = SDL_GPU_LOADOP_DONT_CARE;
    blit.filter = SDL_GPU_FILTER_LINEAR;
    SDL_BlitGPUTexture(frame.cmd, &blit);
}

SDL_GPURenderPass* Renderer::beginOverlayPass(const gfx::Frame& frame, bool clear, const glm::vec4& clearColor)
{
    SDL_GPUColorTargetInfo color{};
    color.texture = frame.swapchain;
    color.clear_color = SDL_FColor{clearColor.r, clearColor.g, clearColor.b, clearColor.a};
    color.load_op = clear ? SDL_GPU_LOADOP_CLEAR : SDL_GPU_LOADOP_LOAD;
    color.store_op = SDL_GPU_STOREOP_STORE;
    return SDL_BeginGPURenderPass(frame.cmd, &color, 1, nullptr);
}

void Renderer::endPass(SDL_GPURenderPass* pass)
{
    if (pass) {
        SDL_EndGPURenderPass(pass);
    }
}

void Renderer::drawScene(SDL_GPUCommandBuffer* cmd, SDL_GPURenderPass* pass, const scene::Scene& scene,
                         const SceneDrawOptions& options)
{
    const entt::entity cameraEntity = scene.activeCamera();
    if (!pass || cameraEntity == entt::null) {
        return;
    }
    const auto& registry = scene.registry();
    const auto& camera = registry.get<scene::CameraComponent>(cameraEntity);
    const auto& cameraTransform = registry.get<scene::TransformComponent>(cameraEntity);
    const f32 aspect = options.aspect > 0.0f ? options.aspect : sceneAspect();
    const glm::mat4 viewProjection = camera.projection(aspect) * scene::CameraComponent::view(cameraTransform);
    const glm::vec3 cameraDir = cameraTransform.rotation * glm::vec3(0.0f, 0.0f, -1.0f);

    if (options.grid) {
        drawGrid(cmd, pass, viewProjection);
    }

    SDL_BindGPUGraphicsPipeline(pass, m_meshPipeline);
    auto view = registry.view<const scene::TransformComponent, const scene::MeshRendererComponent>();
    for (const entt::entity entity : view) {
        const auto& [transform, renderer] = view.get(entity);
        const GpuMesh* mesh = this->mesh(renderer.mesh);
        if (!mesh || !renderer.visible) {
            continue;
        }
        const glm::mat4 model = transform.matrix();
        const MeshDrawUniforms draw{viewProjection * model, glm::transpose(glm::inverse(model)), model};
        const MaterialUniforms material{renderer.tint, glm::vec4(kKeyLightDir, 0.0f), glm::vec4(cameraDir, 0.0f),
                                        glm::vec4(renderer.normalStrength, renderer.flipNormalY ? -1.0f : 1.0f, 0.0f, 0.0f)};
        SDL_PushGPUVertexUniformData(cmd, 0, &draw, sizeof(draw));
        SDL_PushGPUFragmentUniformData(cmd, 0, &material, sizeof(material));

        const GpuTexture* base = texture(renderer.baseColorTexture);
        const GpuTexture* normal = texture(renderer.normalTexture);
        const std::array<SDL_GPUTextureSamplerBinding, 2> samplers = {{
            {(base ? base : &m_textures[scene::kWhiteTexture])->texture, m_sampler},
            {(normal ? normal : &m_textures[scene::kFlatNormalTexture])->texture, m_sampler},
        }};
        SDL_BindGPUFragmentSamplers(pass, 0, samplers.data(), static_cast<Uint32>(samplers.size()));

        const std::array<SDL_GPUBufferBinding, 2> vertices = {{{mesh->vertexBuffer, 0}, {mesh->uvBuffer, 0}}};
        const SDL_GPUBufferBinding indices{mesh->indexBuffer, 0};
        SDL_BindGPUVertexBuffers(pass, 0, vertices.data(), static_cast<Uint32>(vertices.size()));
        SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
        SDL_DrawGPUIndexedPrimitives(pass, mesh->indexCount, 1, 0, 0, 0);
    }

    const OverlayLayer* overlay = options.overlayLayer < m_overlays.size() ? &m_overlays[options.overlayLayer] : nullptr;
    if (overlay && overlay->vertexCount > 0 && overlay->buffer) {
        SDL_BindGPUGraphicsPipeline(pass, m_overlayLinePipeline);
        SDL_PushGPUVertexUniformData(cmd, 0, &viewProjection, sizeof(viewProjection));
        const SDL_GPUBufferBinding binding{overlay->buffer, 0};
        SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
        SDL_DrawGPUPrimitives(pass, overlay->vertexCount, 1, 0, 0);
    }
}

void Renderer::drawGrid(SDL_GPUCommandBuffer* cmd, SDL_GPURenderPass* pass, const glm::mat4& viewProjection)
{
    if (!pass || !m_gridBuffer) {
        return;
    }
    SDL_BindGPUGraphicsPipeline(pass, m_linePipeline);
    SDL_PushGPUVertexUniformData(cmd, 0, &viewProjection, sizeof(viewProjection));
    const SDL_GPUBufferBinding binding{m_gridBuffer, 0};
    SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
    SDL_DrawGPUPrimitives(pass, m_gridVertexCount, 1, 0, 0);
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
