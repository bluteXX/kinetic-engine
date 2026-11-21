#include "kinetic/renderer/VfxRender.h"

#include <unordered_set>

namespace kinetic::renderer {

namespace {

void release(scene::Scene& scene, Renderer& renderer, const VfxRenderState::Entry& entry)
{
    if (scene.registry().valid(entry.entity)) {
        scene.registry().destroy(entry.entity);
    }
    renderer.removeMesh(entry.mesh);
}

} // namespace

void syncVfx(scene::Scene& scene, Renderer& renderer, const vfx::VfxSystem& system, VfxRenderState& state)
{
    auto& registry = scene.registry();
    std::unordered_set<u32> live;
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    for (const vfx::Burst& burst : system.bursts()) {
        live.insert(burst.id);
        const usize count = vfx::VfxSystem::vertexCount(burst);
        if (count == 0) continue;
        positions.resize(count);
        normals.resize(count);
        vfx::VfxSystem::buildMesh(burst, positions, normals);
        auto it = state.bursts.find(burst.id);
        if (it == state.bursts.end()) {
            MeshData data;
            data.positions = positions;
            data.normals = normals;
            data.indices.resize(count);
            for (usize i = 0; i < count; ++i) data.indices[i] = static_cast<u32>(i);
            VfxRenderState::Entry entry;
            entry.mesh = renderer.addMesh("vfx", data, MeshUsage::Dynamic);
            if (entry.mesh == Renderer::kInvalidMesh) continue;
            entry.vertices = count;
            entry.entity = registry.create();
            registry.emplace<scene::TransformComponent>(entry.entity);
            auto& mr = registry.emplace<scene::MeshRendererComponent>(entry.entity);
            mr.mesh = entry.mesh;
            mr.tint = glm::vec4(burst.desc.color, 1.0f);
            state.bursts.emplace(burst.id, entry);
            continue;
        }
        renderer.updateMeshVertices(it->second.mesh, positions, normals);
    }
    for (auto it = state.bursts.begin(); it != state.bursts.end();) {
        if (!live.contains(it->first)) {
            release(scene, renderer, it->second);
            it = state.bursts.erase(it);
        } else {
            ++it;
        }
    }
}

void clearVfx(scene::Scene& scene, Renderer& renderer, VfxRenderState& state)
{
    for (const auto& [id, entry] : state.bursts) release(scene, renderer, entry);
    state.bursts.clear();
}

} // namespace kinetic::renderer
