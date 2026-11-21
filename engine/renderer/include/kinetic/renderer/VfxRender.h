#pragma once

#include "kinetic/renderer/Renderer.h"

#include <kinetic/scene/Scene.h>
#include <kinetic/vfx/Vfx.h>

#include <unordered_map>

namespace kinetic::renderer {

// Draws a VfxSystem's bursts: each burst becomes a hidden-from-the-editor scene entity (no name)
// with a dynamic mesh tinted with the emitter's colour, rebuilt every frame and removed when the
// burst ends.
struct VfxRenderState {
    struct Entry {
        entt::entity entity = entt::null;
        scene::MeshId mesh = 0;
        usize vertices = 0;
    };
    std::unordered_map<u32, Entry> bursts; // by Burst::id
};

void syncVfx(scene::Scene& scene, Renderer& renderer, const vfx::VfxSystem& system, VfxRenderState& state);
// Removes every burst's entity and mesh.
void clearVfx(scene::Scene& scene, Renderer& renderer, VfxRenderState& state);

} // namespace kinetic::renderer
