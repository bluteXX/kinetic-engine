#pragma once

#include "kinetic/core/Types.h"

#include <glm/glm.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace kinetic {

// Surface description as authored in a model file: colours and the texture files to use.
// Paths are already resolved against the model's folder (empty = no such texture). The renderer
// turns this into GPU textures; nothing here touches the disk.
struct MaterialDesc {
    std::string name;
    glm::vec4 baseColorFactor{1.0f};
    std::filesystem::path baseColorTexture;
    std::filesystem::path normalTexture;
    std::filesystem::path roughnessTexture; // not rendered yet (kept for the PBR pass)
    std::filesystem::path metallicTexture;
    f32 normalScale = 1.0f;

    [[nodiscard]] bool hasTextures() const { return !baseColorTexture.empty() || !normalTexture.empty(); }
};

// CPU-side triangle mesh, indexed triangle list (CCW = front). Per vertex: position + normal,
// optionally a texture coordinate and a tangent (xyz = tangent, w = bitangent sign, for normal
// mapping). Produced by the asset loaders, uploaded by the renderer.
struct MeshData {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> uvs;      // empty or one per vertex (glTF convention: v = 0 at the top)
    std::vector<glm::vec4> tangents; // empty or one per vertex
    std::vector<u32> indices;
    MaterialDesc material;

    [[nodiscard]] bool empty() const { return indices.empty(); }
    [[nodiscard]] usize triangleCount() const { return indices.size() / 3; }
    [[nodiscard]] bool hasUvs() const { return !uvs.empty() && uvs.size() == positions.size(); }
    [[nodiscard]] bool hasTangents() const { return !tangents.empty() && tangents.size() == positions.size(); }

    // Axis-aligned bounds of all positions ({0,0,0} for an empty mesh).
    [[nodiscard]] glm::vec3 boundsMin() const;
    [[nodiscard]] glm::vec3 boundsMax() const;
};

// Per-vertex tangents from positions, normals and uvs (accumulated per triangle, orthogonalised
// against the normal; w = handedness such that cross(normal, tangent) * w points up the image,
// matching OpenGL-convention normal maps). Meshes without uvs get an arbitrary tangent frame.
void computeTangents(MeshData& mesh);

inline glm::vec3 MeshData::boundsMin() const
{
    if (positions.empty()) {
        return glm::vec3(0.0f);
    }
    glm::vec3 r = positions[0];
    for (const glm::vec3& p : positions) {
        r = glm::min(r, p);
    }
    return r;
}

inline glm::vec3 MeshData::boundsMax() const
{
    if (positions.empty()) {
        return glm::vec3(0.0f);
    }
    glm::vec3 r = positions[0];
    for (const glm::vec3& p : positions) {
        r = glm::max(r, p);
    }
    return r;
}

} // namespace kinetic
