#include "kinetic/core/Primitives.h"

#include <cmath>

namespace kinetic {

MeshData makeBoxMesh(const glm::vec3& halfExtents, const glm::vec3& center)
{
    MeshData mesh;
    // One quad per face, 4 own vertices each (flat normals). With v = n x u the corners
    // (-u-v, +u-v, +u+v, -u+v) run counter-clockwise around n, i.e. seen from outside.
    const glm::vec3 normals[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (const glm::vec3& n : normals) {
        const glm::vec3 u = std::abs(n.y) > 0.5f ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
        const glm::vec3 v = glm::cross(n, u);
        const auto base = static_cast<u32>(mesh.positions.size());
        for (const glm::vec2 c : {glm::vec2(-1, -1), glm::vec2(1, -1), glm::vec2(1, 1), glm::vec2(-1, 1)}) {
            mesh.positions.push_back(center + (n + u * c.x + v * c.y) * halfExtents);
            mesh.normals.push_back(n);
        }
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    return mesh;
}

MeshData makeCapsuleMesh(f32 radius, f32 halfHeight, const glm::vec3& center, u32 segments, u32 rings)
{
    MeshData mesh;
    const f32 pi = 3.14159265358979f;
    // Latitude rows: top hemisphere (rings + 1 rows, pole to equator), then bottom (equator to pole).
    struct Row {
        f32 y;      // centre offset of the row's sphere
        f32 phi;    // latitude, 0 = north pole, pi = south pole
    };
    std::vector<Row> rows;
    for (u32 r = 0; r <= rings; ++r) {
        rows.push_back({halfHeight, 0.5f * pi * static_cast<f32>(r) / static_cast<f32>(rings)});
    }
    for (u32 r = 0; r <= rings; ++r) {
        rows.push_back({-halfHeight, 0.5f * pi + 0.5f * pi * static_cast<f32>(r) / static_cast<f32>(rings)});
    }

    for (const Row& row : rows) {
        for (u32 s = 0; s <= segments; ++s) {
            const f32 theta = 2.0f * pi * static_cast<f32>(s) / static_cast<f32>(segments);
            const glm::vec3 n(std::sin(row.phi) * std::cos(theta), std::cos(row.phi), std::sin(row.phi) * std::sin(theta));
            mesh.positions.push_back(center + glm::vec3(0.0f, row.y, 0.0f) + n * radius);
            mesh.normals.push_back(n);
        }
    }
    const u32 stride = segments + 1;
    for (u32 r = 0; r + 1 < rows.size(); ++r) {
        for (u32 s = 0; s < segments; ++s) {
            const u32 a = r * stride + s;
            const u32 b = a + stride;
            // Rows go top -> bottom and theta turns from +X towards +Z: (a, a+1, b) is CCW from outside.
            mesh.indices.insert(mesh.indices.end(), {a, a + 1, b, a + 1, b + 1, b});
        }
    }
    return mesh;
}

} // namespace kinetic
