#include "kinetic/assets/MeshLoader.h"

#include "kinetic/core/Log.h"

#include <ufbx.h>

#include <algorithm>
#include <cctype>
#include <memory>
#include <vector>

namespace kinetic::assets {

namespace {

glm::vec3 toGlm(const ufbx_vec3& v)
{
    return {static_cast<f32>(v.x), static_cast<f32>(v.y), static_cast<f32>(v.z)};
}

struct SceneDeleter {
    void operator()(ufbx_scene* scene) const { ufbx_free_scene(scene); }
};

} // namespace

bool isSupportedMeshFile(const std::filesystem::path& path)
{
    std::string ext = path.extension().string();
    std::ranges::transform(ext, ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext == ".fbx" || ext == ".obj";
}

std::optional<MeshData> loadMesh(const std::filesystem::path& path, std::string* error)
{
    const auto fail = [&](std::string reason) -> std::optional<MeshData> {
        KN_ERROR("Mesh import '{}' failed: {}", path.string(), reason);
        if (error) {
            *error = std::move(reason);
        }
        return std::nullopt;
    };

    if (!isSupportedMeshFile(path)) {
        return fail("unsupported file type (expected .fbx or .obj)");
    }

    ufbx_load_opts opts{};
    opts.target_axes = ufbx_axes_right_handed_y_up;
    opts.target_unit_meters = 1.0;
    opts.space_conversion = UFBX_SPACE_CONVERSION_MODIFY_GEOMETRY;
    opts.generate_missing_normals = true;

    ufbx_error ufbxError{};
    const std::unique_ptr<ufbx_scene, SceneDeleter> scene(ufbx_load_file(path.string().c_str(), &opts, &ufbxError));
    if (!scene) {
        char buffer[512];
        ufbx_format_error(buffer, sizeof(buffer), &ufbxError);
        std::string message = buffer;
        while (!message.empty() && (message.back() == '\n' || message.back() == '\r')) {
            message.pop_back();
        }
        return fail(std::move(message));
    }

    MeshData out;
    std::vector<uint32_t> triangle;

    for (const ufbx_node* node : scene->nodes) {
        const ufbx_mesh* mesh = node->mesh;
        if (!mesh || mesh->num_faces == 0) {
            continue;
        }
        const ufbx_matrix& toWorld = node->geometry_to_world;
        const ufbx_matrix normalToWorld = ufbx_matrix_for_normals(&toWorld);
        triangle.resize(mesh->max_face_triangles * 3);

        for (const ufbx_face face : mesh->faces) {
            const u32 numTriangles = ufbx_triangulate_face(triangle.data(), triangle.size(), mesh, face);
            for (u32 i = 0; i < numTriangles * 3; ++i) {
                const uint32_t corner = triangle[i];
                const ufbx_vec3 position = ufbx_get_vertex_vec3(&mesh->vertex_position, corner);
                ufbx_vec3 normal{};
                normal.y = 1.0; // only used if the file has no normals and generation failed
                if (mesh->vertex_normal.exists) {
                    normal = ufbx_get_vertex_vec3(&mesh->vertex_normal, corner);
                }

                glm::vec3 n = toGlm(ufbx_transform_direction(&normalToWorld, normal));
                const f32 len = glm::length(n);
                n = len > 1e-8f ? n / len : glm::vec3(0.0f, 1.0f, 0.0f);

                out.indices.push_back(static_cast<u32>(out.positions.size()));
                out.positions.push_back(toGlm(ufbx_transform_position(&toWorld, position)));
                out.normals.push_back(n);
            }
        }
    }

    if (out.empty()) {
        return fail("file contains no mesh geometry");
    }
    KN_INFO("Mesh import '{}': {} triangles", path.filename().string(), out.triangleCount());
    return out;
}

} // namespace kinetic::assets
