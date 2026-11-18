#include "kinetic/assets/ModelLoader.h"

#include "LoaderInternal.h"

#include "kinetic/core/Log.h"

namespace kinetic::assets {

namespace detail {

void computeNormals(MeshData& mesh)
{
    mesh.normals.assign(mesh.positions.size(), glm::vec3(0.0f));
    for (usize t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const u32 a = mesh.indices[t];
        const u32 b = mesh.indices[t + 1];
        const u32 c = mesh.indices[t + 2];
        // Cross product length is twice the triangle area: larger faces count more.
        const glm::vec3 n = glm::cross(mesh.positions[b] - mesh.positions[a], mesh.positions[c] - mesh.positions[a]);
        mesh.normals[a] += n;
        mesh.normals[b] += n;
        mesh.normals[c] += n;
    }
    for (glm::vec3& n : mesh.normals) {
        const f32 length = glm::length(n);
        n = length > 1e-12f ? n / length : glm::vec3(0.0f, 1.0f, 0.0f);
    }
}

} // namespace detail

bool isSupportedModelFile(const std::filesystem::path& path)
{
    const std::string ext = detail::lowerExtension(path);
    return ext == ".glb" || ext == ".gltf" || ext == ".fbx";
}

std::optional<anim::SkinnedModel> loadSkinnedModel(const std::filesystem::path& path, std::string* error)
{
    std::string reason;
    std::optional<anim::SkinnedModel> model;
    const std::string ext = detail::lowerExtension(path);
    if (ext == ".glb" || ext == ".gltf") {
        model = detail::loadSkinnedModelGltf(path, reason);
    } else if (ext == ".fbx") {
        model = detail::loadSkinnedModelFbx(path, reason);
    } else {
        reason = "unsupported file type (expected .glb, .gltf or .fbx)";
    }

    if (model && !model->skin.isValid(model->skeleton.jointCount())) {
        model.reset();
        reason = "skin weights reference joints outside the skeleton";
    }
    if (!model) {
        if (error) {
            *error = reason;
        }
        return std::nullopt;
    }
    model->name = path.stem().string();
    anim::computeRestBounds(*model);
    KN_INFO("Model '{}': {} triangles, {} joints, {} clip(s)", path.filename().string(), model->skin.mesh.triangleCount(),
            model->skeleton.jointCount(), model->clips.size());
    return model;
}

} // namespace kinetic::assets
