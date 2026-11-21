#include "kinetic/assets/ModelLoader.h"

#include "LoaderInternal.h"

#include "kinetic/assets/TextureLoader.h"
#include "kinetic/animation/Retarget.h"
#include "kinetic/core/Log.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <unordered_map>
#include <vector>

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

void weldVertices(anim::SkinnedMesh& skin)
{
    MeshData& mesh = skin.mesh;
    const usize n = mesh.positions.size();
    const bool uvs = mesh.hasUvs();
    if (n == 0 || mesh.normals.size() != n || skin.joints.size() != n || skin.weights.size() != n) {
        return;
    }
    // Hash of the raw attribute bytes: only exact duplicates are merged.
    struct Key {
        std::array<u32, 20> words{};
        bool operator==(const Key&) const = default;
    };
    struct KeyHash {
        usize operator()(const Key& k) const
        {
            u64 h = 1469598103934665603ull;
            for (const u32 w : k.words) {
                h = (h ^ w) * 1099511628211ull;
            }
            return static_cast<usize>(h);
        }
    };
    const auto keyOf = [&](usize v) {
        Key k;
        usize at = 0;
        const auto put = [&](const f32* values, usize count) {
            for (usize i = 0; i < count; ++i) {
                std::memcpy(&k.words[at++], &values[i], sizeof(u32));
            }
        };
        put(&mesh.positions[v].x, 3);
        put(&mesh.normals[v].x, 3);
        if (uvs) {
            put(&mesh.uvs[v].x, 2);
        } else {
            at += 2;
        }
        for (glm::length_t i = 0; i < 4; ++i) {
            k.words[at++] = skin.joints[v][i];
        }
        put(&skin.weights[v].x, 4);
        return k;
    };

    std::unordered_map<Key, u32, KeyHash> unique;
    unique.reserve(n);
    std::vector<u32> remap(n);
    anim::SkinnedMesh out;
    out.mesh.material = std::move(mesh.material);
    for (usize v = 0; v < n; ++v) {
        const auto [it, inserted] = unique.try_emplace(keyOf(v), static_cast<u32>(out.mesh.positions.size()));
        if (inserted) {
            out.mesh.positions.push_back(mesh.positions[v]);
            out.mesh.normals.push_back(mesh.normals[v]);
            if (uvs) {
                out.mesh.uvs.push_back(mesh.uvs[v]);
            }
            out.joints.push_back(skin.joints[v]);
            out.weights.push_back(skin.weights[v]);
        }
        remap[v] = it->second;
    }
    out.mesh.indices.reserve(mesh.indices.size());
    for (const u32 i : mesh.indices) {
        out.mesh.indices.push_back(i < n ? remap[i] : 0u);
    }
    skin = std::move(out);
}

} // namespace detail

bool isSupportedModelFile(const std::filesystem::path& path)
{
    const std::string ext = detail::lowerExtension(path);
    return ext == ".glb" || ext == ".gltf" || ext == ".fbx";
}

std::optional<SkeletalAnimationAsset> loadAnimationFile(const std::filesystem::path& path, std::string* error)
{
    std::string reason;
    std::optional<SkeletalAnimationAsset> asset;
    const std::string ext = detail::lowerExtension(path);
    if (ext == ".fbx") {
        asset = detail::loadAnimationFbx(path, reason);
    } else if (ext == ".glb" || ext == ".gltf") {
        asset = loadSkeletalAnimation(path);
        if (!asset) reason = "no skin / animation";
    } else {
        reason = "unsupported file type (expected .fbx, .glb or .gltf)";
    }
    if (!asset) {
        if (error) *error = reason;
        return std::nullopt;
    }
    // "mixamo.com", "Take 001", "Armature|Action": a single clip gets the file's name instead.
    if (asset->clips.size() == 1) {
        asset->clips[0] = anim::AnimationClip(path.stem().string(), asset->clips[0].duration(),
                                              std::vector<anim::JointTracks>(asset->clips[0].tracks()));
    }
    return asset;
}

std::vector<std::string> importAnimationsInto(anim::SkinnedModel& model, const std::filesystem::path& file, std::string* error)
{
    std::vector<std::string> added;
    const auto asset = loadAnimationFile(file, error);
    if (!asset) {
        return added;
    }
    const auto map = anim::mapJointsByName(asset->skeleton, model.skeleton);
    const auto matched = std::ranges::count_if(map, [](i32 j) { return j >= 0; });
    if (matched < 3) {
        if (error) *error = "no bones in common with the model (by name)";
        return added;
    }
    for (const anim::AnimationClip& clip : asset->clips) {
        model.clips.push_back(anim::retargetClip(asset->skeleton, clip, model.skeleton, clip.name()));
        added.push_back(clip.name());
    }
    KN_INFO("{}: {} clip(s) retargeted onto '{}' ({} of {} bones matched)", file.filename().string(), added.size(),
            model.name, matched, model.skeleton.jointCount());
    return added;
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
    model->sourcePath = path.generic_string();
    detail::weldVertices(model->skin);
    computeTangents(model->skin.mesh);
    resolveMaterial(model->skin.mesh.material, path);
    anim::computeRestBounds(*model);
    KN_INFO("Model '{}': {} triangles, {} joints, {} clip(s)", path.filename().string(), model->skin.mesh.triangleCount(),
            model->skeleton.jointCount(), model->clips.size());
    return model;
}

} // namespace kinetic::assets
