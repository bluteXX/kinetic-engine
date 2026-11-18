#pragma once

#include <kinetic/animation/Skinning.h>
#include <kinetic/renderer/Renderer.h>

#include <glm/glm.hpp>

#include <filesystem>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace kinetic::editor {

// One asset shown in the drawer: the built-in triangle, a static mesh file, or an animated model.
struct MeshAsset {
    std::string name;
    std::string type; // file type: "BUILT-IN", "FBX", "OBJ", "GLB", "GLTF"
    std::filesystem::path path; // empty for built-ins
    scene::MeshId id = scene::kTriangleMesh; // static mesh (animated models: their rest pose)
    u32 triangles = 0;
    std::shared_ptr<const anim::SkinnedModel> model; // animated models only
    std::string preset; // built-in presets: "character", "floor", "stone" (empty for files)

    [[nodiscard]] bool animated() const { return model != nullptr; }
    [[nodiscard]] bool isPreset() const { return !preset.empty(); }
};

// The "Assets" drawer at the bottom of the editor.
//
// Shows every mesh / model in the project's asset folders (loaded on startup, Refresh picks up
// new files) plus the built-in triangle. Files with a skin or animation become animated assets.
// Tiles are ImGui drag & drop sources carrying the asset index (kAssetPayload): the Viewport
// places a new actor, an actor's Mesh field switches to the asset. Double-click a tile to place
// it in front of the camera.
class AssetDrawer {
public:
    static constexpr const char* kAssetPayload = "KINETIC_ASSET";

    // `folders[0]` receives imported .fbx / .obj files; .glb / .gltf go to the folder named
    // "models" if there is one.
    void init(renderer::Renderer& renderer, std::vector<std::filesystem::path> folders);

    // Built-in preset (shown first, after the triangle).
    void addPreset(MeshAsset asset);

    // Loads files in the folders that are not loaded yet.
    void refresh();
    // Copies `source` into an asset folder (unless it is already there) and loads it.
    bool importFile(const std::filesystem::path& source);

    void draw();

    // Asset double-clicked this frame, if any (index into assets()).
    [[nodiscard]] std::optional<usize> takeSpawnRequest() { return std::exchange(m_spawnRequest, std::nullopt); }

    [[nodiscard]] const std::vector<MeshAsset>& assets() const { return m_assets; }
    [[nodiscard]] const MeshAsset* asset(usize index) const { return index < m_assets.size() ? &m_assets[index] : nullptr; }
    // Asset whose static mesh is `id`, or whose model is `model`.
    [[nodiscard]] const MeshAsset* findByMesh(scene::MeshId id) const;
    [[nodiscard]] const MeshAsset* findByModel(const anim::SkinnedModel* model) const;

    // Combo box listing every asset; returns the picked index. Accepts asset drops too.
    [[nodiscard]] std::optional<usize> assetCombo(const char* label, const MeshAsset* current) const;

private:
    bool loadFile(const std::filesystem::path& path);
    void drawTile(usize index, f32 size);
    [[nodiscard]] std::filesystem::path importFolderFor(const std::filesystem::path& file) const;

    renderer::Renderer* m_renderer = nullptr;
    std::vector<std::filesystem::path> m_folders;
    std::vector<MeshAsset> m_assets;
    std::set<std::filesystem::path> m_loaded; // canonical paths already in the library
    std::optional<usize> m_spawnRequest;
    std::string m_status;
};

} // namespace kinetic::editor
