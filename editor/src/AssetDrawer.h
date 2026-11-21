#pragma once

#include <kinetic/animation/Skinning.h>
#include <kinetic/renderer/Renderer.h>

#include <glm/glm.hpp>

#include <array>
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
    // Material textures from the file (built-in white / flat normal when it has none).
    scene::TextureId baseColorTexture = scene::kWhiteTexture;
    scene::TextureId normalTexture = scene::kFlatNormalTexture;
    // Texture files the material uses (moved together with the model when they live next to it).
    std::vector<std::filesystem::path> textureFiles;

    [[nodiscard]] bool animated() const { return model != nullptr; }
    [[nodiscard]] bool isPreset() const { return !preset.empty(); }
    [[nodiscard]] bool textured() const { return baseColorTexture != scene::kWhiteTexture; }
};

// The "Assets" drawer at the bottom of the editor: a content browser over the project's assets/
// folder.
//
//   left   folder tree ("Built-in" first: triangle + presets; then assets/ and its sub-folders)
//   right  the current folder: sub-folders (double-click to open) and files - mesh / model
//          assets as coloured tiles, images (textures) as grey tiles
//   top    Up, breadcrumb path (every part is a drop target), New Folder, Refresh, Explorer,
//          and "All" (every asset in one flat list, like the old drawer)
//
// Organising: drag a tile or a folder onto a folder (in the tree, the grid or the breadcrumb) to
// move it on disk; a model moves together with the texture files next to it. Right-click for
// Rename / New Folder / Delete (empty folders and images only) / Show in Explorer. Files dropped
// on the window are imported into the current folder.
//
// Asset tiles are also ImGui drag & drop sources carrying the asset index (kAssetPayload): the
// Viewport places a new actor, an actor's Mesh field switches to the asset. Double-click places
// it in front of the camera.
class AssetDrawer {
public:
    static constexpr const char* kAssetPayload = "KINETIC_ASSET";
    static constexpr const char* kPathPayload = "KINETIC_ASSET_PATH"; // folder / image (path string)

    // `root` is the project's assets folder; every file below it is loaded.
    void init(renderer::Renderer& renderer, std::filesystem::path root);

    // Built-in preset (shown in the "Built-in" folder, after the triangle).
    void addPreset(MeshAsset asset);

    // Rescans the folders: loads new files, refreshes the folder tree.
    void refresh();
    // Copies `source` into the current folder (unless it is already below the root) and loads it.
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

    // File operations (also used by the UI). All paths must be below the root. They update the
    // loaded assets' paths and report problems in the status line.
    bool createFolder(const std::filesystem::path& parent, const std::string& name);
    bool moveAsset(usize index, const std::filesystem::path& folder);
    bool movePath(const std::filesystem::path& source, const std::filesystem::path& folder);
    bool renamePath(const std::filesystem::path& source, const std::string& newName);

    [[nodiscard]] const std::filesystem::path& root() const { return m_root; }
    [[nodiscard]] const std::filesystem::path& currentFolder() const { return m_current; }
    void openFolder(const std::filesystem::path& folder);

private:
    struct FolderNode {
        std::filesystem::path path;
        std::vector<FolderNode> children;
    };

    bool loadFile(const std::filesystem::path& path);
    // Uploads the material's textures (shared with other assets using the same files).
    void loadMaterial(const MaterialDesc& material, MeshAsset& asset);
    [[nodiscard]] scene::TextureId loadTexture(const std::filesystem::path& path);
    void rebuildTree();
    void scanCurrentFiles();
    // Rewrites every stored path below `from` to the same place below `to` (after a move / rename).
    void retargetPaths(const std::filesystem::path& from, const std::filesystem::path& to);
    [[nodiscard]] bool insideRoot(const std::filesystem::path& path) const;

    void drawToolbar();
    void drawTree(const FolderNode& node);
    void drawBuiltInNode();
    void drawContents(f32 tileSize);
    void drawFolderTile(const std::filesystem::path& folder, f32 size);
    void drawFileTile(const std::filesystem::path& file, f32 size);
    void drawTile(usize index, f32 size);
    void drawRenamePopup();
    // Makes the last item accept dropped assets / paths and move them into `folder`.
    void folderDropTarget(const std::filesystem::path& folder);
    void contextMenuFor(const std::filesystem::path& path, bool isFolder);

    renderer::Renderer* m_renderer = nullptr;
    std::filesystem::path m_root;
    std::filesystem::path m_current;  // folder shown on the right (empty + m_builtIn = Built-in)
    bool m_builtIn = false;           // showing the virtual Built-in folder
    bool m_flat = false;              // "All": every asset in one list
    FolderNode m_tree;
    std::vector<std::filesystem::path> m_currentFolders; // sub-folders of m_current
    std::vector<std::filesystem::path> m_currentImages;  // other files (textures) in m_current
    bool m_dirty = true;              // rescan the tree / current folder before drawing

    std::vector<MeshAsset> m_assets;
    std::set<std::filesystem::path> m_loaded; // canonical paths already in the library
    std::set<std::filesystem::path> m_animationFiles; // animation-only files (clips of a model)
    std::optional<usize> m_spawnRequest;
    std::string m_status;

    // Rename / new folder popup.
    std::filesystem::path m_renameTarget; // path to rename, or the parent for a new folder
    bool m_renameIsNewFolder = false;
    bool m_openRenamePopup = false;
    std::array<char, 128> m_renameBuffer{};
};

} // namespace kinetic::editor
