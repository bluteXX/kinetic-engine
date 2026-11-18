#pragma once

#include <kinetic/renderer/Renderer.h>

#include <glm/glm.hpp>

#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace kinetic::editor {

// One mesh shown in the drawer: the built-in triangle or a file from the meshes folder.
struct MeshAsset {
    std::string name;
    std::string type; // "BUILT-IN", "FBX", "OBJ"
    std::filesystem::path path; // empty for built-ins
    scene::MeshId id = scene::kTriangleMesh;
    u32 triangles = 0;
};

// The "Assets" drawer at the bottom of the editor.
//
// Shows every mesh in the project's meshes folder (loaded on startup, Refresh picks up new files)
// plus the built-in triangle. Tiles are ImGui drag & drop sources carrying a scene::MeshId
// (payload kMeshPayload): drop one on the Viewport to place an object, or on an object's Mesh
// field in the Inspector to swap its mesh. Double-click a tile to place it in front of the camera.
class AssetDrawer {
public:
    static constexpr const char* kMeshPayload = "KINETIC_MESH";

    void init(renderer::Renderer& renderer, std::filesystem::path folder);

    // Loads files in the folder that are not loaded yet.
    void refresh();
    // Copies `source` into the folder (unless it is already there) and loads it.
    bool importFile(const std::filesystem::path& source);

    void draw();

    // Mesh double-clicked this frame, if any.
    [[nodiscard]] std::optional<scene::MeshId> takeSpawnRequest() { return std::exchange(m_spawnRequest, std::nullopt); }

    [[nodiscard]] const std::filesystem::path& folder() const { return m_folder; }
    [[nodiscard]] const std::vector<MeshAsset>& assets() const { return m_assets; }

private:
    bool loadFile(const std::filesystem::path& path);
    void drawTile(const MeshAsset& asset, f32 size);

    renderer::Renderer* m_renderer = nullptr;
    std::filesystem::path m_folder;
    std::vector<MeshAsset> m_assets;
    std::set<std::filesystem::path> m_loaded; // canonical paths already in the library
    std::optional<scene::MeshId> m_spawnRequest;
    std::string m_status;
};

} // namespace kinetic::editor
