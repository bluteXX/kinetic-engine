#include "AssetDrawer.h"

#include <kinetic/assets/MeshLoader.h>
#include <kinetic/assets/ModelLoader.h>
#include <kinetic/core/Log.h>

#include <SDL3/SDL_misc.h>
#include <imgui.h>

#include <algorithm>
#include <cctype>

namespace kinetic::editor {

namespace {

std::string upperExtension(const std::filesystem::path& path)
{
    std::string ext = path.extension().string();
    if (!ext.empty() && ext[0] == '.') {
        ext.erase(0, 1);
    }
    std::ranges::transform(ext, ext.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return ext;
}

bool isAssetFile(const std::filesystem::path& path)
{
    return assets::isSupportedMeshFile(path) || assets::isSupportedModelFile(path);
}

ImVec4 tileColor(const MeshAsset& asset)
{
    if (asset.animated()) return {0.62f, 0.38f, 0.85f, 1.0f};
    if (asset.type == "FBX") return {0.80f, 0.47f, 0.20f, 1.0f};
    if (asset.type == "OBJ") return {0.27f, 0.62f, 0.38f, 1.0f};
    return {0.35f, 0.42f, 0.58f, 1.0f}; // built-in
}

std::filesystem::path canonicalOrSelf(const std::filesystem::path& path)
{
    std::error_code ec;
    auto result = std::filesystem::weakly_canonical(path, ec);
    return ec ? path : result;
}

} // namespace

void AssetDrawer::init(renderer::Renderer& renderer, std::vector<std::filesystem::path> folders)
{
    m_renderer = &renderer;
    m_folders = std::move(folders);
    if (!m_folders.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(m_folders.front(), ec);
    }

    m_assets.push_back({"Triangle", "BUILT-IN", {}, scene::kTriangleMesh, 1, nullptr});
    refresh();
}

void AssetDrawer::refresh()
{
    std::vector<std::filesystem::path> files;
    std::string folderError;
    for (const auto& folder : m_folders) {
        std::error_code ec;
        if (!std::filesystem::is_directory(folder, ec)) {
            continue;
        }
        for (const auto& entry : std::filesystem::directory_iterator(folder, ec)) {
            if (entry.is_regular_file() && isAssetFile(entry.path())) {
                files.push_back(entry.path());
            }
        }
        if (ec) {
            folderError = "Cannot read " + folder.string() + ": " + ec.message();
        }
    }
    std::ranges::sort(files, {}, [](const std::filesystem::path& p) { return p.filename(); });

    int added = 0;
    for (const auto& file : files) {
        if (!m_loaded.contains(canonicalOrSelf(file)) && loadFile(file)) {
            ++added;
        }
    }
    if (!folderError.empty()) {
        m_status = folderError;
    } else if (added > 0) {
        m_status = "Loaded " + std::to_string(added) + " asset(s)";
    }
}

std::filesystem::path AssetDrawer::importFolderFor(const std::filesystem::path& file) const
{
    const std::string type = upperExtension(file);
    if (type == "GLB" || type == "GLTF") {
        for (const auto& folder : m_folders) {
            if (folder.filename() == "models") {
                return folder;
            }
        }
    }
    return m_folders.empty() ? std::filesystem::path(".") : m_folders.front();
}

bool AssetDrawer::importFile(const std::filesystem::path& source)
{
    if (!isAssetFile(source)) {
        m_status = "Not a mesh or model file (.fbx / .obj / .glb / .gltf): " + source.filename().string();
        return false;
    }

    const std::filesystem::path folder = importFolderFor(source);
    std::filesystem::path destination = folder / source.filename();
    if (canonicalOrSelf(source.parent_path()) != canonicalOrSelf(folder)) {
        if (upperExtension(source) == "GLTF") {
            m_status = "Note: .gltf files with separate .bin / textures need those copied too (prefer .glb)";
        }
        // Don't overwrite a different file with the same name: "Name (2).fbx", "Name (3).fbx", ...
        for (int n = 2; std::filesystem::exists(destination); ++n) {
            destination = folder / (source.stem().string() + " (" + std::to_string(n) + ")" +
                                    source.extension().string());
        }
        std::error_code ec;
        std::filesystem::create_directories(folder, ec);
        std::filesystem::copy_file(source, destination, ec);
        if (ec) {
            m_status = "Copy failed: " + ec.message();
            return false;
        }
    }

    if (m_loaded.contains(canonicalOrSelf(destination))) {
        m_status = destination.filename().string() + " is already in the drawer";
        return true;
    }
    if (!loadFile(destination)) {
        return false;
    }
    m_status = "Added " + destination.filename().string();
    return true;
}

bool AssetDrawer::loadFile(const std::filesystem::path& path)
{
    const std::string name = path.stem().string();
    std::string error;

    // Animated first: an FBX with a skin or animation becomes an animated model.
    if (assets::isSupportedModelFile(path)) {
        if (auto model = assets::loadSkinnedModel(path, &error)) {
            const auto triangles = static_cast<u32>(model->skin.mesh.triangleCount());
            m_assets.push_back({name, upperExtension(path), path, renderer::Renderer::kInvalidMesh, triangles,
                                std::make_shared<const anim::SkinnedModel>(std::move(*model))});
            m_loaded.insert(canonicalOrSelf(path));
            return true;
        }
    }
    if (!assets::isSupportedMeshFile(path)) {
        m_status = path.filename().string() + ": " + error;
        return false;
    }

    const auto data = assets::loadMesh(path, &error);
    if (!data) {
        m_status = path.filename().string() + ": " + error;
        return false;
    }
    const scene::MeshId id = m_renderer->addMesh(name, *data);
    if (id == renderer::Renderer::kInvalidMesh) {
        m_status = path.filename().string() + ": GPU upload failed";
        return false;
    }
    m_loaded.insert(canonicalOrSelf(path));
    m_assets.push_back({name, upperExtension(path), path, id, static_cast<u32>(data->triangleCount()), nullptr});
    return true;
}

void AssetDrawer::draw()
{
    ImGui::Begin("Assets");

    for (const auto& folder : m_folders) {
        ImGui::TextDisabled("%s/", folder.filename().string().c_str());
        ImGui::SameLine();
    }
    if (ImGui::SmallButton("Refresh")) {
        refresh();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Open Folder") && !m_folders.empty()) {
        const std::string url = "file://" + m_folders.front().parent_path().string();
        SDL_OpenURL(url.c_str());
    }
    ImGui::SameLine();
    ImGui::TextDisabled("  drag a tile onto the Viewport, or drop .fbx / .obj / .glb files on the window");
    ImGui::Separator();

    constexpr f32 kTile = 84.0f;
    const ImGuiStyle& style = ImGui::GetStyle();
    const f32 cell = kTile + style.ItemSpacing.x;
    const int columns = std::max(1, static_cast<int>((ImGui::GetContentRegionAvail().x + style.ItemSpacing.x) / cell));

    ImGui::BeginChild("tiles", ImVec2(0, m_status.empty() ? 0.0f : -ImGui::GetFrameHeightWithSpacing()));
    for (usize i = 0; i < m_assets.size(); ++i) {
        if (i % static_cast<usize>(columns) != 0) {
            ImGui::SameLine();
        }
        drawTile(i, kTile);
    }
    ImGui::EndChild();

    if (!m_status.empty()) {
        ImGui::TextDisabled("%s", m_status.c_str());
    }
    ImGui::End();
}

void AssetDrawer::drawTile(usize index, f32 size)
{
    const MeshAsset& asset = m_assets[index];
    ImGui::PushID(static_cast<int>(index));
    ImGui::BeginGroup();

    const ImVec4 color = tileColor(asset);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(color.x * 0.55f, color.y * 0.55f, color.z * 0.55f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(color.x * 0.75f, color.y * 0.75f, color.z * 0.75f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, color);
    const std::string label = asset.animated() ? asset.type + "\nANIM" : asset.type;
    ImGui::Button(label.c_str(), ImVec2(size, size * 0.72f));
    ImGui::PopStyleColor(3);

    if (ImGui::BeginDragDropSource()) {
        if (asset.animated()) {
            ImGui::SetDragDropPayload(kModelPayload, &index, sizeof(index));
            ImGui::Text("%s", asset.name.c_str());
            ImGui::TextDisabled("drop on the Viewport");
        } else {
            ImGui::SetDragDropPayload(kMeshPayload, &asset.id, sizeof(asset.id));
            ImGui::Text("%s", asset.name.c_str());
            ImGui::TextDisabled("drop on the Viewport or a Mesh field");
        }
        ImGui::EndDragDropSource();
    }
    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        m_spawnRequest = index;
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::BeginTooltip();
        ImGui::Text("%s", asset.name.c_str());
        ImGui::TextDisabled("%s, %u triangles", asset.type.c_str(), asset.triangles);
        glm::vec3 extent(0.0f);
        if (asset.animated()) {
            ImGui::TextDisabled("Animated: %zu joints, %zu clip(s)", asset.model->skeleton.jointCount(),
                                asset.model->clips.size());
            extent = asset.model->restBoundsMax - asset.model->restBoundsMin;
        } else if (const renderer::GpuMesh* mesh = m_renderer->mesh(asset.id)) {
            extent = mesh->boundsMax - mesh->boundsMin;
        }
        ImGui::TextDisabled("%.2f x %.2f x %.2f m", static_cast<double>(extent.x), static_cast<double>(extent.y),
                            static_cast<double>(extent.z));
        if (!asset.path.empty()) {
            ImGui::TextDisabled("%s", asset.path.filename().string().c_str());
        }
        ImGui::TextDisabled("Double-click: add to scene");
        ImGui::EndTooltip();
    }

    // Name under the tile, clipped to the tile width.
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(size, ImGui::GetTextLineHeight()));
    ImGui::GetWindowDrawList()->PushClipRect(pos, ImVec2(pos.x + size, pos.y + ImGui::GetTextLineHeight()), true);
    const f32 textWidth = ImGui::CalcTextSize(asset.name.c_str()).x;
    const f32 x = pos.x + std::max(0.0f, (size - textWidth) * 0.5f);
    ImGui::GetWindowDrawList()->AddText(ImVec2(x, pos.y), ImGui::GetColorU32(ImGuiCol_Text), asset.name.c_str());
    ImGui::GetWindowDrawList()->PopClipRect();

    ImGui::EndGroup();
    ImGui::PopID();
}

} // namespace kinetic::editor
