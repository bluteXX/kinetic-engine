#include "AssetDrawer.h"

#include <kinetic/animation/ClipTools.h>
#include <kinetic/animation/Locomotion.h>
#include <kinetic/assets/MeshLoader.h>
#include <kinetic/assets/ModelLoader.h>
#include <kinetic/assets/TextureLoader.h>
#include <kinetic/core/Log.h>

#include <SDL3/SDL_misc.h>
#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <functional>
#include <system_error>

namespace kinetic::editor {

namespace {

namespace fs = std::filesystem;

std::string upperExtension(const fs::path& path)
{
    std::string ext = path.extension().string();
    if (!ext.empty() && ext[0] == '.') {
        ext.erase(0, 1);
    }
    std::ranges::transform(ext, ext.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return ext;
}

bool isAssetFile(const fs::path& path)
{
    return assets::isSupportedMeshFile(path) || assets::isSupportedModelFile(path);
}

ImVec4 tileColor(const MeshAsset& asset)
{
    if (asset.preset == "character") return {0.15f, 0.62f, 0.62f, 1.0f};
    if (asset.isPreset()) return {0.45f, 0.45f, 0.42f, 1.0f};
    if (asset.animated()) return {0.62f, 0.38f, 0.85f, 1.0f};
    if (asset.type == "FBX") return {0.80f, 0.47f, 0.20f, 1.0f};
    if (asset.type == "OBJ") return {0.27f, 0.62f, 0.38f, 1.0f};
    return {0.35f, 0.42f, 0.58f, 1.0f}; // built-in
}

fs::path canonicalOrSelf(const fs::path& path)
{
    std::error_code ec;
    auto result = fs::weakly_canonical(path, ec);
    return ec ? path.lexically_normal() : result;
}

bool hidden(const fs::path& path)
{
    const std::string name = path.filename().string();
    return !name.empty() && name[0] == '.';
}

// `path` relative to `base` if it lies inside it (no "..").
std::optional<fs::path> relativeInside(const fs::path& path, const fs::path& base)
{
    const fs::path rel = canonicalOrSelf(path).lexically_relative(canonicalOrSelf(base));
    if (rel.empty() || *rel.begin() == "..") {
        return std::nullopt;
    }
    return rel;
}

std::string pathString(const fs::path& path) { return path.string(); }

void showInExplorer(const fs::path& folder)
{
    const std::string url = "file:///" + canonicalOrSelf(folder).generic_string();
    SDL_OpenURL(url.c_str());
}

// Draws a folder icon (tab + body) in a tile-sized button area.
void folderIcon(ImDrawList* draw, const ImVec2& p0, const ImVec2& size, bool hovered)
{
    const ImU32 color = hovered ? IM_COL32(236, 196, 92, 255) : IM_COL32(206, 166, 72, 255);
    const ImU32 dark = IM_COL32(150, 116, 44, 255);
    const f32 w = size.x * 0.62f;
    const f32 h = size.y * 0.55f;
    const ImVec2 a(p0.x + (size.x - w) * 0.5f, p0.y + (size.y - h) * 0.55f);
    draw->AddRectFilled(ImVec2(a.x, a.y - h * 0.16f), ImVec2(a.x + w * 0.42f, a.y + 2.0f), dark, 2.0f);
    draw->AddRectFilled(a, ImVec2(a.x + w, a.y + h), color, 3.0f);
}

} // namespace

void AssetDrawer::init(renderer::Renderer& renderer, std::filesystem::path root)
{
    m_renderer = &renderer;
    m_root = canonicalOrSelf(root);
    std::error_code ec;
    fs::create_directories(m_root, ec);
    m_current = m_root;

    m_assets.push_back({"Triangle", "BUILT-IN", {}, scene::kTriangleMesh, 1, nullptr});
    refresh();
}

void AssetDrawer::addPreset(MeshAsset asset)
{
    // Keep presets together right after the built-in triangle.
    auto it = m_assets.begin();
    while (it != m_assets.end() && (it->type == "BUILT-IN" || it->isPreset())) {
        ++it;
    }
    m_assets.insert(it, std::move(asset));
}

bool AssetDrawer::insideRoot(const std::filesystem::path& path) const
{
    return canonicalOrSelf(path) == m_root || relativeInside(path, m_root).has_value();
}

void AssetDrawer::refresh()
{
    std::vector<fs::path> files;
    std::error_code ec;
    fs::recursive_directory_iterator it(m_root, fs::directory_options::skip_permission_denied, ec);
    for (; !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
        if (hidden(it->path())) {
            if (it->is_directory()) {
                it.disable_recursion_pending();
            }
            continue;
        }
        if (it->is_regular_file() && isAssetFile(it->path())) {
            files.push_back(it->path());
        }
    }
    if (ec) {
        m_status = "Cannot read " + m_root.string() + ": " + ec.message();
    }
    std::ranges::sort(files);

    int added = 0;
    for (const auto& file : files) {
        if (!m_loaded.contains(canonicalOrSelf(file)) && loadFile(file)) {
            ++added;
        }
    }
    if (added > 0) {
        m_status = "Loaded " + std::to_string(added) + " asset(s)";
    }
    m_dirty = true;
}

void AssetDrawer::rebuildTree()
{
    const std::function<void(FolderNode&)> scan = [&](FolderNode& node) {
        node.children.clear();
        std::error_code ec;
        for (const auto& entry : fs::directory_iterator(node.path, ec)) {
            if (entry.is_directory() && !hidden(entry.path())) {
                node.children.push_back({entry.path(), {}});
            }
        }
        std::ranges::sort(node.children, {}, [](const FolderNode& n) { return n.path.filename().string(); });
        for (FolderNode& child : node.children) {
            scan(child);
        }
    };
    m_tree = {m_root, {}};
    scan(m_tree);
}

void AssetDrawer::scanCurrentFiles()
{
    m_currentFolders.clear();
    m_currentImages.clear();
    if (m_builtIn || m_flat) {
        return;
    }
    std::error_code ec;
    if (!fs::is_directory(m_current, ec)) {
        m_current = m_root;
    }
    for (const auto& entry : fs::directory_iterator(m_current, ec)) {
        if (hidden(entry.path())) {
            continue;
        }
        if (entry.is_directory()) {
            m_currentFolders.push_back(entry.path());
        } else if (entry.is_regular_file() && (!isAssetFile(entry.path()) || m_animationFiles.contains(canonicalOrSelf(entry.path())))) {
            m_currentImages.push_back(entry.path()); // textures, animation clips and other files
        }
    }
    std::ranges::sort(m_currentFolders);
    std::ranges::sort(m_currentImages);
}

void AssetDrawer::openFolder(const std::filesystem::path& folder)
{
    m_current = canonicalOrSelf(folder);
    m_builtIn = false;
    m_flat = false;
    m_dirty = true;
}

bool AssetDrawer::importFile(const std::filesystem::path& source)
{
    if (!isAssetFile(source)) {
        m_status = "Not a mesh or model file (.fbx / .obj / .glb / .gltf): " + source.filename().string();
        return false;
    }
    // Into the folder being shown (the root while Built-in / All is shown).
    const fs::path folder = (m_builtIn || m_flat) ? m_root : m_current;
    fs::path destination = source;
    if (!insideRoot(source)) {
        if (upperExtension(source) == "GLTF") {
            m_status = "Note: .gltf files with separate .bin / textures need those copied too (prefer .glb)";
        }
        destination = folder / source.filename();
        // Don't overwrite a different file with the same name: "Name (2).fbx", "Name (3).fbx", ...
        for (int n = 2; fs::exists(destination); ++n) {
            destination = folder / (source.stem().string() + " (" + std::to_string(n) + ")" + source.extension().string());
        }
        std::error_code ec;
        fs::create_directories(folder, ec);
        fs::copy_file(source, destination, ec);
        if (ec) {
            m_status = "Copy failed: " + ec.message();
            return false;
        }
    }
    m_dirty = true;
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

// ---------------------------------------------------------------------------------------------- //
// File operations

bool AssetDrawer::createFolder(const std::filesystem::path& parent, const std::string& name)
{
    if (name.empty() || name.find_first_of("/\\:*?\"<>|") != std::string::npos || !insideRoot(parent)) {
        m_status = "Invalid folder name";
        return false;
    }
    const fs::path folder = parent / name;
    std::error_code ec;
    if (fs::exists(folder, ec)) {
        m_status = "'" + name + "' already exists";
        return false;
    }
    fs::create_directory(folder, ec);
    if (ec) {
        m_status = "Cannot create folder: " + ec.message();
        return false;
    }
    m_status = "Created " + name + "/";
    m_dirty = true;
    return true;
}

void AssetDrawer::retargetPaths(const std::filesystem::path& from, const std::filesystem::path& to)
{
    const fs::path a = canonicalOrSelf(from);
    const auto move = [&](fs::path& p) {
        if (p.empty()) {
            return;
        }
        const fs::path c = canonicalOrSelf(p);
        if (c == a) {
            p = to;
        } else if (const auto rel = relativeInside(c, a)) {
            p = to / *rel;
        }
    };
    for (MeshAsset& asset : m_assets) {
        move(asset.path);
        for (fs::path& t : asset.textureFiles) {
            move(t);
        }
        if (!asset.path.empty() && !asset.isPreset()) {
            asset.name = asset.path.stem().string();
        }
    }
    std::set<fs::path> loaded;
    for (fs::path p : m_loaded) {
        move(p);
        loaded.insert(canonicalOrSelf(p));
    }
    m_loaded = std::move(loaded);
    move(m_current);
}

bool AssetDrawer::movePath(const std::filesystem::path& source, const std::filesystem::path& folder)
{
    const fs::path src = canonicalOrSelf(source);
    const fs::path dstFolder = canonicalOrSelf(folder);
    if (!insideRoot(src) || src == m_root || !insideRoot(dstFolder)) {
        return false;
    }
    if (src.parent_path() == dstFolder) {
        return false; // already there
    }
    if (dstFolder == src || relativeInside(dstFolder, src)) {
        m_status = "Cannot move a folder into itself";
        return false;
    }
    const fs::path dst = dstFolder / src.filename();
    std::error_code ec;
    if (fs::exists(dst, ec)) {
        m_status = "'" + src.filename().string() + "' already exists in " + dstFolder.filename().string();
        return false;
    }
    fs::rename(src, dst, ec);
    if (ec) {
        m_status = "Move failed: " + ec.message();
        return false;
    }
    retargetPaths(src, dst);
    m_status = "Moved " + src.filename().string() + " to " + dstFolder.filename().string() + "/";
    m_dirty = true;
    return true;
}

bool AssetDrawer::moveAsset(usize index, const std::filesystem::path& folder)
{
    if (index >= m_assets.size() || m_assets[index].path.empty()) {
        m_status = "Built-in assets cannot be moved";
        return false;
    }
    const fs::path model = canonicalOrSelf(m_assets[index].path);
    const fs::path oldFolder = model.parent_path();
    const fs::path newFolder = canonicalOrSelf(folder);
    // Textures in the model's folder (or below it) travel with it, keeping their layout
    // (e.g. textures/hero_basecolor.png), so the model still finds them next time.
    std::vector<std::pair<fs::path, fs::path>> textures;
    for (const fs::path& t : m_assets[index].textureFiles) {
        if (const auto rel = relativeInside(t, oldFolder)) {
            textures.emplace_back(canonicalOrSelf(t), newFolder / *rel);
        }
    }
    if (!movePath(model, newFolder)) {
        return false;
    }
    std::error_code ec;
    for (const auto& [from, to] : textures) {
        if (!fs::exists(from, ec) || fs::exists(to, ec)) {
            continue;
        }
        fs::create_directories(to.parent_path(), ec);
        fs::rename(from, to, ec);
        if (!ec) {
            retargetPaths(from, to);
        }
    }
    // Remove texture folders left empty (e.g. the old textures/).
    for (const auto& [from, to] : textures) {
        for (fs::path dir = from.parent_path(); dir != oldFolder && relativeInside(dir, oldFolder); dir = dir.parent_path()) {
            if (!fs::is_empty(dir, ec) || !fs::remove(dir, ec)) {
                break;
            }
        }
    }
    if (!textures.empty()) {
        m_status += " (with " + std::to_string(textures.size()) + " texture(s))";
    }
    m_dirty = true;
    return true;
}

bool AssetDrawer::renamePath(const std::filesystem::path& source, const std::string& newName)
{
    const fs::path src = canonicalOrSelf(source);
    if (newName.empty() || newName.find_first_of("/\\:*?\"<>|") != std::string::npos || !insideRoot(src) || src == m_root) {
        m_status = "Invalid name";
        return false;
    }
    std::string name = newName;
    // Files keep their extension unless a new one is typed.
    if (fs::is_regular_file(src) && fs::path(name).extension().empty()) {
        name += src.extension().string();
    }
    const fs::path dst = src.parent_path() / name;
    std::error_code ec;
    if (dst == src) {
        return true;
    }
    if (fs::exists(dst, ec)) {
        m_status = "'" + name + "' already exists";
        return false;
    }
    fs::rename(src, dst, ec);
    if (ec) {
        m_status = "Rename failed: " + ec.message();
        return false;
    }
    retargetPaths(src, dst);
    m_status = "Renamed to " + name;
    m_dirty = true;
    return true;
}

bool AssetDrawer::loadFile(const std::filesystem::path& path)
{
    const std::string name = path.stem().string();
    std::string error;

    // Animated first: an FBX with a skin or animation becomes an animated model.
    if (assets::isSupportedModelFile(path)) {
        if (auto model = assets::loadSkinnedModel(path, &error)) {
            // Static rest-pose copy: used when the model is placed without animation.
            const scene::MeshId restMesh = m_renderer->addMesh(name, anim::skinnedRestMesh(*model));
            if (restMesh == renderer::Renderer::kInvalidMesh) {
                m_status = path.filename().string() + ": GPU upload failed";
                return false;
            }
            // Animation-only files in an "animations" folder next to the model (e.g. Mixamo
            // downloads without skin) are retargeted onto it and become its clips.
            std::error_code ec;
            const std::filesystem::path animations = path.parent_path() / "animations";
            if (std::filesystem::is_directory(animations, ec)) {
                std::vector<std::filesystem::path> files;
                for (const auto& entry : std::filesystem::directory_iterator(animations, ec)) {
                    if (entry.is_regular_file() && assets::isSupportedModelFile(entry.path())) {
                        files.push_back(entry.path());
                    }
                }
                std::ranges::sort(files);
                for (const auto& file : files) {
                    std::string animError;
                    if (!assets::importAnimationsInto(*model, file, &animError).empty()) {
                        m_animationFiles.insert(canonicalOrSelf(file));
                        m_loaded.insert(canonicalOrSelf(file));
                    } else {
                        KN_WARN("{}: {}", file.filename().string(), animError);
                    }
                }
            }
            // Locomotion clips (idle / walk / run / turn): centre roots, close loops, cut in-place
            // turns - so they blend without the body sliding or popping. Attacks: root motion.
            if (!anim::findLocomotionClips(model->skeleton, model->clips, anim::findLegChains(model->skeleton), 1.0f).empty()) {
                for (const std::string& line : anim::prepareLocomotionClips(*model).lines) {
                    KN_INFO("{}: {}", path.filename().string(), line);
                }
                model->locomotionPrepared = true;
            }
            const auto triangles = static_cast<u32>(model->skin.mesh.triangleCount());
            MeshAsset asset{name, upperExtension(path), path, restMesh, triangles, nullptr};
            loadMaterial(model->skin.mesh.material, asset);
            asset.model = std::make_shared<const anim::SkinnedModel>(std::move(*model));
            m_assets.push_back(std::move(asset));
            m_loaded.insert(canonicalOrSelf(path));
            return true;
        }
    }
    // Animation-only file (skeleton + clips, no mesh): it belongs to a model, not the scene.
    if (assets::isSupportedModelFile(path)) {
        if (assets::loadAnimationFile(path)) {
            m_animationFiles.insert(canonicalOrSelf(path));
            m_loaded.insert(canonicalOrSelf(path));
            m_dirty = true;
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
    MeshAsset asset{name, upperExtension(path), path, id, static_cast<u32>(data->triangleCount()), nullptr};
    loadMaterial(data->material, asset);
    m_assets.push_back(std::move(asset));
    return true;
}

void AssetDrawer::loadMaterial(const MaterialDesc& material, MeshAsset& asset)
{
    for (const fs::path& file : {material.baseColorTexture, material.normalTexture, material.roughnessTexture,
                                 material.metallicTexture}) {
        if (!file.empty()) {
            asset.textureFiles.push_back(file);
        }
    }
    if (const scene::TextureId id = loadTexture(material.baseColorTexture); id != renderer::Renderer::kInvalidTexture) {
        asset.baseColorTexture = id;
    }
    if (const scene::TextureId id = loadTexture(material.normalTexture); id != renderer::Renderer::kInvalidTexture) {
        asset.normalTexture = id;
    }
}

scene::TextureId AssetDrawer::loadTexture(const std::filesystem::path& path)
{
    if (path.empty()) {
        return renderer::Renderer::kInvalidTexture;
    }
    const std::string key = canonicalOrSelf(path).string();
    if (const scene::TextureId existing = m_renderer->findTexture(key); existing != renderer::Renderer::kInvalidTexture) {
        return existing;
    }
    std::string error;
    const auto image = assets::loadImage(path, &error);
    if (!image) {
        m_status = path.filename().string() + ": " + error;
        return renderer::Renderer::kInvalidTexture;
    }
    KN_INFO("Texture '{}': {} x {}", path.filename().string(), image->width, image->height);
    return m_renderer->addTexture(key, *image);
}

// ---------------------------------------------------------------------------------------------- //
// UI

void AssetDrawer::draw()
{
    ImGui::Begin("Assets");
    if (m_dirty) {
        rebuildTree();
        scanCurrentFiles();
        m_dirty = false;
    }
    drawToolbar();
    ImGui::Separator();

    const f32 statusHeight = m_status.empty() ? 0.0f : ImGui::GetFrameHeightWithSpacing();
    if (ImGui::BeginTable("##browser", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV,
                          ImVec2(0, -statusHeight))) {
        ImGui::TableSetupColumn("tree", ImGuiTableColumnFlags_WidthFixed, 190.0f);
        ImGui::TableSetupColumn("contents", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableNextRow();

        ImGui::TableSetColumnIndex(0);
        ImGui::BeginChild("##tree");
        drawBuiltInNode();
        drawTree(m_tree);
        ImGui::EndChild();

        ImGui::TableSetColumnIndex(1);
        ImGui::BeginChild("##tiles");
        drawContents(84.0f);
        ImGui::EndChild();
        ImGui::EndTable();
    }
    if (!m_status.empty()) {
        ImGui::TextDisabled("%s", m_status.c_str());
    }
    drawRenamePopup();
    ImGui::End();
}

void AssetDrawer::drawToolbar()
{
    const bool atRoot = m_builtIn || m_flat || m_current == m_root;
    ImGui::BeginDisabled(atRoot);
    if (ImGui::ArrowButton("##up", ImGuiDir_Up)) {
        openFolder(m_current.parent_path());
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled | ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip("Parent folder");
    }
    ImGui::SameLine();

    // Breadcrumb: assets / models / Warrior - each part opens that folder and accepts drops.
    if (m_builtIn) {
        ImGui::TextUnformatted("Built-in");
    } else if (m_flat) {
        ImGui::TextUnformatted("All assets");
    } else {
        fs::path walk = m_root;
        std::vector<fs::path> parts{m_root};
        if (const auto rel = relativeInside(m_current, m_root)) {
            for (const fs::path& part : *rel) {
                walk /= part;
                parts.push_back(walk);
            }
        }
        for (usize i = 0; i < parts.size(); ++i) {
            if (i > 0) {
                ImGui::SameLine(0.0f, 2.0f);
                ImGui::TextDisabled(">");
                ImGui::SameLine(0.0f, 2.0f);
            }
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::SmallButton(parts[i].filename().string().c_str())) {
                openFolder(parts[i]);
            }
            folderDropTarget(parts[i]);
            ImGui::PopID();
        }
    }

    ImGui::SameLine(0.0f, 20.0f);
    ImGui::BeginDisabled(m_builtIn || m_flat);
    if (ImGui::SmallButton("+ Folder")) {
        m_renameTarget = m_current;
        m_renameIsNewFolder = true;
        m_renameBuffer = {};
        std::snprintf(m_renameBuffer.data(), m_renameBuffer.size(), "New Folder");
        m_openRenamePopup = true;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::SmallButton("Refresh")) {
        refresh();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Explorer")) {
        showInExplorer(m_builtIn || m_flat ? m_root : m_current);
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("All", &m_flat)) {
        m_builtIn = false;
        m_dirty = true;
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip("Every asset in one list, from all folders");
    }
    ImGui::SameLine();
    ImGui::TextDisabled("  drag tiles onto folders to move them, onto the Viewport to place them");
}

void AssetDrawer::drawBuiltInNode()
{
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_SpanAvailWidth;
    if (m_builtIn) {
        flags |= ImGuiTreeNodeFlags_Selected;
    }
    ImGui::TreeNodeEx("##builtin", flags, "Built-in");
    if (ImGui::IsItemClicked()) {
        m_builtIn = true;
        m_flat = false;
        m_dirty = true;
    }
}

void AssetDrawer::drawTree(const FolderNode& node)
{
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick |
                               ImGuiTreeNodeFlags_SpanAvailWidth;
    if (node.children.empty()) {
        flags |= ImGuiTreeNodeFlags_Leaf;
    }
    if (node.path == m_root) {
        flags |= ImGuiTreeNodeFlags_DefaultOpen;
    }
    const bool current = !m_builtIn && !m_flat && canonicalOrSelf(node.path) == canonicalOrSelf(m_current);
    if (current) {
        flags |= ImGuiTreeNodeFlags_Selected;
    }
    // Keep the current folder's ancestors open.
    if (!m_builtIn && !m_flat && relativeInside(m_current, node.path)) {
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    }
    const std::string label = node.path.filename().string();
    const bool open = ImGui::TreeNodeEx(pathString(node.path).c_str(), flags, "%s", label.c_str());
    if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
        openFolder(node.path);
    }
    folderDropTarget(node.path);
    if (node.path != m_root && ImGui::BeginDragDropSource()) {
        const std::string payload = pathString(node.path);
        ImGui::SetDragDropPayload(kPathPayload, payload.c_str(), payload.size() + 1);
        ImGui::Text("%s/", label.c_str());
        ImGui::TextDisabled("drop on a folder to move it");
        ImGui::EndDragDropSource();
    }
    contextMenuFor(node.path, true);
    if (open) {
        for (const FolderNode& child : node.children) {
            drawTree(child);
        }
        ImGui::TreePop();
    }
}

void AssetDrawer::drawContents(f32 tileSize)
{
    const ImGuiStyle& style = ImGui::GetStyle();
    const f32 cell = tileSize + style.ItemSpacing.x;
    const int columns = std::max(1, static_cast<int>((ImGui::GetContentRegionAvail().x + style.ItemSpacing.x) / cell));
    int column = 0;
    const auto next = [&] {
        if (column++ % columns != 0) {
            ImGui::SameLine();
        }
    };

    if (m_builtIn || m_flat) {
        for (usize i = 0; i < m_assets.size(); ++i) {
            const bool builtIn = m_assets[i].path.empty();
            if (m_builtIn != builtIn && !m_flat) {
                continue;
            }
            next();
            drawTile(i, tileSize);
        }
    } else {
        for (const fs::path& folder : m_currentFolders) {
            next();
            drawFolderTile(folder, tileSize);
        }
        const fs::path here = canonicalOrSelf(m_current);
        for (usize i = 0; i < m_assets.size(); ++i) {
            if (!m_assets[i].path.empty() && canonicalOrSelf(m_assets[i].path).parent_path() == here) {
                next();
                drawTile(i, tileSize);
            }
        }
        for (const fs::path& file : m_currentImages) {
            next();
            drawFileTile(file, tileSize);
        }
        if (column == 0) {
            ImGui::TextDisabled("Empty folder. Drag assets here, or drop .fbx / .obj / .glb files on the window.");
        }
    }

    // Right-click on empty space: folder actions.
    if (!m_builtIn && !m_flat && ImGui::BeginPopupContextWindow("##contentsMenu",
                                                                ImGuiPopupFlags_MouseButtonRight | ImGuiPopupFlags_NoOpenOverItems)) {
        if (ImGui::MenuItem("New Folder")) {
            m_renameTarget = m_current;
            m_renameIsNewFolder = true;
            m_renameBuffer = {};
            std::snprintf(m_renameBuffer.data(), m_renameBuffer.size(), "New Folder");
            m_openRenamePopup = true;
        }
        if (ImGui::MenuItem("Refresh")) {
            refresh();
        }
        if (ImGui::MenuItem("Show in Explorer")) {
            showInExplorer(m_current);
        }
        ImGui::EndPopup();
    }
}

void AssetDrawer::drawFolderTile(const std::filesystem::path& folder, f32 size)
{
    ImGui::PushID(pathString(folder).c_str());
    ImGui::BeginGroup();
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 box(size, size * 0.72f);
    ImGui::InvisibleButton("##folder", box);
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(p0, ImVec2(p0.x + box.x, p0.y + box.y), hovered ? IM_COL32(60, 62, 70, 255) : IM_COL32(44, 46, 52, 255), 4.0f);
    folderIcon(draw, p0, box, hovered);
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        openFolder(folder);
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip("%s/\nDouble-click to open, drag to move, drop assets here", folder.filename().string().c_str());
    }
    folderDropTarget(folder);
    if (ImGui::BeginDragDropSource()) {
        const std::string payload = pathString(folder);
        ImGui::SetDragDropPayload(kPathPayload, payload.c_str(), payload.size() + 1);
        ImGui::Text("%s/", folder.filename().string().c_str());
        ImGui::EndDragDropSource();
    }
    contextMenuFor(folder, true);

    const std::string name = folder.filename().string();
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(size, ImGui::GetTextLineHeight()));
    draw->PushClipRect(pos, ImVec2(pos.x + size, pos.y + ImGui::GetTextLineHeight()), true);
    const f32 x = pos.x + std::max(0.0f, (size - ImGui::CalcTextSize(name.c_str()).x) * 0.5f);
    draw->AddText(ImVec2(x, pos.y), ImGui::GetColorU32(ImGuiCol_Text), name.c_str());
    draw->PopClipRect();
    ImGui::EndGroup();
    ImGui::PopID();
}

void AssetDrawer::drawFileTile(const std::filesystem::path& file, f32 size)
{
    ImGui::PushID(pathString(file).c_str());
    ImGui::BeginGroup();
    const bool image = assets::isSupportedImageFile(file);
    const bool clip = m_animationFiles.contains(canonicalOrSelf(file));
    ImGui::PushStyleColor(ImGuiCol_Button, clip ? ImVec4(0.30f, 0.22f, 0.40f, 1.0f) : ImVec4(0.24f, 0.25f, 0.28f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, clip ? ImVec4(0.40f, 0.30f, 0.52f, 1.0f) : ImVec4(0.32f, 0.33f, 0.37f, 1.0f));
    const std::string label = upperExtension(file) + (image ? "\nTEXTURE" : (clip ? "\nCLIP" : ""));
    ImGui::Button(label.c_str(), ImVec2(size, size * 0.72f));
    ImGui::PopStyleColor(2);
    if (ImGui::BeginDragDropSource()) {
        const std::string payload = pathString(file);
        ImGui::SetDragDropPayload(kPathPayload, payload.c_str(), payload.size() + 1);
        ImGui::Text("%s", file.filename().string().c_str());
        ImGui::EndDragDropSource();
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        std::error_code ec;
        const auto bytes = fs::file_size(file, ec);
        ImGui::SetTooltip("%s\n%.1f MB%s", file.filename().string().c_str(), static_cast<double>(bytes) / (1024.0 * 1024.0),
                          clip ? "\nAnimation clip: retargeted onto the model in the folder above it" : "");
    }
    contextMenuFor(file, false);

    const std::string name = file.filename().string();
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(size, ImGui::GetTextLineHeight()));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(pos, ImVec2(pos.x + size, pos.y + ImGui::GetTextLineHeight()), true);
    const f32 x = pos.x + std::max(0.0f, (size - ImGui::CalcTextSize(name.c_str()).x) * 0.5f);
    draw->AddText(ImVec2(x, pos.y), ImGui::GetColorU32(ImGuiCol_TextDisabled), name.c_str());
    draw->PopClipRect();
    ImGui::EndGroup();
    ImGui::PopID();
}

void AssetDrawer::folderDropTarget(const std::filesystem::path& folder)
{
    if (!ImGui::BeginDragDropTarget()) {
        return;
    }
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetPayload)) {
        usize index = 0;
        std::memcpy(&index, payload->Data, sizeof(index));
        moveAsset(index, folder);
    }
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kPathPayload)) {
        movePath(fs::path(static_cast<const char*>(payload->Data)), folder);
    }
    ImGui::EndDragDropTarget();
}

void AssetDrawer::contextMenuFor(const std::filesystem::path& path, bool isFolder)
{
    if (!ImGui::BeginPopupContextItem("##item")) {
        return;
    }
    const bool root = canonicalOrSelf(path) == m_root;
    if (isFolder && ImGui::MenuItem("Open")) {
        openFolder(path);
    }
    if (isFolder && ImGui::MenuItem("New Folder Inside")) {
        m_renameTarget = path;
        m_renameIsNewFolder = true;
        m_renameBuffer = {};
        std::snprintf(m_renameBuffer.data(), m_renameBuffer.size(), "New Folder");
        m_openRenamePopup = true;
    }
    if (!root && ImGui::MenuItem("Rename")) {
        m_renameTarget = path;
        m_renameIsNewFolder = false;
        m_renameBuffer = {};
        const std::string name = isFolder ? path.filename().string() : path.stem().string();
        std::snprintf(m_renameBuffer.data(), m_renameBuffer.size(), "%s", name.c_str());
        m_openRenamePopup = true;
    }
    std::error_code ec;
    const bool canDelete = !root && (isFolder ? fs::is_empty(path, ec) : !isAssetFile(path));
    if (ImGui::MenuItem("Delete", nullptr, false, canDelete)) {
        if (fs::remove(path, ec)) {
            m_status = "Deleted " + path.filename().string();
        } else {
            m_status = "Delete failed: " + ec.message();
        }
        m_dirty = true;
    }
    if (!canDelete && !root && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip(isFolder ? "Only empty folders can be deleted here" : "Loaded assets are not deleted from the editor");
    }
    if (ImGui::MenuItem("Show in Explorer")) {
        showInExplorer(isFolder ? path : path.parent_path());
    }
    ImGui::EndPopup();
}

void AssetDrawer::drawRenamePopup()
{
    if (m_openRenamePopup) {
        ImGui::OpenPopup("##rename");
        m_openRenamePopup = false;
    }
    if (!ImGui::BeginPopup("##rename")) {
        return;
    }
    ImGui::TextUnformatted(m_renameIsNewFolder ? "New folder name" : "Rename to");
    if (ImGui::IsWindowAppearing()) {
        ImGui::SetKeyboardFocusHere();
    }
    ImGui::SetNextItemWidth(240.0f);
    const bool enter = ImGui::InputText("##name", m_renameBuffer.data(), m_renameBuffer.size(),
                                        ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);
    if (enter || ImGui::Button("OK")) {
        const std::string name = m_renameBuffer.data();
        const bool ok = m_renameIsNewFolder ? createFolder(m_renameTarget, name) : renamePath(m_renameTarget, name);
        if (ok) {
            ImGui::CloseCurrentPopup();
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
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
    std::string label = asset.animated() ? asset.type + "\nANIM" : asset.type;
    if (asset.preset == "character") {
        label = "CHARACTER\nPHYS";
    } else if (asset.isPreset()) {
        label = "PRESET\n" + asset.name;
    }
    ImGui::Button(label.c_str(), ImVec2(size, size * 0.72f));
    ImGui::PopStyleColor(3);
    if (!asset.path.empty()) {
        contextMenuFor(asset.path, false);
    }

    if (ImGui::BeginDragDropSource()) {
        ImGui::SetDragDropPayload(kAssetPayload, &index, sizeof(index));
        ImGui::Text("%s", asset.name.c_str());
        ImGui::TextDisabled("drop on the Viewport or an actor's Mesh field");
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
        if (asset.preset == "character") {
            ImGui::TextDisabled("Playable physics character (Play: WASD, LMB attack, R ragdoll)");
        } else if (asset.preset == "floor" || asset.preset == "stone") {
            ImGui::TextDisabled("Static box collider");
        }
        if (asset.animated()) {
            ImGui::TextDisabled("Animated: %zu joints, %zu clip(s)", asset.model->skeleton.jointCount(),
                                asset.model->clips.size());
            for (const anim::AnimationClip& clip : asset.model->clips) {
                const bool physics = clip.kind() == anim::ClipKind::Physics;
                ImGui::TextColored(physics ? ImVec4(1.0f, 0.6f, 0.25f, 1.0f) : ImVec4(0.45f, 0.7f, 1.0f, 1.0f), "  %s %s",
                                   physics ? "[PHYS]" : "[KIN] ", clip.name().c_str());
            }
            extent = asset.model->restBoundsMax - asset.model->restBoundsMin;
        } else if (const renderer::GpuMesh* mesh = m_renderer->mesh(asset.id)) {
            extent = mesh->boundsMax - mesh->boundsMin;
        }
        ImGui::TextDisabled("%.2f x %.2f x %.2f m", static_cast<double>(extent.x), static_cast<double>(extent.y),
                            static_cast<double>(extent.z));
        if (!asset.path.empty()) {
            const auto rel = relativeInside(asset.path, m_root);
            ImGui::TextDisabled("%s", (rel ? *rel : asset.path).generic_string().c_str());
            if (asset.textured()) {
                ImGui::TextDisabled("Textured (%zu texture file(s))", asset.textureFiles.size());
            }
        }
        ImGui::TextDisabled("Double-click: add to scene. Drag onto a folder: move. Right-click: rename.");
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

const MeshAsset* AssetDrawer::findByMesh(scene::MeshId id) const
{
    const auto it = std::ranges::find(m_assets, id, &MeshAsset::id);
    return it != m_assets.end() ? &*it : nullptr;
}

const MeshAsset* AssetDrawer::findByModel(const anim::SkinnedModel* model) const
{
    const auto it = std::ranges::find_if(m_assets, [&](const MeshAsset& a) { return a.model.get() == model; });
    return it != m_assets.end() ? &*it : nullptr;
}

std::optional<usize> AssetDrawer::assetCombo(const char* label, const MeshAsset* current) const
{
    std::optional<usize> picked;
    if (ImGui::BeginCombo(label, current ? current->name.c_str() : "<none>")) {
        for (usize i = 0; i < m_assets.size(); ++i) {
            const MeshAsset& asset = m_assets[i];
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::Selectable(asset.name.c_str(), &asset == current)) {
                picked = i;
            }
            ImGui::SameLine();
            ImGui::TextDisabled("%s%s, %u tris", asset.type.c_str(), asset.animated() ? " ANIM" : "", asset.triangles);
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetPayload)) {
            usize index = 0;
            std::memcpy(&index, payload->Data, sizeof(index));
            picked = index;
        }
        ImGui::EndDragDropTarget();
    }
    return picked;
}

} // namespace kinetic::editor
