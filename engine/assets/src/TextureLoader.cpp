#include "kinetic/assets/TextureLoader.h"

#include "kinetic/core/Log.h"

#include <SDL3/SDL_surface.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <string_view>
#include <system_error>

namespace kinetic::assets {

namespace {

namespace fs = std::filesystem;

std::string lower(std::string s)
{
    std::ranges::transform(s, s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool isFile(const fs::path& p)
{
    std::error_code ec;
    return !p.empty() && fs::is_regular_file(p, ec);
}

// File named `name` (case-insensitive) somewhere below `root` (bounded depth), or empty.
fs::path searchBelow(const fs::path& root, const std::string& name, int maxDepth)
{
    std::error_code ec;
    if (!fs::is_directory(root, ec)) {
        return {};
    }
    const std::string wanted = lower(name);
    fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, ec);
    for (; !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
        if (it.depth() > maxDepth) {
            it.disable_recursion_pending();
            continue;
        }
        if (it->is_regular_file(ec) && lower(it->path().filename().string()) == wanted) {
            return it->path();
        }
    }
    return {};
}

// Texture-set suffixes, per map. The first entry is the canonical one.
constexpr std::array<std::string_view, 6> kBaseColorKeys = {"basecolor", "base_color", "albedo", "diffuse", "color", "col"};
constexpr std::array<std::string_view, 4> kNormalKeys = {"normal", "normal_gl", "nrm", "norm"};
constexpr std::array<std::string_view, 2> kRoughnessKeys = {"roughness", "rough"};
constexpr std::array<std::string_view, 3> kMetallicKeys = {"metallic", "metalness", "metal"};

template <usize N>
fs::path findCompanion(const fs::path& baseColor, std::string_view baseKey, const std::array<std::string_view, N>& keys)
{
    const std::string stem = lower(baseColor.stem().string());
    const usize at = stem.rfind(baseKey);
    if (at == std::string::npos) {
        return {};
    }
    std::error_code ec;
    for (const std::string_view key : keys) {
        const std::string wanted = stem.substr(0, at) + std::string(key) + stem.substr(at + baseKey.size());
        for (const auto& entry : fs::directory_iterator(baseColor.parent_path(), ec)) {
            if (entry.is_regular_file(ec) && lower(entry.path().stem().string()) == wanted &&
                isSupportedImageFile(entry.path())) {
                return entry.path();
            }
        }
    }
    return {};
}

} // namespace

bool isSupportedImageFile(const std::filesystem::path& path)
{
    const std::string ext = lower(path.extension().string());
    return ext == ".png" || ext == ".bmp";
}

std::optional<ImageData> loadImage(const std::filesystem::path& path, std::string* error)
{
    const auto fail = [&](std::string reason) -> std::optional<ImageData> {
        KN_ERROR("Image '{}': {}", path.string(), reason);
        if (error) {
            *error = std::move(reason);
        }
        return std::nullopt;
    };
    if (!isSupportedImageFile(path)) {
        return fail("unsupported image type (expected .png or .bmp)");
    }
    SDL_Surface* loaded = SDL_LoadSurface(path.string().c_str());
    if (!loaded) {
        return fail(SDL_GetError());
    }
    SDL_Surface* rgba = SDL_ConvertSurface(loaded, SDL_PIXELFORMAT_RGBA32);
    SDL_DestroySurface(loaded);
    if (!rgba) {
        return fail(SDL_GetError());
    }
    ImageData image;
    image.width = static_cast<u32>(rgba->w);
    image.height = static_cast<u32>(rgba->h);
    image.rgba.resize(usize{image.width} * image.height * 4);
    const usize row = usize{image.width} * 4;
    const auto* src = static_cast<const u8*>(rgba->pixels);
    for (u32 y = 0; y < image.height; ++y) {
        std::memcpy(image.rgba.data() + y * row, src + static_cast<usize>(y) * static_cast<usize>(rgba->pitch), row);
    }
    SDL_DestroySurface(rgba);
    return image;
}

std::filesystem::path findTextureFile(const std::filesystem::path& reference, const std::filesystem::path& modelFile)
{
    if (reference.empty()) {
        return {};
    }
    // Paths from other machines may use either separator.
    std::string normalized = reference.string();
    std::ranges::replace(normalized, '\\', '/');
    const fs::path ref(normalized);
    const fs::path folder = modelFile.parent_path();
    const fs::path name = ref.filename();

    for (const fs::path& candidate : {ref, folder / ref, folder / name, folder / "textures" / name,
                                      folder / "Textures" / name}) {
        if (isFile(candidate)) {
            return candidate.lexically_normal();
        }
    }
    return searchBelow(folder, name.string(), 3);
}

void resolveMaterial(MaterialDesc& material, const std::filesystem::path& modelFile)
{
    for (fs::path* texture : {&material.baseColorTexture, &material.normalTexture, &material.roughnessTexture,
                              &material.metallicTexture}) {
        if (!texture->empty()) {
            const fs::path found = findTextureFile(*texture, modelFile);
            if (found.empty()) {
                KN_WARN("Model '{}': texture '{}' not found", modelFile.filename().string(), texture->string());
            }
            *texture = found;
        }
    }
    if (material.baseColorTexture.empty()) {
        return;
    }
    // The rest of the texture set, from the base colour's name.
    const std::string stem = lower(material.baseColorTexture.stem().string());
    for (const std::string_view key : kBaseColorKeys) {
        if (stem.find(key) == std::string::npos) {
            continue;
        }
        if (material.normalTexture.empty()) {
            material.normalTexture = findCompanion(material.baseColorTexture, key, kNormalKeys);
        }
        if (material.roughnessTexture.empty()) {
            material.roughnessTexture = findCompanion(material.baseColorTexture, key, kRoughnessKeys);
        }
        if (material.metallicTexture.empty()) {
            material.metallicTexture = findCompanion(material.baseColorTexture, key, kMetallicKeys);
        }
        break;
    }
}

} // namespace kinetic::assets
