#pragma once

#include "kinetic/core/Image.h"
#include "kinetic/core/MeshData.h"

#include <filesystem>
#include <optional>
#include <string>

namespace kinetic::assets {

// .png / .bmp (decoded by SDL).
[[nodiscard]] bool isSupportedImageFile(const std::filesystem::path& path);

// Decodes an image file to 8-bit RGBA.
[[nodiscard]] std::optional<ImageData> loadImage(const std::filesystem::path& path, std::string* error = nullptr);

// Finds the file a model refers to as `reference` (often a path from the artist's machine, e.g.
// "../../../../warrior_basecolor.png" or "C:/Users/x/tex/a.png"). Tries, in order: the path as
// is, relative to the model's folder, then the file name in the model's folder, its "textures"
// sub-folder and anywhere below the model's folder (case-insensitive). Empty if not found.
[[nodiscard]] std::filesystem::path findTextureFile(const std::filesystem::path& reference,
                                                    const std::filesystem::path& modelFile);

// Resolves every texture path of `material` with findTextureFile, then fills maps the file does
// not reference from the texture set next to the base colour: "hero_basecolor.png" also finds
// "hero_normal.png", "hero_roughness.png", "hero_metallic.png" (common suffixes: basecolor,
// base_color, albedo, diffuse, color / normal, nrm / roughness, rough / metallic, metalness).
void resolveMaterial(MaterialDesc& material, const std::filesystem::path& modelFile);

} // namespace kinetic::assets
