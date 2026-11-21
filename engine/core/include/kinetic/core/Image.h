#pragma once

#include "kinetic/core/Types.h"

#include <vector>

namespace kinetic {

// Decoded 8-bit RGBA image, rows top to bottom.
struct ImageData {
    u32 width = 0;
    u32 height = 0;
    std::vector<u8> rgba; // width * height * 4

    [[nodiscard]] bool empty() const { return width == 0 || height == 0 || rgba.size() < usize{width} * height * 4; }
};

} // namespace kinetic
