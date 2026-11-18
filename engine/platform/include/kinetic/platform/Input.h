#pragma once

#include <SDL3/SDL_scancode.h>

namespace kinetic::input {

// Polled keyboard state for the current frame (valid after events were pumped).
[[nodiscard]] bool isKeyDown(SDL_Scancode key);

} // namespace kinetic::input
