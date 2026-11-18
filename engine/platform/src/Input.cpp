#include "kinetic/platform/Input.h"

#include <SDL3/SDL_keyboard.h>

namespace kinetic::input {

bool isKeyDown(SDL_Scancode key)
{
    int count = 0;
    const bool* state = SDL_GetKeyboardState(&count);
    return key >= 0 && key < count && state[key];
}

} // namespace kinetic::input
