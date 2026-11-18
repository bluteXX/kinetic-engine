#include "kinetic/platform/Window.h"

#include "kinetic/core/Log.h"

#include <SDL3/SDL.h>

namespace kinetic {

Window::~Window()
{
    destroy();
}

bool Window::create(const WindowDesc& desc)
{
    SDL_WindowFlags flags = SDL_WINDOW_HIGH_PIXEL_DENSITY;
    if (desc.resizable) {
        flags |= SDL_WINDOW_RESIZABLE;
    }
    m_window = SDL_CreateWindow(desc.title.c_str(), static_cast<int>(desc.width), static_cast<int>(desc.height), flags);
    if (!m_window) {
        KN_ERROR("SDL_CreateWindow failed: {}", SDL_GetError());
        return false;
    }
    return true;
}

void Window::destroy()
{
    if (m_window) {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
    }
}

bool Window::isMinimized() const
{
    return (SDL_GetWindowFlags(m_window) & SDL_WINDOW_MINIMIZED) != 0;
}

void Window::sizeInPixels(u32& width, u32& height) const
{
    int w = 0;
    int h = 0;
    SDL_GetWindowSizeInPixels(m_window, &w, &h);
    width = static_cast<u32>(w);
    height = static_cast<u32>(h);
}

} // namespace kinetic
