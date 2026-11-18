#pragma once

#include "kinetic/core/Types.h"

#include <string>

struct SDL_Window;

namespace kinetic {

struct WindowDesc {
    std::string title = "Kinetic";
    u32 width = 1600;
    u32 height = 900;
    bool resizable = true;
};

class Window {
public:
    Window() = default;
    ~Window();
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    bool create(const WindowDesc& desc);
    void destroy();

    [[nodiscard]] SDL_Window* handle() const { return m_window; }
    [[nodiscard]] bool isMinimized() const;
    void sizeInPixels(u32& width, u32& height) const;

private:
    SDL_Window* m_window = nullptr;
};

} // namespace kinetic
