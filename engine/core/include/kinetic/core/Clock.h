#pragma once

#include "kinetic/core/Types.h"

#include <chrono>

namespace kinetic {

// Monotonic clock measuring seconds since construction / last reset.
class Clock {
public:
    Clock() : m_start(std::chrono::steady_clock::now()) {}

    void reset() { m_start = std::chrono::steady_clock::now(); }

    [[nodiscard]] f64 elapsedSeconds() const
    {
        return std::chrono::duration<f64>(std::chrono::steady_clock::now() - m_start).count();
    }

private:
    std::chrono::steady_clock::time_point m_start;
};

} // namespace kinetic
