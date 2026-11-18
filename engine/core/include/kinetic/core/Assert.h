#pragma once

#include "kinetic/core/Log.h"

#include <cstdlib>

#if defined(_MSC_VER)
#define KN_DEBUG_BREAK() __debugbreak()
#elif defined(__clang__) || defined(__GNUC__)
#define KN_DEBUG_BREAK() __builtin_trap()
#else
#define KN_DEBUG_BREAK() std::abort()
#endif

#ifdef KINETIC_DEBUG
#define KN_ASSERT(cond, ...)                                                              \
    do {                                                                                  \
        if (!(cond)) {                                                                    \
            KN_ERROR("Assertion failed: {} ({}:{})", #cond, __FILE__, __LINE__);          \
            __VA_OPT__(KN_ERROR(__VA_ARGS__);)                                            \
            KN_DEBUG_BREAK();                                                             \
        }                                                                                 \
    } while (false)
#else
#define KN_ASSERT(cond, ...) ((void)0)
#endif
