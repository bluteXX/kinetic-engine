#include "kinetic/core/Log.h"

#include <spdlog/sinks/stdout_color_sinks.h>

namespace kinetic::log {

namespace {
std::shared_ptr<spdlog::logger> g_engine;
}

void init()
{
    if (g_engine) {
        return;
    }
    g_engine = spdlog::stdout_color_mt("kinetic");
    g_engine->set_pattern("%^[%T.%e] [%n] %v%$");
#ifdef KINETIC_DEBUG
    g_engine->set_level(spdlog::level::trace);
#else
    g_engine->set_level(spdlog::level::info);
#endif
}

spdlog::logger& engine()
{
    if (!g_engine) {
        init();
    }
    return *g_engine;
}

} // namespace kinetic::log
