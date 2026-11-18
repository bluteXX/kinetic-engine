#pragma once

#include <spdlog/spdlog.h>

namespace kinetic::log {

void init();
spdlog::logger& engine();

} // namespace kinetic::log

#define KN_TRACE(...) ::kinetic::log::engine().trace(__VA_ARGS__)
#define KN_INFO(...)  ::kinetic::log::engine().info(__VA_ARGS__)
#define KN_WARN(...)  ::kinetic::log::engine().warn(__VA_ARGS__)
#define KN_ERROR(...) ::kinetic::log::engine().error(__VA_ARGS__)
