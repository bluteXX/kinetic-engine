cmake_minimum_required(VERSION 3.25)

project(KineticEngine
    VERSION 0.1.0
    DESCRIPTION "Cross-platform 3D engine focused on character animation"
    LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Put all executables and shared libs next to each other so shaders/assets resolve the same way everywhere.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(KINETIC_BUILD_SANDBOX "Build the sandbox app"            ON)
option(KINETIC_BUILD_EDITOR  "Build the editor app"             ON)
option(KINETIC_BUILD_TESTS   "Build unit tests"                 ON)
option(KINETIC_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(CPM)
include(CompilerWarnings)
include(Dependencies)
include(Shaders)

add_subdirectory(engine)

if(KINETIC_BUILD_SANDBOX)
    add_subdirectory(sandbox)
endif()

if(KINETIC_BUILD_EDITOR)
    add_subdirectory(editor)
endif()

if(KINETIC_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
