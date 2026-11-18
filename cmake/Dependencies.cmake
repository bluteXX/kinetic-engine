# All third-party code is pulled with CPM and pinned to exact tags.
# Set CPM_SOURCE_CACHE (the presets do) so every build dir shares one download.

# --- SDL3: window, input and the SDL_GPU API (Metal / Vulkan / D3D12) -------
CPMAddPackage(
    NAME SDL3
    SYSTEM YES
    GITHUB_REPOSITORY libsdl-org/SDL
    GIT_TAG release-3.4.20
    OPTIONS
        "SDL_SHARED OFF"
        "SDL_STATIC ON"
        "SDL_TEST_LIBRARY OFF"
        "SDL_EXAMPLES OFF"
        "SDL_TESTS OFF"
)

# --- Math ---------------------------------------------------------------------
CPMAddPackage(
    NAME glm
    SYSTEM YES
    GITHUB_REPOSITORY g-truc/glm
    GIT_TAG 1.0.3
    OPTIONS "GLM_BUILD_TESTS OFF" "BUILD_SHARED_LIBS OFF"
)

# --- Logging ------------------------------------------------------------------
CPMAddPackage(
    NAME spdlog
    SYSTEM YES
    GITHUB_REPOSITORY gabime/spdlog
    VERSION 1.17.0
    OPTIONS "SPDLOG_BUILD_EXAMPLE OFF" "SPDLOG_BUILD_TESTS OFF"
)

# --- ECS ----------------------------------------------------------------------
CPMAddPackage(
    NAME EnTT
    GITHUB_REPOSITORY skypjack/entt
    GIT_TAG v3.16.0
    DOWNLOAD_ONLY YES
)
add_library(EnTT INTERFACE)
target_include_directories(EnTT SYSTEM INTERFACE ${EnTT_SOURCE_DIR}/src)
add_library(EnTT::EnTT ALIAS EnTT)

# --- glTF loader (header-only, implementation lives in engine/assets) --------
CPMAddPackage(
    NAME cgltf
    GITHUB_REPOSITORY jkuhlmann/cgltf
    GIT_TAG v1.15
    DOWNLOAD_ONLY YES
)
add_library(cgltf INTERFACE)
target_include_directories(cgltf SYSTEM INTERFACE ${cgltf_SOURCE_DIR})

# --- FBX / OBJ loader (single .c file, compiled in engine/assets) ------------
CPMAddPackage(
    NAME ufbx
    GITHUB_REPOSITORY ufbx/ufbx
    GIT_TAG v0.23.1
    DOWNLOAD_ONLY YES
)
add_library(ufbx STATIC ${ufbx_SOURCE_DIR}/ufbx.c)
target_include_directories(ufbx SYSTEM PUBLIC ${ufbx_SOURCE_DIR})
set_target_properties(ufbx PROPERTIES FOLDER "third_party")

# --- Physics: Jolt ---------------------------------------------------------------
# Static MSVC runtime OFF: the rest of the engine uses the default /MD runtime.
CPMAddPackage(
    NAME JoltPhysics
    SYSTEM YES
    GITHUB_REPOSITORY jrouwe/JoltPhysics
    GIT_TAG v5.3.0
    SOURCE_SUBDIR Build
    OPTIONS
        "TARGET_UNIT_TESTS OFF"
        "TARGET_HELLO_WORLD OFF"
        "TARGET_PERFORMANCE_TEST OFF"
        "TARGET_SAMPLES OFF"
        "TARGET_VIEWER OFF"
        "ENABLE_ALL_WARNINGS OFF"
        "USE_STATIC_MSVC_RUNTIME_LIBRARY OFF"
        "INTERPROCEDURAL_OPTIMIZATION OFF"
        "DOUBLE_PRECISION OFF"
        "OVERRIDE_CXX_FLAGS OFF"
)
if(TARGET Jolt)
    set_target_properties(Jolt PROPERTIES FOLDER "third_party")
endif()

# --- Dear ImGui (docking branch) + SDL3 / SDL_GPU backends ---------------------
CPMAddPackage(
    NAME imgui
    GITHUB_REPOSITORY ocornut/imgui
    GIT_TAG v1.93.0-docking
    DOWNLOAD_ONLY YES
)
add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlgpu3.cpp
)
target_include_directories(imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC SDL3::SDL3)

# --- Tests ----------------------------------------------------------------------
if(KINETIC_BUILD_TESTS)
    CPMAddPackage(
        NAME doctest
        GITHUB_REPOSITORY doctest/doctest
        GIT_TAG v2.5.3
        DOWNLOAD_ONLY YES
    )
    add_library(doctest INTERFACE)
    target_include_directories(doctest SYSTEM INTERFACE ${doctest_SOURCE_DIR})
    add_library(doctest::doctest ALIAS doctest)
endif()
