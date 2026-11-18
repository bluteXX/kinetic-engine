# Every engine module is its own static library: kinetic::<name>.
# Dependencies (a module only includes the ones listed after the arrow):
#
#   core
#   animation -> core                  (pure CPU, no SDL / GPU)
#   assets    -> animation             (glTF import, FBX/OBJ meshes via ufbx)
#   scene     -> animation             (EnTT; animator component)
#   platform  -> core                  (SDL3 window / input)
#   gfx       -> platform              (SDL_GPU device, shaders, frames)
#   physics   -> core                  (Jolt, hidden behind PhysicsWorld)
#   renderer  -> gfx, scene
#   ui        -> gfx                   (Dear ImGui)
#   app       -> renderer, ui          (main loop)
#
# Keep `animation` free of SDL / GPU code: it has to stay testable headless
# and portable (e.g. into an Unreal plugin later).

function(kinetic_add_module name)
    cmake_parse_arguments(ARG "" "" "PUBLIC_DEPS;PRIVATE_DEPS" ${ARGN})
    file(GLOB_RECURSE sources CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/${name}/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/${name}/include/*.h)
    add_library(kinetic_${name} STATIC ${sources})
    add_library(kinetic::${name} ALIAS kinetic_${name})
    target_include_directories(kinetic_${name}
        PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/${name}/include
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/${name}/src)
    target_link_libraries(kinetic_${name} PUBLIC ${ARG_PUBLIC_DEPS} PRIVATE ${ARG_PRIVATE_DEPS})
    kinetic_set_warnings(kinetic_${name})
    set_target_properties(kinetic_${name} PROPERTIES FOLDER "engine")
endfunction()

kinetic_add_module(core      PUBLIC_DEPS glm::glm spdlog::spdlog)
kinetic_add_module(animation PUBLIC_DEPS kinetic::core)
kinetic_add_module(assets    PUBLIC_DEPS kinetic::animation PRIVATE_DEPS cgltf ufbx)
kinetic_add_module(scene     PUBLIC_DEPS kinetic::animation EnTT::EnTT)
kinetic_add_module(platform  PUBLIC_DEPS kinetic::core SDL3::SDL3)
kinetic_add_module(gfx       PUBLIC_DEPS kinetic::platform)
kinetic_add_module(physics   PUBLIC_DEPS kinetic::core PRIVATE_DEPS Jolt)
kinetic_add_module(renderer  PUBLIC_DEPS kinetic::gfx kinetic::scene)
kinetic_add_module(ui        PUBLIC_DEPS kinetic::gfx imgui)
kinetic_add_module(app       PUBLIC_DEPS kinetic::renderer kinetic::ui)

target_compile_definitions(kinetic_core PUBLIC
    GLM_FORCE_DEPTH_ZERO_TO_ONE   # SDL_GPU (Metal/D3D/Vulkan) uses 0..1 depth
    GLM_ENABLE_EXPERIMENTAL
    $<$<CONFIG:Debug>:KINETIC_DEBUG>)

# One umbrella target for applications.
add_library(kinetic_engine INTERFACE)
add_library(kinetic::engine ALIAS kinetic_engine)
target_link_libraries(kinetic_engine INTERFACE
    kinetic::app kinetic::assets kinetic::animation kinetic::scene kinetic::physics)

if(MSVC)
    # /Zc:preprocessor -> standard preprocessor (__VA_OPT__), /utf-8 -> required by fmt/spdlog
    target_compile_options(kinetic_core PUBLIC /Zc:preprocessor /utf-8)
endif()
