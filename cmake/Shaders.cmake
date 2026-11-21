# Shaders are authored in GLSL (shaders/src) and shipped pre-compiled (shaders/compiled):
#   *.spv  -> Vulkan (Windows, Linux)
#   *.msl  -> Metal  (macOS)
# Regenerate with: python3 tools/compile_shaders.py  (needs glslang (Vulkan SDK or .tools/glslang) + spirv-cross for .msl)

set(KINETIC_SHADER_DIR ${CMAKE_SOURCE_DIR}/shaders/compiled)

# All executables share CMAKE_RUNTIME_OUTPUT_DIRECTORY, so shaders/ and assets/ are copied there
# once, by a single target. Per-executable POST_BUILD copies raced each other in parallel builds
# (two `copy_directory` into the same folder -> "No such file or directory").
get_property(_kinetic_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(_kinetic_multi_config)
    set(KINETIC_RUNTIME_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<CONFIG>)
else()
    set(KINETIC_RUNTIME_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()

add_custom_target(kinetic_runtime_files
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different ${KINETIC_SHADER_DIR} ${KINETIC_RUNTIME_DIR}/shaders
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different ${CMAKE_SOURCE_DIR}/assets ${KINETIC_RUNTIME_DIR}/assets
    COMMENT "Copying shaders and assets to ${KINETIC_RUNTIME_DIR}")

# kinetic_copy_shaders(<target>) / kinetic_copy_assets(<target>) — make sure shaders / assets
# are next to the executable when <target> is built.
function(kinetic_copy_shaders target)
    add_dependencies(${target} kinetic_runtime_files)
endfunction()

function(kinetic_copy_assets target)
    add_dependencies(${target} kinetic_runtime_files)
endfunction()
