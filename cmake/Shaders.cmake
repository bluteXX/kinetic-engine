# Shaders are authored in GLSL (shaders/src) and shipped pre-compiled (shaders/compiled):
#   *.spv  -> Vulkan (Windows, Linux)
#   *.msl  -> Metal  (macOS)
# Regenerate with: python3 tools/compile_shaders.py  (needs glslangValidator + spirv-cross, both in the Vulkan SDK)

set(KINETIC_SHADER_DIR ${CMAKE_SOURCE_DIR}/shaders/compiled)

# kinetic_copy_shaders(<target>) — copies compiled shaders next to the executable after each build.
function(kinetic_copy_shaders target)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${KINETIC_SHADER_DIR} $<TARGET_FILE_DIR:${target}>/shaders
        COMMENT "Copying shaders for ${target}")
endfunction()

# kinetic_copy_assets(<target>) — copies the assets/ folder next to the executable.
function(kinetic_copy_assets target)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:${target}>/assets
        COMMENT "Copying assets for ${target}")
endfunction()
