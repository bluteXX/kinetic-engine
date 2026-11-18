add_executable(kinetic_editor src/main.cpp src/AssetDrawer.cpp src/AssetDrawer.h)
# The editor works on the project's own assets/ folder, so meshes added in the Assets drawer land
# in the repository (not only in the build output). Falls back to <exe>/assets if it is missing.
target_compile_definitions(kinetic_editor PRIVATE KINETIC_PROJECT_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(kinetic_editor PRIVATE kinetic::engine)
kinetic_set_warnings(kinetic_editor)
kinetic_copy_shaders(kinetic_editor)
kinetic_copy_assets(kinetic_editor)
set_target_properties(kinetic_editor PROPERTIES OUTPUT_NAME "KineticEditor" FOLDER "apps")
