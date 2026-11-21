add_executable(kinetic_tests
    main.cpp
    animation/PoseOpsTests.cpp
    animation/AnimationClipTests.cpp
    animation/AnimationPlayerTests.cpp
    animation/SkinningTests.cpp
    animation/BlendTreeTests.cpp
    animation/StateMachineTests.cpp
    animation/ClipToolsTests.cpp
    animation/BlendTreeIOTests.cpp
    assets/GltfLoaderTests.cpp
    assets/MeshLoaderTests.cpp
    assets/ModelLoaderTests.cpp
    assets/HeroAssetTests.cpp
    scene/CameraTests.cpp
    core/RayTests.cpp
    core/JsonTests.cpp
    core/SwordMeshTests.cpp
    physics/PhysicsWorldTests.cpp
    physics/SwordStrikeTests.cpp
    game/PlayModeTests.cpp
    game/CharacterControllerTests.cpp
    game/HeroPlayTests.cpp
    game/AssetDrawerTests.cpp
    game/ComponentGizmosTests.cpp
    game/HeroComboTests.cpp
    game/HeroArmTests.cpp
    game/VfxTests.cpp
    ${CMAKE_SOURCE_DIR}/editor/src/SwordStrikeTest.cpp
    ${CMAKE_SOURCE_DIR}/editor/src/CharacterRig.cpp
    ${CMAKE_SOURCE_DIR}/editor/src/PlayMode.cpp
    ${CMAKE_SOURCE_DIR}/editor/src/CharacterSetup.cpp
    ${CMAKE_SOURCE_DIR}/editor/src/AssetDrawer.cpp
    ${CMAKE_SOURCE_DIR}/editor/src/ComponentGizmos.cpp)
target_link_libraries(kinetic_tests PRIVATE kinetic::animation kinetic::assets kinetic::scene kinetic::physics
    kinetic::gameplay kinetic::physanim kinetic::renderer kinetic::vfx imgui doctest::doctest)
target_include_directories(kinetic_tests PRIVATE ${CMAKE_SOURCE_DIR}/editor/src) # editor prototypes under test
target_compile_definitions(kinetic_tests PRIVATE KINETIC_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
kinetic_set_warnings(kinetic_tests)
add_test(NAME kinetic_tests COMMAND kinetic_tests)
