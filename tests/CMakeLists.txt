add_executable(kinetic_tests
    main.cpp
    animation/PoseOpsTests.cpp
    animation/AnimationClipTests.cpp
    animation/AnimationPlayerTests.cpp
    animation/SkinningTests.cpp
    assets/GltfLoaderTests.cpp
    assets/MeshLoaderTests.cpp
    assets/ModelLoaderTests.cpp
    scene/CameraTests.cpp
    core/RayTests.cpp
    physics/PhysicsWorldTests.cpp
    physics/SwordStrikeTests.cpp
    ${CMAKE_SOURCE_DIR}/editor/src/SwordStrikeTest.cpp)
target_link_libraries(kinetic_tests PRIVATE kinetic::animation kinetic::assets kinetic::scene kinetic::physics
    kinetic::renderer imgui doctest::doctest)
target_include_directories(kinetic_tests PRIVATE ${CMAKE_SOURCE_DIR}/editor/src) # editor prototypes under test
target_compile_definitions(kinetic_tests PRIVATE KINETIC_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
kinetic_set_warnings(kinetic_tests)
add_test(NAME kinetic_tests COMMAND kinetic_tests)
