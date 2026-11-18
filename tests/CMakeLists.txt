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
    core/RayTests.cpp)
target_link_libraries(kinetic_tests PRIVATE kinetic::animation kinetic::assets kinetic::scene doctest::doctest)
target_compile_definitions(kinetic_tests PRIVATE KINETIC_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
kinetic_set_warnings(kinetic_tests)
add_test(NAME kinetic_tests COMMAND kinetic_tests)
