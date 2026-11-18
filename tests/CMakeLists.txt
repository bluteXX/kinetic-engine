add_executable(kinetic_tests
    main.cpp
    animation/PoseOpsTests.cpp
    animation/AnimationClipTests.cpp
    assets/GltfLoaderTests.cpp)
target_link_libraries(kinetic_tests PRIVATE kinetic::animation kinetic::assets doctest::doctest)
target_compile_definitions(kinetic_tests PRIVATE KINETIC_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
kinetic_set_warnings(kinetic_tests)
add_test(NAME kinetic_tests COMMAND kinetic_tests)
