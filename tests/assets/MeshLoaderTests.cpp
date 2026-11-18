#include <doctest/doctest.h>

#include <kinetic/assets/MeshLoader.h>

#include <glm/gtc/epsilon.hpp>

using namespace kinetic;

TEST_CASE("Cube.obj loads as 12 triangles with unit normals")
{
    std::string error;
    const auto mesh = assets::loadMesh(KINETIC_TEST_ASSETS_DIR "/meshes/Cube.obj", &error);
    REQUIRE_MESSAGE(mesh.has_value(), error);

    CHECK(mesh->triangleCount() == 12);
    CHECK(mesh->positions.size() == mesh->normals.size());
    for (const u32 index : mesh->indices) {
        CHECK(index < mesh->positions.size());
    }
    for (const glm::vec3& n : mesh->normals) {
        CHECK(glm::length(n) == doctest::Approx(1.0f).epsilon(1e-4));
    }
    CHECK(glm::all(glm::epsilonEqual(mesh->boundsMin(), glm::vec3(-0.5f), 1e-4f)));
    CHECK(glm::all(glm::epsilonEqual(mesh->boundsMax(), glm::vec3(0.5f), 1e-4f)));

    // Every triangle winds counter-clockwise around its outward normal.
    for (usize t = 0; t < mesh->triangleCount(); ++t) {
        const glm::vec3& a = mesh->positions[mesh->indices[t * 3 + 0]];
        const glm::vec3& b = mesh->positions[mesh->indices[t * 3 + 1]];
        const glm::vec3& c = mesh->positions[mesh->indices[t * 3 + 2]];
        CHECK(glm::dot(glm::cross(b - a, c - a), mesh->normals[mesh->indices[t * 3]]) > 0.0f);
    }
}

TEST_CASE("loadMesh reports errors")
{
    std::string error;
    CHECK_FALSE(assets::loadMesh("does_not_exist.fbx", &error).has_value());
    CHECK_FALSE(error.empty());

    CHECK_FALSE(assets::loadMesh(KINETIC_TEST_ASSETS_DIR "/models/RiggedFigure.glb", &error).has_value());
    CHECK(error.find("unsupported") != std::string::npos);

    CHECK(assets::isSupportedMeshFile("a/b/Model.FBX"));
    CHECK(assets::isSupportedMeshFile("x.obj"));
    CHECK_FALSE(assets::isSupportedMeshFile("x.glb"));
}

TEST_CASE("Torus.fbx loads in meters, Y up")
{
    std::string error;
    const auto mesh = assets::loadMesh(KINETIC_TEST_ASSETS_DIR "/meshes/Torus.fbx", &error);
    REQUIRE_MESSAGE(mesh.has_value(), error);

    CHECK(mesh->triangleCount() == 32 * 16 * 2); // 32 x 16 quads
    // Outer radius 0.5 m in XZ, tube radius 0.15 m in Y.
    const glm::vec3 size = mesh->boundsMax() - mesh->boundsMin();
    CHECK(size.x == doctest::Approx(1.0f).epsilon(1e-3));
    CHECK(size.z == doctest::Approx(1.0f).epsilon(1e-2));
    CHECK(size.y == doctest::Approx(0.3f).epsilon(1e-2));
    for (const glm::vec3& n : mesh->normals) {
        CHECK(glm::length(n) == doctest::Approx(1.0f).epsilon(1e-4));
    }
}
