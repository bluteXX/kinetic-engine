#include <doctest/doctest.h>

#include <kinetic/animation/Skinning.h>

#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/matrix_transform.hpp>

using namespace kinetic;

namespace {

anim::SkinnedMesh twoVertexMesh()
{
    anim::SkinnedMesh skin;
    skin.mesh.positions = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};
    skin.mesh.normals = {{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}};
    skin.mesh.indices = {0, 1, 0};
    skin.joints = {glm::uvec4(0, 0, 0, 0), glm::uvec4(0, 1, 0, 0)};
    skin.weights = {glm::vec4(1, 0, 0, 0), glm::vec4(0.5f, 0.5f, 0, 0)};
    return skin;
}

bool near(const glm::vec3& a, const glm::vec3& b) { return glm::all(glm::epsilonEqual(a, b, 1e-5f)); }

} // namespace

TEST_CASE("skinVertices with identity matrices keeps the bind pose")
{
    const anim::SkinnedMesh skin = twoVertexMesh();
    REQUIRE(skin.isValid(2));
    const std::vector<glm::mat4> matrices(2, glm::mat4(1.0f));
    std::vector<glm::vec3> positions(2);
    std::vector<glm::vec3> normals(2);
    anim::skinVertices(skin, matrices, positions, normals);
    CHECK(near(positions[0], skin.mesh.positions[0]));
    CHECK(near(positions[1], skin.mesh.positions[1]));
    CHECK(near(normals[1], skin.mesh.normals[1]));
}

TEST_CASE("skinVertices blends joint transforms by weight")
{
    const anim::SkinnedMesh skin = twoVertexMesh();
    const std::vector<glm::mat4> matrices = {
        glm::mat4(1.0f),
        glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 2.0f)),
    };
    std::vector<glm::vec3> positions(2);
    std::vector<glm::vec3> normals(2);
    anim::skinVertices(skin, matrices, positions, normals);
    CHECK(near(positions[0], {1.0f, 0.0f, 0.0f}));       // only joint 0
    CHECK(near(positions[1], {0.0f, 1.0f, 1.0f}));       // halfway between both joints

    // Rotating the only joint rotates the normal too.
    const std::vector<glm::mat4> rotated = {
        glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0, 1, 0)),
        glm::mat4(1.0f),
    };
    anim::skinVertices(skin, rotated, positions, normals);
    CHECK(near(positions[0], {0.0f, 0.0f, -1.0f}));
    CHECK(near(normals[0], {1.0f, 0.0f, 0.0f}));
}

TEST_CASE("normalizeWeights and isValid")
{
    CHECK(glm::all(glm::epsilonEqual(anim::normalizeWeights({2, 2, 0, 0}), glm::vec4(0.5f, 0.5f, 0, 0), 1e-6f)));
    CHECK(anim::normalizeWeights(glm::vec4(0.0f)) == glm::vec4(0.0f));

    anim::SkinnedMesh skin = twoVertexMesh();
    CHECK_FALSE(skin.isValid(1)); // vertex 1 uses joint 1
    skin.weights.pop_back();
    CHECK_FALSE(skin.isValid(2)); // array sizes differ
}
