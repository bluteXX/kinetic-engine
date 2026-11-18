#include <doctest/doctest.h>

#include <kinetic/animation/PoseOps.h>

#include <glm/gtc/epsilon.hpp>

#include <array>

using namespace kinetic;
using namespace kinetic::anim;

namespace {

bool near(const glm::vec3& a, const glm::vec3& b, float eps = 1e-4f)
{
    return glm::all(glm::epsilonEqual(a, b, eps));
}

// root -> child, each offset 1 unit along +Y.
Skeleton makeChain()
{
    Pose rest(2);
    rest.translations[1] = {0.0f, 1.0f, 0.0f};
    return Skeleton({"root", "child"}, {kNoParent, 0}, rest);
}

} // namespace

TEST_CASE("skeleton validates parent ordering")
{
    CHECK(makeChain().isValid());
    CHECK(makeChain().findJoint("child") == 1u);
    CHECK_FALSE(makeChain().findJoint("missing").has_value());
}

TEST_CASE("blendPoses interpolates translation and takes the short rotation arc")
{
    Pose a(1);
    Pose b(1);
    b.translations[0] = {2.0f, 0.0f, 0.0f};
    // Same rotation, opposite sign: blending must not pass through a degenerate quaternion.
    a.rotations[0] = glm::angleAxis(glm::radians(90.0f), glm::vec3(0, 1, 0));
    b.rotations[0] = -a.rotations[0];

    Pose out;
    blendPoses(a, b, 0.5f, out);
    CHECK(near(out.translations[0], {1.0f, 0.0f, 0.0f}));
    CHECK(std::abs(glm::dot(out.rotations[0], a.rotations[0])) == doctest::Approx(1.0f));
}

TEST_CASE("localToModel concatenates parent transforms")
{
    const Skeleton skeleton = makeChain();
    Pose pose = skeleton.restPose();
    // Rotate the root 90 degrees around Z: the child (at +Y) must end up at -X.
    pose.rotations[0] = glm::angleAxis(glm::radians(90.0f), glm::vec3(0, 0, 1));

    std::array<glm::mat4, 2> model{};
    localToModel(skeleton, pose, model);
    CHECK(near(glm::vec3(model[1][3]), {-1.0f, 0.0f, 0.0f}));
}

TEST_CASE("skinning matrices are identity in the bind pose")
{
    const Skeleton chain = makeChain();
    std::array<glm::mat4, 2> model{};
    localToModel(chain, chain.restPose(), model);

    std::vector<glm::mat4> inverseBind = {glm::inverse(model[0]), glm::inverse(model[1])};
    const Skeleton skeleton({"root", "child"}, chain.parents(), chain.restPose(), inverseBind);

    std::array<glm::mat4, 2> skin{};
    computeSkinningMatrices(skeleton, model, skin);
    for (const auto& m : skin) {
        CHECK(near(glm::vec3(m[3]), glm::vec3(0.0f)));
        CHECK(near(glm::vec3(m[0]), {1.0f, 0.0f, 0.0f}));
    }
}
