#include <doctest/doctest.h>

#include <kinetic/animation/PoseOps.h>
#include <kinetic/assets/GltfLoader.h>

#include <glm/gtc/epsilon.hpp>

using namespace kinetic;

TEST_CASE("RiggedFigure.glb loads a valid skeleton and clip")
{
    const auto asset = assets::loadSkeletalAnimation(KINETIC_TEST_ASSETS_DIR "/models/RiggedFigure.glb");
    REQUIRE(asset.has_value());

    const anim::Skeleton& skeleton = asset->skeleton;
    CHECK(skeleton.jointCount() > 1);
    CHECK(skeleton.isValid());
    REQUIRE_FALSE(asset->clips.empty());
    CHECK(asset->clips[0].duration() > 0.0f);
    CHECK(asset->clips[0].jointCount() == skeleton.jointCount());

    // In the rest pose model * inverseBind should be (close to) identity for every joint.
    std::vector<glm::mat4> model(skeleton.jointCount());
    std::vector<glm::mat4> skin(skeleton.jointCount());
    anim::localToModel(skeleton, skeleton.restPose(), model);
    anim::computeSkinningMatrices(skeleton, model, skin);
    for (const auto& m : skin) {
        CHECK(glm::all(glm::epsilonEqual(glm::vec3(m[3]), glm::vec3(0.0f), 1e-3f)));
    }
}
