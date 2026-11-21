// End-to-end checks on a real Tripo3D / Mixamo character, if it is present in assets/models.
// The asset is large and not part of the repository, so these tests skip when it is missing.

#include <kinetic/animation/BlendTree.h>
#include <kinetic/animation/IK.h>
#include <kinetic/animation/PoseOps.h>
#include <kinetic/assets/ModelLoader.h>
#include <kinetic/assets/TextureLoader.h>

#include <doctest/doctest.h>

#include <filesystem>

using namespace kinetic;

namespace {

const std::filesystem::path kWarrior = std::filesystem::path(KINETIC_TEST_ASSETS_DIR) / "models" / "Warrior" / "Warrior.fbx";

i32 clipIndex(const anim::SkinnedModel& model, const char* name)
{
    for (usize i = 0; i < model.clips.size(); ++i) {
        if (model.clips[i].name() == name) {
            return static_cast<i32>(i);
        }
    }
    return -1;
}

} // namespace

TEST_CASE("Hero asset: Tripo warrior loads with uvs, tangents, its texture set and two legs")
{
    if (!std::filesystem::exists(kWarrior)) {
        MESSAGE("assets/models/Warrior/Warrior.fbx not present - skipped");
        return;
    }
    std::string error;
    const auto model = assets::loadSkinnedModel(kWarrior, &error);
    REQUIRE_MESSAGE(model.has_value(), error);
    const MeshData& mesh = model->skin.mesh;

    CHECK(mesh.hasUvs());
    CHECK(mesh.hasTangents());
    // Welding: far fewer vertices than triangle corners.
    CHECK(mesh.positions.size() < mesh.indices.size() / 2);
    MESSAGE("vertices " << mesh.positions.size() << " for " << mesh.triangleCount() << " triangles");

    // The file points at a texture path from the author's machine; the loader finds it next to
    // the model and completes the set from the file names.
    CHECK(std::filesystem::exists(mesh.material.baseColorTexture));
    CHECK(std::filesystem::exists(mesh.material.normalTexture));
    CHECK(mesh.material.normalTexture.filename().string().find("normal") != std::string::npos);
    MESSAGE("base colour " << mesh.material.baseColorTexture.string());
    MESSAGE("normal      " << mesh.material.normalTexture.string());

    const auto legs = anim::findLegChains(model->skeleton);
    REQUIRE(legs.size() == 2);
    CHECK(model->skeleton.names()[legs[0].foot].find("Foot") != std::string::npos);
    CHECK(anim::findPelvis(model->skeleton, legs) >= 0);

    const i32 idle = clipIndex(*model, "idle");
    const i32 walk = clipIndex(*model, "walk");
    const i32 run = clipIndex(*model, "run");
    REQUIRE(idle >= 0);
    REQUIRE(walk >= 0);
    REQUIRE(run >= 0);
    const f32 idleSpeed = anim::estimateGroundSpeed(model->skeleton, model->clips[static_cast<usize>(idle)], legs);
    const f32 walkSpeed = anim::estimateGroundSpeed(model->skeleton, model->clips[static_cast<usize>(walk)], legs);
    const f32 runSpeed = anim::estimateGroundSpeed(model->skeleton, model->clips[static_cast<usize>(run)], legs);
    MESSAGE("ground speed (model units/s): idle " << idleSpeed << ", walk " << walkSpeed << ", run " << runSpeed);
    CHECK(idleSpeed < 0.2f * walkSpeed);
    CHECK(walkSpeed > 0.2f);
    CHECK(runSpeed > walkSpeed * 1.3f);
}

TEST_CASE("Hero asset: base colour texture decodes")
{
    if (!std::filesystem::exists(kWarrior)) {
        return;
    }
    const auto model = assets::loadSkinnedModel(kWarrior);
    REQUIRE(model.has_value());
    std::string error;
    const auto image = assets::loadImage(model->skin.mesh.material.baseColorTexture, &error);
    REQUIRE_MESSAGE(image.has_value(), error);
    CHECK(image->width >= 256);
    CHECK(image->rgba.size() == usize{image->width} * image->height * 4);
    MESSAGE("base colour " << image->width << " x " << image->height);
}

TEST_CASE("Hero asset: foot IK keeps the warrior's feet on a raised step")
{
    if (!std::filesystem::exists(kWarrior)) {
        return;
    }
    const auto model = assets::loadSkinnedModel(kWarrior);
    REQUIRE(model.has_value());
    const anim::Skeleton& skeleton = model->skeleton;
    const auto legs = anim::findLegChains(skeleton);
    REQUIRE(legs.size() == 2);

    // Idle pose standing at y = 0; the ground under the left foot (legs[1], +X) is 15 cm higher.
    const i32 idle = clipIndex(*model, "idle");
    anim::Pose pose = skeleton.restPose();
    model->clips[static_cast<usize>(idle)].sample(0.0f, true, pose);
    std::vector<glm::mat4> before(skeleton.jointCount());
    anim::localToModel(skeleton, pose, before);
    const f32 splitX = (before[legs[0].foot][3].x + before[legs[1].foot][3].x) * 0.5f;
    const anim::GroundQuery ground = [splitX](const glm::vec3& from, f32 maxDistance) -> std::optional<anim::GroundHit> {
        const f32 h = from.x > splitX ? 0.15f : 0.0f;
        if (from.y - maxDistance > h) {
            return std::nullopt;
        }
        return anim::GroundHit{{from.x, h, from.z}, {0.0f, 1.0f, 0.0f}};
    };
    anim::FootIKSettings settings;
    settings.enabled = true;
    anim::FootIKState state;
    anim::applyFootIK(skeleton, legs, glm::mat4(1.0f), ground, settings, state, 0.0f, pose);

    std::vector<glm::mat4> after(skeleton.jointCount());
    anim::localToModel(skeleton, pose, after);
    const f32 lowRaise = after[legs[0].foot][3].y - before[legs[0].foot][3].y;
    const f32 highRaise = after[legs[1].foot][3].y - before[legs[1].foot][3].y;
    MESSAGE("foot on flat ground moved " << lowRaise << " m, foot on the step moved " << highRaise << " m");
    CHECK(std::abs(lowRaise) < 0.01f);
    CHECK(highRaise == doctest::Approx(0.15f).epsilon(0.07));
}
