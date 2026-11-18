#include <doctest/doctest.h>

#include <kinetic/animation/PoseOps.h>
#include <kinetic/assets/MeshLoader.h>
#include <kinetic/assets/ModelLoader.h>

#include <glm/gtc/epsilon.hpp>

#include <filesystem>
#include <fstream>

using namespace kinetic;

namespace {

// Skins the model in its rest pose and returns the resulting positions.
std::vector<glm::vec3> skinRestPose(const anim::SkinnedModel& model)
{
    const usize joints = model.skeleton.jointCount();
    std::vector<glm::mat4> world(joints);
    std::vector<glm::mat4> skin(joints);
    anim::localToModel(model.skeleton, model.skeleton.restPose(), world);
    anim::computeSkinningMatrices(model.skeleton, world, skin);

    const usize n = model.skin.mesh.positions.size();
    std::vector<glm::vec3> positions(n);
    std::vector<glm::vec3> normals(n);
    anim::skinVertices(model.skin, skin, positions, normals);
    return positions;
}

void checkSkin(const anim::SkinnedModel& model)
{
    CHECK(model.skeleton.isValid());
    CHECK(model.skin.isValid(model.skeleton.jointCount()));
    CHECK_FALSE(model.skin.mesh.empty());
    for (const glm::vec4& w : model.skin.weights) {
        CHECK(w.x + w.y + w.z + w.w == doctest::Approx(1.0f).epsilon(1e-4));
    }
    for (const anim::AnimationClip& clip : model.clips) {
        CHECK(clip.jointCount() == model.skeleton.jointCount());
        CHECK(clip.duration() > 0.0f);
    }
}

} // namespace

TEST_CASE("RiggedFigure.glb loads as a skinned model standing along +Y")
{
    std::string error;
    const auto model = assets::loadSkinnedModel(KINETIC_TEST_ASSETS_DIR "/models/RiggedFigure.glb", &error);
    REQUIRE_MESSAGE(model.has_value(), error);
    checkSkin(*model);
    CHECK(model->name == "RiggedFigure");
    CHECK_FALSE(model->clips.empty());

    // The mesh is authored Z-up (1.45 m tall along Z) under a "Z_UP" node that rotates it to
    // Y-up. That node is part of the skeleton, so the rest pose stands along +Y.
    const glm::vec3 meshSize = model->skin.mesh.boundsMax() - model->skin.mesh.boundsMin();
    CHECK(meshSize.z == doctest::Approx(1.45f).epsilon(0.01));
    const glm::vec3 restSize = model->restBoundsMax - model->restBoundsMin;
    CHECK(restSize.y == doctest::Approx(1.45f).epsilon(0.01));
    CHECK(restSize.z < 0.5f);
    CHECK(model->restBoundsMin.y == doctest::Approx(0.0f).epsilon(0.01));
}

TEST_CASE("TestTentacle_Skinned.fbx: skinned rest pose matches the static mesh")
{
    const char* path = KINETIC_TEST_ASSETS_DIR "/meshes/TestTentacle_Skinned.fbx";
    std::string error;
    const auto model = assets::loadSkinnedModel(path, &error);
    REQUIRE_MESSAGE(model.has_value(), error);
    checkSkin(*model);
    MESSAGE("tentacle: ", model->skeleton.jointCount(), " joints, ", model->clips.size(), " clips, ",
            model->skin.mesh.triangleCount(), " triangles");

    const auto staticMesh = assets::loadMesh(path, &error);
    REQUIRE_MESSAGE(staticMesh.has_value(), error);
    CHECK(staticMesh->triangleCount() == model->skin.mesh.triangleCount());

    // Bind pose (in meters, Y up) must line up with the static import.
    const std::vector<glm::vec3> skinned = skinRestPose(*model);
    MeshData posed;
    posed.positions = skinned;
    const f32 size = glm::length(staticMesh->boundsMax() - staticMesh->boundsMin());
    const f32 tolerance = std::max(size * 0.01f, 1e-4f);
    CHECK(glm::all(glm::epsilonEqual(posed.boundsMin(), staticMesh->boundsMin(), tolerance)));
    CHECK(glm::all(glm::epsilonEqual(posed.boundsMax(), staticMesh->boundsMax(), tolerance)));

    // Halfway through the clip the tentacle has moved, but stays attached and the same length.
    REQUIRE_FALSE(model->clips.empty());
    const anim::AnimationClip& clip = model->clips[0];
    anim::Pose pose = model->skeleton.restPose();
    clip.sample(clip.duration() * 0.5f, false, pose);
    std::vector<glm::mat4> world(model->skeleton.jointCount());
    std::vector<glm::mat4> skin(model->skeleton.jointCount());
    anim::localToModel(model->skeleton, pose, world);
    anim::computeSkinningMatrices(model->skeleton, world, skin);
    std::vector<glm::vec3> animated(skinned.size());
    std::vector<glm::vec3> normals(skinned.size());
    anim::skinVertices(model->skin, skin, animated, normals);

    f32 maxMove = 0.0f;
    f32 maxRadius = 0.0f;
    for (usize i = 0; i < animated.size(); ++i) {
        maxMove = std::max(maxMove, glm::length(animated[i] - skinned[i]));
        maxRadius = std::max(maxRadius, glm::length(animated[i]));
    }
    CHECK(maxMove > size * 0.02f);
    CHECK(maxRadius < size * 1.5f);
}

TEST_CASE("loadSkinnedModel rejects static and unsupported files")
{
    std::string error;
    CHECK_FALSE(assets::loadSkinnedModel(KINETIC_TEST_ASSETS_DIR "/meshes/Torus.fbx", &error).has_value());
    CHECK(error.find("no skin") != std::string::npos);
    CHECK_FALSE(assets::loadSkinnedModel(KINETIC_TEST_ASSETS_DIR "/meshes/Cube.obj", &error).has_value());
    CHECK(assets::isSupportedModelFile("a/Hero.GLB"));
    CHECK(assets::isSupportedModelFile("a/Hero.fbx"));
    CHECK_FALSE(assets::isSupportedModelFile("a/Hero.obj"));
}

TEST_CASE("glTF: transforms of non-joint ancestors (scaled Armature node) are part of the skeleton")
{
    // One joint "Hips" under an "Armature" node scaled by 0.01 (typical Blender export). The joint
    // sits at 1 m; the inverse bind matrix includes the Armature scale, as the glTF spec requires.
    constexpr const char* kGltf = R"json({"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0,2]}],"nodes":[{"name":"Armature","scale":[0.01,0.01,0.01],"children":[1]},{"name":"Hips","translation":[0,100,0]},{"name":"Body","mesh":0,"skin":0}],"meshes":[{"primitives":[{"attributes":{"POSITION":0,"JOINTS_0":1,"WEIGHTS_0":2},"indices":4}]}],"skins":[{"joints":[1],"inverseBindMatrices":3}],"buffers":[{"byteLength":168,"uri":"data:application/octet-stream;base64,AAAAAAAAgD8AAAAAAACAPwAAgD8AAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAgD8AAAAAAAAAAAAAAAAAAIA/AAAAAAAAAAAAAAAAAADIQgAAAAAAAAAAAAAAAAAAAAAAAMhCAAAAAAAAAAAAAAAAAAAAAAAAyEIAAAAAAAAAAAAAyMIAAAAAAACAPwAAAQACAAAA"}],"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":12},{"buffer":0,"byteOffset":48,"byteLength":48},{"buffer":0,"byteOffset":96,"byteLength":64},{"buffer":0,"byteOffset":160,"byteLength":6}],"accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,1,0],"max":[1,2,0]},{"bufferView":1,"componentType":5121,"count":3,"type":"VEC4"},{"bufferView":2,"componentType":5126,"count":3,"type":"VEC4"},{"bufferView":3,"componentType":5126,"count":1,"type":"MAT4"},{"bufferView":4,"componentType":5123,"count":3,"type":"SCALAR"}]})json";
    const auto path = std::filesystem::temp_directory_path() / "kinetic_scaled_armature.gltf";
    {
        std::ofstream file(path, std::ios::binary);
        file << kGltf;
    }
    std::string error;
    const auto model = assets::loadSkinnedModel(path, &error);
    std::filesystem::remove(path);
    REQUIRE_MESSAGE(model.has_value(), error);
    checkSkin(*model);
    REQUIRE(model->skeleton.jointCount() == 2);
    CHECK_FALSE(model->jointIsBone(0)); // Armature: helper joint
    CHECK(model->jointIsBone(1));       // Hips

    // Rest pose must reproduce the bind pose exactly, not a 100x / 0.01x version of it.
    const std::vector<glm::vec3> skinned = skinRestPose(*model);
    for (usize i = 0; i < skinned.size(); ++i) {
        CHECK(glm::all(glm::epsilonEqual(skinned[i], model->skin.mesh.positions[i], 1e-4f)));
    }
}
