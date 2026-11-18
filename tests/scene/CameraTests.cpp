#include <doctest/doctest.h>

#include <kinetic/scene/Scene.h>

#include <glm/gtc/epsilon.hpp>

using namespace kinetic;
using namespace kinetic::scene;

namespace {

bool near(const glm::vec3& a, const glm::vec3& b, float eps = 1e-4f)
{
    return glm::all(glm::epsilonEqual(a, b, eps));
}

glm::vec3 project(const glm::mat4& m, const glm::vec3& p)
{
    const glm::vec4 clip = m * glm::vec4(p, 1.0f);
    return glm::vec3(clip) / clip.w;
}

} // namespace

TEST_CASE("lookAtRotation points local -Z at the target")
{
    const glm::vec3 position{1.0f, 2.0f, 3.0f};
    const glm::vec3 target{-2.0f, 0.5f, -1.0f};
    const glm::quat q = lookAtRotation(position, target);
    CHECK(near(q * glm::vec3(0, 0, -1), glm::normalize(target - position)));
    CHECK((q * glm::vec3(0, 1, 0)).y > 0.0f); // stays upright
}

TEST_CASE("camera view moves the world into camera space")
{
    TransformComponent t;
    t.position = {0.0f, 1.0f, 5.0f};
    t.rotation = lookAtRotation(t.position, {0.0f, 1.0f, 0.0f});
    const glm::mat4 view = CameraComponent::view(t);

    CHECK(near(glm::vec3(view * glm::vec4(t.position, 1.0f)), glm::vec3(0.0f)));
    CHECK(near(glm::vec3(view * glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)), glm::vec3(0.0f, 0.0f, -5.0f)));
}

TEST_CASE("camera projection maps near/far to depth 0..1")
{
    CameraComponent cam;
    cam.nearPlane = 0.1f;
    cam.farPlane = 100.0f;
    const glm::mat4 proj = cam.projection(16.0f / 9.0f);

    CHECK(project(proj, {0, 0, -0.1f}).z == doctest::Approx(0.0f).epsilon(1e-4));
    CHECK(project(proj, {0, 0, -100.0f}).z == doctest::Approx(1.0f).epsilon(1e-4));
    // A point up and to the right of the view axis lands in the upper-right of NDC.
    const glm::vec3 ndc = project(proj, {0.5f, 0.5f, -5.0f});
    CHECK(ndc.x > 0.0f);
    CHECK(ndc.y > 0.0f);
}

TEST_CASE("scene factories and active camera")
{
    Scene scene;
    CHECK(bool(scene.activeCamera() == entt::null));

    const entt::entity camera = scene.createCamera("Camera", {0, 0, 3}, {0, 0, 0});
    const entt::entity mesh = scene.createMesh("Triangle", BuiltinMesh::Triangle, {1, 0, 0, 1});

    CHECK(scene.activeCamera() == camera);
    CHECK(scene.registry().all_of<MeshRendererComponent>(mesh));
    CHECK(scene.registry().get<MeshRendererComponent>(mesh).tint == glm::vec4(1, 0, 0, 1));
    CHECK(std::string(builtinMeshName(BuiltinMesh::Triangle)) == "Triangle");
}

TEST_CASE("eulerDegreesNear stays continuous past 90 degrees on Y")
{
    // glm's XYZ decomposition limits Y to [-90, 90]; (0, 100, 0) comes back as (180, 80, 180).
    const glm::vec3 typed{0.0f, 100.0f, 0.0f};
    const glm::quat q = glm::quat(glm::radians(typed));

    // glm's raw decomposition flips to the other solution...
    const glm::vec3 raw = glm::degrees(glm::eulerAngles(q));
    CHECK_FALSE(near(raw, typed, 1e-2f));
    // ...the hinted one gives back what was typed.
    CHECK(near(eulerDegreesNear(q, {0.0f, 95.0f, 0.0f}), typed, 1e-2f));
}

TEST_CASE("eulerDegreesNear always represents the same rotation")
{
    const glm::vec3 samples[] = {{10, 20, 30}, {170, -45, 90}, {-120, 80, -150}, {0, 0, 179}, {359, 0, 0}};
    const glm::vec3 hints[] = {{0, 0, 0}, {180, 180, 180}, {-200, 30, 400}};
    for (const glm::vec3& s : samples) {
        const glm::quat q = glm::quat(glm::radians(s));
        for (const glm::vec3& h : hints) {
            const glm::quat back = glm::quat(glm::radians(eulerDegreesNear(q, h)));
            CHECK(std::abs(glm::dot(back, q)) == doctest::Approx(1.0f).epsilon(1e-4));
        }
    }
    // Wraps by 360 toward the hint.
    CHECK(near(eulerDegreesNear(glm::quat(glm::radians(glm::vec3(10, 0, 0))), {365, 0, 0}), {370, 0, 0}, 1e-2f));
}
