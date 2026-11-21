// Hit effects (kinetic::vfx) and their asset files.

#include <kinetic/vfx/Vfx.h>

#include <doctest/doctest.h>

#include <filesystem>

using namespace kinetic;

TEST_CASE("VFX: the hit effects in assets/vfx load, one per material")
{
    vfx::VfxLibrary library;
    const usize loaded = library.loadDirectory(std::filesystem::path(KINETIC_TEST_ASSETS_DIR) / "vfx");
    CHECK(loaded >= 5);
    for (const char* name : {"hit_stone", "hit_metal", "hit_wood", "hit_dirt", "hit_flesh"}) {
        CAPTURE(name);
        const vfx::VfxDesc* desc = library.find(name);
        REQUIRE(desc != nullptr);
        CHECK_FALSE(desc->emitters.empty());
    }
    // Colours match the material: wood brown, flesh red, metal sparks bright.
    const auto& wood = library.find("hit_wood")->emitters.front().color;
    CHECK(wood.r > wood.b);
    const auto& flesh = library.find("hit_flesh")->emitters.front().color;
    CHECK(flesh.r > 4.0f * flesh.g);
    const auto& metal = library.find("hit_metal")->emitters.front().color;
    CHECK(metal.r + metal.g + metal.b > 2.0f);
}

TEST_CASE("VFX: a burst flies out of the surface, falls, bounces on the ground and ends")
{
    vfx::VfxDesc desc;
    vfx::EmitterDesc e;
    e.count = 20;
    e.speed = {2.0f, 3.0f};
    e.spread = 30.0f;
    e.lifetime = {0.5f, 0.6f};
    e.drag = 0.0f;
    e.scaleWithSpeed = false;
    desc.emitters.push_back(e);
    const auto roundTrip = vfx::parseVfx(vfx::vfxToJson(desc));
    REQUIRE(roundTrip.has_value());
    CHECK(roundTrip->emitters.front().count == 20);

    vfx::VfxSystem system;
    system.setGround(0.0f);
    const glm::vec3 point(0.0f, 1.0f, 0.0f);
    const glm::vec3 normal(1.0f, 0.0f, 0.0f); // a wall facing +X
    system.spawn(desc, point, normal, glm::vec3(0.0f, 0.0f, -5.0f));
    REQUIRE(system.bursts().size() == 1);
    CHECK(system.particleCount() == 20);
    for (const vfx::Particle& p : system.bursts().front().particles) {
        CHECK(glm::dot(p.velocity, normal) > 0.0f); // out of the wall
    }
    std::vector<glm::vec3> positions(vfx::VfxSystem::vertexCount(system.bursts().front()));
    std::vector<glm::vec3> normals(positions.size());
    vfx::VfxSystem::buildMesh(system.bursts().front(), positions, normals);
    CHECK(glm::length(positions.front() - point) < 0.1f);

    f32 lowest = 10.0f;
    for (int i = 0; i < 20; ++i) {
        system.update(0.02f);
        for (const vfx::Burst& b : system.bursts()) {
            for (const vfx::Particle& p : b.particles) lowest = std::min(lowest, p.position.y);
        }
    }
    CHECK(lowest < 0.9f);   // gravity
    CHECK(lowest >= 0.0f);  // not through the ground
    for (int i = 0; i < 40; ++i) system.update(0.02f);
    CHECK(system.bursts().empty());
}
