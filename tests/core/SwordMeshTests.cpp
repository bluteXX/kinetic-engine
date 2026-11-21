// The procedural longsword: a closed, outward-facing mesh that fits the weapon's collision box.

#include <kinetic/core/Sword.h>
#include <kinetic/scene/Components.h>

#include <doctest/doctest.h>

using namespace kinetic;

TEST_CASE("Sword mesh: grip at the origin, blade along +Y inside the weapon's collision box")
{
    const MeshData m = makeSwordMesh();
    REQUIRE_FALSE(m.empty());
    REQUIRE(m.hasUvs());
    REQUIRE(m.hasTangents());
    CHECK(m.normals.size() == m.positions.size());
    for (const u32 i : m.indices) REQUIRE(i < m.positions.size());
    CHECK(m.triangleCount() > 2000);
    CHECK(m.triangleCount() < 40000);
    const glm::vec3 lo = m.boundsMin();
    const glm::vec3 hi = m.boundsMax();
    MESSAGE("bounds " << lo.x << " " << lo.y << " " << lo.z << " .. " << hi.x << " " << hi.y << " " << hi.z);
    CHECK(lo.y == doctest::Approx(-0.172f).epsilon(0.02));
    CHECK(hi.y == doctest::Approx(1.05f).epsilon(0.01));
    CHECK(hi.x == doctest::Approx(0.112f).epsilon(0.05)); // the guard
    CHECK(std::abs(lo.x + hi.x) < 1e-3f);                 // symmetric
    CHECK(hi.z < 0.035f);                                 // flat: the pommel is the thickest part

    // The blade (above the guard) stays within the default collision box (a little margin at the tip).
    const scene::WeaponComponent weapon;
    for (const glm::vec3& p : m.positions) {
        if (p.y < 0.14f) continue;
        CHECK(std::abs(p.x) <= weapon.halfExtents.x + 2e-3f);
        CHECK(std::abs(p.z) <= weapon.halfExtents.z + 1e-4f);
        CHECK(p.y <= weapon.center.y + weapon.halfExtents.y + 0.012f);
    }
    // Normals point outwards (away from the sword's axis or the pommel's centre) and windings agree.
    usize outward = 0;
    usize faces = 0;
    for (usize t = 0; t + 2 < m.indices.size(); t += 3) {
        const glm::vec3 a = m.positions[m.indices[t]];
        const glm::vec3 b = m.positions[m.indices[t + 1]];
        const glm::vec3 c = m.positions[m.indices[t + 2]];
        const glm::vec3 face = glm::cross(b - a, c - a);
        if (glm::length(face) < 1e-12f) continue;
        ++faces;
        const glm::vec3 n = m.normals[m.indices[t]] + m.normals[m.indices[t + 1]] + m.normals[m.indices[t + 2]];
        CHECK(glm::dot(face, n) > 0.0f);
        const glm::vec3 centre = (a + b + c) / 3.0f;
        const glm::vec3 axis = centre.y < -0.11f ? glm::vec3(0.0f, -0.142f, 0.0f) : glm::vec3(0.0f, centre.y, 0.0f);
        outward += glm::dot(face, centre - axis) >= 0.0f ? 1 : 0;
    }
    CHECK(static_cast<f32>(outward) > 0.97f * static_cast<f32>(faces));
    for (const glm::vec2& uv : m.uvs) {
        CHECK(uv.x >= 0.0f);
        CHECK(uv.x <= 1.0f);
        CHECK(uv.y >= 0.0f);
        CHECK(uv.y <= 1.0f);
    }
}

TEST_CASE("Sword texture: steel, darker fuller, blackened guard steel and brown leather")
{
    const ImageData img = makeSwordTexture();
    REQUIRE_FALSE(img.empty());
    const auto px = [&](f32 u, f32 v) {
        const u32 x = static_cast<u32>(u * static_cast<f32>(img.width));
        const u32 y = static_cast<u32>(v * static_cast<f32>(img.height));
        const u8* p = &img.rgba[(static_cast<usize>(y) * img.width + x) * 4];
        return glm::vec3(p[0], p[1], p[2]) / 255.0f;
    };
    const glm::vec3 edge = px(0.03f, 0.5f);
    const glm::vec3 flat = px(0.37f, 0.5f);
    const glm::vec3 fuller = px(0.56f, 0.5f);
    const glm::vec3 dark = px(0.69f, 0.5f);
    const glm::vec3 leather = px(0.8f, 0.33f);
    CHECK(edge.x > flat.x);
    CHECK(flat.x > fuller.x);
    CHECK(fuller.x > dark.x);
    CHECK(flat.z >= flat.x); // cool steel
    CHECK(leather.x > leather.z + 0.05f); // warm leather (or its dark cord, still reddish)
}
