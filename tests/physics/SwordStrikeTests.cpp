#include <doctest/doctest.h>

#include "SwordStrikeTest.h"

#include <cmath>
#include <string>

using namespace kinetic;

namespace {

bool finite(const glm::vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

} // namespace

TEST_CASE("Sword strike: the blade hits the stone and the forces stay physical")
{
    editor::SwordStrikeTest test;
    REQUIRE(test.createHeadless());

    // Run the strike; the deepest contact penetration tells whether the blade stays on the stone.
    f32 deepest = 0.0f;
    for (int i = 0; i < 288; ++i) { // 1.2 s, checked after every 240 Hz step
        test.advance(1.0f / 240.0f);
        const physics::BodyState& sword = test.world().body(test.sword());
        REQUIRE(finite(sword.position));
        for (const physics::ContactPoint& c : test.world().contacts()) {
            deepest = std::max(deepest, c.depth);
        }
    }

    REQUIRE_FALSE(test.hits().empty());
    const auto* strike = test.strongestHit();
    REQUIRE(strike != nullptr);
    CHECK(strike->peakContact > 50.0f);
    CHECK(deepest < 0.01f); // CCD + contacts keep the blade on top of the stone

    const std::string names[] = {"shoulder", "elbow", "wrist", "grip"};
    for (usize j = 0; j < 4; ++j) {
        const auto& h = strike->joints[j];
        CHECK(std::isfinite(h.peakTorque));
        CHECK(std::isfinite(h.peakForce));
        MESSAGE(names[j], ": impulse ", glm::length(h.impulse), " N s, muscle impulse ", h.angularImpulse,
                " N m s, peak load ", h.peakLoad * 100.0f, " %, absorbed ", h.absorbedEnergy, " J");
    }
    MESSAGE(test.hits().size(), " hit(s); strike at ", strike->start, " s, ", strike->duration * 1000.0f, " ms, peak ",
            strike->peakContact, " N, impulse ", glm::length(strike->impulse), " N s");
    // A muscle can never exceed its strength.
    CHECK(strike->joints[2].peakLoad <= 1.001f);
}

TEST_CASE("Sword strike: the contact impulse does not depend on the step rate")
{
    const auto strikeImpulse = [](f32 hz) {
        editor::SwordStrikeTest test;
        test.setStepRate(hz);
        REQUIRE(test.createHeadless());
        test.advance(1.2f);
        const auto* strike = test.strongestHit();
        REQUIRE(strike != nullptr);
        return glm::length(strike->impulse);
    };
    const f32 coarse = strikeImpulse(240.0f);
    const f32 fine = strikeImpulse(960.0f);
    MESSAGE("impulse at 240 Hz: ", coarse, " N s, at 960 Hz: ", fine, " N s");
    CHECK(coarse == doctest::Approx(fine).epsilon(0.1));
}

TEST_CASE("Sword strike: the blade rebounds right after the impact and does not keep pressing")
{
    editor::SwordStrikeTest test;
    REQUIRE(test.createHeadless());
    constexpr f32 kStep = 1.0f / 960.0f;

    const auto tipVelocityY = [&] {
        const physics::BodyState& s = test.world().body(test.sword());
        const glm::vec3 tip = s.position + s.rotation * glm::vec3(0.0f, -1.0f, 0.0f);
        return (s.linearVelocity + glm::cross(s.angularVelocity, tip - s.centerOfMass)).y;
    };

    f32 time = 0.0f;
    f32 impact = -1.0f;
    f32 velocityBefore = 0.0f;
    f32 velocityAfter = 0.0f;
    f32 lastContact = 0.0f;
    while (time < 1.6f) {
        const f32 before = tipVelocityY();
        test.advance(kStep);
        time += kStep;
        if (test.world().body(test.sword()).inContact) {
            if (impact < 0.0f) {
                impact = time;
                velocityBefore = before;
            }
            lastContact = time;
        }
        if (impact >= 0.0f && velocityAfter == 0.0f && time >= impact + 0.02f) {
            velocityAfter = tipVelocityY();
        }
    }
    REQUIRE(impact > 0.0f);
    MESSAGE("impact at ", impact, " s: tip vy ", velocityBefore, " -> ", velocityAfter, " m/s 20 ms later; last contact ",
            (lastContact - impact) * 1000.0f, " ms after the impact");
    CHECK(velocityBefore < -2.0f); // the blade came down fast
    CHECK(velocityAfter > 0.0f);   // and bounced up straight away
    CHECK(lastContact - impact < 0.1f); // no pressing / grinding on the stone afterwards
}
