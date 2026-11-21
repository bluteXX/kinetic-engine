#include "kinetic/animation/IK.h"

#include "kinetic/animation/PoseOps.h"

#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <string>
#include <string_view>

namespace kinetic::anim {

namespace {

constexpr glm::quat kIdentity{1.0f, 0.0f, 0.0f, 0.0f};

glm::vec3 position(const glm::mat4& m) { return glm::vec3(m[3]); }

glm::vec3 anyPerpendicular(const glm::vec3& v)
{
    const glm::vec3 axis = std::abs(v.x) < 0.9f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    return glm::normalize(glm::cross(v, axis));
}

// Shortest rotation taking direction u to direction v (both non-zero).
glm::quat rotationBetween(const glm::vec3& u, const glm::vec3& v)
{
    const f32 lu = glm::length(u);
    const f32 lv = glm::length(v);
    if (lu < 1e-8f || lv < 1e-8f) {
        return kIdentity;
    }
    const glm::vec3 a = u / lu;
    const glm::vec3 b = v / lv;
    const f32 d = glm::dot(a, b);
    if (d < -0.99999f) {
        return glm::angleAxis(3.14159265f, anyPerpendicular(a));
    }
    return glm::normalize(glm::rotation(a, b));
}

std::string lower(std::string_view s)
{
    std::string r(s);
    std::ranges::transform(r, r.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return r;
}

bool contains(const std::string& s, std::string_view part) { return s.find(part) != std::string::npos; }

f32 smoothFactor(f32 rate, f32 dt) { return dt <= 0.0f ? 1.0f : 1.0f - std::exp(-rate * dt); }

void toWorld(const Skeleton& skeleton, const Pose& local, const glm::mat4& world, std::vector<glm::mat4>& model,
             std::vector<glm::mat4>& out)
{
    model.resize(skeleton.jointCount());
    out.resize(skeleton.jointCount());
    localToModel(skeleton, local, model);
    for (usize j = 0; j < model.size(); ++j) {
        out[j] = world * model[j];
    }
}

} // namespace

// ---------------------------------------------------------------------------------------------- //

TwoBoneSolution solveTwoBone(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& target,
                             const glm::vec3& poleHint)
{
    TwoBoneSolution s;
    const f32 lab = glm::length(b - a);
    const f32 lbc = glm::length(c - b);
    const glm::vec3 toTarget = target - a;
    const f32 targetDistance = glm::length(toTarget);
    if (lab < 1e-6f || lbc < 1e-6f || targetDistance < 1e-6f) {
        return s;
    }
    s.reached = targetDistance <= lab + lbc && targetDistance >= std::abs(lab - lbc);
    const f32 dist = std::clamp(targetDistance, std::abs(lab - lbc) + 1e-4f, lab + lbc - 1e-4f);
    const glm::vec3 dir = toTarget / targetDistance;

    // Bend direction: where the mid joint currently sits off the root -> end line.
    const glm::vec3 ac = c - a;
    const glm::vec3 acDir = glm::length(ac) > 1e-6f ? glm::normalize(ac) : dir;
    glm::vec3 bend = (b - a) - acDir * glm::dot(b - a, acDir);
    if (glm::length(bend) < 1e-4f * lab) {
        bend = poleHint;
    }
    bend -= dir * glm::dot(bend, dir);
    bend = glm::length(bend) > 1e-6f ? glm::normalize(bend) : anyPerpendicular(dir);

    // Law of cosines: angle at the root between the target line and the upper bone.
    const f32 cosA = std::clamp((lab * lab + dist * dist - lbc * lbc) / (2.0f * lab * dist), -1.0f, 1.0f);
    const f32 sinA = std::sqrt(std::max(0.0f, 1.0f - cosA * cosA));
    const glm::vec3 newB = a + dir * (lab * cosA) + bend * (lab * sinA);

    s.upper = rotationBetween(b - a, newB - a);
    const glm::vec3 c1 = newB + s.upper * (c - b);
    s.lower = rotationBetween(c1 - newB, a + dir * dist - newB);
    return s;
}

glm::quat rotationOf(const glm::mat4& m)
{
    glm::mat3 r(m);
    for (int i = 0; i < 3; ++i) {
        const f32 l = glm::length(r[i]);
        if (l > 1e-12f) {
            r[i] /= l;
        }
    }
    return glm::normalize(glm::quat_cast(r));
}

void setWorldRotation(const Skeleton& skeleton, Pose& local, std::span<const glm::mat4> world, usize joint,
                      const glm::quat& worldRotation)
{
    const i32 parent = skeleton.parents()[joint];
    const glm::quat parentRotation = parent >= 0 ? rotationOf(world[static_cast<usize>(parent)]) : kIdentity;
    local.rotations[joint] = glm::normalize(glm::inverse(parentRotation) * worldRotation);
}

// ---------------------------------------------------------------------------------------------- //

std::vector<LegChain> findLegChains(const Skeleton& skeleton)
{
    static constexpr std::array<std::string_view, 12> kExclude = {"toe", "end", "ball", "twist", "_ik", "ik_", "helper",
                                                                  "pole", "target", "ctrl", "heel", "nub"};
    const auto& names = skeleton.names();
    const auto& parents = skeleton.parents();
    std::vector<LegChain> legs;
    for (usize j = 0; j < skeleton.jointCount(); ++j) {
        const std::string name = lower(names[j]);
        if (!contains(name, "foot") && !contains(name, "ankle")) {
            continue;
        }
        if (std::ranges::any_of(kExclude, [&](std::string_view x) { return contains(name, x); })) {
            continue;
        }
        const i32 knee = parents[j];
        const i32 hip = knee >= 0 ? parents[static_cast<usize>(knee)] : -1;
        if (hip < 0 || parents[static_cast<usize>(hip)] < 0) {
            continue;
        }
        LegChain leg{static_cast<usize>(hip), static_cast<usize>(knee), j, -1};
        for (usize c = j + 1; c < skeleton.jointCount(); ++c) {
            if (parents[c] == static_cast<i32>(j)) {
                leg.toe = static_cast<i32>(c);
                break;
            }
        }
        legs.push_back(leg);
    }
    // Order by rest X so results are stable.
    std::vector<glm::mat4> model(skeleton.jointCount());
    localToModel(skeleton, skeleton.restPose(), model);
    std::ranges::sort(legs, [&](const LegChain& l, const LegChain& r) { return model[l.foot][3].x < model[r.foot][3].x; });
    return legs;
}

i32 findPelvis(const Skeleton& skeleton, std::span<const LegChain> legs)
{
    if (legs.empty()) {
        return -1;
    }
    const i32 pelvis = skeleton.parents()[legs.front().upper];
    for (const LegChain& leg : legs) {
        if (skeleton.parents()[leg.upper] != pelvis) {
            return -1;
        }
    }
    return pelvis;
}

f32 estimateGroundSpeed(const Skeleton& skeleton, const AnimationClip& clip, std::span<const LegChain> legs)
{
    if (legs.empty() || clip.duration() <= 0.0f || clip.jointCount() != skeleton.jointCount()) {
        return 0.0f;
    }
    const i32 pelvis = findPelvis(skeleton, legs);
    const usize samples = std::clamp(static_cast<usize>(clip.duration() * 60.0f), usize{30}, usize{2000});
    const f32 step = clip.duration() / static_cast<f32>(samples);

    // feet[leg][sample] = foot position, rel[leg][sample] = foot - pelvis (model space).
    std::vector<std::vector<glm::vec3>> feet(legs.size(), std::vector<glm::vec3>(samples + 1));
    std::vector<std::vector<glm::vec3>> rel(legs.size(), std::vector<glm::vec3>(samples + 1));
    Pose pose;
    std::vector<glm::mat4> model(skeleton.jointCount());
    for (usize i = 0; i <= samples; ++i) {
        pose = skeleton.restPose();
        clip.sample(static_cast<f32>(i) * step, false, pose);
        localToModel(skeleton, pose, model);
        const glm::vec3 p = pelvis >= 0 ? position(model[static_cast<usize>(pelvis)]) : glm::vec3(0.0f);
        for (usize l = 0; l < legs.size(); ++l) {
            feet[l][i] = position(model[legs[l].foot]);
            rel[l][i] = feet[l][i] - p;
        }
    }

    f32 sum = 0.0f;
    usize count = 0;
    for (usize l = 0; l < legs.size(); ++l) {
        f32 lo = feet[l][0].y;
        f32 hi = lo;
        for (const glm::vec3& f : feet[l]) {
            lo = std::min(lo, f.y);
            hi = std::max(hi, f.y);
        }
        const f32 planted = lo + (hi - lo) * 0.33f;
        for (usize i = 0; i < samples; ++i) {
            if (feet[l][i].y > planted || feet[l][i + 1].y > planted) {
                continue;
            }
            const glm::vec3 d = rel[l][i + 1] - rel[l][i];
            sum += glm::length(glm::vec2(d.x, d.z)) / step;
            ++count;
        }
    }
    return count > 0 ? sum / static_cast<f32>(count) : 0.0f;
}

// ---------------------------------------------------------------------------------------------- //

void applyFootIK(const Skeleton& skeleton, std::span<const LegChain> legs, const glm::mat4& world,
                 const GroundQuery& ground, const FootIKSettings& settings, FootIKState& state, f32 dt, Pose& local)
{
    if (!settings.enabled || legs.empty() || !ground || local.jointCount() != skeleton.jointCount()) {
        return;
    }
    const usize n = legs.size();
    if (state.offsets.size() != n) {
        state = FootIKState{};
        state.offsets.assign(n, 0.0f);
        state.hits.assign(n, GroundHit{});
        state.grounded.assign(n, 0);
        state.targets.assign(n, glm::vec3(0.0f));
    }
    std::vector<glm::mat4> model;
    std::vector<glm::mat4> w;
    if (state.restAnkleHeight < 0.0f) {
        toWorld(skeleton, skeleton.restPose(), glm::mat4(1.0f), model, w);
        f32 h = 0.0f;
        for (const LegChain& leg : legs) {
            h += position(w[leg.foot]).y;
        }
        state.restAnkleHeight = std::max(h / static_cast<f32>(n), 0.0f);
    }
    const f32 scale = glm::length(glm::vec3(world[1]));
    const f32 groundY = world[3].y;
    const f32 blend = state.initialized ? smoothFactor(settings.smoothing, dt) : 1.0f;
    state.initialized = true;

    toWorld(skeleton, local, world, model, w);
    std::vector<glm::vec3> animated(n);
    for (usize i = 0; i < n; ++i) {
        animated[i] = position(w[legs[i].foot]);
        const glm::vec3 from(animated[i].x, groundY + settings.probeHeight, animated[i].z);
        const auto hit = ground(from, settings.probeHeight + settings.maxStepDown);
        f32 offset = 0.0f;
        state.grounded[i] = hit ? 1 : 0;
        if (hit) {
            state.hits[i] = *hit;
            offset = std::clamp(hit->point.y - groundY, -settings.maxStepDown, settings.probeHeight);
        }
        state.offsets[i] += (offset - state.offsets[i]) * blend;
    }

    // Pelvis: down by the lowest foot's offset, so that leg can reach without over-stretching.
    f32 pelvisTarget = 0.0f;
    for (const f32 o : state.offsets) {
        pelvisTarget = std::min(pelvisTarget, o);
    }
    pelvisTarget = std::max(pelvisTarget, -settings.maxPelvisDrop);
    state.pelvisOffset += (pelvisTarget - state.pelvisOffset) * blend;
    const i32 pelvis = findPelvis(skeleton, legs);
    if (pelvis >= 0 && std::abs(state.pelvisOffset) > 1e-6f) {
        const i32 parent = skeleton.parents()[static_cast<usize>(pelvis)];
        const glm::mat3 parentWorld = parent >= 0 ? glm::mat3(w[static_cast<usize>(parent)]) : glm::mat3(world);
        local.translations[static_cast<usize>(pelvis)] += glm::inverse(parentWorld) * glm::vec3(0.0f, state.pelvisOffset, 0.0f);
        toWorld(skeleton, local, world, model, w);
    }

    const glm::vec3 forward = glm::normalize(glm::mat3(world) * glm::vec3(0.0f, 0.0f, 1.0f));
    for (usize i = 0; i < n; ++i) {
        const LegChain& leg = legs[i];
        const glm::vec3 target = animated[i] + glm::vec3(0.0f, state.offsets[i], 0.0f);
        state.targets[i] = target;
        const glm::quat upperWorld = rotationOf(w[leg.upper]);
        const glm::quat lowerWorld = rotationOf(w[leg.lower]);
        const glm::quat footWorld = rotationOf(w[leg.foot]);

        const TwoBoneSolution s =
            solveTwoBone(position(w[leg.upper]), position(w[leg.lower]), position(w[leg.foot]), target, forward);
        const glm::quat newUpper = glm::normalize(s.upper * upperWorld);
        const glm::quat newLower = glm::normalize(s.lower * s.upper * lowerWorld);
        setWorldRotation(skeleton, local, w, leg.upper, newUpper);
        local.rotations[leg.lower] = glm::normalize(glm::inverse(newUpper) * newLower);

        // Foot: keep its animated world orientation, tilted to the slope while it is planted.
        glm::quat newFoot = footWorld;
        if (state.grounded[i] && settings.footAlign > 0.0f) {
            const f32 lift = (animated[i].y - groundY) - state.restAnkleHeight * scale;
            const f32 planted = std::clamp(1.0f - lift / 0.1f, 0.0f, 1.0f); // fades out over 10 cm of lift
            const glm::quat tilt = rotationBetween(glm::vec3(0.0f, 1.0f, 0.0f), state.hits[i].normal);
            newFoot = glm::normalize(glm::slerp(kIdentity, tilt, settings.footAlign * planted) * footWorld);
        }
        local.rotations[leg.foot] = glm::normalize(glm::inverse(newLower) * newFoot);
    }
}

} // namespace kinetic::anim
