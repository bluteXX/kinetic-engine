#include "kinetic/animation/Locomotion.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <string_view>

namespace kinetic::anim {

namespace {

std::string lower(std::string_view s)
{
    std::string r(s);
    std::ranges::transform(r, r.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return r;
}

struct Role {
    const char* name;
    std::array<const char*, 3> keys;
};
constexpr std::array<Role, 3> kRoles = {{
    {"idle", {"idle", "stand", "breath"}},
    {"walk", {"walk", nullptr, nullptr}},
    {"run", {"run", "jog", "sprint"}},
}};

} // namespace

std::vector<LocomotionClip> findLocomotionClips(const Skeleton& skeleton, std::span<const AnimationClip> clips,
                                                std::span<const LegChain> legs, f32 scale)
{
    std::vector<LocomotionClip> found;
    for (const Role& role : kRoles) {
        i32 best = -1;
        for (usize i = 0; i < clips.size(); ++i) {
            const std::string name = lower(clips[i].name());
            const bool match = std::ranges::any_of(role.keys, [&](const char* k) { return k && name.find(k) != std::string::npos; });
            // Prefer the plainest name ("walk" over "walk_left_strafe").
            if (match && (best < 0 || name.size() < clips[static_cast<usize>(best)].name().size())) {
                best = static_cast<i32>(i);
            }
        }
        if (best < 0) {
            continue;
        }
        LocomotionClip c;
        c.clip = best;
        c.role = role.name;
        c.speed = c.role == "idle" ? 0.0f : estimateGroundSpeed(skeleton, clips[static_cast<usize>(best)], legs) * scale;
        found.push_back(c);
    }
    return found;
}

std::optional<BlendTree> makeLocomotionTree(const Skeleton& skeleton, std::span<const AnimationClip> clips,
                                            std::span<const LegChain> legs, f32 scale)
{
    std::vector<LocomotionClip> found = findLocomotionClips(skeleton, clips, legs, scale);
    if (std::ranges::none_of(found, [](const LocomotionClip& c) { return c.role != "idle"; })) {
        return std::nullopt;
    }
    // Moving clips with no measurable speed (no legs found): typical human speeds.
    for (LocomotionClip& c : found) {
        if (c.role != "idle" && c.speed < 0.05f) {
            c.speed = c.role == "walk" ? 1.4f : 3.5f;
        }
    }
    // In-place turns (root yaw extracted, see ClipTools::prepareLocomotionClips).
    std::vector<i32> turns;
    f32 maxTurn = 0.0f;
    for (usize i = 0; i < clips.size(); ++i) {
        if (std::abs(clips[i].rootYawRate()) > 0.3f) {
            turns.push_back(static_cast<i32>(i));
            maxTurn = std::max(maxTurn, std::abs(clips[i].rootYawRate()));
        }
    }

    BlendTree tree;
    tree.name = "Locomotion";
    f32 maxSpeed = 1.0f;
    for (const LocomotionClip& c : found) {
        maxSpeed = std::max(maxSpeed, c.speed);
    }
    const i32 speed = tree.addParameter("speed", 0.0f, 0.0f, std::ceil(maxSpeed * 1.5f));
    const f32 turnRange = std::max(std::ceil(maxTurn * 1.5f), 1.0f);
    const i32 turn = tree.addParameter("turn", 0.0f, -turnRange, turnRange);
    const i32 grounded = tree.addParameter("grounded", 1.0f, 0.0f, 1.0f, ParameterType::Bool);
    const i32 jump = tree.addParameter("jump", 0.0f, 0.0f, 1.0f, ParameterType::Trigger);

    const bool twoD = !turns.empty();
    const i32 blend = twoD ? tree.addBlend2D("Locomotion", turn, speed) : tree.addBlend1D("Locomotion", speed);
    for (const LocomotionClip& c : found) {
        std::string nodeName = c.role;
        nodeName[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(nodeName[0])));
        const i32 node = tree.addClip(nodeName, c.clip);
        tree.addChild(blend, node, twoD ? 0.0f : c.speed, twoD ? c.speed : 0.0f);
        if (c.role == "idle") {
            // An idle is not a step cycle (often much longer): it keeps its own clock.
            for (BlendChild& child : tree.nodes[static_cast<usize>(blend)].children) {
                if (child.node == node) child.sync = false;
            }
        }
    }
    for (const i32 t : turns) {
        const f32 rate = clips[static_cast<usize>(t)].rootYawRate();
        const i32 node = tree.addClip(rate > 0.0f ? "Turn Left" : "Turn Right", t);
        tree.addChild(blend, node, rate, 0.0f);
        tree.nodes[static_cast<usize>(blend)].children.back().sync = false;
    }
    if (twoD) {
        // Walking / running while turning is still walking / running (there are no turning walk
        // clips): the moving clips also sit at both ends of the turn axis, so only turning on the
        // spot plays the turn clips.
        for (const LocomotionClip& c : found) {
            if (c.role == "idle") {
                continue;
            }
            for (const f32 side : {1.0f, -1.0f}) {
                std::string nodeName = c.role;
                nodeName[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(nodeName[0])));
                nodeName += side > 0.0f ? " (turning left)" : " (turning right)";
                tree.addChild(blend, tree.addClip(nodeName, c.clip), side * maxTurn, c.speed);
            }
        }
    }
    tree.root = blend;

    // State machine: Locomotion, plus Jump when the model has a jump clip.
    const i32 locomotion = tree.addState("Locomotion", blend, {260.0f, 120.0f});
    for (usize i = 0; i < clips.size(); ++i) {
        std::string name = clips[i].name();
        std::ranges::transform(name, name.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        if (name.find("jump") == std::string::npos) {
            continue;
        }
        const i32 clipNode = tree.addClip("Jump", static_cast<i32>(i));
        tree.nodes[static_cast<usize>(clipNode)].loop = false;
        const i32 jumpState = tree.addState("Jump", clipNode, {520.0f, 120.0f});
        const i32 up = tree.addTransition(locomotion, jumpState, 0.1f);
        tree.stateMachine.transitions[static_cast<usize>(up)].conditions.push_back({jump, ConditionOp::IsTrue, 0.0f});
        const i32 down = tree.addTransition(jumpState, locomotion, 0.25f);
        auto& back = tree.stateMachine.transitions[static_cast<usize>(down)];
        back.hasExitTime = true;
        back.exitTime = 0.8f;
        back.conditions.push_back({grounded, ConditionOp::IsTrue, 0.0f});
        break;
    }
    tree.stateMachine.defaultState = locomotion;
    return tree;
}

} // namespace kinetic::anim
