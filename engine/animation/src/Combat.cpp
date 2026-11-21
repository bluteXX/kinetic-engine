#include "kinetic/animation/Combat.h"

#include "kinetic/animation/PoseOps.h"

#include <algorithm>
#include <cmath>
#include <array>
#include <cctype>
#include <cstdio>

namespace kinetic::anim {

namespace {

std::string lower(std::string s)
{
    std::ranges::transform(s, s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool containsAny(const std::string& s, std::initializer_list<const char*> keys)
{
    return std::ranges::any_of(keys, [&](const char* k) { return s.find(k) != std::string::npos; });
}

} // namespace

std::vector<i32> findAttackClips(std::span<const AnimationClip> clips)
{
    std::vector<i32> attacks;
    for (usize i = 0; i < clips.size(); ++i) {
        if (containsAny(lower(clips[i].name()), {"attack", "slash", "strike", "combo", "swing"})) {
            attacks.push_back(static_cast<i32>(i));
        }
    }
    std::ranges::sort(attacks, [&](i32 a, i32 b) { return clips[static_cast<usize>(a)].name() < clips[static_cast<usize>(b)].name(); });
    return attacks;
}

i32 findCombatIdle(std::span<const AnimationClip> clips)
{
    for (usize i = 0; i < clips.size(); ++i) {
        const std::string name = lower(clips[i].name());
        if (name.find("idle") != std::string::npos && containsAny(name, {"combat", "fight", "guard", "battle"})) {
            return static_cast<i32>(i);
        }
    }
    return -1;
}

i32 findWeaponHand(const Skeleton& skeleton)
{
    i32 anyHand = -1;
    for (usize j = 0; j < skeleton.jointCount(); ++j) {
        std::string name = lower(skeleton.names()[j]);
        if (const usize at = name.rfind(':'); at != std::string::npos) name = name.substr(at + 1);
        if (name == "righthand" || name == "hand_r" || name == "r_hand" || name == "hand.r") {
            return static_cast<i32>(j);
        }
        if (anyHand < 0 && name.find("hand") != std::string::npos && name.find("thumb") == std::string::npos &&
            name.find("index") == std::string::npos) {
            anyHand = static_cast<i32>(j);
        }
    }
    return anyHand;
}

glm::mat4 weaponSocket(const Skeleton& skeleton, i32 hand)
{
    if (hand < 0 || static_cast<usize>(hand) >= skeleton.jointCount()) {
        return glm::mat4(1.0f);
    }
    i32 finger = -1;
    i32 thumb = -1;
    for (usize j = 0; j < skeleton.jointCount(); ++j) {
        if (skeleton.parents()[j] != hand) continue;
        const std::string name = lower(skeleton.names()[j]);
        if (name.find("thumb") != std::string::npos) thumb = static_cast<i32>(j);
        else if (name.find("middle") != std::string::npos || (finger < 0 && name.find("index") != std::string::npos)) finger = static_cast<i32>(j);
    }
    if (finger < 0 || thumb < 0) {
        return glm::mat4(1.0f);
    }
    std::vector<glm::mat4> model(skeleton.jointCount());
    localToModel(skeleton, skeleton.restPose(), model);
    const glm::vec3 h(model[static_cast<usize>(hand)][3]);
    const glm::vec3 f(model[static_cast<usize>(finger)][3]);
    const glm::vec3 th(model[static_cast<usize>(thumb)][3]);
    const glm::vec3 x = glm::normalize(f - h);
    glm::vec3 y = th - h;
    y = glm::normalize(y - x * glm::dot(y, x));
    const glm::vec3 z = glm::cross(x, y);
    glm::mat4 socket(1.0f);
    socket[0] = glm::vec4(x, 0.0f);
    socket[1] = glm::vec4(y, 0.0f);
    socket[2] = glm::vec4(z, 0.0f);
    socket[3] = glm::vec4(h + (f - h) * 0.55f, 1.0f); // grip: in the palm, near the finger roots
    return glm::inverse(model[static_cast<usize>(hand)]) * socket;
}

f32 estimateStrikeTime(const Skeleton& skeleton, const AnimationClip& clip, usize joint)
{
    constexpr usize kSamples = 90;
    Pose pose;
    std::vector<glm::mat4> model(skeleton.jointCount());
    glm::vec3 previous(0.0f);
    f32 best = 0.0f;
    f32 bestTime = 0.5f;
    for (usize i = 0; i <= kSamples; ++i) {
        const f32 t = static_cast<f32>(i) / kSamples;
        pose = skeleton.restPose();
        clip.sample(t * clip.duration(), false, pose);
        localToModel(skeleton, pose, model);
        const glm::vec3 p(model[joint][3]);
        if (i > 0) {
            const f32 speed = glm::length(p - previous);
            if (speed > best) {
                best = speed;
                bestTime = t;
            }
        }
        previous = p;
    }
    return bestTime;
}

namespace {

// Bone positions relative to the hips (model space) - a pose "shape" independent of root motion.
void poseShape(const Skeleton& skeleton, const AnimationClip& clip, f32 normalized, i32 hips, std::vector<glm::vec3>& out)
{
    Pose pose = skeleton.restPose();
    clip.sample(normalized * clip.duration(), false, pose);
    std::vector<glm::mat4> model(skeleton.jointCount());
    localToModel(skeleton, pose, model);
    const glm::vec3 origin = hips >= 0 ? glm::vec3(model[static_cast<usize>(hips)][3]) : glm::vec3(0.0f);
    out.resize(model.size());
    for (usize j = 0; j < model.size(); ++j) {
        out[j] = glm::vec3(model[j][3]) - origin;
    }
}

i32 hipsOf(const Skeleton& skeleton)
{
    for (usize j = 0; j < skeleton.jointCount(); ++j) {
        std::string name = lower(skeleton.names()[j]);
        if (const usize at = name.rfind(':'); at != std::string::npos) name = name.substr(at + 1);
        if (name == "hips" || name == "pelvis") return static_cast<i32>(j);
    }
    return -1;
}

} // namespace

f32 findBestTransitionTime(const Skeleton& skeleton, const AnimationClip& a, const AnimationClip& b, f32 from, f32 to)
{
    from = std::clamp(from, 0.0f, 1.0f);
    to = std::clamp(to, from, 1.0f);
    const i32 hips = hipsOf(skeleton);
    std::vector<glm::vec3> target;
    std::vector<glm::vec3> shape;
    poseShape(skeleton, b, 0.0f, hips, target);
    constexpr int kSamples = 40;
    std::vector<f32> distance(kSamples + 1);
    f32 best = 1e30f;
    for (int i = 0; i <= kSamples; ++i) {
        const f32 t = from + (to - from) * static_cast<f32>(i) / kSamples;
        poseShape(skeleton, a, t, hips, shape);
        f32 d = 0.0f;
        for (usize j = 0; j < shape.size(); ++j) {
            const glm::vec3 diff = shape[j] - target[j];
            d += glm::dot(diff, diff);
        }
        distance[static_cast<usize>(i)] = d;
        best = std::min(best, d);
    }
    // Earliest time where most (75%) of the pose difference is gone: responsive, and the crossfade
    // covers the rest.
    const f32 worst = *std::max_element(distance.begin(), distance.end());
    const f32 good = best + (worst - best) * 0.25f;
    for (int i = 0; i <= kSamples; ++i) {
        if (distance[static_cast<usize>(i)] <= good + 1e-6f) {
            return from + (to - from) * static_cast<f32>(i) / kSamples;
        }
    }
    return from;
}

ComboReport addAttackCombo(BlendTree& tree, const Skeleton& skeleton, std::span<const AnimationClip> clips,
                           std::span<const i32> attackClips, i32 combatIdleClip, i32 locomotionState,
                           const ComboSettings& settings)
{
    ComboReport report;
    if (attackClips.empty()) {
        return report;
    }
    const i32 attack = tree.addParameter("attack", 0.0f, 0.0f, 1.0f, ParameterType::Trigger);
    tree.parameters[static_cast<usize>(attack)].bufferTime = settings.inputBuffer;
    const i32 moveInput = tree.addParameter("moveInput", 0.0f, 0.0f, 1.0f);
    const TransitionCondition attackPressed{attack, ConditionOp::IsTrue, 0.0f};
    const TransitionCondition moving{moveInput, ConditionOp::Greater, 0.1f};

    // Combat idle: the stance between attacks (the "base pose").
    i32 rest = locomotionState;
    if (combatIdleClip >= 0) {
        const i32 node = tree.addClip("Combat Idle", combatIdleClip);
        rest = report.combatIdleState = tree.addState("Combat Idle", node, {260.0f, 300.0f});
        if (locomotionState >= 0) {
            const i32 relax = tree.addTransition(rest, locomotionState, settings.blendOut);
            tree.stateMachine.transitions[static_cast<usize>(relax)].conditions.push_back(moving);
            const i32 calm = tree.addTransition(rest, locomotionState, 0.6f);
            auto& t = tree.stateMachine.transitions[static_cast<usize>(calm)];
            t.hasExitTime = true;
            t.exitTime = settings.combatIdleTime;
        }
    }

    const i32 hand = findWeaponHand(skeleton);
    std::vector<f32> strikes;
    char line[160];
    // Strikes first (the combo windows need the next attack's strike-independent first frame).
    for (const i32 clip : attackClips) {
        strikes.push_back(hand >= 0 ? estimateStrikeTime(skeleton, clips[static_cast<usize>(clip)], static_cast<usize>(hand)) : 0.5f);
    }
    std::vector<f32> windows(attackClips.size(), 0.95f);
    for (usize i = 0; i + 1 < attackClips.size(); ++i) {
        windows[i] = findBestTransitionTime(skeleton, clips[static_cast<usize>(attackClips[i])], clips[static_cast<usize>(attackClips[i + 1])],
                                            std::min(strikes[i] + settings.comboDelay, 0.9f), 0.9f);
    }
    for (usize i = 0; i < attackClips.size(); ++i) {
        const i32 clip = attackClips[i];
        const i32 node = tree.addClip("Attack " + std::to_string(i + 1), clip);
        tree.nodes[static_cast<usize>(node)].loop = false;
        const i32 state = tree.addState("Attack " + std::to_string(i + 1), node, {560.0f + 230.0f * static_cast<f32>(i), 300.0f});
        AnimState& st = tree.stateMachine.states[static_cast<usize>(state)];
        st.lockMovement = true;
        st.turnWindow = settings.turnWindow;
        report.attackStates.push_back(state);

        const f32 strike = strikes[i];
        std::snprintf(line, sizeof(line), "Attack %zu = %s: strike at %.0f%%, combo from %.0f%% (best pose match), cancel from %.0f%%",
                      i + 1, clips[static_cast<usize>(clip)].name().c_str(), static_cast<double>(strike * 100.0f),
                      static_cast<double>(windows[i] * 100.0f), static_cast<double>(std::min(strike + settings.recoveryDelay, 0.9f) * 100.0f));
        report.lines.emplace_back(line);
    }

    // Transitions (first match wins, in this order per state: next attack, cancel, play out).
    for (usize i = 0; i < report.attackStates.size(); ++i) {
        const i32 state = report.attackStates[i];
        const f32 strike = strikes[i];

        // Into the first attack: from locomotion and from the combat idle.
        if (i == 0) {
            for (const i32 from : {locomotionState, report.combatIdleState}) {
                if (from >= 0) {
                    const i32 t = tree.addTransition(from, state, settings.blendIn);
                    tree.stateMachine.transitions[static_cast<usize>(t)].conditions.push_back(attackPressed);
                }
            }
        }
        if (i + 1 < report.attackStates.size()) {
            const i32 t = tree.addTransition(state, report.attackStates[i + 1], settings.comboBlend);
            auto& next = tree.stateMachine.transitions[static_cast<usize>(t)];
            next.conditions.push_back(attackPressed);
            next.hasExitTime = true;
            next.exitTime = windows[i];
        }
        if (locomotionState >= 0) {
            const i32 t = tree.addTransition(state, locomotionState, settings.cancelBlend);
            auto& cancel = tree.stateMachine.transitions[static_cast<usize>(t)];
            cancel.conditions.push_back(moving);
            cancel.hasExitTime = true;
            cancel.exitTime = std::max(std::min(strike + settings.recoveryDelay, 0.9f), windows[i]);
        }
        if (rest >= 0) {
            const i32 t = tree.addTransition(state, rest, settings.blendOut);
            auto& end = tree.stateMachine.transitions[static_cast<usize>(t)];
            end.hasExitTime = true;
            end.exitTime = 0.92f;
        }
    }
    return report;
}

namespace {

bool hasAttackCondition(const AnimTransition& t, i32 attack)
{
    return std::ranges::any_of(t.conditions, [&](const TransitionCondition& c) { return c.parameter == attack && c.op == ConditionOp::IsTrue; });
}

} // namespace

std::vector<i32> attackChain(const BlendTree& tree)
{
    std::vector<i32> chain;
    const StateMachine& sm = tree.stateMachine;
    const i32 attack = tree.findParameter("attack");
    if (attack < 0) {
        return chain;
    }
    const auto isAttack = [&](i32 s) { return sm.validState(s) && sm.states[static_cast<usize>(s)].lockMovement; };
    // The first attack: entered with the trigger from a state that is not an attack.
    for (const AnimTransition& t : sm.transitions) {
        if (isAttack(t.to) && !isAttack(t.from) && hasAttackCondition(t, attack)) {
            chain.push_back(t.to);
            break;
        }
    }
    while (!chain.empty()) {
        i32 next = -1;
        for (const AnimTransition& t : sm.transitions) {
            if (t.from == chain.back() && isAttack(t.to) && hasAttackCondition(t, attack) && std::ranges::find(chain, t.to) == chain.end()) {
                next = t.to;
                break;
            }
        }
        if (next < 0) break;
        chain.push_back(next);
    }
    return chain;
}

i32 stateClip(const BlendTree& tree, i32 state)
{
    if (!tree.stateMachine.validState(state)) {
        return -1;
    }
    const i32 node = tree.stateMachine.states[static_cast<usize>(state)].motion;
    if (!tree.validNode(node) || tree.nodes[static_cast<usize>(node)].type != BlendNodeType::Clip) {
        return -1;
    }
    return tree.nodes[static_cast<usize>(node)].clip;
}

namespace {

struct BladeSample {
    glm::quat blade{1.0f, 0.0f, 0.0f, 0.0f}; // blade frame rotation, model space
    glm::vec3 axis{0.0f, 1.0f, 0.0f};         // blade direction, model space
    glm::vec3 velocity{0.0f};                 // of the measured point, across the blade axis
};

glm::quat rotationOf(const glm::mat4& m)
{
    glm::mat3 r(m);
    for (int i = 0; i < 3; ++i) {
        const f32 l = glm::length(r[i]);
        if (l > 1e-8f) r[i] /= l;
    }
    return glm::normalize(glm::quat_cast(r));
}

std::vector<f32> sampleTimes(const AnimationClip& clip)
{
    const usize n = std::max<usize>(2, static_cast<usize>(std::ceil(clip.duration() * 60.0f)));
    std::vector<f32> times(n + 1);
    for (usize i = 0; i <= n; ++i) times[i] = clip.duration() * static_cast<f32>(i) / static_cast<f32>(n);
    return times;
}

// Blade frames at `times`, with velocities from central differences.
std::vector<BladeSample> sampleBlade(const Skeleton& skeleton, const AnimationClip& clip, i32 hand, const glm::mat4& socket, f32 point,
                                     const std::vector<f32>& times)
{
    Pose pose;
    std::vector<glm::mat4> model(skeleton.jointCount());
    const auto frame = [&](f32 t) {
        pose = skeleton.restPose();
        clip.sample(std::clamp(t, 0.0f, clip.duration()), false, pose);
        localToModel(skeleton, pose, model);
        return model[static_cast<usize>(hand)] * socket;
    };
    std::vector<BladeSample> out(times.size());
    const f32 h = times.size() > 1 ? times[1] - times[0] : 1.0f / 60.0f;
    for (usize i = 0; i < times.size(); ++i) {
        const glm::mat4 b = frame(times[i]);
        BladeSample& s = out[i];
        s.blade = rotationOf(b);
        s.axis = glm::normalize(glm::vec3(b[1]));
        const f32 t0 = std::max(times[i] - h, 0.0f);
        const f32 t1 = std::min(times[i] + h, clip.duration());
        if (t1 > t0) {
            const glm::vec3 p0(frame(t0) * glm::vec4(0.0f, point, 0.0f, 1.0f));
            const glm::vec3 p1(frame(t1) * glm::vec4(0.0f, point, 0.0f, 1.0f));
            const glm::vec3 v = (p1 - p0) / (t1 - t0);
            s.velocity = v - s.axis * glm::dot(v, s.axis);
        }
    }
    return out;
}

// Angle (rad) to turn the blade about its axis so an edge faces its motion, in (-pi/2, pi/2].
f32 edgeTwist(const BladeSample& s)
{
    const glm::vec3 local = glm::inverse(s.blade) * s.velocity;
    f32 t = std::atan2(-local.z, local.x); // turns +X onto the motion
    constexpr f32 kHalf = 1.5707963f;
    if (t > kHalf) t -= 2.0f * kHalf;
    if (t <= -kHalf) t += 2.0f * kHalf;
    return t;
}

} // namespace

f32 bladeFlatness(const Skeleton& skeleton, const AnimationClip& clip, i32 hand, const glm::mat4& socket, f32 point)
{
    if (hand < 0 || static_cast<usize>(hand) >= skeleton.jointCount() || clip.duration() <= 0.0f) return 0.0f;
    const auto samples = sampleBlade(skeleton, clip, hand, socket, point, sampleTimes(clip));
    f32 sum = 0.0f;
    f32 weight = 0.0f;
    for (const BladeSample& s : samples) {
        const f32 w = glm::dot(s.velocity, s.velocity);
        sum += w * std::abs(edgeTwist(s));
        weight += w;
    }
    return weight > 0.0f ? glm::degrees(sum / weight) : 0.0f;
}

namespace {

// One pass of alignBladeEdge; returns the largest turn added (degrees).
f32 alignBladeEdgePass(const Skeleton& skeleton, AnimationClip& clip, i32 hand, const glm::mat4& socket, f32 point)
{
    f32 maxTwist = 0.0f;
    const std::vector<f32> times = sampleTimes(clip);
    const auto samples = sampleBlade(skeleton, clip, hand, socket, point, times);
    const usize n = samples.size();

    // Wanted twist per frame, kept continuous (a double-edged blade may use either edge: take the
    // one nearer the previous frame's choice, within +-120 degrees), faded out while slow.
    f32 fastest = 0.0f;
    for (const BladeSample& s : samples) fastest = std::max(fastest, glm::length(s.velocity));
    std::vector<f32> wanted(n, 0.0f);
    std::vector<f32> weight(n, 0.0f);
    f32 last = 0.0f;
    constexpr f32 kPi = 3.14159265f;
    const f32 limit = glm::radians(120.0f);
    for (usize i = 0; i < n; ++i) {
        const f32 speed = glm::length(samples[i].velocity);
        const f32 x = std::clamp((speed - 0.15f * fastest) / std::max(0.3f * fastest, 1e-4f), 0.0f, 1.0f);
        weight[i] = x * x * (3.0f - 2.0f * x);
        if (weight[i] <= 0.0f) continue;
        f32 best = edgeTwist(samples[i]);
        for (const f32 c : {best - kPi, best + kPi}) {
            if (std::abs(c) <= limit && std::abs(c - last) < std::abs(best - last)) best = c;
        }
        wanted[i] = best;
        last = best;
    }
    // Smooth (gaussian, sigma 0.05 s): the weighted twist over the smoothed weight, then faded by
    // the weight - the full turn while fast, back to the clip's own hand while slow.
    const f32 dt = times[1] - times[0];
    const int radius = std::max(1, static_cast<int>(std::lround(0.12f / dt)));
    const f32 sigma = 0.025f / dt;
    std::vector<f32> twist(n, 0.0f);
    for (usize i = 0; i < n; ++i) {
        f32 num = 0.0f;
        f32 den = 0.0f;
        f32 gsum = 0.0f;
        for (int k = -radius; k <= radius; ++k) {
            const i64 j = static_cast<i64>(i) + k;
            if (j < 0 || j >= static_cast<i64>(n)) continue;
            const f32 g = std::exp(-0.5f * static_cast<f32>(k * k) / (sigma * sigma));
            num += g * weight[static_cast<usize>(j)] * wanted[static_cast<usize>(j)];
            den += g * weight[static_cast<usize>(j)];
            gsum += g;
        }
        const f32 fade = gsum > 0.0f ? den / gsum : 0.0f;
        twist[i] = den > 1e-5f ? num / den * std::min(1.0f, fade * 1.5f) : 0.0f;
        maxTwist = std::max(maxTwist, glm::degrees(std::abs(twist[i])));
    }

    // New hand rotation keys: the clip's hand turned about the blade axis (model space).
    const i32 parent = skeleton.parents()[static_cast<usize>(hand)];
    Pose pose;
    std::vector<glm::mat4> model(skeleton.jointCount());
    Track<glm::quat> track;
    track.times = times;
    track.values.resize(n);
    for (usize i = 0; i < n; ++i) {
        pose = skeleton.restPose();
        clip.sample(times[i], false, pose);
        localToModel(skeleton, pose, model);
        const glm::quat local = pose.rotations[static_cast<usize>(hand)];
        const glm::quat parentRotation = parent >= 0 ? rotationOf(model[static_cast<usize>(parent)]) : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        const glm::quat turn = glm::angleAxis(twist[i], samples[i].axis);
        glm::quat q = glm::normalize(glm::inverse(parentRotation) * turn * parentRotation * local);
        if (i > 0 && glm::dot(q, track.values[i - 1]) < 0.0f) q = -q;
        track.values[i] = q;
    }
    auto& tracks = clip.tracks();
    if (tracks.size() <= static_cast<usize>(hand)) tracks.resize(static_cast<usize>(hand) + 1);
    tracks[static_cast<usize>(hand)].rotation = std::move(track);
    return maxTwist;
}

} // namespace

EdgeAlignment alignBladeEdge(const Skeleton& skeleton, AnimationClip& clip, i32 hand, const glm::mat4& socket, f32 point)
{
    EdgeAlignment report;
    if (hand < 0 || static_cast<usize>(hand) >= skeleton.jointCount() || clip.duration() <= 0.0f) return report;
    report.before = bladeFlatness(skeleton, clip, hand, socket, point);
    report.after = report.before;
    const AnimationClip original = clip;
    // Turning the hand moves the blade a little too (it is not on the wrist's axis), so the rest
    // is corrected in a few more passes, each keeping the result only while it helps.
    for (int pass = 0; pass < 6; ++pass) {
        AnimationClip trial = clip;
        const f32 twist = alignBladeEdgePass(skeleton, trial, hand, socket, point);
        const f32 flatness = bladeFlatness(skeleton, trial, hand, socket, point);
        if (flatness > report.after - 0.5f) break;
        clip = std::move(trial);
        report.after = flatness;
        (void)twist;
    }
    // How far the hand was turned, at most (the parent bones are untouched).
    Pose a;
    Pose b;
    for (const f32 t : sampleTimes(clip)) {
        a = skeleton.restPose();
        b = skeleton.restPose();
        original.sample(t, false, a);
        clip.sample(t, false, b);
        const glm::quat d = glm::inverse(a.rotations[static_cast<usize>(hand)]) * b.rotations[static_cast<usize>(hand)];
        report.maxTwist = std::max(report.maxTwist, glm::degrees(2.0f * std::acos(std::min(1.0f, std::abs(glm::normalize(d).w)))));
    }
    return report;
}

} // namespace kinetic::anim
