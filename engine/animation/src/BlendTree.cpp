#include "kinetic/animation/BlendTree.h"

#include "kinetic/animation/PoseOps.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>

namespace kinetic::anim {

namespace {

constexpr f32 kMinDuration = 1e-3f;
constexpr int kMaxDepth = 64;

const AnimationClip* clipOf(const BlendNode& node, std::span<const AnimationClip> clips)
{
    return node.clip >= 0 && static_cast<usize>(node.clip) < clips.size() ? &clips[static_cast<usize>(node.clip)] : nullptr;
}

f32 wrap01(f32 x)
{
    x -= std::floor(x);
    return x >= 1.0f ? 0.0f : x;
}

f32 parameterValue(const BlendTreeInstance& inst, i32 p)
{
    return p >= 0 && static_cast<usize>(p) < inst.parameters.size() ? inst.parameters[static_cast<usize>(p)] : 0.0f;
}

i32 fixIndex(i32 index, i32 removed) { return index == removed ? -1 : (index > removed ? index - 1 : index); }

} // namespace

// ---------------------------------------------------------------------------------------------- //
// Graph editing

i32 BlendTree::addParameter(std::string parameterName, f32 defaultValue, f32 min, f32 max, ParameterType type)
{
    if (const i32 existing = findParameter(parameterName); existing >= 0) {
        return existing;
    }
    parameters.push_back({std::move(parameterName), defaultValue, min, max, type});
    return static_cast<i32>(parameters.size() - 1);
}

i32 BlendTree::findParameter(std::string_view parameterName) const
{
    for (usize i = 0; i < parameters.size(); ++i) {
        if (parameters[i].name == parameterName) {
            return static_cast<i32>(i);
        }
    }
    return -1;
}

i32 BlendTree::addClip(std::string nodeName, i32 clip, f32 playbackSpeed)
{
    BlendNode node;
    node.type = BlendNodeType::Clip;
    node.name = std::move(nodeName);
    node.clip = clip;
    node.playbackSpeed = playbackSpeed;
    nodes.push_back(std::move(node));
    const auto index = static_cast<i32>(nodes.size() - 1);
    if (root < 0) {
        root = index;
    }
    return index;
}

i32 BlendTree::addBlend1D(std::string nodeName, i32 parameter)
{
    BlendNode node;
    node.type = BlendNodeType::Blend1D;
    node.name = std::move(nodeName);
    node.parameter = parameter;
    nodes.push_back(std::move(node));
    const auto index = static_cast<i32>(nodes.size() - 1);
    if (root < 0) {
        root = index;
    }
    return index;
}

i32 BlendTree::addBlend2D(std::string nodeName, i32 parameterX, i32 parameterY)
{
    const i32 index = addBlend1D(std::move(nodeName), parameterX);
    nodes[static_cast<usize>(index)].type = BlendNodeType::Blend2D;
    nodes[static_cast<usize>(index)].parameterY = parameterY;
    nodes[static_cast<usize>(index)].scalePlayback = false;
    return index;
}

void BlendTree::addChild(i32 blend, i32 child, f32 threshold, f32 thresholdY)
{
    if (!validNode(blend) || !validNode(child) || !nodes[static_cast<usize>(blend)].isBlend()) {
        return;
    }
    nodes[static_cast<usize>(blend)].children.push_back({child, threshold, thresholdY, true});
    sortChildren(blend);
}

void BlendTree::sortChildren(i32 blend)
{
    if (!validNode(blend) || nodes[static_cast<usize>(blend)].type != BlendNodeType::Blend1D) {
        return;
    }
    auto& children = nodes[static_cast<usize>(blend)].children;
    std::ranges::stable_sort(children, {}, &BlendChild::threshold);
}

void BlendTree::removeNode(i32 node)
{
    if (!validNode(node)) {
        return;
    }
    nodes.erase(nodes.begin() + node);
    for (BlendNode& n : nodes) {
        std::erase_if(n.children, [node](const BlendChild& c) { return c.node == node; });
        for (BlendChild& c : n.children) {
            c.node = fixIndex(c.node, node);
        }
    }
    for (AnimState& s : stateMachine.states) {
        s.motion = fixIndex(s.motion, node);
    }
    root = fixIndex(root, node);
    if (root < 0 && !nodes.empty()) {
        root = 0;
    }
}

i32 BlendTree::addState(std::string stateName, i32 motion, glm::vec2 position)
{
    stateMachine.states.push_back({std::move(stateName), motion, position});
    const auto index = static_cast<i32>(stateMachine.states.size() - 1);
    if (stateMachine.defaultState < 0) {
        stateMachine.defaultState = index;
    }
    return index;
}

i32 BlendTree::addTransition(i32 from, i32 to, f32 duration)
{
    AnimTransition t;
    t.from = from;
    t.to = to;
    t.duration = duration;
    stateMachine.transitions.push_back(std::move(t));
    return static_cast<i32>(stateMachine.transitions.size() - 1);
}

void BlendTree::removeState(i32 state)
{
    auto& sm = stateMachine;
    if (!sm.validState(state)) {
        return;
    }
    sm.states.erase(sm.states.begin() + state);
    std::erase_if(sm.transitions, [state](const AnimTransition& t) { return t.from == state || t.to == state; });
    for (AnimTransition& t : sm.transitions) {
        t.from = t.from < 0 ? -1 : fixIndex(t.from, state);
        t.to = fixIndex(t.to, state);
    }
    sm.defaultState = fixIndex(sm.defaultState, state);
    if (sm.defaultState < 0 && !sm.states.empty()) {
        sm.defaultState = 0;
    }
}

i32 BlendTree::stateOfNode(i32 node) const
{
    const std::function<bool(i32, i32, int)> contains = [&](i32 at, i32 wanted, int depth) -> bool {
        if (!validNode(at) || depth > kMaxDepth) {
            return false;
        }
        if (at == wanted) {
            return true;
        }
        for (const BlendChild& c : nodes[static_cast<usize>(at)].children) {
            if (contains(c.node, wanted, depth + 1)) {
                return true;
            }
        }
        return false;
    };
    for (usize s = 0; s < stateMachine.states.size(); ++s) {
        if (contains(stateMachine.states[s].motion, node, 0)) {
            return static_cast<i32>(s);
        }
    }
    return -1;
}

i32 BlendTree::entryNode() const
{
    if (!stateMachine.empty()) {
        const i32 s = stateMachine.validState(stateMachine.defaultState) ? stateMachine.defaultState : 0;
        return stateMachine.states[static_cast<usize>(s)].motion;
    }
    return root;
}

bool BlendTree::isValid(usize clipCount) const
{
    std::vector<u8> state(nodes.size(), 0); // 0 new, 1 on stack, 2 done
    const std::function<bool(i32)> visit = [&](i32 n) -> bool {
        if (!validNode(n)) {
            return false;
        }
        u8& s = state[static_cast<usize>(n)];
        if (s == 1) {
            return false;
        }
        if (s == 2) {
            return true;
        }
        s = 1;
        const BlendNode& node = nodes[static_cast<usize>(n)];
        if (node.type == BlendNodeType::Clip) {
            if (node.clip < 0 || static_cast<usize>(node.clip) >= clipCount) {
                return false;
            }
        } else {
            const auto validParam = [&](i32 p) { return p >= 0 && static_cast<usize>(p) < parameters.size(); };
            if (!validParam(node.parameter) || node.children.empty() ||
                (node.type == BlendNodeType::Blend2D && !validParam(node.parameterY))) {
                return false;
            }
            for (const BlendChild& c : node.children) {
                if (!visit(c.node)) {
                    return false;
                }
            }
        }
        s = 2;
        return true;
    };
    if (stateMachine.empty()) {
        return visit(root);
    }
    for (const AnimState& s : stateMachine.states) {
        if (!visit(s.motion)) {
            return false;
        }
    }
    for (const AnimTransition& t : stateMachine.transitions) {
        if (!stateMachine.validState(t.to) || (t.from >= 0 && !stateMachine.validState(t.from))) {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------------------------- //
// Instance

void BlendTreeInstance::prepare(const BlendTree& tree)
{
    if (parameters.size() != tree.parameters.size()) {
        const usize old = parameters.size();
        parameters.resize(tree.parameters.size());
        for (usize i = old; i < parameters.size(); ++i) {
            parameters[i] = tree.parameters[i].defaultValue;
        }
    }
    phase.resize(tree.nodes.size(), 0.0f);
    weight.resize(tree.nodes.size(), 0.0f);
    triggerAge.resize(tree.parameters.size(), 0.0f);
    if (!tree.stateMachine.validState(state)) {
        state = -1;
    }
    if (!tree.stateMachine.validState(previousState)) {
        previousState = -1;
    }
}

void BlendTreeInstance::setParameter(const BlendTree& tree, std::string_view name, f32 value)
{
    prepare(tree);
    if (const i32 p = tree.findParameter(name); p >= 0) {
        parameters[static_cast<usize>(p)] = value;
    }
}

void BlendTreeInstance::setTrigger(const BlendTree& tree, std::string_view name)
{
    setParameter(tree, name, 1.0f);
    if (const i32 p = tree.findParameter(name); p >= 0) {
        triggerAge[static_cast<usize>(p)] = 0.0f;
    }
}

f32 BlendTreeInstance::parameter(const BlendTree& tree, std::string_view name) const
{
    return parameterValue(*this, tree.findParameter(name));
}

namespace {

void resetPhase(const BlendTree& tree, BlendTreeInstance& inst, i32 node, int depth)
{
    if (!tree.validNode(node) || depth > kMaxDepth) {
        return;
    }
    inst.phase[static_cast<usize>(node)] = 0.0f;
    for (const BlendChild& c : tree.nodes[static_cast<usize>(node)].children) {
        resetPhase(tree, inst, c.node, depth + 1);
    }
}

} // namespace

void BlendTreeInstance::forceState(const BlendTree& tree, i32 s)
{
    prepare(tree);
    if (!tree.stateMachine.validState(s)) {
        return;
    }
    state = s;
    previousState = -1;
    stateTime = 0.0f;
    transitionElapsed = transitionDuration = 0.0f;
    resetPhase(tree, *this, tree.stateMachine.states[static_cast<usize>(s)].motion, 0);
}

// ---------------------------------------------------------------------------------------------- //
// Weights

std::vector<f32> gradientBandWeights(std::span<const glm::vec2> samples, const glm::vec2& point)
{
    const usize n = samples.size();
    std::vector<f32> w(n, 0.0f);
    if (n == 0) {
        return w;
    }
    if (n == 1) {
        w[0] = 1.0f;
        return w;
    }
    f32 sum = 0.0f;
    for (usize i = 0; i < n; ++i) {
        f32 weight = 1.0f;
        for (usize j = 0; j < n; ++j) {
            if (i == j) {
                continue;
            }
            const glm::vec2 ij = samples[j] - samples[i];
            const f32 len2 = glm::dot(ij, ij);
            if (len2 < 1e-12f) {
                continue; // coincident samples
            }
            weight = std::min(weight, 1.0f - glm::dot(point - samples[i], ij) / len2);
        }
        w[i] = std::max(weight, 0.0f);
        sum += w[i];
    }
    if (sum <= 1e-8f) {
        // Outside every band: the nearest sample takes it all.
        usize nearest = 0;
        for (usize i = 1; i < n; ++i) {
            if (glm::length(samples[i] - point) < glm::length(samples[nearest] - point)) {
                nearest = i;
            }
        }
        std::ranges::fill(w, 0.0f);
        w[nearest] = 1.0f;
        return w;
    }
    for (f32& x : w) {
        x /= sum;
    }
    return w;
}

Blend1DSegment blend1DSegment(const BlendTree& tree, const BlendTreeInstance& instance, i32 node)
{
    Blend1DSegment s;
    if (!tree.validNode(node)) {
        return s;
    }
    const BlendNode& n = tree.nodes[static_cast<usize>(node)];
    if (n.type != BlendNodeType::Blend1D || n.children.empty()) {
        return s;
    }
    const f32 value = parameterValue(instance, n.parameter);
    const auto& c = n.children;
    if (value <= c.front().threshold) {
        s.a = s.b = c.front().node;
        return s;
    }
    if (value >= c.back().threshold) {
        s.a = s.b = c.back().node;
        const f32 last = c.back().threshold;
        if (n.scalePlayback && last > 1e-4f) {
            s.playbackScale = value / last;
        }
        return s;
    }
    for (usize i = 0; i + 1 < c.size(); ++i) {
        if (value <= c[i + 1].threshold) {
            const f32 span = c[i + 1].threshold - c[i].threshold;
            s.a = c[i].node;
            s.b = c[i + 1].node;
            s.t = span > 1e-6f ? (value - c[i].threshold) / span : 1.0f;
            return s;
        }
    }
    s.a = s.b = c.back().node;
    return s;
}

std::vector<f32> blendChildWeights(const BlendTree& tree, const BlendTreeInstance& instance, i32 node)
{
    if (!tree.validNode(node)) {
        return {};
    }
    const BlendNode& n = tree.nodes[static_cast<usize>(node)];
    std::vector<f32> w(n.children.size(), 0.0f);
    if (n.type == BlendNodeType::Blend1D) {
        const Blend1DSegment s = blend1DSegment(tree, instance, node);
        // First matching child for each end (children could repeat a node; the first one wins).
        bool doneA = false;
        bool doneB = false;
        for (usize i = 0; i < n.children.size(); ++i) {
            if (!doneA && n.children[i].node == s.a) {
                w[i] += s.a == s.b ? 1.0f : 1.0f - s.t;
                doneA = true;
            } else if (!doneB && s.b != s.a && n.children[i].node == s.b) {
                w[i] += s.t;
                doneB = true;
            }
        }
    } else if (n.type == BlendNodeType::Blend2D) {
        std::vector<glm::vec2> samples;
        samples.reserve(n.children.size());
        for (const BlendChild& c : n.children) {
            samples.emplace_back(c.threshold, c.thresholdY);
        }
        w = gradientBandWeights(samples, {parameterValue(instance, n.parameter), parameterValue(instance, n.parameterY)});
    }
    return w;
}

f32 blendNodeDuration(const BlendTree& tree, std::span<const AnimationClip> clips, const BlendTreeInstance& instance,
                      i32 node)
{
    if (!tree.validNode(node)) {
        return 1.0f;
    }
    const BlendNode& n = tree.nodes[static_cast<usize>(node)];
    if (n.type == BlendNodeType::Clip) {
        const AnimationClip* clip = clipOf(n, clips);
        const f32 speed = std::max(std::abs(n.playbackSpeed), 1e-3f);
        return clip ? std::max(clip->duration(), kMinDuration) / speed : 1.0f;
    }
    const std::vector<f32> w = blendChildWeights(tree, instance, node);
    f32 syncedWeight = 0.0f;
    f32 synced = 0.0f;
    f32 all = 0.0f;
    for (usize i = 0; i < n.children.size(); ++i) {
        if (w[i] <= 0.0f) {
            continue;
        }
        const f32 d = blendNodeDuration(tree, clips, instance, n.children[i].node);
        all += w[i] * d;
        if (n.children[i].sync) {
            synced += w[i] * d;
            syncedWeight += w[i];
        }
    }
    // The shared cycle follows the synced children only.
    const f32 duration = syncedWeight > 1e-4f ? synced / syncedWeight : (all > 0.0f ? all : 1.0f);
    const f32 scale = n.type == BlendNodeType::Blend1D ? blend1DSegment(tree, instance, node).playbackScale : 1.0f;
    return std::max(duration / scale, kMinDuration);
}

// ---------------------------------------------------------------------------------------------- //
// Update / evaluate

namespace {

// Recursively advances clocks. `synced`: the parent dictates this node's phase.
void advance(const BlendTree& tree, std::span<const AnimationClip> clips, BlendTreeInstance& inst, i32 node, f32 dt,
             f32 weight, const f32* synced, int depth)
{
    if (!tree.validNode(node) || depth > kMaxDepth) {
        return;
    }
    const auto i = static_cast<usize>(node);
    inst.weight[i] += weight;
    const BlendNode& n = tree.nodes[i];
    f32& phase = inst.phase[i];
    const f32 phaseBefore = phase;
    if (synced) {
        phase = *synced;
    } else if (n.type == BlendNodeType::Clip && !n.loop) {
        phase = std::min(phase + dt / blendNodeDuration(tree, clips, inst, node), 1.0f);
    } else {
        phase = wrap01(phase + dt / blendNodeDuration(tree, clips, inst, node));
    }
    if (!n.isBlend()) {
        // Root motion of this clip over the step, weighted.
        const AnimationClip* clip = clipOf(n, clips);
        if (clip && clip->hasRootMotion() && weight > 0.0f && dt > 0.0f) {
            const f32 d = clip->duration();
            glm::vec3 delta = clip->sampleRootMotion(phase * d) - clip->sampleRootMotion(phaseBefore * d);
            if (phase < phaseBefore) { // looped
                delta = clip->sampleRootMotion(d) - clip->sampleRootMotion(phaseBefore * d) + clip->sampleRootMotion(phase * d);
            }
            inst.rootMotion += delta * weight;
        }
        return;
    }
    const std::vector<f32> w = blendChildWeights(tree, inst, node);
    const f32 scale = n.type == BlendNodeType::Blend1D ? blend1DSegment(tree, inst, node).playbackScale : 1.0f;
    for (usize c = 0; c < n.children.size(); ++c) {
        // Synced: every child follows the shared clock, so inactive ones are aligned when they
        // fade in. Unsynced: each runs on its own clock (inactive ones keep running too).
        const bool follow = n.syncPhase && n.children[c].sync;
        const f32 shared = phase;
        advance(tree, clips, inst, n.children[c].node, follow ? 0.0f : dt * scale, weight * w[c], follow ? &shared : nullptr,
                depth + 1);
    }
}

void evaluateNode(const BlendTree& tree, const Skeleton& skeleton, std::span<const AnimationClip> clips,
                  BlendTreeInstance& inst, i32 node, Pose& out, usize depth)
{
    out = skeleton.restPose();
    if (!tree.validNode(node) || depth > static_cast<usize>(kMaxDepth)) {
        return;
    }
    const auto i = static_cast<usize>(node);
    const BlendNode& n = tree.nodes[i];
    if (n.type == BlendNodeType::Clip) {
        const AnimationClip* clip = clipOf(n, clips);
        if (clip && clip->jointCount() == skeleton.jointCount()) {
            const f32 p = n.playbackSpeed < 0.0f ? 1.0f - inst.phase[i] : inst.phase[i];
            clip->sample(p * clip->duration(), n.loop, out);
        }
        return;
    }
    // Heaviest child first, then fold the others in.
    const std::vector<f32> w = blendChildWeights(tree, inst, node);
    std::vector<usize> order(w.size());
    std::iota(order.begin(), order.end(), usize{0});
    std::ranges::sort(order, [&](usize a, usize b) { return w[a] > w[b]; });
    if (order.empty() || w[order[0]] <= 0.0f) {
        return;
    }
    evaluateNode(tree, skeleton, clips, inst, n.children[order[0]].node, out, depth + 1);
    if (depth >= inst.scratch.size()) {
        return; // deeper than the tree has nodes: only possible with a cycle
    }
    f32 accumulated = w[order[0]];
    for (usize k = 1; k < order.size(); ++k) {
        const f32 wk = w[order[k]];
        if (wk < 1e-3f) {
            break;
        }
        Pose& other = inst.scratch[depth];
        evaluateNode(tree, skeleton, clips, inst, n.children[order[k]].node, other, depth + 1);
        accumulated += wk;
        blendPoses(out, other, wk / accumulated, out);
    }
}

bool conditionHolds(const BlendTree& tree, const BlendTreeInstance& inst, const TransitionCondition& c)
{
    if (c.parameter < 0 || static_cast<usize>(c.parameter) >= tree.parameters.size()) {
        return false;
    }
    const f32 v = inst.parameters[static_cast<usize>(c.parameter)];
    switch (c.op) {
    case ConditionOp::Greater: return v > c.value;
    case ConditionOp::Less: return v < c.value;
    case ConditionOp::IsTrue: return v > 0.5f;
    case ConditionOp::IsFalse: return v <= 0.5f;
    }
    return false;
}

void runStateMachine(const BlendTree& tree, std::span<const AnimationClip> clips, BlendTreeInstance& inst)
{
    const StateMachine& sm = tree.stateMachine;
    if (inst.state < 0) {
        inst.forceState(tree, sm.validState(sm.defaultState) ? sm.defaultState : 0);
    }
    if (inst.inTransition()) {
        return; // no interruptions
    }
    const i32 motion = sm.states[static_cast<usize>(inst.state)].motion;
    const f32 normalized = inst.stateTime / blendNodeDuration(tree, clips, inst, motion);
    // Any-state transitions first, then the current state's.
    for (const bool any : {true, false}) {
        for (usize t = 0; t < sm.transitions.size(); ++t) {
            const AnimTransition& tr = sm.transitions[t];
            if ((any && tr.from != -1) || (!any && tr.from != inst.state) || tr.to == inst.state || !sm.validState(tr.to)) {
                continue;
            }
            if (tr.conditions.empty() && !tr.hasExitTime) {
                continue; // would fire every frame
            }
            if (tr.hasExitTime && normalized < tr.exitTime) {
                continue;
            }
            if (!std::ranges::all_of(tr.conditions, [&](const TransitionCondition& c) { return conditionHolds(tree, inst, c); })) {
                continue;
            }
            for (const TransitionCondition& c : tr.conditions) { // consume triggers
                if (tree.parameters[static_cast<usize>(c.parameter)].type == ParameterType::Trigger) {
                    inst.parameters[static_cast<usize>(c.parameter)] = 0.0f;
                }
            }
            const i32 from = inst.state;
            inst.forceState(tree, tr.to);
            if (tr.duration > 0.0f) {
                inst.previousState = from;
                inst.transitionDuration = tr.duration;
                inst.transitionElapsed = 0.0f;
            }
            inst.lastTransition = static_cast<i32>(t);
            return;
        }
    }
}

} // namespace

void updateBlendTree(const BlendTree& tree, std::span<const AnimationClip> clips, BlendTreeInstance& instance, f32 dt)
{
    instance.prepare(tree);
    std::ranges::fill(instance.weight, 0.0f);
    instance.rootMotion = glm::vec3(0.0f);
    dt = std::max(dt, 0.0f);
    const StateMachine& sm = tree.stateMachine;
    if (sm.empty()) {
        advance(tree, clips, instance, tree.root, dt, 1.0f, nullptr, 0);
        return;
    }
    runStateMachine(tree, clips, instance);
    // Input buffer: triggers nobody used expire after their buffer time.
    for (usize p = 0; p < tree.parameters.size(); ++p) {
        const BlendParameter& param = tree.parameters[p];
        if (param.type != ParameterType::Trigger || instance.parameters[p] <= 0.5f) {
            instance.triggerAge[p] = 0.0f;
            continue;
        }
        instance.triggerAge[p] += dt;
        if (param.bufferTime > 0.0f && instance.triggerAge[p] > param.bufferTime) {
            instance.parameters[p] = 0.0f;
            instance.triggerAge[p] = 0.0f;
        }
    }
    const f32 alpha = instance.inTransition() ? instance.transitionWeight() : 1.0f;
    if (instance.inTransition()) {
        advance(tree, clips, instance, sm.states[static_cast<usize>(instance.previousState)].motion, dt, 1.0f - alpha,
                nullptr, 0);
    }
    advance(tree, clips, instance, sm.states[static_cast<usize>(instance.state)].motion, dt, alpha, nullptr, 0);
    instance.stateTime += dt;
    if (instance.inTransition()) {
        instance.transitionElapsed += dt;
        if (instance.transitionElapsed >= instance.transitionDuration) {
            instance.previousState = -1;
        }
    }
}

void evaluateBlendTree(const BlendTree& tree, const Skeleton& skeleton, std::span<const AnimationClip> clips,
                       BlendTreeInstance& instance, Pose& out)
{
    instance.prepare(tree);
    // One scratch pose per level; sized up front so references stay valid during recursion.
    if (instance.scratch.size() < tree.nodes.size() + 1) {
        instance.scratch.resize(tree.nodes.size() + 1);
    }
    const StateMachine& sm = tree.stateMachine;
    if (sm.empty()) {
        evaluateNode(tree, skeleton, clips, instance, tree.root, out, 0);
        return;
    }
    if (instance.state < 0) {
        instance.forceState(tree, sm.validState(sm.defaultState) ? sm.defaultState : 0);
    }
    evaluateNode(tree, skeleton, clips, instance, sm.states[static_cast<usize>(instance.state)].motion, out, 0);
    if (instance.inTransition()) {
        evaluateNode(tree, skeleton, clips, instance, sm.states[static_cast<usize>(instance.previousState)].motion,
                     instance.transitionPose, 0);
        blendPoses(instance.transitionPose, out, instance.transitionWeight(), out);
    }
}

} // namespace kinetic::anim
