#include "kinetic/animation/BlendTreeIO.h"

namespace kinetic::anim {

namespace {

const char* nodeTypeName(BlendNodeType t)
{
    switch (t) {
    case BlendNodeType::Clip: return "clip";
    case BlendNodeType::Blend1D: return "blend1d";
    case BlendNodeType::Blend2D: return "blend2d";
    }
    return "clip";
}

BlendNodeType nodeTypeFrom(const std::string& s)
{
    return s == "blend1d" ? BlendNodeType::Blend1D : s == "blend2d" ? BlendNodeType::Blend2D : BlendNodeType::Clip;
}

const char* parameterTypeName(ParameterType t)
{
    return t == ParameterType::Bool ? "bool" : t == ParameterType::Trigger ? "trigger" : "float";
}

ParameterType parameterTypeFrom(const std::string& s)
{
    return s == "bool" ? ParameterType::Bool : s == "trigger" ? ParameterType::Trigger : ParameterType::Float;
}

const char* opName(ConditionOp op)
{
    switch (op) {
    case ConditionOp::Greater: return ">";
    case ConditionOp::Less: return "<";
    case ConditionOp::IsTrue: return "true";
    case ConditionOp::IsFalse: return "false";
    }
    return ">";
}

ConditionOp opFrom(const std::string& s)
{
    return s == "<" ? ConditionOp::Less : s == "true" ? ConditionOp::IsTrue : s == "false" ? ConditionOp::IsFalse : ConditionOp::Greater;
}

} // namespace

Json blendTreeToJson(const BlendTree& tree, const Skeleton& skeleton, std::span<const AnimationClip> clips)
{
    const auto parameterName = [&](i32 p) -> Json {
        return p >= 0 && static_cast<usize>(p) < tree.parameters.size() ? Json(tree.parameters[static_cast<usize>(p)].name) : Json();
    };
    Json out = Json::object();
    out.set("version", 1);
    out.set("name", tree.name);
    out.set("root", tree.root);

    Json parameters = Json::array();
    for (const BlendParameter& p : tree.parameters) {
        Json j = Json::object();
        j.set("name", p.name);
        j.set("type", parameterTypeName(p.type));
        j.set("default", p.defaultValue);
        j.set("min", p.min);
        j.set("max", p.max);
        if (p.bufferTime > 0.0f) j.set("buffer", p.bufferTime);
        parameters.push(std::move(j));
    }
    out.set("parameters", std::move(parameters));

    Json nodes = Json::array();
    for (const BlendNode& n : tree.nodes) {
        Json j = Json::object();
        j.set("type", nodeTypeName(n.type));
        j.set("name", n.name);
        if (n.type == BlendNodeType::Clip) {
            j.set("clip", n.clip >= 0 && static_cast<usize>(n.clip) < clips.size() ? Json(clips[static_cast<usize>(n.clip)].name()) : Json());
            j.set("speed", n.playbackSpeed);
            j.set("loop", n.loop);
            if (!n.physicsJoints.empty()) {
                Json bones = Json::array();
                for (const i32 joint : n.physicsJoints) {
                    if (joint >= 0 && static_cast<usize>(joint) < skeleton.jointCount()) bones.push(skeleton.names()[static_cast<usize>(joint)]);
                }
                j.set("physicsBones", std::move(bones));
                j.set("physicsStrength", n.physicsStrength);
            }
        } else {
            j.set("parameter", parameterName(n.parameter));
            if (n.type == BlendNodeType::Blend2D) j.set("parameterY", parameterName(n.parameterY));
            j.set("syncPhase", n.syncPhase);
            j.set("scalePlayback", n.scalePlayback);
            Json children = Json::array();
            for (const BlendChild& c : n.children) {
                Json cj = Json::object();
                cj.set("node", c.node);
                cj.set("threshold", c.threshold);
                if (n.type == BlendNodeType::Blend2D) cj.set("thresholdY", c.thresholdY);
                cj.set("sync", c.sync);
                children.push(std::move(cj));
            }
            j.set("children", std::move(children));
        }
        nodes.push(std::move(j));
    }
    out.set("nodes", std::move(nodes));

    const StateMachine& sm = tree.stateMachine;
    Json machine = Json::object();
    machine.set("default", sm.defaultState);
    machine.set("entryPosition", sm.entryPosition);
    machine.set("anyStatePosition", sm.anyStatePosition);
    Json states = Json::array();
    for (const AnimState& s : sm.states) {
        Json j = Json::object();
        j.set("name", s.name);
        j.set("motion", s.motion);
        j.set("position", s.position);
        j.set("lockMovement", s.lockMovement);
        j.set("turnWindow", s.turnWindow);
        states.push(std::move(j));
    }
    machine.set("states", std::move(states));
    Json transitions = Json::array();
    for (const AnimTransition& t : sm.transitions) {
        Json j = Json::object();
        j.set("from", t.from);
        j.set("to", t.to);
        j.set("duration", t.duration);
        j.set("hasExitTime", t.hasExitTime);
        j.set("exitTime", t.exitTime);
        Json conditions = Json::array();
        for (const TransitionCondition& c : t.conditions) {
            Json cj = Json::object();
            cj.set("parameter", parameterName(c.parameter));
            cj.set("op", opName(c.op));
            cj.set("value", c.value);
            conditions.push(std::move(cj));
        }
        j.set("conditions", std::move(conditions));
        transitions.push(std::move(j));
    }
    machine.set("transitions", std::move(transitions));
    out.set("stateMachine", std::move(machine));
    return out;
}

std::optional<BlendTree> blendTreeFromJson(const Json& json, const Skeleton& skeleton, std::span<const AnimationClip> clips,
                                           std::string* warnings)
{
    if (!json.isObject() || !json["nodes"].isArray()) {
        if (warnings) *warnings += "not an animation graph; ";
        return std::nullopt;
    }
    const auto warn = [&](const std::string& w) {
        if (warnings) *warnings += w + "; ";
    };
    BlendTree tree;
    tree.name = json["name"].asString().empty() ? tree.name : json["name"].asString();

    for (const Json& j : json["parameters"].items()) {
        BlendParameter p;
        p.name = j["name"].asString();
        p.type = parameterTypeFrom(j["type"].asString());
        p.defaultValue = j["default"].asFloat();
        p.min = j["min"].asFloat(0.0f);
        p.max = j["max"].asFloat(1.0f);
        p.bufferTime = j["buffer"].asFloat(0.0f);
        tree.parameters.push_back(std::move(p));
    }
    const auto parameterIndex = [&](const Json& name) -> i32 {
        if (!name.isString()) return -1;
        const i32 p = tree.findParameter(name.asString());
        if (p < 0) warn("unknown parameter " + name.asString());
        return p;
    };
    const auto clipIndex = [&](const std::string& name) -> i32 {
        for (usize i = 0; i < clips.size(); ++i) {
            if (clips[i].name() == name) return static_cast<i32>(i);
        }
        return -1;
    };

    for (const Json& j : json["nodes"].items()) {
        BlendNode n;
        n.type = nodeTypeFrom(j["type"].asString());
        n.name = j["name"].asString();
        if (n.type == BlendNodeType::Clip) {
            const std::string clipName = j["clip"].asString();
            n.clip = clipIndex(clipName);
            if (n.clip < 0 && !clipName.empty()) warn("missing clip " + clipName);
            n.playbackSpeed = j["speed"].asFloat(1.0f);
            n.loop = j["loop"].asBool(true);
            for (const Json& bone : j["physicsBones"].items()) {
                if (const auto joint = skeleton.findJoint(bone.asString())) {
                    n.physicsJoints.push_back(static_cast<i32>(*joint));
                } else {
                    warn("missing bone " + bone.asString());
                }
            }
            n.physicsStrength = j["physicsStrength"].asFloat(1.0f);
        } else {
            n.parameter = parameterIndex(j["parameter"]);
            n.parameterY = parameterIndex(j["parameterY"]);
            n.syncPhase = j["syncPhase"].asBool(true);
            n.scalePlayback = j["scalePlayback"].asBool(true);
            for (const Json& c : j["children"].items()) {
                BlendChild child;
                child.node = c["node"].asInt(-1);
                child.threshold = c["threshold"].asFloat();
                child.thresholdY = c["thresholdY"].asFloat();
                child.sync = c["sync"].asBool(true);
                n.children.push_back(child);
            }
        }
        tree.nodes.push_back(std::move(n));
    }
    tree.root = json["root"].asInt(-1);

    const Json& machine = json["stateMachine"];
    StateMachine& sm = tree.stateMachine;
    sm.entryPosition = machine["entryPosition"].asVec2(sm.entryPosition);
    sm.anyStatePosition = machine["anyStatePosition"].asVec2(sm.anyStatePosition);
    for (const Json& j : machine["states"].items()) {
        AnimState s;
        s.name = j["name"].asString();
        s.motion = j["motion"].asInt(-1);
        s.position = j["position"].asVec2();
        s.lockMovement = j["lockMovement"].asBool(false);
        s.turnWindow = j["turnWindow"].asFloat(0.0f);
        sm.states.push_back(std::move(s));
    }
    sm.defaultState = machine["default"].asInt(-1);
    for (const Json& j : machine["transitions"].items()) {
        AnimTransition t;
        t.from = j["from"].asInt(-1);
        t.to = j["to"].asInt(-1);
        t.duration = j["duration"].asFloat(0.2f);
        t.hasExitTime = j["hasExitTime"].asBool(false);
        t.exitTime = j["exitTime"].asFloat(0.9f);
        for (const Json& c : j["conditions"].items()) {
            TransitionCondition cond;
            cond.parameter = parameterIndex(c["parameter"]);
            cond.op = opFrom(c["op"].asString());
            cond.value = c["value"].asFloat();
            t.conditions.push_back(cond);
        }
        sm.transitions.push_back(std::move(t));
    }

    if (!tree.isValid(clips.size())) {
        warn("the saved graph does not fit this model");
        return std::nullopt;
    }
    return tree;
}

} // namespace kinetic::anim
