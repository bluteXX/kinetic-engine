// Saving animation graphs: everything edited survives a JSON round trip, clips and bones by name.

#include <kinetic/animation/BlendTreeIO.h>

#include <doctest/doctest.h>

using namespace kinetic;
using namespace kinetic::anim;

namespace {

Skeleton threeBones()
{
    Pose rest(3);
    return Skeleton({"Hips", "Spine", "RightHand"}, {-1, 0, 1}, rest);
}

AnimationClip clip(const char* name)
{
    AnimationClip c(name, 1.0f, {});
    return c;
}

} // namespace

TEST_CASE("BlendTreeIO: a graph with states, transitions and physics bones round-trips")
{
    const Skeleton skeleton = threeBones();
    std::vector<AnimationClip> clips = {clip("Idle"), clip("Walk"), clip("Attack")};
    BlendTree tree;
    tree.name = "Hero";
    const i32 speed = tree.addParameter("speed", 0.0f, 0.0f, 4.0f);
    const i32 attack = tree.addParameter("attack", 0.0f, 0.0f, 1.0f, ParameterType::Trigger);
    tree.parameters[static_cast<usize>(attack)].bufferTime = 0.6f;
    const i32 blend = tree.addBlend1D("Locomotion", speed);
    tree.addChild(blend, tree.addClip("Idle", 0), 0.0f);
    tree.addChild(blend, tree.addClip("Walk", 1), 1.4f);
    const i32 hit = tree.addClip("Attack", 2);
    tree.nodes[static_cast<usize>(hit)].loop = false;
    tree.nodes[static_cast<usize>(hit)].physicsJoints = {2, 1};
    tree.nodes[static_cast<usize>(hit)].physicsStrength = 1.5f;
    tree.root = blend;
    const i32 loco = tree.addState("Locomotion", blend, {100.0f, 50.0f});
    const i32 atk = tree.addState("Attack", hit, {300.0f, 50.0f});
    tree.stateMachine.states[static_cast<usize>(atk)].lockMovement = true;
    tree.stateMachine.states[static_cast<usize>(atk)].turnWindow = 0.15f;
    tree.stateMachine.defaultState = loco;
    const i32 t = tree.addTransition(loco, atk, 0.15f);
    tree.stateMachine.transitions[static_cast<usize>(t)].conditions.push_back({attack, ConditionOp::IsTrue, 0.0f});
    const i32 back = tree.addTransition(atk, loco, 0.3f);
    tree.stateMachine.transitions[static_cast<usize>(back)].hasExitTime = true;
    tree.stateMachine.transitions[static_cast<usize>(back)].exitTime = 0.92f;
    REQUIRE(tree.isValid(clips.size()));

    const Json json = blendTreeToJson(tree, skeleton, clips);
    // Clips are stored by name: reordering the model's clips keeps the graph right.
    std::vector<AnimationClip> reordered = {clip("Attack"), clip("Idle"), clip("Walk")};
    std::string warnings;
    const auto parsed = Json::parse(json.dump());
    REQUIRE(parsed.has_value());
    const auto loaded = blendTreeFromJson(*parsed, skeleton, reordered, &warnings);
    REQUIRE(loaded.has_value());
    CHECK(warnings.empty());
    CHECK(loaded->name == "Hero");
    REQUIRE(loaded->nodes.size() == tree.nodes.size());
    const BlendNode& a = loaded->nodes[static_cast<usize>(hit)];
    CHECK(reordered[static_cast<usize>(a.clip)].name() == "Attack");
    CHECK_FALSE(a.loop);
    CHECK(a.physicsJoints == std::vector<i32>{2, 1});
    CHECK(a.physicsStrength == doctest::Approx(1.5f));
    CHECK(loaded->nodes[static_cast<usize>(blend)].children.size() == 2);
    CHECK(loaded->nodes[static_cast<usize>(blend)].children[1].threshold == doctest::Approx(1.4f));
    CHECK(loaded->parameters[static_cast<usize>(attack)].type == ParameterType::Trigger);
    CHECK(loaded->parameters[static_cast<usize>(attack)].bufferTime == doctest::Approx(0.6f));
    REQUIRE(loaded->stateMachine.states.size() == 2);
    CHECK(loaded->stateMachine.states[1].lockMovement);
    CHECK(loaded->stateMachine.states[1].position == glm::vec2(300.0f, 50.0f));
    CHECK(loaded->stateMachine.defaultState == loco);
    REQUIRE(loaded->stateMachine.transitions.size() == 2);
    CHECK(loaded->stateMachine.transitions[0].conditions.front().parameter == attack);
    CHECK(loaded->stateMachine.transitions[0].conditions.front().op == ConditionOp::IsTrue);
    CHECK(loaded->stateMachine.transitions[1].hasExitTime);
    CHECK(loaded->stateMachine.transitions[1].exitTime == doctest::Approx(0.92f));
    // Same text again.
    CHECK(blendTreeToJson(*loaded, skeleton, reordered).dump() == json.dump());
}

TEST_CASE("BlendTreeIO: missing clips are reported")
{
    const Skeleton skeleton = threeBones();
    std::vector<AnimationClip> clips = {clip("Idle")};
    BlendTree tree;
    tree.root = tree.addClip("Idle", 0);
    const Json json = blendTreeToJson(tree, skeleton, clips);
    std::vector<AnimationClip> other = {clip("Run")};
    std::string warnings;
    CHECK_FALSE(blendTreeFromJson(json, skeleton, other, &warnings).has_value());
    CHECK(warnings.find("Idle") != std::string::npos);
}
