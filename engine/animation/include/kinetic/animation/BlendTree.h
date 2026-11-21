#pragma once

#include "kinetic/animation/AnimationClip.h"
#include "kinetic/animation/Skeleton.h"

#include <algorithm>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace kinetic::anim {

// Animation graph: parameters, a pool of blend nodes and (optionally) a state machine on top.
//
// Blend nodes, evaluated every frame into one pose:
//   Clip     plays one clip of the model at `playbackSpeed` (looping, or holding the last frame)
//   Blend1D  picks its two children around a parameter value and blends them, e.g.
//            "speed": Idle @ 0, Walk @ 1.4, Run @ 3.8 (thresholds in the parameter's units)
//   Blend2D  children placed on a plane of two parameters (e.g. turn rate x speed), weighted by
//            gradient band interpolation (Unity's "freeform cartesian")
//
// Children of a blend with `syncPhase` run on one shared, normalised clock (0..1 per cycle),
// so walk and run cycles stay aligned (left foot down at the same moment) while they blend; the
// cycle length is blended too. Children with sync off (an idle, a turn) keep their own clock.
// With `scalePlayback`, values past the last 1D threshold play faster in proportion.
//
// State machine (Unity's Animator): states, each playing one node; transitions between them (or
// from "any state") fire when all their conditions hold (and the exit time is reached, if set),
// crossfading over `duration`. Without states, the graph plays `root`.
//
// The graph is an asset (shared by instances); per-instance state lives in BlendTreeInstance.
enum class BlendNodeType : u8 { Clip, Blend1D, Blend2D };

struct BlendChild {
    i32 node = -1;
    f32 threshold = 0.0f;  // 1D position, or X on a 2D plane
    f32 thresholdY = 0.0f; // Y on a 2D plane
    bool sync = true;      // follows the parent's clock when the parent syncs (off e.g. for an idle)
};

struct BlendNode {
    BlendNodeType type = BlendNodeType::Clip;
    std::string name;
    // Clip
    i32 clip = -1;
    f32 playbackSpeed = 1.0f;
    bool loop = true; // false: holds the last frame (one-shots like a jump)
    // Physics animation (kinetic::physanim): while this clip leads the pose, the bones listed here
    // are simulated - their bodies follow the clip with muscles and react to what they hit; the
    // other bones play normally. E.g. an attack: the weapon arm; a leap attack: the whole body (it
    // can then be knocked over). Joint indices of the skeleton; empty = plain animation.
    std::vector<i32> physicsJoints;
    f32 physicsStrength = 1.0f; // multiplies the muscles' strength
    // Blends
    i32 parameter = -1;  // 1D value / 2D X
    i32 parameterY = -1; // 2D Y
    std::vector<BlendChild> children; // 1D: sorted by threshold (BlendTree::sortChildren)
    bool syncPhase = true;
    bool scalePlayback = true;

    [[nodiscard]] bool isBlend() const { return type != BlendNodeType::Clip; }
};

enum class ParameterType : u8 { Float, Bool, Trigger };

struct BlendParameter {
    std::string name;
    f32 defaultValue = 0.0f;
    f32 min = 0.0f; // UI range (floats)
    f32 max = 1.0f;
    ParameterType type = ParameterType::Float;
    // Triggers: how long a set trigger waits to be used, seconds (an input buffer: press attack
    // during an attack and it fires when the combo window opens). 0 = until a transition uses it.
    f32 bufferTime = 0.0f;
};

enum class ConditionOp : u8 { Greater, Less, IsTrue, IsFalse };

struct TransitionCondition {
    i32 parameter = -1;
    ConditionOp op = ConditionOp::Greater;
    f32 value = 0.0f;
};

struct AnimState {
    std::string name;
    i32 motion = -1;               // blend node played in this state
    glm::vec2 position{0.0f};      // editor graph position
    // Gameplay hints read by the character controller:
    bool lockMovement = false;     // committed action (attack): no walking, root motion only
    f32 turnWindow = 0.0f;         // seconds at the start of the state the character may still turn
};

struct AnimTransition {
    i32 from = -1; // -1 = any state
    i32 to = -1;
    std::vector<TransitionCondition> conditions; // all must hold
    f32 duration = 0.2f;    // crossfade, seconds
    bool hasExitTime = false;
    f32 exitTime = 0.9f;    // normalised time of the source state's motion
};

struct StateMachine {
    std::vector<AnimState> states;
    std::vector<AnimTransition> transitions;
    i32 defaultState = -1;
    glm::vec2 entryPosition{40.0f, 120.0f};
    glm::vec2 anyStatePosition{40.0f, 220.0f};

    [[nodiscard]] bool empty() const { return states.empty(); }
    [[nodiscard]] bool validState(i32 s) const { return s >= 0 && static_cast<usize>(s) < states.size(); }
};

class BlendTree {
public:
    std::string name = "Animation Graph";
    std::vector<BlendNode> nodes;
    std::vector<BlendParameter> parameters;
    i32 root = -1;            // played when there is no state machine
    StateMachine stateMachine;

    i32 addParameter(std::string parameterName, f32 defaultValue = 0.0f, f32 min = 0.0f, f32 max = 1.0f,
                     ParameterType type = ParameterType::Float);
    [[nodiscard]] i32 findParameter(std::string_view parameterName) const;

    i32 addClip(std::string nodeName, i32 clip, f32 playbackSpeed = 1.0f);
    i32 addBlend1D(std::string nodeName, i32 parameter);
    i32 addBlend2D(std::string nodeName, i32 parameterX, i32 parameterY);
    // Adds `child` under the blend `blend` (1D: kept sorted).
    void addChild(i32 blend, i32 child, f32 threshold, f32 thresholdY = 0.0f);
    void sortChildren(i32 blend);
    // Removes a node and every reference to it (children, root, state motions); indices above it
    // shift down by one.
    void removeNode(i32 node);

    i32 addState(std::string stateName, i32 motion, glm::vec2 position = glm::vec2(0.0f));
    i32 addTransition(i32 from, i32 to, f32 duration = 0.2f);
    void removeState(i32 state);
    // The state playing `node` or a blend containing it, or -1.
    [[nodiscard]] i32 stateOfNode(i32 node) const;

    [[nodiscard]] bool validNode(i32 node) const { return node >= 0 && static_cast<usize>(node) < nodes.size(); }
    // The node played when nothing else decides: the default state's motion, or `root`.
    [[nodiscard]] i32 entryNode() const;
    // Every index in range, no cycles, and something to play.
    [[nodiscard]] bool isValid(usize clipCount) const;
};

// Playback state of one graph on one object.
struct BlendTreeInstance {
    std::vector<f32> parameters; // values, by BlendTree::parameters index (bools / triggers: 0 / 1)
    std::vector<f32> phase;      // per node, 0..1 through its cycle
    std::vector<f32> weight;     // per node, its share of the final pose (last update; for UI)
    std::vector<Pose> scratch;   // evaluation memory, one pose per tree level
    std::vector<f32> triggerAge; // seconds each set trigger has been waiting
    glm::vec3 rootMotion{0.0f};  // root motion of the last update (model space), weighted
    Pose transitionPose;         // the previous state's pose during a crossfade

    // State machine.
    i32 state = -1;
    i32 previousState = -1;      // valid while crossfading
    f32 stateTime = 0.0f;        // seconds in `state`
    f32 transitionElapsed = 0.0f;
    f32 transitionDuration = 0.0f;
    i32 lastTransition = -1;     // index of the transition that fired last (UI)
    // crossfadeTo(..., freeze): the crossfade starts from `snapshot`, the pose shown at that
    // moment (it holds still instead of playing on, and can be re-aimed mid-blend without a pop).
    bool fromSnapshot = false;
    Pose snapshot;
    Pose lastOutput;             // the last evaluated pose

    // Sizes everything for `tree` and sets parameters to their defaults (if the count changed).
    void prepare(const BlendTree& tree);
    void setParameter(const BlendTree& tree, std::string_view name, f32 value);
    // Triggers stay set until a transition consumes them (or their buffer time runs out).
    void setTrigger(const BlendTree& tree, std::string_view name);
    // The state being played (nullptr without a state machine).
    [[nodiscard]] const AnimState* currentState(const BlendTree& tree) const
    {
        return tree.stateMachine.validState(state) ? &tree.stateMachine.states[static_cast<usize>(state)] : nullptr;
    }
    [[nodiscard]] f32 parameter(const BlendTree& tree, std::string_view name) const;
    [[nodiscard]] bool inTransition() const { return previousState >= 0; }
    [[nodiscard]] f32 transitionWeight() const
    {
        return transitionDuration > 0.0f ? std::min(transitionElapsed / transitionDuration, 1.0f) : 1.0f;
    }
    // Jumps to a state immediately (editor preview).
    void forceState(const BlendTree& tree, i32 state);
    // Crossfades into `state` over `duration` s from gameplay, outside the graph's transitions
    // (e.g. an attack interrupted by its blade hitting a wall). With `freeze` the state faded
    // out of holds the pose last shown (the swing stops where it was stopped) instead of playing
    // on; calling it again mid-blend starts the new crossfade from the pose shown then.
    void crossfadeTo(const BlendTree& tree, i32 state, f32 duration, bool freeze = false);
};

// Advances clocks and the state machine by dt and refreshes node weights.
void updateBlendTree(const BlendTree& tree, std::span<const AnimationClip> clips, BlendTreeInstance& instance, f32 dt);

// Local pose of the graph at the instance's current state (rest pose + blended clips).
void evaluateBlendTree(const BlendTree& tree, const Skeleton& skeleton, std::span<const AnimationClip> clips,
                       BlendTreeInstance& instance, Pose& out);

// Cycle length of a node in seconds at the instance's parameter values (blended for blends).
[[nodiscard]] f32 blendNodeDuration(const BlendTree& tree, std::span<const AnimationClip> clips,
                                    const BlendTreeInstance& instance, i32 node);

// For a Blend1D at the instance's parameter value: the two children to mix and the weight of the
// second (0 = only the first).
struct Blend1DSegment {
    i32 a = -1;
    i32 b = -1;
    f32 t = 0.0f;
    f32 playbackScale = 1.0f; // > 1 past the last threshold with scalePlayback
};
[[nodiscard]] Blend1DSegment blend1DSegment(const BlendTree& tree, const BlendTreeInstance& instance, i32 node);

// Weight of every child of a blend node (same order as BlendNode::children; sums to 1).
[[nodiscard]] std::vector<f32> blendChildWeights(const BlendTree& tree, const BlendTreeInstance& instance, i32 node);

// Gradient band interpolation weights of `point` among `samples` (sum to 1).
[[nodiscard]] std::vector<f32> gradientBandWeights(std::span<const glm::vec2> samples, const glm::vec2& point);

} // namespace kinetic::anim
