#pragma once

#include "kinetic/animation/BlendTree.h"

#include <span>
#include <string>
#include <vector>

namespace kinetic::anim {

// Melee combos in the style of Dark Souls, built from plain attack clips:
//
//   - one "attack" trigger with an input buffer: a press is remembered for `inputBuffer` seconds,
//     so pressing during an attack queues the next one;
//   - every attack is committed: its state locks movement (the character controller only applies
//     the clip's root motion) and allows a short turn window at its start;
//   - the next attack can only start after the current one has struck (combo window opens
//     `comboDelay` after the strike); a buffered press fires right then;
//   - after `recoveryDelay` past the strike, moving cancels the recovery into locomotion;
//   - without input the attack plays out into the combat idle (the "base pose"), which returns to
//     locomotion when moving or after a few seconds; attacking from it starts the chain again.
//
// The strike moment of each clip is found from its weapon hand's peak speed.

struct ComboSettings {
    f32 inputBuffer = 1.2f;    // s a press waits for the combo window (long enough for a press
                               // anywhere in the swing to carry into the next attack)
    f32 comboDelay = 0.05f;    // earliest combo window, normalised time after the strike; the
                               // window opens at the best-matching pose after that (see below)
    f32 recoveryDelay = 0.3f;  // normalised time after the strike when moving may cancel
    f32 blendIn = 0.15f;       // s crossfade from locomotion / idle into the first attack
    f32 comboBlend = 0.22f;    // s crossfade from one attack into the next
    f32 blendOut = 0.3f;       // s crossfade from an attack into the combat idle
    f32 endTime = 0.97f;       // normalised time an attack without input starts fading into the
                               // combat idle (the combo window stays open until then)
    f32 cancelBlend = 0.45f;   // s crossfade when moving cancels the recovery (no snapping)
    f32 turnWindow = 0.15f;    // s at the start of an attack the character may still turn
    f32 combatIdleTime = 3.0f; // cycles of combat idle before relaxing into locomotion
};

// Attack clips by name (attack / slash / strike / combo), in name order (Attack1, Attack2, ...).
[[nodiscard]] std::vector<i32> findAttackClips(std::span<const AnimationClip> clips);
// The combat idle (a name with "idle" and "combat" / "fight" / "guard" / "battle"), or -1.
[[nodiscard]] i32 findCombatIdle(std::span<const AnimationClip> clips);

// Normalised time in [from, to] of clip `a` whose pose is closest to the first frame of clip `b`
// (joint positions relative to the hips): a smooth moment to crossfade from a into b. Takes the
// earliest time where 75% of the pose difference (worst -> best in the range) is gone, so combos
// stay responsive.
[[nodiscard]] f32 findBestTransitionTime(const Skeleton& skeleton, const AnimationClip& a, const AnimationClip& b, f32 from,
                                         f32 to);

// Normalised time (0..1) at which `joint` (the weapon hand) moves fastest: the strike.
[[nodiscard]] f32 estimateStrikeTime(const Skeleton& skeleton, const AnimationClip& clip, usize joint);
// The weapon hand: "RightHand" (or "hand_r", ...), else the first joint named *hand*; -1 if none.
[[nodiscard]] i32 findWeaponHand(const Skeleton& skeleton);

// Where a weapon sits in `hand` (its joint space): the grip in the palm (between the wrist and the
// finger roots), +Y along the blade (the thumb side, perpendicular to the fingers), +X along the
// fingers. Uses the rest pose; identity if the hand has no finger / thumb joints.
[[nodiscard]] glm::mat4 weaponSocket(const Skeleton& skeleton, i32 hand);
// The same from a pose that holds the weapon (the combat idle, else the first attack): the grip
// goes through the fist - the middle finger curls around it, so the circle through its joints is
// the hole - along the fist (index side = blade, little finger side = pommel), the edge facing
// the knuckles. Falls back to the rest-pose socket when no clip curls the fingers.
[[nodiscard]] glm::mat4 weaponSocket(const Skeleton& skeleton, i32 hand, std::span<const AnimationClip> clips);

struct ComboReport {
    std::vector<i32> attackStates;
    i32 combatIdleState = -1;
    std::vector<std::string> lines;
};

// Adds the combo to `tree` (parameters "attack" (trigger) and "moveInput" (0..1), states
// "Combat Idle" and "Attack 1..N", transitions as above). `locomotionState` is where moving
// returns to (-1 = none: attacks return to the combat idle only).
ComboReport addAttackCombo(BlendTree& tree, const Skeleton& skeleton, std::span<const AnimationClip> clips,
                           std::span<const i32> attackClips, i32 combatIdleClip, i32 locomotionState,
                           const ComboSettings& settings = {});

// Blade edge: a sword cuts through the air edge first. Mixamo-style attacks are animated without
// a weapon, so the hand often turns the blade's flat into the swing. `socket` is the weapon in the
// hand's space (blade along +Y, edges along +-X, flats facing +-Z); `point` is where on the blade
// (socket-space Y) its motion is measured.
//
// bladeFlatness: how far (degrees) the swing is from edge-first, averaged with the blade speed
// squared as weight (the fast parts count): 0 = always the edge, 90 = always the flat.
[[nodiscard]] f32 bladeFlatness(const Skeleton& skeleton, const AnimationClip& clip, i32 hand, const glm::mat4& socket,
                                f32 point = 0.7f);

// Turns the hand about the blade's axis, frame by frame, so an edge leads the motion (either
// edge: a double-edged blade turns at most ~90 degrees), only while the blade moves fast, smoothed
// over ~0.1 s, and rewrites the hand's rotation keys (60 per second). The grip stays in the hand.
struct EdgeAlignment {
    f32 before = 0.0f;   // bladeFlatness, degrees
    f32 after = 0.0f;
    f32 maxTwist = 0.0f; // largest turn added to the hand, degrees
};
EdgeAlignment alignBladeEdge(const Skeleton& skeleton, AnimationClip& clip, i32 hand, const glm::mat4& socket, f32 point = 0.7f);

// The combo's attack states in order (Attack 1, 2, ...): the states entered with the "attack"
// trigger, following the chain of attack -> attack transitions. Empty when there is no combo.
[[nodiscard]] std::vector<i32> attackChain(const BlendTree& tree);
// The clip played by state `state` (its motion is a clip node), or -1.
[[nodiscard]] i32 stateClip(const BlendTree& tree, i32 state);

} // namespace kinetic::anim
