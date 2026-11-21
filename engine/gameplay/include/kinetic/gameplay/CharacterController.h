#pragma once

#include <kinetic/scene/Scene.h>

#include <string>

namespace kinetic::gameplay {

// A kinematic character that walks on the scene's colliders: camera-relative movement with
// acceleration, turning towards the movement, ground snapping, steps up to `stepHeight`, walls
// (collider boxes) block it, gravity when there is no ground. It never simulates bodies; physics
// characters are a separate thing (the editor's Play mode ragdolls).
//
// If the entity has an AnimatorComponent with an animation graph, the controller writes every
// update (when the graph has these parameters):
//   speed     horizontal speed, m/s            turn       yaw rate, rad/s (+ = turning left)
//   grounded  bool                             jump       trigger, set when a jump starts
//   moveInput stick / WASD amount, 0..1        attack     trigger, set when attack is pressed
// and follows the graph back: states with lockMovement stop walking (the clip's root motion moves
// the character instead; turning only during the state's turnWindow).
struct CharacterControllerComponent {
    f32 walkSpeed = 1.5f;     // m/s
    f32 runSpeed = 4.0f;      // m/s
    f32 acceleration = 10.0f; // m/s^2 towards the wanted velocity
    f32 turnSpeed = 10.0f;    // rad/s
    f32 radius = 0.3f;        // m (collision against walls)
    f32 height = 1.8f;        // m (collision against walls)
    f32 stepHeight = 0.4f;    // m: ledges up to this are stepped onto
    f32 gravity = 20.0f;      // m/s^2
    f32 jumpSpeed = 4.5f;     // m/s upwards when jumping (0 = no jumping)
    bool faceBeforeMoving = true; // move where the character faces (turn in place first) instead of strafing
    std::string speedParameter = "speed";
    std::string turnParameter = "turn";

    // Runtime state.
    glm::vec3 velocity{0.0f}; // horizontal (xz) + vertical (y)
    f32 yaw = 0.0f;           // radians; facing = (sin yaw, 0, cos yaw)
    bool grounded = true;
    f32 turnRate = 0.0f;      // rad/s, smoothed
    bool initialized = false; // yaw taken from the transform on the first update
};

// Input for one character for one update.
struct CharacterInput {
    glm::vec2 move{0.0f}; // x = right, y = forward relative to `cameraYaw`; length <= 1
    bool run = false;
    bool jump = false;    // pressed this update
    bool attack = false;  // pressed this update (sets the graph's "attack" trigger)
    f32 cameraYaw = 0.0f; // camera forward = (sin yaw, 0, cos yaw)
};

// Moves one character (TransformComponent position + yaw rotation) and feeds its blend tree.
void updateCharacter(scene::Scene& scene, entt::entity entity, const CharacterInput& input, f32 dt);

// The horizontal speed of a character (m/s).
[[nodiscard]] f32 horizontalSpeed(const CharacterControllerComponent& c);

} // namespace kinetic::gameplay
