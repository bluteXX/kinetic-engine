# Architecture

## Goals

1. **Animation first.** The engine exists to build and test animation systems; rendering serves them.
2. **Runs the same on macOS and Windows** from one CMake project.
3. **Small, understandable layers** — each module is a static library with explicit dependencies.

## Modules and dependencies

```
core
animation -> core                  pure CPU math, no SDL / GPU
assets    -> animation             glTF / FBX / OBJ import (cgltf, ufbx), PNG / BMP (SDL)
scene     -> animation             EnTT registry + components (AnimatorComponent), collider
                                   raycasts, the pose stage of animation (AnimatorSystem)
physics   -> core                  Jolt Physics behind PhysicsWorld (pimpl, no Jolt headers leak)
gameplay  -> scene                 character controller (kinematic, walks on colliders)
physanim  -> scene, physics        physics animation: bones on muscles per clip, hits
vfx       -> core                  particle effects from assets/vfx/*.vfx (hit sparks, splinters...)
platform  -> core                  SDL3 window / input
gfx       -> platform              SDL_GPU device, shaders, frame lifecycle
renderer  -> gfx, scene, vfx
ui        -> gfx                   Dear ImGui
app       -> renderer, ui          Application + main loop
```

A module may only include headers of modules it links. This is enforced by CMake: a missing
dependency is a compile error, not a convention.

**Why `animation` has no GPU dependency:** it takes a skeleton, clips and time and produces
matrices. That keeps it unit-testable without a window and portable — the same code can later be
wrapped in an Unreal plugin or a command-line baking tool.

## Data flow per frame

```
events ─► fixed update ×N ─► update ─► ImGui ─► render
              │
              └─ animation: sample clips → blend → IK → local pose
                            → localToModel → skinning matrices → CPU skinning ──► vertex upload
```

- **Animated objects** have a `MeshRendererComponent` pointing at their *own* dynamic mesh
  (`MeshUsage::Dynamic`) plus an `AnimatorComponent` (shared `SkinnedModel` + `AnimationPlayer`,
  optionally a blend tree and foot IK). `scene::updateAnimatorPoses` (headless) evaluates the
  pose: clip player with crossfade, or the blend tree; then foot IK against the colliders.
  `renderer::updateAnimations` runs it, then skins
  the vertices on the CPU (`anim::skinVertices`) and stages them; `Renderer::uploadPendingData`
  copies them in the frame's copy pass. CPU skinning needs no new shaders; GPU skinning (joint
  matrices in a storage buffer, skinned in `mesh.vert`) replaces it once heavier characters need
  it. The editor advances animation per frame (after the UI, so scrubbing shows immediately).

- **Fixed timestep** (`AppDesc::fixedTimestep`, 60 Hz): animation and physics advance in
  constant steps, so results don't depend on frame rate. Rendering gets `interpolationAlpha`
  to interpolate between the last two ticks. Frame time is clamped to 0.25 s to avoid the
  "spiral of death" after a breakpoint.
- **Frame lifecycle** (`gfx::Device`): acquire command buffer + swapchain → copy passes
  (ImGui upload, mesh uploads, skinned vertices, overlay lines) → scene pass into the offscreen
  *scene target* (colour + depth) → [sandbox: blit to swapchain] → overlay pass on the swapchain
  (ImGui) → submit. The editor sizes the scene target to its Viewport panel and draws it there as
  an ImGui image (`AppDesc::presentScene = false`).
- **Camera** is an entity with `CameraComponent` + `TransformComponent`; it looks down local −Z.
  `Renderer::drawScene` renders the floor grid and every `MeshRendererComponent` from the scene's
  first camera.
- **Meshes** live in the renderer's library; `MeshRendererComponent::mesh` is an index into it
  (0 = built-in triangle). `assets::loadMesh` turns .fbx / .obj into `MeshData` (Y-up, meters,
  transforms baked, triangulated) via ufbx; `Renderer::addMesh` uploads it.
- **Actors:** every scene object is an entity with Name + Transform. The editor creates empty
  actors ("Add Actor") and adds components in the Inspector: *Mesh Renderer* (any drawer asset,
  tint, visibility) and, for assets with a skeleton, *Animator* (clip, loop, speed, play/pause).
  `renderer::enableAnimation` / `disableAnimation` swap between the asset's shared static
  rest-pose mesh and a per-instance dynamic mesh + `AnimatorComponent`.
- **Editor assets:** the *Assets* drawer is a content browser over the project's `assets/`
  folder (so additions end up in the repo): folder tree + Built-in (triangle, presets), the
  current folder's sub-folders, assets and texture files, breadcrumb, "All" flat view. Every
  model / mesh file below `assets/` is loaded on startup. Dragging tiles or folders onto a folder
  moves them on disk (a model takes the textures next to it along); right-click renames, creates
  folders, deletes empty folders. Dropped files are imported into the current folder. Files with a
  skin or animation become animated (ANIM) assets via `assets::loadSkinnedModel` and also get a
  static rest-pose mesh. Tiles drag an asset index (`kAssetPayload`) onto the Viewport (new actor)
  or an actor's Mesh field.
- **Selection:** clicking the Viewport (press + release with < 4 px movement; dragging still looks
  around) casts `renderer::cameraRay` and `renderer::pickEntity` tests every visible mesh's
  triangles in local space — animated actors in their current pose. The selection gets a bounding
  box outline in the overlay; Del / F / Esc delete, frame and deselect.
- **Inspector + Actor Editor:** the Inspector holds only the name, transform and quick toggles
  (visible, state / clip, foot IK, playable, collider). Everything else happens in the *Actor
  Editor* (double-click an actor): left, a preview of the actor alone (own `RenderTarget`, orbit
  camera, overlay layer 1) over its components (list + the selected one's settings, provided by
  the app through `ActorEditorHost`); right, tabs *Animation Graph* (state machine canvas with
  pan / zoom, parameter list with live values, and the selected state's motion - clip, 1D / 2D
  blend with its plot - or transition's conditions), *Clips & Timeline* and *Skeleton*. It drives
  the actor's own `AnimatorComponent`, so the preview and the Viewport show the same pose.
- **Extra render targets:** `Renderer::resizeTarget` / `beginTargetPass` render any scene into
  any `RenderTarget`; apps record such passes in `Application::onRenderOffscreen`, before the main
  scene pass. Overlay lines are per layer (`setOverlayLines(lines, layer)`).

- **Physics animation (physanim):** `layoutBody` finds a character's rigid bodies from any skeleton
  (a capsule per bone long enough to matter, short bones such as fingers riding on their parent,
  thickness from the skin) and `PhysicalBody` simulates them: ball joints with 3-axis muscles sized
  from the inertia they move (the weapon welded to its hand). Each clip node of the graph lists its
  *physics bones* (`BlendNode::physicsJoints`, set in the Actor Editor: None / Weapon arm / Full
  body / any bones); while that clip leads the pose those bodies are dynamic and their muscles
  follow the animation (~30 ms ahead), the rest stay animated (riding on simulated parents). Weapon
  hits are judged by `HitTracker` (rebound / glance / stop / cut / stuck), hard ones trigger a
  reflex, and each one sends a `HitEvent` ("on hit"). With the pelvis simulated a force-limited
  balance assist holds the body on the animation (it never falls over). In Play a hard hit on a
  wall gives a hit stop, then the rest of the attack becomes a crossfade (from the pose shown, see
  `BlendTreeInstance::crossfadeTo`) into the combo's next attack when one is queued, else into the
  combat idle; the ground does not interrupt an attack. The result goes back as `AnimatorComponent::boneOverrides`
  (model space, blendable with `overrideWeight`). Play mode evaluates the animation before the
  physics (`scene::evaluateAnimatorPoses` / `finishAnimatorPoses`), so the drawn bones and weapons
  are the simulated ones in the same frame. `findStrikePoint` / `strikeTarget` place test targets
  in any attack's path.
- **Blade edge (anim::alignBladeEdge):** attacks animated without a weapon often swing the blade flat;
  this turns the hand about the blade axis (while it moves fast, smoothed, a few passes) so an edge
  leads. `bladeFlatness` measures it. Actor Editor > Clips: "Align blade edge" (saved in the actor file).
- **Effects (vfx):** `VfxLibrary` loads `assets/vfx/*.vfx` (JSON: emitters of sparks, shards, puffs,
  drops with colour, speed, spread, lifetime, gravity), `VfxSystem` runs the bursts and
  `renderer::syncVfx` draws them as tinted dynamic meshes. In Play every hit plays the struck
  collider's `onHit` effect, by default `hit_<material>`.
- **Saving (core Json, BlendTreeIO):** an actor's graph (with physics bones) and weapon sockets are
  saved next to its model as `<model>.actor` (`editor::saveActor`), automatically when they change;
  "Make Playable Hero" loads it.
- **Characters (gameplay):** `gameplay::CharacterControllerComponent` + `updateCharacter` move an
  entity camera-relative with acceleration and turning, snap it to the ground under it
  (`scene::raycastColliders`), step onto ledges up to `stepHeight`, push it out of collider walls
  and let it fall without ground. It writes its horizontal speed to the entity's blend tree
  parameter (`speed`), so a locomotion tree follows the movement. The editor's *Make Playable
  Hero* sets up any humanoid model this way (scale to a height, locomotion tree measured from the
  clips, foot IK, controller); Play mode drives such heroes with WASD / Shift.
- **Materials:** loaders fill `MeshData::material` (base colour / normal / roughness / metallic
  file references); `assets::resolveMaterial` finds files whose paths point at the author's
  machine (searching the model's folder and `textures/`) and completes a texture set from the
  base colour's name (`*_basecolor.png` -> `*_normal.png`, ...). `Renderer::addTexture` uploads
  RGBA8 with a GPU-generated mip chain; `MeshRendererComponent` holds base colour + normal texture
  ids (0 / 1 = built-in white / flat normal). Meshes have two vertex streams: position + normal +
  tangent (rewritten by skinning), and uv (static). `mesh.frag` samples both maps (tangent-space
  normal map, OpenGL convention; DirectX maps with `flipNormalY`).

## Animation data

- **`SkinnedModel`** = bind-pose mesh with ≤ 4 joint influences per vertex + `Skeleton` + clips.
  Loaders guarantee skinning the rest pose reproduces the model as authored:
  - *glTF*: all ancestors of the skin's joints become helper joints (joint world transforms
    include them, e.g. a `Z_UP` or 0.01-scaled `Armature` node).
  - *FBX*: joints are every bone plus its ancestors up to the scene root (which holds the axis /
    unit conversion, `UFBX_SPACE_CONVERSION_TRANSFORM_ROOT`). The rest pose is derived from the
    skin clusters' bind matrices, because node transforms as loaded may already be posed by the
    current take. Animation stacks are baked with `ufbx_bake_anim` to linear keys. Meshes without
    a skin are bound rigidly to their own node.
  - Helper joints are flagged (`SkinnedModel::isBone`) and hidden from skeleton overlays.

- **`Pose` is Structure-of-Arrays** — separate arrays of translations, rotations and scales.
  Blending, sampling and IK iterate those arrays linearly (cache-friendly, easy to SIMD later).
- **`Skeleton` invariant:** `parents[i] < i`. The glTF importer sorts joints by depth to
  guarantee it, so model-space transforms are one forward pass with no recursion.
- **Rotations blend with nlerp on the shortest arc** (`dot < 0 → negate`). It is cheaper than
  slerp and commutative for multi-way blends; clip sampling between keys uses slerp.
- **Additive layers** store `additive - reference` and apply it on top of a base pose.
- **Animation graph** (`anim::BlendTree`, asset) + `BlendTreeInstance` (per object: parameter
  values, per-node phase and weight, state machine state). Parameters are Float / Bool / Trigger.
  Blend nodes: *Clip* (looping or one-shot), *Blend1D* (two neighbours around a value) and
  *Blend2D* (gradient band interpolation over two parameters, Unity's freeform cartesian). Synced
  children share one normalised clock whose cycle length is blended, so walk and run stay in
  step; children with sync off (idle, turns) keep their own clock. On top, an optional *state
  machine* (Unity Animator): states play a node, transitions (from a state or Any State) fire when
  all conditions hold and the exit time is reached, crossfade over their duration and consume
  triggers.
- **Clip processing** (`ClipTools.h`, the importer's "animation settings"): centre the root of
  off-centre in-place clips, Loop Pose (close the loop), sub-clips, root yaw extraction (the pose
  stops turning, `rootYawRate` tells the controller how fast to turn), mirroring by left / right
  joint names. `prepareLocomotionClips` applies them to idle / walk / run and cuts an in-place,
  mirrored turn pair out of a turn clip; the editor runs it on import.
- **Retargeting** (`Retarget.h`): `assets::loadAnimationFile` reads skeleton + clips from files
  without a mesh (Mixamo "without skin"); `retargetClip` maps bones by name (namespace prefixes
  ignored) and applies each bone's model-space change from the source rest pose to the target's
  rest pose (different bone axes are fine), scaling the hips' travel by the hip-height ratio.
  The editor imports every file in an `animations/` folder next to a model this way.
- **Root motion:** `extractRootMotion` moves a clip's hip travel into `AnimationClip::rootMotion`;
  the graph accumulates the weighted root motion of each update (`BlendTreeInstance::rootMotion`)
  and the character controller moves the character by it.
- **Combat** (`Combat.h`, Dark Souls style): `addAttackCombo` chains attack clips with an
  "attack" trigger whose `bufferTime` is the input buffer; each attack state locks movement
  (`AnimState::lockMovement`, short `turnWindow`), the next attack may start only after the
  current one's strike (found from the weapon hand's peak speed), moving cancels after the
  recovery point, and without input it plays out into the combat idle.
- **Locomotion analysis:** `anim::findLegChains` finds legs by joint names (Mixamo, Unreal,
  Blender rigs); `estimateGroundSpeed` measures how fast the planted foot moves relative to the
  pelvis - the speed an in-place clip was authored for. `makeLocomotionTree` builds the locomotion
  graph from that: a 2D blend (turn rate x speed: idle, walk, run, turn left / right) or 1D on
  speed, plus Locomotion / Jump states - thresholds match the movement, so feet do not slide.
- **IK:** `solveTwoBone` (analytic, keeps the animated knee direction) and `applyFootIK`: probe the
  ground under each ankle, lower the pelvis for the lowest foot, solve each leg so the ankle sits
  at ground + the clip's own lift, tilt planted feet to the ground normal; offsets are smoothed.

## Physics

- **`physics::PhysicsWorld`** wraps Jolt (single-threaded job system, 24 velocity / 6 position
  solver iterations, 2 mm penetration slop). It is built for *measuring* force transmission:
  - **Muscles** are hinge position motors with a spring (stiffness, damping) and a torque limit
    (strength). Jolt applies torque limits in position mode too.
  - Every joint reports the force it applies to its child (`GetTotalLambdaPosition / dt`) and the
    muscle / limit torques (`GetTotalLambdaMotor / dt`): one collision step per `step()` keeps the
    lambdas per step.
  - **Contact force on a body** = momentum change - gravity - joint forces. Exact because damping
    is 0 and the solver applies exactly those impulses.
  - Verified by tests: a muscle holding a horizontal rod carries m g L/2 (= J^T F), saturates at
    its strength, a resting box's contact force equals its weight.
- **Rigid impacts are instantaneous** (one step): per-step forces scale with the step rate, the
  impulses do not (tested: 240 Hz vs 960 Hz within 2 %). Analyses report impulses, angular
  impulses and absorbed energy; peak forces are estimated from an assumed contact duration.
  Jolt's CCD is linear only, so fast *rotating* blades need small steps (960 Hz) to stay out of
  the target.
- **Sword strike prototype** (`editor/src/SwordStrikeTest`, Physics menu): planar arm (torso,
  upper arm, forearm, hand, welded sword) with anthropometric masses and biomechanical strengths,
  driven by an `AnimationClip`; the stone is placed on the blade path from the clip's kinematics.
  Debug draw: shapes, centres of mass, joint axes, joint force vectors, muscle torque arcs (sweep
  and colour = load), contact force and points, grip force and torque, velocities, weights, the
  animation's target pose, blade trail, viewport labels. The Physics window has the live table
  (incl. J^T F per joint), the hit list with per-joint distribution, graphs, muscle tuning,
  pause / step / slow motion / freeze on hit.

- **Clip kinds** (`anim::ClipKind`): *Kinematic* clips are shown as sampled; *Physics* clips are
  targets for the muscles of a simulated body. The editor marks them [KIN] (blue) / [PHYS] (orange)
  in the Inspector, the Animation Editor and the Assets tooltips.
- **Play mode** (`editor/src/PlayMode`, F5): a physics world is built from the scene's
  `ColliderComponent`s (static boxes) and `CharacterComponent`s. A character (`CharacterRig`:
  16 bodies, hinges + swing-twist joints, masses / strengths from biomechanics) is kinematic while
  walking (bodies follow Idle / Walk via `moveKinematic`); an attack makes the sword arm dynamic and
  its muscles follow the Strike clip with the hit reaction; R makes every body dynamic (limp
  ragdoll). The simulated bodies are written back as the AnimatorComponent's local pose
  (`externalPose`) and skinned like any other model. Stop restores every transform.

## Graphics

- **SDL_GPU** abstracts Metal / Vulkan / D3D12. The device asks for SPIR-V and MSL, so the
  backend is Metal on macOS and Vulkan on Windows. Adding DXIL (via SDL_shadercross) would
  enable D3D12.
- **Shaders** are GLSL → SPIR-V (glslang) → MSL (SPIRV-Cross), committed pre-compiled so the
  build has no shader-tool dependency. The MSL entry point is `main0`. `tools/compile_shaders.py`
  finds glslang on PATH (Vulkan SDK) or in `.tools/glslang/` (a glslang release, git-ignored).
- **GPU debugging:** RenderDoc / PIX on Windows, Xcode's Metal frame capture on macOS.

## Conventions

- Namespaces: `kinetic::<module>` (`kinetic::anim` for animation).
- Headers: `engine/<module>/include/kinetic/<module>/X.h`, sources in `src/`.
- Units: meters, seconds, radians. Right-handed; glTF content is used as authored.
- Errors: functions that can fail return `bool` / `std::optional` and log the reason.
