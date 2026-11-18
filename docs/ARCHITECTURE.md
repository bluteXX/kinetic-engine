# Architecture

## Goals

1. **Animation first.** The engine exists to build and test animation systems; rendering serves them.
2. **Runs the same on macOS and Windows** from one CMake project.
3. **Small, understandable layers** — each module is a static library with explicit dependencies.

## Modules and dependencies

```
core
animation -> core                  pure CPU math, no SDL / GPU
assets    -> animation             glTF / FBX / OBJ import (cgltf, ufbx)
scene     -> animation             EnTT registry + components (AnimatorComponent)
physics   -> core                  Jolt Physics behind PhysicsWorld (pimpl, no Jolt headers leak)
platform  -> core                  SDL3 window / input
gfx       -> platform              SDL_GPU device, shaders, frame lifecycle
renderer  -> gfx, scene
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
  (`MeshUsage::Dynamic`) plus an `AnimatorComponent` (shared `SkinnedModel` + `AnimationPlayer`).
  `renderer::updateAnimations` advances each player, evaluates the pose (with crossfade), skins
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
- **Editor assets:** the *Assets* drawer loads every file in `assets/meshes/` and
  `assets/models/` on startup (the project folders, so additions end up in the repo). Files with a
  skin or animation become animated (ANIM) assets via `assets::loadSkinnedModel` and also get a
  static rest-pose mesh. Tiles drag an asset index (`kAssetPayload`) onto the Viewport (new actor)
  or an actor's Mesh field.
- **Selection:** clicking the Viewport (press + release with < 4 px movement; dragging still looks
  around) casts `renderer::cameraRay` and `renderer::pickEntity` tests every visible mesh's
  triangles in local space — animated actors in their current pose. The selection gets a bounding
  box outline in the overlay; Del / F / Esc delete, frame and deselect.
- **Animation Editor:** the Inspector only holds settings; working with animation happens in a
  separate window ("Edit Animation...", or double-click an animated actor): clip list, skeleton
  tree, a preview of the model alone (own `RenderTarget`, orbit camera, overlay layer 1) and the
  transport + timeline. It drives the actor's own `AnimatorComponent`, and the preview draws the
  actor's already-skinned instance mesh, so both views always show the same pose.
- **Extra render targets:** `Renderer::resizeTarget` / `beginTargetPass` render any scene into
  any `RenderTarget`; apps record such passes in `Application::onRenderOffscreen`, before the main
  scene pass. Overlay lines are per layer (`setOverlayLines(lines, layer)`).

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
  build has no shader-tool dependency. The MSL entry point is `main0`.
- **GPU debugging:** RenderDoc / PIX on Windows, Xcode's Metal frame capture on macOS.

## Conventions

- Namespaces: `kinetic::<module>` (`kinetic::anim` for animation).
- Headers: `engine/<module>/include/kinetic/<module>/X.h`, sources in `src/`.
- Units: meters, seconds, radians. Right-handed; glTF content is used as authored.
- Errors: functions that can fail return `bool` / `std::optional` and log the reason.
