# Architecture

## Goals

1. **Animation first.** The engine exists to build and test animation systems; rendering serves them.
2. **Runs the same on macOS and Windows** from one CMake project.
3. **Small, understandable layers** — each module is a static library with explicit dependencies.

## Modules and dependencies

```
core
animation -> core                  pure CPU math, no SDL / GPU
assets    -> animation             glTF import
scene     -> core                  EnTT registry + components
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
                            → localToModel → skinning matrices ──► GPU (milestone 4)
```

- **Fixed timestep** (`AppDesc::fixedTimestep`, 60 Hz): animation and physics advance in
  constant steps, so results don't depend on frame rate. Rendering gets `interpolationAlpha`
  to interpolate between the last two ticks. Frame time is clamped to 0.25 s to avoid the
  "spiral of death" after a breakpoint.
- **Frame lifecycle** (`gfx::Device`): acquire command buffer + swapchain → copy passes
  (ImGui upload, mesh uploads, later: skinning matrices) → scene pass into the offscreen
  *scene target* (colour + depth) → [sandbox: blit to swapchain] → overlay pass on the swapchain
  (ImGui) → submit. The editor sizes the scene target to its Viewport panel and draws it there as
  an ImGui image (`AppDesc::presentScene = false`).
- **Camera** is an entity with `CameraComponent` + `TransformComponent`; it looks down local −Z.
  `Renderer::drawScene` renders the floor grid and every `MeshRendererComponent` from the scene's
  first camera.
- **Meshes** live in the renderer's library; `MeshRendererComponent::mesh` is an index into it
  (0 = built-in triangle). `assets::loadMesh` turns .fbx / .obj into `MeshData` (Y-up, meters,
  transforms baked, triangulated) via ufbx; `Renderer::addMesh` uploads it.
- **Editor assets:** the *Assets* drawer loads every .fbx / .obj in `assets/meshes/` on startup
  (the project folder, so additions end up in the repo). Tiles are ImGui drag & drop sources
  carrying a `MeshId`; the Viewport and the Inspector's Mesh field accept them.

## Animation data

- **`Pose` is Structure-of-Arrays** — separate arrays of translations, rotations and scales.
  Blending, sampling and IK iterate those arrays linearly (cache-friendly, easy to SIMD later).
- **`Skeleton` invariant:** `parents[i] < i`. The glTF importer sorts joints by depth to
  guarantee it, so model-space transforms are one forward pass with no recursion.
- **Rotations blend with nlerp on the shortest arc** (`dot < 0 → negate`). It is cheaper than
  slerp and commutative for multi-way blends; clip sampling between keys uses slerp.
- **Additive layers** store `additive - reference` and apply it on top of a base pose.

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
