# avm-stitch

**English** | [中文](README.zh-CN.md)

Rust workspace that stitches N fisheye cameras into a 3D AVM (Around View Monitor / 360° bird's-eye) surround view.

- **Pure algorithm core** — `avm-core` holds the geometry and camera math with no GPU, no image and no OS dependency.
- **Headless rendering** — `avm-render` draws four views offscreen with wgpu, so it runs in containers and CI on any Vulkan/Metal/DX12 adapter (software rasterisers included).
- **N cameras** — up to 8 by architecture; configuration accepts N ∈ [1, 8]. Typical installs use 4 (front/rear/left/right) or 8 cameras.
- **Optional window** — `avm-viewer` opens the same scene interactively and can present a fixed number of frames and save a screenshot.

## Contents

1. [What it does](#1-what-it-does)
2. [Workspace layout](#2-workspace-layout)
3. [Bowl model](#3-bowl-model)
4. [Bake](#4-bake)
5. [Camera models](#5-camera-models)
6. [Blend weights](#6-blend-weights)
7. [Renderer](#7-renderer)
8. [Engine and data flow](#8-engine-and-data-flow)
9. [Depth module](#9-depth-module)
10. [Build and run](#10-build-and-run)
11. [Verification and parity](#11-verification-and-parity)
12. [Calibration JSON](#12-calibration-json)
13. [Extending beyond 8 cameras](#13-extending-beyond-8-cameras)
14. [Known limitations](#14-known-limitations)
15. [History](#15-history)

## 1. What it does

An AVM system mounts N fisheye cameras around a vehicle (FOV typically > 180°), so adjacent views overlap on the ground. Pasting them into a flat top-down image does not work: the same ground patch has different pixel density in each camera, and a flat plane shows a visible stretch discontinuity at every seam.

The industry-standard fix is the **bowl model**: model the world around the car as a bowl that is flat under the vehicle and rises smoothly toward the rim. Rendering a virtual camera at this surface compresses distant ground naturally, so all camera images meet without distortion at the seams.

```
                [Front]
                  F
        +---------+---------+      F/R/L/R = fisheye cameras
        | overlap | overlap |      (FOV > 180 deg)
   L    |    +---------+    |    R
  e ----+    |   CAR   |    +---- i
  f    |    +---------+    |    g
  t    |         |         |    h
        | overlap | overlap |
        +---------+---------+
                  R
                [Rear]
```

The pipeline has three stages:

1. **Geometry** — build a triangle mesh from `BowlParams`.
2. **Bake** — for every vertex, project it into every camera and precompute normalized UVs and blend weights on the CPU.
3. **Render** — upload the N camera frames as a texture array; a WGSL shader samples each camera at its baked UV and sums them weighted.

All geometry and blending costs happen once at configuration; per-frame work is a texture upload and one draw pass per view.

## 2. Workspace layout

| crate | role | notable dependencies |
|---|---|---|
| `avm-core` | camera models, extrinsics, bowl geometry, blend weights, depth, calibration JSON | `nalgebra`, `serde`, `thiserror` — no GPU |
| `avm-render` | wgpu offscreen renderer, WGSL shader, vertex packing, texture readback | `wgpu`, `naga`, `bytemuck`, `pollster` |
| `avm-engine` | `configure` / `render_frame` orchestration and validation | `avm-core`, `avm-render`, `image` |
| `avm-viewer` | interactive window and trackball controller; also the viewer binary | `avm-render`, `winit`, `wgpu`, `image` |
| `avm-cli` | prints calibration, derived bowl parameters and mesh statistics | `avm-core` only |

The dependency direction is enforced by tests: `avm-core`'s resolved closure never reaches a GPU or image crate, and `avm-cli`'s closure never reaches `wgpu`, `winit`, `avm-render` or `avm-viewer`.

## 3. Bowl model

`avm-core::bowl::BowlParams` defines the immutable parameter object that owns the inputs the geometry derives from — it holds everything the mesh needs, so it is the information expert for bowl geometry:

- `generate_block_boat_mesh` — the mesh.
- `distance_from_flat_platform`, `bowl_height`, `grid_segments_per_axis` — the radial profile.
- `default_view_poses`, `home_view_pose`, `clip_planes_for` — view helpers.

`BowlParams::for_car_size(car_length, car_width, axis_x)` derives a full parameter set from the vehicle footprint. With `scale = car_length / 5.6`:

| Parameter | Value | Notes |
|---|---|---|
| `flat_platform_radius` | `2.0 × scale` | rounded-corner margin around the car footprint |
| `grid_step` | `0.2 × scale` | mesh spacing (m); grid ≈ 126×126 at L = 5.6 |
| `outer_radius` | `12.6 × scale` | radial extent of the bowl |
| `slope_coefficient` | `0.09 × scale^-1.2` | rim slope |
| `height_exponent` | `2.2` | fixed profile shape |
| `max_height` | `8.6 × scale` | vertices above this are clipped |
| `car_center` | `(axis_x / 2, 0)` | rear-axle frame |

Height profile and footprint:

```
 z ^      rim: z = slope_coefficient * distance_from_flat_platform^height_exponent   (capped at max_height)
   |    ___                          ___
   |   /    \___                ___/    \
   |  /         \___      ___/           \
   | /             \____/                 \
   |/                                     \
 0 +----|---------flat z=0--------|--------\----> rd
        | car footprint           |      outer_radius
     rounded rectangle         curved slope
     (L/2 + flat_platform_radius, W/2 + flat_platform_radius)
```

The same formula covers cars and ships: the bundled sample data contains a 5.27 m car (`data_4`) and a 211.58 m ship (`data_8`).

`avm-core::bowl::BowlModel::new(params, cameras)` builds the mesh and bakes it in the constructor. There is no half-initialized state. Accessors are index-based (`vertex_count` / `baked_vertex` / `triangle_count`), so the internal layout stays a private implementation detail.

## 4. Bake

`BowlModel`'s constructor computes, for every vertex:

- `uv[N]` — normalized pixel coordinates in each camera image (`pixel / width`, `pixel / height`); invisible cameras stay `(0, 0)`.
- `weight[N]` — blend weight per camera, normalized over the visible set so the sum is 1; invisible cameras get 0.

Per vertex the bake:

1. computes the azimuth-sector blend weights (`BlendWeightsCalculator`);
2. gates visibility: inside the image rectangle and inside the fisheye circle;
3. multiplies visible cameras by the edge feather factor;
4. normalizes over the cameras that are visible **and** have non-zero weight.

`BakedVertex` is allocated per actual camera count N, not for the 8-camera maximum. The 8-camera cap lives only in the GPU layout and in configuration validation; the shader activates only the first N layers.

## 5. Camera models

`avm-core::camera::CameraModel` is the trait shared by three fisheye models:

- Batch and single-point projection: `project_points`, `project_camera_point`.
- Gating: `fisheye_circle_radius`, `inside_fisheye_circle`, `edge_feather(u, v, feather_px = 40.0)`.

`edge_feather` is a hard gate, not a soft falloff: the dark rim of a fisheye image is pure black, and interpolation cannot recover it, so pixels within `feather_px` of the circle edge return 0.

`make_camera_model("kb" | "table" | "ocam", params)` constructs `KbCameraModel` / `LutCameraModel` / `OcamCameraModel`:

| Model | Params | Notes |
|---|---|---|
| `kb` | `KbParams` | width, height, fx, fy, cx, cy, k1..k4, fisheye_radius (0 = auto inscribed circle) |
| `table` | `LutParams` | lookup-table projection |
| `ocam` | `OcamParams` | OCamCalib-style polynomial |

The kb model uses a hand-written equirectangular fisheye polynomial (`θ = atan2(r_xy, z)`, θ ∈ [0, π]) as the single source of truth in `KbCameraModel::project_camera_point`; the batch loop delegates to the same formula.

`Camera` bundles extrinsics and the model as one unit:

```rust
let camera = Camera::new(extrinsics, make_camera_model("kb", &CameraModelParams::Kb(kb_params))?);
camera.width();
let mut uv = [[0.0, 0.0]; points.len()];
let mut valid = [0u8; points.len()];
camera.project_points(&points, &mut uv, &mut valid)?;  // camera-frame points -> pixels
camera.inside_fisheye_circle(u, v);
camera.edge_feather(u, v, 40.0);
camera.extrinsics();                 // T_wc pose
```

`CameraExtrinsics` stores the T_wc pose (camera → world) as a row-major 3×3 `R` and a translation in metres, with the camera y axis pointing down. It provides `to_camera` / `to_world` / `rotate_camera_to_world` / `rotate_world_to_camera` / `position`.

World point → pixel is the composition `model.project_camera_point(extr.to_camera(p_world))`: extrinsics handle pose, the model handles imaging.

## 6. Blend weights

`BlendWeightsCalculator` implements an azimuth-sector algorithm that is independent of N (4 and 8 cameras use the same code):

1. The constructor projects each camera to its azimuth `extrinsics().azimuth_deg()` and sorts the `(azimuth, camera_index)` pairs.
2. Sector boundaries are the midpoints between adjacent camera azimuths.
3. `compute_blend_half_angle(N) = 52.0 / N` degrees is the half-width of the transition band.
4. `weights_for(p_world)` locates the ground point's azimuth:
   - inside a boundary ± `52°/N` band → the two adjacent cameras blend with a cosine smoothstep (0.5 each at the boundary);
   - outside every band → the nearest camera owns the point with weight 1.0.
5. `normalize_weights(blend_weights, edge_factors, normalized_weights)` renormalizes over visible cameras after edge feathering, so black fisheye rims never enter a seam.

```
 weight
 1.0 |________                          ________
             \                        /        A owns (nearest)
 0.5 |         \                     /
              |                     |     <- boundary: A = B = 0.5
 0.0 |          \__________________/          B owns
     +----------+--------+---------+---------> azimuth
               b-h      b       b+h
              blend band:  h = 52/N deg  (N=4 -> 13 deg)
```

## 7. Renderer

`avm-render` is the only crate that talks to the GPU. It owns the shader and the draw path used by both offscreen rendering and the window.

- **`PackedVertex`** packs the baked data into one dense vertex (120 bytes) with a fixed 8-camera layout, so the shader stays simple:

```
location 0   vec3  position   (VCS -> render basis: x,y,z -> x,z,-y)
location 1   vec3  normal
location 2   vec4  uv0 = (u0, v0, u1, v1)
location 3   vec4  uv1 = (u2, v2, u3, v3)    8 cameras, 4 x vec4
location 4   vec4  uv2 = (u4, v4, u5, v5)
location 5   vec4  uv3 = (u6, v6, u7, v7)
location 6   vec4  weights0 = (w0, w1, w2, w3)   2 x vec4
location 7   vec4  weights1 = (w4, w5, w6, w7)
```

- **Shader** (`shader.wgsl`): the fragment shader unrolls all 8 cameras explicitly to avoid dynamic sampler-array indexing. Each camera contributes `textureSample(camera_textures, camera_sampler, vec3(uv_k, k)).rgb * w_k` when `num_cameras > k && w_k > 0.001`.
- **`SceneData`** converts a `BowlModel` into vertices, indices and the N camera images; the images become the N layers of a texture array.
- **`GpuSceneRenderer`** builds the pipeline (color-format parameterised), the camera array, the vertex/index buffers and records one pass per `ViewSpec`. Depth is `Depth32Float`, compare `Less`.
- **`render_all_views` / `render_all_views_with`** create the adapter, render every view offscreen, and read each frame back as RGBA8 with `TextureReadback`. Without an adapter they return `RenderError::NoAdapter` instead of panicking.
- View poses and clip planes come from `BowlParams`: `default_view_poses()`, `home_view_pose()`, `clip_planes_for(max_eye_to_target)`. Default view names are `3d_front`, `3d_rear`, `3d_left`, `3d_right`.
- The scene is drawn with the background colour `BACKGROUND_CLEAR` = RGB(30, 36, 50).

## 8. Engine and data flow

`avm-engine::StitchEngine` exposes a two-stage API:

```
configure(Vec<Camera>, &BowlParams, width: u32, height: u32)          once
    |   (Camera owns extrinsics + model; ownership moves into the engine)
    |
    | BowlModel::new                  mesh + baked uv/weight, CPU
    v
SceneData::from_bowl_model           PackedVertex buffer + camera images
    ^
    | render_frame(&[RgbaImage])      per frame: texture upload + one pass per view
    |
    v
Vec<(String, RgbaImage)>             ("3d_front" .. "3d_right")
```

`width` and `height` are required arguments (Rust has no default parameters).
Validation is explicit: `configure` returns `EngineError` for N outside [1, 8], a missing camera model, mismatched image sizes, a non-positive grid step or outer radius, and a negative slope coefficient or height exponent.

## 9. Depth module

`avm-core::depth` contains spherical rectification and **analytic ground-plane depth**:

```rust
make_rectification_basis(a: &CameraExtrinsics, b: &CameraExtrinsics) -> RectificationBasis;
compute_ground_depth_map(&basis, a, b, &grid, options: GroundDepthOptions) -> GroundDepthMap;
triangulate_depth(dir_a_world, origin_a, dir_b_world, origin_b) -> f64;
```

`GroundDepthMap` holds `depth`, `disparity`, and `valid` maps. `compute_ground_depth_map` does **not** read images: for each cell of a spherical grid it assumes the point lies on the ground plane `z = ground_z`, intersects the ray with that plane to obtain the depth λ, and computes the *predicted* disparity of that point in camera B.

Status: geometric only. There is no stereo matcher and no measured disparity yet, and it currently has no production caller. It is the groundwork for image-based ground depth estimation.

## 10. Build and run

The workspace needs a Rust toolchain (edition 2021, rust-version 1.85) and, for rendering, a wgpu adapter. No system SDK is installed by the build.

```bash
cargo build --workspace
cargo test --workspace

# calibration statistics: no GPU, no display
cargo run -p avm-cli -- examples/data_4

# interactive viewer: needs a display (X11 or Wayland)
cargo run -p avm-viewer -- examples/data_4
```

`avm-cli` prints the cameras, the derived bowl parameters, the mesh size and the baked weight range. `avm-viewer` opens a 1280×720 window showing the home view of the same scene; left drag rotates, right drag zooms (dolly), middle drag pans, and the wheel zooms.

For scripted runs the viewer accepts:

```bash
cargo run -p avm-viewer -- examples/data_4 --frames 2
cargo run -p avm-viewer -- examples/data_4 --frames 1 --screenshot shot.png
```

`--frames N` presents N frames and exits; `--screenshot PATH` writes the first presented frame as PNG and fails the run if no frame could be captured. Without a display the viewer exits non-zero with an explicit message.

## 11. Verification and parity

```bash
cargo test -p avm-render --test parity
```

The parity test renders `examples/data_4` at 960×640 and compares each of the four views with the stored reference PNGs in `crates/avm-render/tests/fixtures/parity/data_4/`. It requires, per channel, PSNR > 45 dB and a mean absolute difference < 1.0; the current fixtures sit at 86+ dB on the worst channel. Those fixtures were captured from the original renderer and are kept as external ground truth, so a shader or matrix change that alters output fails here.

GPU-backed tests skip when the machine has no adapter; parity deliberately fails instead of skipping, so a silent GPU-less run cannot claim parity.

## 12. Calibration JSON

```json
{
  "cameras": [
    {
      "name": "front",
      "image": "front.png",
      "width": 1920,
      "height": 1080,
      "intrinsics": {"fx": 0.0, "fy": 0.0, "cx": 0.0, "cy": 0.0,
                     "k1": 0.0, "k2": 0.0, "k3": 0.0, "k4": 0.0},
      "extrinsics": {"r": [[1,0,0],[0,1,0],[0,0,1]], "t": [0, 0, 0]}
    }
  ],
  "car": {"length": 5.27, "width": 2.17, "axis_x": 2.9,
          "origin_convention": "rear_axle"}
}
```

`avm-core::calibration::Calibration::load_from_path` reads it; `bowl_params()` derives the bowl from the `car` block.

## 13. Extending beyond 8 cameras

The geometry and blend-weight algorithms are independent of N. The 8-camera cap is only in the GPU layout and validation:

| Bottleneck | Location | Current state |
|---|---|---|
| Vertex attribute layout | `avm-render::vertex` slot constants | UV: 4 × vec4 (locations 2-5); weights: 2 × vec4 (locations 6-7) |
| Fragment shader | `shader.wgsl` | 8 unrolled camera branches |
| Configuration validation | `StitchEngine::configure` | N ∈ [1, 8] |

For 16 cameras:

1. Repack UVs as 8 × vec4 and weights as 4 × vec4, and extend the vertex layout.
2. Unroll 16 shader branches; the `w_k > 0.001` threshold still applies.
3. Raise the configuration limit to 16.
4. No changes needed in `BlendWeightsCalculator` (half-angle becomes `52/16 = 3.25°`), `BowlModel`, or the texture-array upload.

## 14. Known limitations

- **Windowed output is checked only by the smoke test** — with a display, `avm-viewer/tests/windowed_smoke.rs` compares the screenshot byte-for-byte with the offscreen render of the same view; without a display and without `xvfb-run` it skips with an explicit message.
- **Depth module is analytic only** — it computes ground-plane depth and predicted disparity from geometry; it does not read images, has no stereo matcher, and currently has no production caller.
- **QNX is not verified** — no target toolchain run has been performed; the core crate is OS-independent by construction but the GPU path assumes a Vulkan/Metal/DX12 adapter.
- **Parity fixtures cover one dataset** — `data_4` at 960×640, four views. Other sizes, datasets and camera counts have no reference images.
- **8-camera GPU layout cap** — see [section 13](#13-extending-beyond-8-cameras).
- **Deferred refactors** — the migration's known debt is tracked in [`docs/known-debt.md`](docs/known-debt.md).

## 15. History

This repository was ported from a C++20/OSG implementation. That implementation is no longer in the tree; it is preserved at tag `archive/cpp-implementation`, together with the reference-image driver, for anyone who needs to compare behaviour or regenerate fixtures. The migration test map in `docs/migration-test-mapping.md` records how each original case was ported or exempted.
