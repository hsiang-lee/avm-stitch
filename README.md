# avm-stitch

**English** | [中文](README.zh-CN.md)

C++20 static library that stitches N fisheye cameras into a 3D AVM (Around View Monitor / 360° bird's-eye) surround view.

- **Pure algorithm API** — structs and `cv::Mat` in/out (`Configure` + `RenderFrame`); no JSON, no protocol dependency.
- **Headless rendering** — EGL surfaceless offscreen context; no X server or display, runs in Docker/CI.
- **Embeddable** — static library, no `main`, namespace `avm_stitch`, CMake + vcpkg.
- **N cameras** — up to 8 by architecture; `Configure` accepts N ∈ [1, 8]. Typical installs use 4 (front/rear/left/right) or 8 cameras.

## Contents

1. [What it does](#1-what-it-does)
2. [Bowl model](#2-bowl-model)
3. [Bake](#3-bake)
4. [Camera models](#4-camera-models)
5. [Blend weights](#5-blend-weights)
6. [Rendering](#6-rendering)
7. [Depth module](#7-depth-module)
8. [Architecture and data flow](#8-architecture-and-data-flow)
9. [Repository layout](#9-repository-layout)
10. [Build and test](#10-build-and-test)
11. [Example: avm_viewer](#11-example-avm_viewer)
12. [Integration](#12-integration)
13. [Extending beyond 8 cameras](#13-extending-beyond-8-cameras)
14. [Known limitations](#14-known-limitations)

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
3. **Render** — upload the N camera frames as a `Texture2DArray`; a GLSL 330 shader samples each camera at its baked UV and sums them weighted.

All geometry and blending costs happen once at `Configure`; per-frame work is a texture upload and one draw pass per view.

## 2. Bowl model

`src/bowl_model/bowl_params.hpp` defines `BowlParams`, an immutable parameter object that owns the parameters the geometry derives from — it holds every input the mesh needs, so it is the information expert for bowl geometry:

- `GenerateBlockBoatMesh(vertices, normals, triangles)` — the mesh.
- `DistanceFromFlatPlatform`, `BowlHeight`, `GridSegmentsPerAxis` — the radial profile.
- `DefaultViewPoses`, `HomeViewPose`, `HomeViewDistance`, `ClipPlanesFor` — view helpers.

`BowlParams::ForCarSize(car_length, car_width, axis_x)` derives a full parameter set from the vehicle footprint. With `scale = car_length / 5.6`:

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

`src/bowl_model/bowl_model.cpp` — `BowlModel(params, cameras)` builds the mesh and bakes it in the constructor. There is no half-initialized state. Accessors are index-based (`vertex_count` / `vertex(i)` / `normal_count` / `normal(i)` / `triangle_count` / `triangle(i)`), so the internal layout stays a private implementation detail.

## 3. Bake

`BowlModel`'s constructor computes, for every vertex:

- `uv[N]` — normalized pixel coordinates in each camera image (`pixel / width`, `pixel / height`); invisible cameras stay `(0, 0)`.
- `weight[N]` — blend weight per camera, normalized over the visible set so the sum is 1; invisible cameras get 0.

Per vertex the bake:

1. computes the azimuth-sector blend weights (`BlendWeightsCalculator`);
2. gates visibility: inside the image rectangle and inside the fisheye circle;
3. multiplies visible cameras by the edge feather factor;
4. normalizes over the cameras that are visible **and** have non-zero weight.

`BakedVertex` is allocated per actual camera count N, not for the 8-camera maximum. The 8-camera cap lives only in the GPU layout and in `Configure` validation; the shader activates only the first N layers via `uNumCameras`.

## 4. Camera models

`src/camera_model/camera_model.hpp` defines the `CameraModel` base class shared by three fisheye models:

- Batch and single-point projection: `ProjectPoints(…)`, `ProjectCameraPoint(P_cam)`.
- Gating: `FisheyeCircleRadius()`, `InsideFisheyeCircle(u, v)`, `EdgeFeather(u, v, feather_px = 40.0)`.

`EdgeFeather` is a hard gate, not a soft falloff: the dark rim of a fisheye image is pure black, and interpolation cannot recover it, so pixels within `feather_px` of the circle edge return 0.

`MakeCameraModel("kb" | "table" | "ocam", params)` constructs `KbCameraModel` / `LutCameraModel` / `OcamCameraModel`:

| Model | Params | Notes |
|---|---|---|
| `kb` | `KbParams` | width, height, fx, fy, cx, cy, k1..k4, fisheye_radius (0 = auto inscribed circle) |
| `table` | `LutParams` | lookup-table projection |
| `ocam` | `OcamParams` | OCamCalib-style polynomial |

The kb model uses a hand-written equirectangular fisheye polynomial (`θ = atan2(r_xy, z)`, θ ∈ [0, π]) as the single source of truth in `KbCameraModel::ProjectCameraPoint`; the batch loop delegates to the same formula.

`Camera` (`src/camera_model/camera.hpp`) bundles extrinsics and the model pointer as one unit:

```cpp
Camera cam(CameraExtrinsics{R, t}, MakeCameraModel("kb", kb_params));
cam.Valid();                 // model present?
cam.Width(); cam.Height();
cam.ProjectPoints(...);      // camera-frame points -> pixels
cam.InsideFisheyeCircle(u, v);
cam.EdgeFeather(u, v);
cam.Extrinsics();            // CameraExtrinsics
```

`CameraExtrinsics` (`src/camera_model/camera_extrinsics.hpp`) stores the T_wc pose (camera → world) as a 3×3 row-major R and a translation in metres, following the OpenCV convention (camera y axis points down). It provides `ToCamera` / `ToWorld` / `RotateCameraToWorld` / `RotateWorldToCamera` / `position()`.

World point → pixel is the composition `model.ProjectCameraPoint(extr.ToCamera(P_world))`: extrinsics handle pose, the model handles imaging.

## 5. Blend weights

`src/bowl_model/blend_weights.cpp` implements an azimuth-sector algorithm that is independent of N (4 and 8 cameras use the same code):

1. The constructor projects each camera to its azimuth `Extrinsics().AzimuthDeg()` and sorts the `(azimuth, camera_index)` pairs.
2. Sector boundaries are the midpoints between adjacent camera azimuths (`SectorBoundaryDeg`).
3. `ComputeBlendHalfAngle(N) = 52.0 / N` degrees is the half-width of the transition band.
4. `WeightsFor(P_world)` locates the ground point's azimuth:
   - inside a boundary ± `52°/N` band → the two adjacent cameras blend with a cosine smoothstep (0.5 each at the boundary);
   - outside every band → the nearest camera owns the point with weight 1.0.
5. `NormalizeWeights(blend_weights, edge_factors, normalized_weights)` renormalizes over visible cameras after edge feathering, so black fisheye rims never enter a seam.

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

## 6. Rendering

`src/render/` is OSG (OpenSceneGraph) + GLSL 330, offscreen by default (with an optional `Windowed` debug mode).

- **`StitchScene`** (`stitch_scene.cpp`) owns the scene graph and packs the baked data into an `osg::Geometry`. The attribute layout is fixed for 8 cameras so the shader stays simple:

```
location 0   vec3  position   (VCS -> OpenGL: x,y,z -> x,z,-y)
location 1   vec3  normal
location 2   vec4  uvCoord0 = (u0, v0, u1, v1)
location 3   vec4  uvCoord1 = (u2, v2, u3, v3)    8 cameras, 4 x vec4
location 4   vec4  uvCoord2 = (u4, v4, u5, v5)
location 5   vec4  uvCoord3 = (u6, v6, u7, v7)
location 6   vec4  weights0  = (w0, w1, w2, w3)   2 x vec4
location 7   vec4  weights1  = (w4, w5, w6, w7)
```

- **Shader** (GLSL 330, assembled into the `StitchScene` state set): the fragment shader unrolls all 8 cameras explicitly to avoid dynamic sampler-array indexing. Each camera contributes `texture(uCameraTextures, vec3(uv_k, k)).rgb * w_k` when `fragNumCameras > k && w_k > 0.001`.
- **`StitchScene::UploadCameraImages`** uploads the N frames as the N layers of a `Texture2DArray` (BGR → BGRA, owned copy, non-power-of-two resize disabled).
- **`OSGViewRenderer`** (`osg_view_renderer.cpp`) creates one FBO camera per view pose. `RenderAllViews()` renders all views and reads them back as BGR `cv::Mat` (default 960×640, clear colour RGB(30,36,50), row flip to compensate the GL bottom-left origin). `RenderMode::Windowed` opens a resizable window with a trackball manipulator instead.
- View poses and clip planes come from `BowlParams`: `DefaultViewPoses()`, `HomeViewPose()`, `HomeViewDistance()`, `ClipPlanesFor(max_eye_to_target)`. Default view names are `3d_front`, `3d_rear`, `3d_left`, `3d_right`.
- **`EGLHeadlessContext`** implements EGL surfaceless (Mesa) desktop GL 3.3 compatibility profile as an `osgViewer::GraphicsWindow`. The 16×16 pbuffer only holds the context; actual drawing goes to FBOs, so no window and no X server are needed.

## 7. Depth module

`src/depth/` contains spherical rectification and **analytic ground-plane depth**:

```cpp
RectificationBasis MakeRectificationBasis(const CameraExtrinsics& a,
                                          const CameraExtrinsics& b);
GroundDepthMap ComputeGroundDepthMap(const RectificationBasis& basis,
                                     const CameraExtrinsics& a,
                                     const CameraExtrinsics& b,
                                     const SphericalGrid& grid,
                                     const GroundDepthOptions& opts = {});
double TriangulateDepth(const cv::Vec3d& dir_a_world, const cv::Vec3d& origin_a,
                        const cv::Vec3d& dir_b_world, const cv::Vec3d& origin_b);
```

`GroundDepthMap` holds `depth`, `disparity`, and `valid` maps. `ComputeGroundDepthMap` does **not** read images: for each cell of a spherical grid it assumes the point lies on the ground plane `z = ground_z`, intersects the ray with that plane to obtain the depth λ, and computes the *predicted* disparity of that point in camera B.

Status: this module is geometric only. There is no stereo matcher and no measured disparity yet, and it currently has no production caller. It is the groundwork for image-based ground depth estimation.

## 8. Architecture and data flow

`StitchEngine` (`src/core/stitch_engine.cpp`) exposes a two-stage API:

```
Configure(std::vector<Camera>, BowlParams, width = 960, height = 640)   once
    |   (Camera owns extrinsics + model; ownership moves into the engine)
    |
    | BowlModel(params, cameras)   mesh + baked uv/weight, CPU
    v
StitchScene(bowl, w, h)            osg::Geometry + GLSL 330
    ^
    | UploadCameraImages -> Texture2DArray (N layers)   per frame
RenderFrame(std::vector<cv::Mat>)
    |
    v
OSGViewRenderer (one FBO camera per ViewPose)
    |
    v
{"3d_front": cv::Mat, "3d_rear": cv::Mat, "3d_left": cv::Mat, "3d_right": cv::Mat}
```

Validation is explicit: `Configure` throws `std::invalid_argument` for N outside [1, 8] or otherwise inconsistent arguments.

## 9. Repository layout

```
src/
├── camera_model/                  camera models (pure math)
│   ├── camera_extrinsics.hpp      T_wc pose value class (header-only)
│   ├── camera_model.hpp / .cpp    CameraModel base, Kb/Lut/Ocam params, MakeCameraModel
│   ├── camera.hpp / .cpp          Camera: extrinsics + model as one unit
│   └── kb_ / lut_ / ocam_camera_model.{hpp,cpp}
├── bowl_model/                    bowl geometry and blending (CPU precompute)
│   ├── bowl_params.hpp / .cpp     BowlParams + ForCarSize + mesh/view helpers
│   ├── bowl_model.hpp / .cpp      BowlModel: constructor builds mesh and bakes
│   └── blend_weights.hpp / .cpp   BlendWeightsCalculator (N-agnostic)
├── common/
│   └── angle_utils.hpp            angle normalization/conversion helpers
├── depth/                         spherical rectification + analytic ground depth
│   ├── depth_common.hpp
│   ├── spherical_rectification.{hpp,cpp}
│   └── ground_depth.{hpp,cpp}
├── render/                        GPU / OSG
│   ├── stitch_scene.hpp / .cpp    scene graph, GLSL 330, Texture2DArray
│   ├── osg_view_renderer.hpp / .cpp  offscreen multi-view renderer + readback
│   └── egl_headless_context.hpp / .cpp  surfaceless EGL GL 3.3
└── core/
    └── stitch_engine.hpp / .cpp   Configure + RenderFrame

examples/
├── avm_viewer.cpp                 interactive bowl viewer
├── data_4/                        4-camera sample (5.27 m car)
└── data_8/                        8-camera sample (211.58 m ship)

tests/                             15 gtest executables / 210 test cases
├── test_support/                  shared fixtures (header-only)
├── camera_model/                  projection, extrinsics, Camera, factory
├── bowl_model/                    bowl params, mesh/bake, blend weights
├── depth/                         spherical rectification, ground depth
├── render/                        scene graph, renderer, EGL context
├── common/                        angle utilities
└── core/                          engine end-to-end + regression
```

## 10. Build and test

Dependencies come from the vcpkg manifest (`gtest`, `opencv4` with `calib3d`/`jpeg`/`png`, `osg`, `nlohmann-json`) plus system EGL (`find_package(OpenGL COMPONENTS EGL)`). The library itself has no JSON dependency; `nlohmann-json` is used only by the `avm_viewer` example to read `calibration.json`.

```bash
export VCPKG_ROOT=/path/to/vcpkg
cmake --preset default          # Unix Makefiles, build/
cmake --build build             # incremental; do not delete build/
ctest --test-dir build          # 210 tests in 15 gtest executables
```

The GL-gated suites (`test_stitch_engine`, `test_egl_headless_context`, `test_osg_view_renderer`) probe for a headless EGL context and **skip** when none is available; set `AVM_STITCH_TEST_GL=1` to force them on.

## 11. Example: avm_viewer

```bash
./build/examples/avm_viewer ../../examples/data_8
```

`avm_viewer` reads `<dir>/calibration.json` with nlohmann-json, loads the images with OpenCV, derives the bowl via `BowlParams::ForCarSize`, and opens an OSG window. Controls: left drag = rotate, right drag = zoom, middle drag = pan.

`calibration.json` schema:

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

## 12. Integration

```cpp
#include "bowl_model/bowl_params.hpp"
#include "camera_model/camera.hpp"
#include "camera_model/camera_model.hpp"
#include "core/stitch_engine.hpp"

using namespace avm_stitch;

std::vector<Camera> cameras;
cameras.emplace_back(CameraExtrinsics{R, t}, MakeCameraModel("kb", kb_params));
// ... N cameras, N in [1, 8]

StitchEngine engine;
engine.Configure(std::move(cameras),
                 BowlParams::ForCarSize(5.27, 2.17, 2.9),
                 960, 640);

std::vector<cv::Mat> frames;  // N BGR images, same order as cameras
auto views = engine.RenderFrame(frames);
// views["3d_front"], views["3d_rear"], views["3d_left"], views["3d_right"]
```

- `Configure` takes ownership of the `Camera` objects (and therefore of their models).
- `RenderFrame` requires the same camera order as `Configure`.
- Errors are reported as exceptions (`std::invalid_argument` for configuration errors).
- Protocol adaptation (queue tasks, base64 decoding, result envelopes, `ParseBowlParams`) lives in **avm-worker**, not in this library.

## 13. Extending beyond 8 cameras

The geometry and blend-weight algorithms are independent of N. The 8-camera cap is only in the GPU layout and validation:

| Bottleneck | Location | Current state |
|---|---|---|
| Vertex attribute layout | `StitchScene` slot constants | UV: 4 × vec4 (locations 2-5); weights: 2 × vec4 (locations 6-7) |
| Fragment shader | `StitchScene` fragment source | 8 unrolled camera branches |
| Configure validation | `StitchEngine::Configure` | N ∈ [1, 8] |

For 16 cameras:

1. Repack UVs as 8 × vec4 (locations 2-9) and weights as 4 × vec4 (locations 10-13). GLSL 330 guarantees at least 16 vertex attributes, so 0-13 is safe in principle; verify on the target GPU.
2. Unroll 16 shader branches; the `WEIGHT_MIN` threshold still applies.
3. Raise the `Configure` limit to 16.
4. No changes needed in `BlendWeightsCalculator` (half-angle becomes `52/16 = 3.25°`), `BowlModel`, or `UploadCameraImages`.

## 14. Known limitations

- **Depth module is analytic only** — `src/depth/` computes ground-plane depth and predicted disparity from geometry; it does not read images, has no stereo matcher, and currently has no production caller.
- **JSON integration is external** — task contracts, backend factories, result envelopes, camera-name parsing and `ParseBowlParams` live in avm-worker (`avm_stitch_integration` / `avm_renderer_bridge`). This library exposes only the C++ algorithm API.
- **GL tests need EGL** — the GL-gated suites skip without a headless EGL context (`AVM_STITCH_TEST_GL=1` forces them on).
- **OSG/vcpkg quirk** — `osg_ModelViewProjectionMatrix` is not updated automatically; the MVP uniform must be `osg::Matrixf` (a `Matrixd` triggers `GL_INVALID_OPERATION`). Re-test when the OSG build changes.
- **8-camera GPU layout cap** — see [section 13](#13-extending-beyond-8-cameras).
