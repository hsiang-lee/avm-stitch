# avm-stitch

[English](README.md) | **中文**

C++20 AVM（Around View Monitor，环视 / 360° 鸟瞰）拼接静态库：把 N 路鱼眼相机画面拼成车辆周围的三维环视视图。

- **纯算法 API** — 结构体 / `cv::Mat` 进出（`Configure` + `RenderFrame`），不依赖 JSON，也不依赖任务协议。
- **无窗口渲染** — EGL surfaceless 离屏上下文，不需要 X server / 显示器，可直接跑在 Docker / CI 中。
- **可嵌入** — 纯静态库，无 `main`，命名空间 `avm_stitch`，CMake + vcpkg 管理。
- **N 路相机** — 架构上限 8 路，`Configure` 接受 N ∈ [1, 8]；典型配置为 4 路（前/后/左/右）或 8 路。

## 目录

1. [它做什么](#1-它做什么)
2. [碗模型](#2-碗模型)
3. [烘焙](#3-烘焙)
4. [相机模型](#4-相机模型)
5. [融合权重](#5-融合权重)
6. [渲染](#6-渲染)
7. [深度模块](#7-深度模块)
8. [架构与数据流](#8-架构与数据流)
9. [目录结构](#9-目录结构)
10. [构建与测试](#10-构建与测试)
11. [示例：avm_viewer](#11-示例avm_viewer)
12. [集成方式](#12-集成方式)
13. [扩展到 8 路以上](#13-扩展到-8-路以上)
14. [已知限制](#14-已知限制)

## 1. 它做什么

AVM 系统在车辆前后左右安装 N 路鱼眼相机（FOV 通常超过 180°），相邻视野在地面上重叠。把画面直接拼到一张俯视平面上并不可行：同一块地面在不同相机里的像素密度不同，平面拼接会在每条接缝处出现明显的拉伸断层。

行业通行解法是**碗模型**：把车辆周围建模成一只中间平坦、四周缓缓升起的碗。虚拟相机看向这只碗时，远处地面被自然压缩，各路画面在碗面上恰好平滑衔接。

```
                    [Front]
                      F
            +---------+---------+      F/R/L/R = 鱼眼相机
            | overlap | overlap |      (FOV > 180°)
       L    |    +---------+    |    R
      e ----+    |   CAR   |    +---- i
      f    |    +---------+    |    g
      t    |         |         |    h
            | overlap | overlap |
            +---------+---------+
                      R
                    [Rear]
```

整条流水线分三步：

1. **几何** — 由 `BowlParams` 生成三角网格。
2. **烘焙** — 对每个顶点，投影进每一路相机，在 CPU 上预计算归一化 UV 和融合权重。
3. **渲染** — 把 N 路画面作为 `Texture2DArray` 上传；GLSL 330 着色器按烘焙好的 UV 采样各相机，再按权重求和。

几何与融合全部在 `Configure` 时算一次；每帧只做纹理上传和每个视图一次绘制。

## 2. 碗模型

`src/bowl_model/bowl_params.hpp` 定义 `BowlParams`：不可变参数对象，持有几何所依赖的参数——它握有网格所需的全部输入，是碗几何的信息专家：

- `GenerateBlockBoatMesh(vertices, normals, triangles)` — 网格生成。
- `DistanceFromFlatPlatform`、`BowlHeight`、`GridSegmentsPerAxis` — 径向剖面。
- `DefaultViewPoses`、`HomeViewPose`、`HomeViewDistance`、`ClipPlanesFor` — 视图辅助。

`BowlParams::ForCarSize(car_length, car_width, axis_x)` 按车辆尺寸派生整套参数。令 `scale = car_length / 5.6`：

| 参数 | 取值 | 说明 |
|---|---|---|
| `flat_platform_radius` | `2.0 × scale` | 车体轮廓外的圆角外扩量 |
| `grid_step` | `0.2 × scale` | 网格步长（m）；L = 5.6 时约 126×126 |
| `outer_radius` | `12.6 × scale` | 碗的径向范围 |
| `slope_coefficient` | `0.09 × scale^-1.2` | 碗沿坡度 |
| `height_exponent` | `2.2` | 固定剖面形状 |
| `max_height` | `8.6 × scale` | 超过此高度的顶点被裁剪 |
| `car_center` | `(axis_x / 2, 0)` | 后轴坐标系 |

高度剖面与 footprint：

```
 z ^      rim: z = slope_coefficient * distance_from_flat_platform^height_exponent   (max_height 封顶)
   |    ___                          ___
   |   /    \___                ___/    \
   |  /         \___      ___/           \
   | /             \____/                 \
   |/                                     \
 0 +----|---------平地 z=0--------|--------\----> rd
        | 车体 footprint          |      outer_radius
     圆角矩形                 弧形坡面
     (L/2 + flat_platform_radius, W/2 + flat_platform_radius)
```

同一套公式覆盖汽车和船舶：内置样例数据包含 5.27 m 汽车（`data_4`）和 211.58 m 船舶（`data_8`）。

`src/bowl_model/bowl_model.cpp` — `BowlModel(params, cameras)` 在构造函数里生成网格并完成烘焙，不存在半初始化状态。数据经细粒度索引访问（`vertex_count` / `vertex(i)` / `normal_count` / `normal(i)` / `triangle_count` / `triangle(i)`），内部布局是纯实现细节。

## 3. 烘焙

`BowlModel` 构造函数对每个顶点计算：

- `uv[N]` — 在该路相机图像中的归一化像素坐标（`pixel / width`、`pixel / height`）；不可见的相机槽保持 `(0, 0)`。
- `weight[N]` — 每路相机的融合权重，在可见集合内归一化，和为 1；不可见相机为 0。

逐顶点烘焙流程：

1. 计算方位角扇区融合权重（`BlendWeightsCalculator`）；
2. 判定可见性：在图像矩形内且在鱼眼有效圆内；
3. 可见相机乘上边缘羽化系数；
4. 在“可见且权重非零”的相机集合内归一化。

`BakedVertex` 按实际相机数 N 分配，不为 8 路上限预留空槽。8 路上限只存在于 GPU 布局与 `Configure` 校验中；着色器按 `uNumCameras` 只激活前 N 层。

## 4. 相机模型

`src/camera_model/camera_model.hpp` 定义三种鱼眼模型共享的 `CameraModel` 基类：

- 批量 / 单点投影：`ProjectPoints(…)`、`ProjectCameraPoint(P_cam)`。
- 门控：`FisheyeCircleRadius()`、`InsideFisheyeCircle(u, v)`、`EdgeFeather(u, v, feather_px = 40.0)`。

`EdgeFeather` 是硬门限而非软衰减：鱼眼图像边缘是纯黑，线性插值无法挽回，因此距离有效圆边缘不足 `feather_px` 的像素直接返回 0。

`MakeCameraModel("kb" | "table" | "ocam", params)` 构造 `KbCameraModel` / `LutCameraModel` / `OcamCameraModel`：

| 模型 | 参数 | 说明 |
|---|---|---|
| `kb` | `KbParams` | width, height, fx, fy, cx, cy, k1..k4, fisheye_radius（0 = 自动内切圆） |
| `table` | `LutParams` | 查找表投影 |
| `ocam` | `OcamParams` | OCamCalib 风格多项式 |

kb 模型使用手写等距鱼眼多项式（`θ = atan2(r_xy, z)`，θ ∈ [0, π]），单一事实源为 `KbCameraModel::ProjectCameraPoint`；批量循环委托同一公式。

`Camera`（`src/camera_model/camera.hpp`）把外参与模型指针统一成对：

```cpp
Camera cam(CameraExtrinsics{R, t}, MakeCameraModel("kb", kb_params));
cam.Valid();                 // 模型是否存在？
cam.Width(); cam.Height();
cam.ProjectPoints(...);      // 相机系点 -> 像素
cam.InsideFisheyeCircle(u, v);
cam.EdgeFeather(u, v);
cam.Extrinsics();            // CameraExtrinsics
```

`CameraExtrinsics`（`src/camera_model/camera_extrinsics.hpp`）保存 T_wc 位姿（camera → world），内部为 3×3 行主序 R 与平移（米），遵循 OpenCV 惯例（相机 y 轴朝下）。提供 `ToCamera` / `ToWorld` / `RotateCameraToWorld` / `RotateWorldToCamera` / `position()`。

世界点 → 像素的组合：`model.ProjectCameraPoint(extr.ToCamera(P_world))`——外参负责位姿，模型负责成像。

## 5. 融合权重

`src/bowl_model/blend_weights.cpp` 实现与 N 无关的方位角扇区算法（4 路和 8 路同一套代码）：

1. 构造时把每路相机投影成方位角 `Extrinsics().AzimuthDeg()`，对 `(方位角, 相机下标)` 排序。
2. 扇区边界取相邻相机方位角的中点（`SectorBoundaryDeg`）。
3. `ComputeBlendHalfAngle(N) = 52.0 / N` 度，是过渡带半角。
4. `WeightsFor(P_world)` 定位地面点方位角：
   - 落在某边界 ± `52°/N` 过渡带内 → 相邻两路按余弦 smoothstep 混合（边界处各 0.5）；
   - 过渡带之外 → 最近相机独占，权重 1.0。
5. `NormalizeWeights(blend_weights, edge_factors, normalized_weights)` 在边缘羽化后按可见相机重新归一化，保证黑边不进入接缝。

```
 weight
 1.0 |________                          ________
             \                        /        A 独占（最近）
 0.5 |         \                     /
              |                     |     <- 边界：A = B = 0.5
 0.0 |          \__________________/          B 独占
     +----------+--------+---------+---------> 方位角
               b-h      b       b+h
              过渡带：h = 52/N 度（N=4 -> 13 度）
```

## 6. 渲染

`src/render/` 基于 OSG（OpenSceneGraph）+ GLSL 330，默认离屏（可选 `Windowed` 调试模式）。

- **`StitchScene`**（`stitch_scene.cpp`）拥有场景图，把烘焙数据装配成 `osg::Geometry`。顶点属性布局按 8 路固定打包：

```
location 0   vec3  position   (VCS -> OpenGL: x,y,z -> x,z,-y)
location 1   vec3  normal
location 2   vec4  uvCoord0 = (u0, v0, u1, v1)
location 3   vec4  uvCoord1 = (u2, v2, u3, v3)    8 路，4 x vec4
location 4   vec4  uvCoord2 = (u4, v4, u5, v5)
location 5   vec4  uvCoord3 = (u6, v6, u7, v7)
location 6   vec4  weights0  = (w0, w1, w2, w3)   2 x vec4
location 7   vec4  weights1  = (w4, w5, w6, w7)
```

- **着色器**（GLSL 330，装配进 `StitchScene` 的 state set）：片元着色器把 8 路显式展开，避免动态采样器数组索引。每路在 `fragNumCameras > k && w_k > 0.001` 时贡献 `texture(uCameraTextures, vec3(uv_k, k)).rgb * w_k`。
- **`StitchScene::UploadCameraImages`** 把 N 路画面作为 `Texture2DArray` 的 N 个图层上传（BGR → BGRA，自持拷贝，禁用非 2 次幂重缩放）。
- **`OSGViewRenderer`**（`osg_view_renderer.cpp`）为每个视图位姿建一个 FBO 相机。`RenderAllViews()` 渲染全部视图并回读为 BGR `cv::Mat`（默认 960×640，清屏色 RGB(30,36,50)，回读后翻转补偿 GL 左下角原点）。`RenderMode::Windowed` 则打开可缩放窗口并挂 trackball 操作器。
- 视图位姿与裁剪面来自 `BowlParams`：`DefaultViewPoses()`、`HomeViewPose()`、`HomeViewDistance()`、`ClipPlanesFor(max_eye_to_target)`。默认视图名为 `3d_front`、`3d_rear`、`3d_left`、`3d_right`。
- **`EGLHeadlessContext`** 以 EGL surfaceless（Mesa）实现桌面 GL 3.3 兼容 Profile 的 `osgViewer::GraphicsWindow`。16×16 pbuffer 只负责持有上下文，真正的绘制全部进 FBO——无窗口、无 X server。

## 7. 深度模块

`src/depth/` 包含球面矫正与**解析地面深度**：

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

`GroundDepthMap` 含 `depth`、`disparity`、`valid` 三张图。`ComputeGroundDepthMap` **不读图像**：对球面网格每格，假设点位于地面平面 `z = ground_z`，射线与该平面求交得到深度 λ，再计算该点在 B 相机中的**预测**视差。

当前状态：该模块只有几何推导，没有立体匹配，也没有测量视差；目前没有生产调用方。它是图像化地面深度估计的基础设施。

## 8. 架构与数据流

`StitchEngine`（`src/core/stitch_engine.cpp`）提供两阶段 API：

```
Configure(std::vector<Camera>, BowlParams, width = 960, height = 640)   一次性
    |   (Camera 持有外参 + 模型；所有权移入引擎)
    |
    | BowlModel(params, cameras)   网格 + 烘焙 uv/weight，CPU
    v
StitchScene(bowl, w, h)            osg::Geometry + GLSL 330
    ^
    | UploadCameraImages -> Texture2DArray（N 层）      每帧
RenderFrame(std::vector<cv::Mat>)
    |
    v
OSGViewRenderer（每个 ViewPose 一个 FBO 相机）
    |
    v
{"3d_front": cv::Mat, "3d_rear": cv::Mat, "3d_left": cv::Mat, "3d_right": cv::Mat}
```

参数校验是显式的：N 不在 [1, 8] 或参数不一致时，`Configure` 抛 `std::invalid_argument`。

## 9. 目录结构

```
src/
├── camera_model/                  相机模型（纯数学）
│   ├── camera_extrinsics.hpp      T_wc 位姿值类（header-only）
│   ├── camera_model.hpp / .cpp    CameraModel 基类、Kb/Lut/Ocam 参数、MakeCameraModel
│   ├── camera.hpp / .cpp          Camera：外参 + 模型作为一体
│   └── kb_ / lut_ / ocam_camera_model.{hpp,cpp}
├── bowl_model/                    碗几何与融合（CPU 预计算）
│   ├── bowl_params.hpp / .cpp     BowlParams + ForCarSize + 网格/视图辅助
│   ├── bowl_model.hpp / .cpp      BowlModel：构造即生成网格并烘焙
│   └── blend_weights.hpp / .cpp   BlendWeightsCalculator（与 N 无关）
├── common/
│   └── angle_utils.hpp            角度归一化/换算辅助
├── depth/                         球面矫正 + 解析地面深度
│   ├── depth_common.hpp
│   ├── spherical_rectification.{hpp,cpp}
│   └── ground_depth.{hpp,cpp}
├── render/                        GPU / OSG
│   ├── stitch_scene.hpp / .cpp    场景图、GLSL 330、Texture2DArray
│   ├── osg_view_renderer.hpp / .cpp  离屏多视图渲染 + 回读
│   └── egl_headless_context.hpp / .cpp  surfaceless EGL GL 3.3
└── core/
    └── stitch_engine.hpp / .cpp   Configure + RenderFrame

examples/
├── avm_viewer.cpp                 交互式碗模型查看器
├── data_4/                        4 路样例（5.27 m 汽车）
└── data_8/                        8 路样例（211.58 m 船舶）

tests/                             15 个 gtest 可执行文件 / 210 个测试用例
├── test_support/                  共享测试基建（header-only）
├── camera_model/                  投影、外参、Camera、工厂
├── bowl_model/                    碗参数、网格/烘焙、融合权重
├── depth/                         球面矫正、地面深度
├── render/                        场景图、渲染器、EGL 上下文
├── common/                        角度工具
└── core/                          引擎端到端 + 回归
```

## 10. 构建与测试

依赖由 vcpkg manifest 提供（`gtest`、`opencv4`（`calib3d`/`jpeg`/`png`）、`osg`、`nlohmann-json`），另加系统 EGL（`find_package(OpenGL COMPONENTS EGL)`）。库本身不依赖 JSON；`nlohmann-json` 仅由 `avm_viewer` 示例读取 `calibration.json` 时使用。

```bash
export VCPKG_ROOT=/path/to/vcpkg
cmake --preset default          # Unix Makefiles，build/
cmake --build build             # 增量编译；不要删 build/
ctest --test-dir build          # 15 个 gtest 可执行文件，共 210 个用例
```

GL 门控用例（`test_stitch_engine`、`test_egl_headless_context`、`test_osg_view_renderer`）会探测 headless EGL 上下文，不可用时 **SKIP**；设置 `AVM_STITCH_TEST_GL=1` 可强制开启。

## 11. 示例：avm_viewer

```bash
./build/examples/avm_viewer ../../examples/data_8
```

`avm_viewer` 用 nlohmann-json 读取 `<dir>/calibration.json`，用 OpenCV 加载图像，经 `BowlParams::ForCarSize` 派生碗，然后打开 OSG 窗口。操作：左键拖动 = 旋转，右键拖动 = 缩放，中键拖动 = 平移。

`calibration.json` 结构：

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

## 12. 集成方式

```cpp
#include "bowl_model/bowl_params.hpp"
#include "camera_model/camera.hpp"
#include "camera_model/camera_model.hpp"
#include "core/stitch_engine.hpp"

using namespace avm_stitch;

std::vector<Camera> cameras;
cameras.emplace_back(CameraExtrinsics{R, t}, MakeCameraModel("kb", kb_params));
// ... N 路，N ∈ [1, 8]

StitchEngine engine;
engine.Configure(std::move(cameras),
                 BowlParams::ForCarSize(5.27, 2.17, 2.9),
                 960, 640);

std::vector<cv::Mat> frames;  // N 路 BGR 图像，顺序与 cameras 一致
auto views = engine.RenderFrame(frames);
// views["3d_front"]、views["3d_rear"]、views["3d_left"]、views["3d_right"]
```

- `Configure` 接管 `Camera` 对象（连带其模型）的所有权。
- `RenderFrame` 的相机顺序必须与 `Configure` 一致。
- 错误以异常表达（配置错误抛 `std::invalid_argument`）。
- 协议适配（队列任务、base64 解码、结果信封、`ParseBowlParams`）位于 **avm-worker**，不在本库。

## 13. 扩展到 8 路以上

几何与融合算法本身与 N 无关；8 路上限只在 GPU 布局和校验中：

| 瓶颈 | 位置 | 现状 |
|---|---|---|
| 顶点属性布局 | `StitchScene` 槽位常量 | UV：4 × vec4（location 2-5）；权重：2 × vec4（location 6-7） |
| 片元着色器 | `StitchScene` 片元源码 | 8 路展开分支 |
| Configure 校验 | `StitchEngine::Configure` | N ∈ [1, 8] |

扩到 16 路需要：

1. UV 重打包为 8 × vec4（location 2-9），权重为 4 × vec4（location 10-13）。GLSL 330 保证至少 16 个顶点属性，0-13 理论安全，但需在目标 GPU 上验证。
2. 着色器展开 16 路分支；`WEIGHT_MIN` 阈值机制可沿用。
3. `Configure` 上限放到 16。
4. **不需要动**：`BlendWeightsCalculator`（半角自动变为 `52/16 = 3.25°`）、`BowlModel`、`UploadCameraImages`。

## 14. 已知限制

- **深度模块只有解析推导** — `src/depth/` 从几何计算地面深度与预测视差，不读图像、无立体匹配、目前无生产调用方。
- **JSON 集成在外部** — 任务契约、后端工厂、结果信封、相机名解析、`ParseBowlParams` 均在 avm-worker（`avm_stitch_integration` / `avm_renderer_bridge`）。本库只暴露 C++ 算法 API。
- **GL 测试依赖 EGL** — GL 门控用例在没有 headless EGL 上下文时 SKIP（`AVM_STITCH_TEST_GL=1` 强制开启）。
- **OSG / vcpkg 怪癖** — `osg_ModelViewProjectionMatrix` 不会自动更新；MVP uniform 必须使用 `osg::Matrixf`（`Matrixd` 会触发 `GL_INVALID_OPERATION`）。升级 OSG 构建时需复测。
- **8 路 GPU 布局上限** — 见[第 13 节](#13-扩展到-8-路以上)。
