# avm-stitch

[English](README.md) | **中文**

把 N 路鱼眼相机拼接成 3D AVM（Around View Monitor / 360° 鸟瞰）环视的 Rust package。

- **纯算法核心** — `camera`、`bowl`、`depth`、`calibration` 模块承载几何与相机数学，零 GPU、零图像库、零 OS 依赖，且不依赖任何 feature。
- **离屏渲染** — `render` feature 用 wgpu 离屏绘制四个视角，可在容器与 CI 中跑在任何 Vulkan/Metal/DX12 适配器上（含软件光栅化器）。
- **N 路相机** — 架构上限 8 路；配置接受 N ∈ [1, 8]。常见部署为 4 路（前/后/左/右）或 8 路。
- **可选窗口** — `viewer` feature 以交互方式打开同一场景，也能只呈现固定帧数并保存截图。

## 目录

1. [它做什么](#1-它做什么)
2. [Package 结构](#2-package-结构)
3. [碗模型](#3-碗模型)
4. [烘焙](#4-烘焙)
5. [相机模型](#5-相机模型)
6. [融合权重](#6-融合权重)
7. [渲染器](#7-渲染器)
8. [引擎与数据流](#8-引擎与数据流)
9. [深度模块](#9-深度模块)
10. [构建与运行](#10-构建与运行)
11. [验证与对拍](#11-验证与对拍)
12. [标定 JSON](#12-标定-json)
13. [扩展到 8 路以上](#13-扩展到-8-路以上)
14. [已知限制](#14-已知限制)
15. [历史](#15-历史)

## 1. 它做什么

AVM 系统在车辆四周安装 N 路鱼眼相机（FOV 通常 > 180°），相邻视角在地面上互相重叠。把它们直接贴进一张平面俯视图行不通：同一块地面在每个相机里的像素密度不同，平面在每个接缝处都会出现可见的拉伸断裂。

业界标准解法是**碗模型**：把车周围的世界建模成一只碗——车底平坦，向边缘平滑抬升。在这张曲面上渲染虚拟相机，远处地面会被自然压缩，于是所有相机图像在接缝处无缝相接。

```
                [Front]
                  F
        +---------+---------+      F/R/L/R = 鱼眼相机
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

流水线分三段：

1. **几何** — 由 `BowlParams` 生成三角网格。
2. **烘焙** — 每个顶点投影进每个相机，在 CPU 上预先算好归一化 UV 与融合权重。
3. **渲染** — N 帧相机图像作为纹理数组上传；WGSL shader 按烘焙 UV 采样每路相机并加权求和。

所有几何与融合开销只发生在配置阶段；逐帧工作只有一次纹理上传和每个视角一次 draw pass。

## 2. Package 结构

整个仓库是一个 Cargo package `avm-stitch`，用 feature 划分边界，而不是拆成多个 crate：

| feature | 内容 | 主要依赖 |
|---|---|---|
| （始终启用） | `camera`、`bowl`、`depth`、`calibration`、`angle_utils` | `nalgebra`、`serde`、`thiserror` — 无 GPU、无图像库、无 OS 依赖 |
| `render` | `render` 模块（wgpu 离屏路径、WGSL shader、顶点打包、纹理回读）与 `engine` | `wgpu`、`naga`、`bytemuck`、`pollster`、`image` |
| `viewer` | `render::windowed` 与 `avm_viewer` 示例 | `winit` |

`default = ["render", "viewer"]`；`avm-cli` 二进制两个 feature 都不需要。

```
src/lib.rs               模块根
src/camera/              相机模型、外参
src/bowl/                碗几何与融合权重
src/depth/               球面校正与地面深度
src/calibration.rs       标定 JSON
src/angle_utils.rs       角度工具
src/engine.rs            StitchEngine                     #[cfg(feature = "render")]
src/render/              wgpu 渲染路径                     #[cfg(feature = "render")]
src/render/windowed.rs   交互窗口                          #[cfg(feature = "viewer")]
src/bin/avm-cli.rs       统计 CLI（只用核心模块）
examples/avm_viewer.rs   交互 viewer                       required-features = ["viewer"]
tests/*.rs               集成测试
tests/fixtures/parity/   对拍参考图
```

依赖方向由测试强制：`cargo tree --no-default-features` 的解析闭包不得出现 `wgpu`、`winit`、`naga`，且 CPU-only 构建不会读取 `src/render` 下的任何文件。

## 3. 碗模型

`avm_stitch::bowl::BowlParams` 是不可变参数对象，拥有几何所依赖的全部输入——网格需要的一切都在它手里，因此它是碗几何的 information expert：

- `generate_block_boat_mesh` — 网格。
- `distance_from_flat_platform`、`bowl_height`、`grid_segments_per_axis` — 径向剖面。
- `default_view_poses`、`home_view_pose`、`clip_planes_for` — 视角辅助。

`BowlParams::for_car_size(car_length, car_width, axis_x)` 由车辆 footprint 推导整套参数。令 `scale = car_length / 5.6`：

| 参数 | 取值 | 说明 |
|---|---|---|
| `flat_platform_radius` | `2.0 × scale` | 车体 footprint 外的一圈圆角余量 |
| `grid_step` | `0.2 × scale` | 网格间距（米）；L = 5.6 时约 126×126 |
| `outer_radius` | `12.6 × scale` | 碗的径向范围 |
| `slope_coefficient` | `0.09 × scale^-1.2` | 边缘坡度 |
| `height_exponent` | `2.2` | 固定剖面形状 |
| `max_height` | `8.6 × scale` | 超过此高度的顶点被裁掉 |
| `car_center` | `(axis_x / 2, 0)` | 后轴坐标系 |

高度剖面与 footprint：

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

同一公式覆盖轿车与轮船：仓库自带 5.27 m 轿车（`data_4`）与 211.58 m 轮船（`data_8`）样例。

`avm_stitch::bowl::BowlModel::new(params, cameras)` 在构造函数里建网格并完成烘焙，不存在半初始化状态。访问器按索引提供（`vertex_count` / `baked_vertex` / `triangle_count`），内部布局保持私有。

## 4. 烘焙

`BowlModel` 的构造函数对每个顶点计算：

- `uv[N]` — 每路相机图像中的归一化像素坐标（`pixel / width`、`pixel / height`）；不可见相机保持 `(0, 0)`。
- `weight[N]` — 每路相机的融合权重，在可见集合上归一化使和为 1；不可见相机为 0。

逐顶点的烘焙流程：

1. 计算方位扇区的融合权重（`BlendWeightsCalculator`）；
2. 可见性门控：落在图像矩形内且落在鱼眼圆内；
3. 可见相机乘以边缘羽化因子；
4. 在“可见且权重非零”的相机上归一化。

`BakedVertex` 按实际相机数 N 分配，而不是按 8 路上限。8 路上限只存在于 GPU 布局与配置校验；shader 只激活前 N 层。

## 5. 相机模型

`avm_stitch::camera::CameraModel` 是三种鱼眼模型共享的 trait：

- 批量与单点投影：`project_points`、`project_camera_point`。
- 门控：`fisheye_circle_radius`、`inside_fisheye_circle`、`edge_feather(u, v, feather_px = 40.0)`。

`edge_feather` 是硬门限而非软衰减：鱼眼图像暗边是纯黑，插值无法恢复，因此圆边缘 `feather_px` 内的像素直接返回 0。

`make_camera_model("kb" | "table" | "ocam", params)` 构造 `KbCameraModel` / `LutCameraModel` / `OcamCameraModel`：

| 模型 | 参数 | 说明 |
|---|---|---|
| `kb` | `KbParams` | width、height、fx、fy、cx、cy、k1..k4、fisheye_radius（0 = 自动内切圆） |
| `table` | `LutParams` | 查表投影 |
| `ocam` | `OcamParams` | OCamCalib 风格多项式 |

kb 模型在 `KbCameraModel::project_camera_point` 里用一份手写的等距鱼眼多项式（`θ = atan2(r_xy, z)`，θ ∈ [0, π]）作为唯一真源；批量循环复用同一公式。

`Camera` 把外参与模型组合成一个整体：

```rust
let camera = Camera::new(extrinsics, make_camera_model("kb", &CameraModelParams::Kb(kb_params))?);
camera.width();
let mut uv = [[0.0, 0.0]; points.len()];
let mut valid = [0u8; points.len()];
camera.project_points(&points, &mut uv, &mut valid)?;  // 相机系点 -> 像素
camera.inside_fisheye_circle(u, v);
camera.edge_feather(u, v, 40.0);
camera.extrinsics();                 // T_wc 位姿
```

`CameraExtrinsics` 保存 T_wc 位姿（相机 → 世界）：行主序 3×3 `R` 加米制平移，相机 y 轴朝下。提供 `to_camera` / `to_world` / `rotate_camera_to_world` / `rotate_world_to_camera` / `position`。

世界点 → 像素是复合 `model.project_camera_point(extr.to_camera(p_world))`：外参管位姿，模型管成像。

## 6. 融合权重

`BlendWeightsCalculator` 实现与 N 无关的方位扇区算法（4 路与 8 路共用同一份代码）：

1. 构造函数把每路相机投影到方位角 `extrinsics().azimuth_deg()`，并对 `(方位角, 相机下标)` 排序。
2. 扇区边界是相邻相机方位角的中点。
3. `compute_blend_half_angle(N) = 52.0 / N` 度是过渡带的半宽。
4. `weights_for(p_world)` 定位地面点的方位角：
   - 落在边界 ± `52°/N` 带内 → 相邻两路相机按余弦 smoothstep 融合（边界处各 0.5）；
   - 落在所有带外 → 最近相机独占，权重 1.0。
5. `normalize_weights(blend_weights, edge_factors, normalized_weights)` 在边缘羽化后于可见相机上重新归一化，黑色鱼眼边缘永远不会进入接缝。

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

## 7. 渲染器

`render` 模块是 package 中唯一接触 GPU 的部分。shader 与绘制路径由离屏渲染和窗口共用。

- **`PackedVertex`** 把烘焙数据打包成一个稠密顶点（120 字节），布局固定为 8 路，shader 因而保持简单：

```
location 0   vec3  position   (VCS -> 渲染基：x,y,z -> x,z,-y)
location 1   vec3  normal
location 2   vec4  uv0 = (u0, v0, u1, v1)
location 3   vec4  uv1 = (u2, v2, u3, v3)    8 路，4 x vec4
location 4   vec4  uv2 = (u4, v4, u5, v5)
location 5   vec4  uv3 = (u6, v6, u7, v7)
location 6   vec4  weights0 = (w0, w1, w2, w3)   2 x vec4
location 7   vec4  weights1 = (w4, w5, w6, w7)
```

- **Shader**（`shader.wgsl`）：片元 shader 显式展开 8 路相机，避免动态 sampler 数组下标。当 `num_cameras > k && w_k > 0.001` 时，每路贡献 `textureSample(camera_textures, camera_sampler, vec3(uv_k, k)).rgb * w_k`。
- **`SceneData`** 把 `BowlModel` 转成顶点、索引与 N 路相机图像；图像成为纹理数组的 N 个层。
- **`GpuSceneRenderer`** 负责管线（颜色格式参数化）、相机数组、顶点/索引缓冲，并为每个 `ViewSpec` 录制一次 pass。深度为 `Depth32Float`，比较函数 `Less`。
- **`render_all_views` / `render_all_views_with`** 创建适配器、离屏渲染每个视角，并用 `TextureReadback` 按 RGBA8 回读。没有适配器时返回 `RenderError::NoAdapter`，不会 panic。
- 视角与裁剪面来自 `BowlParams`：`default_view_poses()`、`home_view_pose()`、`clip_planes_for(max_eye_to_target)`。默认视角名为 `3d_front`、`3d_rear`、`3d_left`、`3d_right`。
- 场景以背景色 `BACKGROUND_CLEAR` = RGB(30, 36, 50) 绘制。

## 8. 引擎与数据流

`avm_stitch::engine::StitchEngine` 暴露两段式 API：

```
configure(Vec<Camera>, &BowlParams, width: u32, height: u32)          一次
    |   (Camera 拥有外参与模型；所有权移入引擎)
    |
    | BowlModel::new                  网格 + 烘焙 uv/weight，CPU
    v
SceneData::from_bowl_model           PackedVertex 缓冲 + 相机图像
    ^
    | render_frame(&[RgbaImage])      逐帧：纹理上传 + 每视角一次 pass
    |
    v
Vec<(String, RgbaImage)>             ("3d_front" .. "3d_right")
```

`width` 与 `height` 是必传参数（Rust 没有默认参数）。校验是显式的：N 不在 [1, 8]、缺少相机模型、图像尺寸不一致、grid step 或 outer radius 非正、坡度系数或高度指数为负时，`configure` 返回 `EngineError`。

## 9. 深度模块

`avm_stitch::depth` 包含球面校正与**解析地面深度**：

```rust
make_rectification_basis(a: &CameraExtrinsics, b: &CameraExtrinsics) -> RectificationBasis;
compute_ground_depth_map(&basis, a, b, &grid, options: GroundDepthOptions) -> GroundDepthMap;
triangulate_depth(dir_a_world, origin_a, dir_b_world, origin_b) -> f64;
```

`GroundDepthMap` 保存 `depth`、`disparity`、`valid` 三张图。`compute_ground_depth_map` **不读取图像**：对球面网格的每个单元，它假定点位于地面 `z = ground_z`，把射线与该平面求交得到深度 λ，再计算该点在相机 B 中的*预测*视差。

状态：纯几何。尚无立体匹配器，也没有实测视差，目前没有生产调用方。它是基于图像的地面深度估计的前置工作。

## 10. 构建与运行

该 package 需要 Rust 工具链（edition 2021，rust-version 1.85）；渲染还需要一个 wgpu 适配器。构建过程不安装任何系统 SDK。

```bash
cargo build
cargo test

# 标定统计：不需要 GPU，也不需要显示环境
cargo run --bin avm-cli -- examples/data_4

# 同样的统计，但完全不编译 GPU 栈
cargo run --no-default-features --bin avm-cli -- examples/data_4

# 交互窗口：需要显示环境（X11 或 Wayland）
cargo run --example avm_viewer -- examples/data_4
```

`avm-cli` 打印相机、派生碗参数、网格规模与烘焙权重范围。`avm_viewer` 示例以 1280×720 打开同一场景的 home 视角；左键拖动旋转，右键拖动缩放（dolly），中键拖动平移，滚轮缩放。

脚本化运行支持：

```bash
cargo run --example avm_viewer -- examples/data_4 --frames 2
cargo run --example avm_viewer -- examples/data_4 --frames 1 --screenshot shot.png
```

`--frames N` 呈现 N 帧后退出；`--screenshot PATH` 把第一帧写成 PNG，若始终无法取帧则以非零退出码结束。没有显示环境时 viewer 会以明确信息非零退出。

## 11. 验证与对拍

```bash
cargo test --test parity
```

对拍测试以 960×640 渲染 `examples/data_4`，把四个视角与 `tests/fixtures/parity/data_4/` 中的参考 PNG 比较。逐通道要求 PSNR > 45 dB、平均绝对差 < 1.0；当前夹具最差通道为 86+ dB。参考图取自原始渲染器，作为外部真值保留，因此任何改变输出的 shader 或矩阵改动都会在这里失败。

依赖 GPU 的测试在机器没有适配器时跳过；对拍测试故意失败而不跳过，避免无 GPU 的运行悄悄声称通过。

## 12. 标定 JSON

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

由 `avm_stitch::calibration::Calibration::load_from_path` 读取；`bowl_params()` 从 `car` 段派生碗参数。

## 13. 扩展到 8 路以上

几何与融合权重算法与 N 无关。8 路上限只存在于 GPU 布局与校验：

| 瓶颈 | 位置 | 现状 |
|---|---|---|
| 顶点属性布局 | `avm_stitch::render::vertex` 槽常量 | UV：4 × vec4（location 2-5）；权重：2 × vec4（location 6-7） |
| 片元 shader | `shader.wgsl` | 8 段展开的相机分支 |
| 配置校验 | `StitchEngine::configure` | N ∈ [1, 8] |

扩展到 16 路：

1. 把 UV 重排为 8 × vec4、权重重排为 4 × vec4，并扩展顶点布局。
2. 展开 16 段 shader 分支；`w_k > 0.001` 阈值仍适用。
3. 把配置上限提高到 16。
4. `BlendWeightsCalculator`（半角变为 `52/16 = 3.25°`）、`BowlModel` 与纹理数组上传均无需改动。

## 14. 已知限制

- **窗口画面只由冒烟测试检查** — 有显示环境时，`tests/windowed_smoke.rs` 会把截图与同视角离屏渲染逐字节比较；无显示且无 `xvfb-run` 时，测试会打印原因并明确跳过。
- **深度模块仅解析** — 它从几何计算地面深度与预测视差；不读取图像，没有立体匹配器，目前没有生产调用方。
- **QNX 未验证** — 尚未在目标工具链上运行；核心模块在构造上不依赖 OS，但 GPU 路径假定存在 Vulkan/Metal/DX12 适配器。
- **对拍夹具只覆盖一个数据集** — `data_4`、960×640、四个视角。其他尺寸、数据集与相机数没有参考图。
- **8 路 GPU 布局上限** — 见[第 13 节](#13-扩展到-8-路以上)。
- **延后的重构** — 迁移期已知债务记录在 [`docs/known-debt.md`](docs/known-debt.md)。

## 15. 历史

本仓库由一份 C++20/OSG 实现迁移而来。该实现已从代码树中删除，保留在 tag `archive/cpp-implementation`，连同参考图生成工具一起，供需要对照行为或重建夹具时取用。迁移测试映射记录在 `docs/migration-test-mapping.md`，逐条说明每个原始用例是移植还是豁免。
