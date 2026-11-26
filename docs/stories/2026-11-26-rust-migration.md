# Epic: avm-stitch 算法核心与渲染骨架移植到 Rust

## Epic 目标

把 avm-stitch 的 C++ 算法核心（camera model / bowl model）与渲染骨架（OSG + GLSL 330）
按行为对等原则移植为 Rust workspace，目标平台为 Linux 桌面。C++ 源码在本 Epic 全程保留，
作为规格来源与参考实现；C++ 文件不改动、不删除。

验收方式：`cargo build --workspace` 通过 + `cargo test --workspace` 全绿。

## 非目标（本 Epic 明确不做）

- 不删除/改写 C++ 源码、CMake 构建、vcpkg 依赖。
- 不做功能扩展：不新增 CameraRig、不改标定语义、不做多线程/异步调度、不做窗口渲染器。
- 不支持 QNX（后续批次另立故事）。
- 不引入 OpenCV / OSG 依赖；不使用 `image` 以外的图像 IO。
- 不改 `examples/data_4`、`examples/data_8` 数据。

## 契约（后续批次必须遵守）

- crate 边界：
  - `avm-core`（lib）：纯算法 + 标定解析。零 GPU、零 OpenCV、零 OSG、零 OS 依赖。
  - `avm-render`（lib）：wgpu + WGSL，只依赖 `avm-core` 的数据结构。
  - `avm-cli`（bin）：读 calibration.json，跑烘焙，打印统计。
- 依赖白名单：`nalgebra`（数学）、`serde`/`serde_json`（JSON）、`thiserror`（错误）、
  `image`（图像 IO）、`wgpu`/`naga`（仅 render）、`bytemuck`（仅 render）。
- 数值行为与 C++ 一致，包含退化输入（如全 0 默认 `BowlParams`）；Rust 可用
  `Result`/`Option` 表达非法构造，但不得改变 C++ 已定义的行为。
- 命名：C++ 的 `PascalCase` 方法映射为 Rust `snake_case`；`Camera` 携带 `name`
  （默认空串），`CameraExtrinsics` 保持 VCS→相机外参语义。

## 验收标准

### B1

- **A1 crate 布局与边界**：workspace 含 `avm-core`（lib）、`avm-render`（lib）、
  `avm-cli`（bin）三个 crate。`avm-core` 零 GPU/OpenCV/OSG/OS 依赖——不仅直接依赖，
  `cargo tree -p avm-core` 的传递闭包也不得出现 `wgpu`/`naga`/`image`/`avm-render`；
  `avm-cli` 不得链接 GPU 栈。
- **A2 依赖白名单**：只允许 `nalgebra`（数学，二选一，需说明理由）、`serde`/`serde_json`、
  `thiserror`、`image`（唯一图像 IO）、`wgpu`/`naga`/`bytemuck`/`pollster`（仅 render）。
  不引入 OpenCV。
- **A3 camera 对等**：外参（含 `relative_rotation_from`、`azimuth_deg`）、kb 多项式、
  table LUT、ocam 三类模型；门控保持 C++ 的 0/1 硬门限与 `fisheye_circle_radius`
  自动内切圆；`Camera` 组合 `name`（默认空串）+ 外参 + 模型；模型工厂保持
  `"kb"/"table"/"ocam"` 字符串分派与参数不匹配错误语义。
- **A4 bowl 对等**：`BowlParams::for_car_size` 缩放律、`distance_from_flat_platform`、
  `bowl_height`、`grid_segments_per_axis`、`generate_block_boat_mesh`（含中心差分法线）；
  扇区混合权重（方位角排序、扇区边界、`52/N` 余弦 smoothstep）、`normalize_weights` 的
  edge-factor 归一化与 D3 回退；`BowlModel` 构造即烘焙（uv + weight）。
- **A5 数值一致**：所有 C++ 数值用例容差一致；退化输入行为一致（如全 0 默认
  `BowlParams` 产生 `grid_segments_per_axis()==0`、1 顶点、0 三角形，对齐 glibc
  `lround(NaN)` 截断为 0 的路径），并且不 panic。Rust 可用 `Result`/`Option` 表达非法
  构造，但不得改变 C++ 已定义的行为；无法对齐的点记入 `OPEN_ITEMS`。
- **A6 测试迁移**：C++ 单元测试逐条翻译为 `#[test]`，全 workspace ≥60 个测试通过；
  未翻译用例逐条记入 `OPEN_ITEMS`（附原因）。
- **A7 渲染骨架**：顶点布局与 C++ 一致（位置 3 / 法线 3 / uv 4×vec4 / 权重 2×vec4，
  共 8 路）；WGSL 移植 GLSL 330（`texture_2d_array`、逐路 `weight > 0.001` 硬门限、
  `num_cameras` 只激活前 N 层）；naga 解析 + 校验用例；`render_all_views` 离屏
  FBO → buffer 读回，无适配器时返回 `Err`（不 panic），用例跳过而非失败；
  `num_cameras <= 8` 且 `camera_images.len() == num_cameras`。
- **A8 CLI 可跑**：`cargo run -p avm-cli -- examples/data_4` 打印相机数量、顶点数、
  三角形数、权重和区间；`data_8` 同样可用。
- **A9 C++ 零改动**：不修改 `src/`、`tests/`、`examples/`、CMake 文件；`git status`
  只出现新增 Rust 文件与故事文档。

### B1.5（B1 审查修复）

- **I-1 离屏碗场景必须有内容**：fixture 相机必须真正看向碗（look-at 且鱼眼半径大于
  40 px feather 带），并断言 ① 烘焙结果存在 `weight > 0.001` 的顶点；② 渲染帧在期望
  区域存在非黑像素、角落仍为背景。全黑帧不得通过。
- **I-2 深度约定对齐 wgpu**：投影矩阵使用 `[0,1]` 深度（`m22 = far/(near-far)`、
  `m23 = far*near/(near-far)`、`m32 = -1`）；回归用例断言 `near < z < far` 的点满足
  `0 <= clip.z <= clip.w`，近平面映射 0、远平面映射 1。
- **M-1 `clip_planes_for` 不 panic**：退化为 `near > far` 时按 libstdc++
  `min(max(v,lo),hi)` 语义取 `near = far/1000`（全 0 默认参数得 `near == far == 0`），
  不再触发 `f64::clamp` 的 panic；补默认参数用例。
- **M-2 忽略构建目录**：`.gitignore` 含 `/target`，不依赖 `.git/info/exclude` 的本地补丁。
- **M-3 clippy 干净**：故意复刻 NaN 语义的 `!(a < b)`、`module_inception` 加
  `#[allow]` 与原因注释；`all-targets` 下无 warning/error。
- **M-4 依赖契约加严**：`avm-core` manifest 不含 `avm-render`，且其依赖闭包不含
  `wgpu`/`naga`/`image`。
- **M-5 渲染前置校验**：`num_cameras > MAX_CAMERAS` 与
  `camera_images.len() != num_cameras` 均返回 `Err`（对齐 C++ `Configure` 对 N9 的异常），
  各有一条用例。
- **M-6 ViewPose/ClipPlanes 纯数学用例移植**：`tests/render/test_osg_view_renderer.cpp`
  的 4 视角/名称/fovy 60/共同 target/车辆中心偏移/scale 倍数/HomeViewPose 与手搭值一致
  等用例逐条对齐（不改断言强度），外加 `clip_planes_for` 三条。
- **G-1 外部真值锚点**：`ProjectionTest.MatchesOpenCV` 的期望像素由仓库外一次性
  OpenCV 4.12 `cv::fisheye::projectPoints` 程序算出并硬编码为测试常量，Rust 侧仍零
  OpenCV 依赖。
- **G-2 故事文档补全**：本文件补齐 A1–A9 正文与 B1.5 AC。

## 本批次 B1 故事清单与 AC

| 故事 | 内容 | AC |
|------|------|----|
| S1 | workspace 骨架（三个 crate + 依赖约束） | A1, A2, A9 |
| S2 | camera 模块（外参、kb/table/ocam、门控、Camera 组合、工厂） | A3, A5, A6 |
| S3 | bowl 模块（BowlParams、BlendWeightsCalculator、BowlModel 烘焙） | A4, A5, A6 |
| S4 | 标定 JSON 加载 + CLI | A8 |
| S5 | wgpu 离屏渲染骨架（顶点打包、WGSL、naga 校验、render_all_views） | A7 |

批次标注：B1 = S1 → S2 → S3 → S4 → S5，单批次单 commit。
B1.5 = B1 审查的 I-1/I-2/M-1..M-6/G-1/G-2，单独 commit（不 amend B1）。
后续批次（不在本批次）：QNX 目标、窗口渲染器、性能对齐、`OPEN_ITEMS` 中延后的坏味道。
