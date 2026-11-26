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

## 本批次 B1 故事清单与 AC

| 故事 | 内容 | AC |
|------|------|----|
| S1 | workspace 骨架（三个 crate + 依赖约束） | A1, A2, A9 |
| S2 | camera 模块（外参、kb/table/ocam、门控、Camera 组合、工厂） | A3, A5, A6 |
| S3 | bowl 模块（BowlParams、BlendWeightsCalculator、BowlModel 烘焙） | A4, A5, A6 |
| S4 | 标定 JSON 加载 + CLI | A8 |
| S5 | wgpu 离屏渲染骨架（顶点打包、WGSL、naga 校验、render_all_views） | A7 |

批次标注：B1 = S1 → S2 → S3 → S4 → S5，单批次单 commit。
后续批次（不在本批次）：C++ 测试未移植项对齐、QNX 目标、窗口渲染器、性能对齐。
