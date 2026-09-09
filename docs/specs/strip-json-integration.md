# JSON 集成层剥离（avm-stitch / avm-worker 双仓库第三次搬迁）设计规范

> **状态:** 草稿

**目标:** 将 avm-stitch 中的 JSON 集成层（backend 接口/工厂、OSG backend 的 JSON 信封适配、结果构建、相机标签解析、碗参数 JSON 解析）**搬迁**（非删除）到其唯一消费者 avm-worker，使 avm-stitch 成为第三方中立拼接库——只暴露纯算法 API（结构体 / `cv::Mat` 进出），库内 JSON 依赖归零。

**架构:** avm-stitch 移除 nlohmann-json 依赖，对外只剩 `StitchEngine`（`Configure` / `RenderFrame`）+ geometry / render / view_config 纯算法层；JSON 门面搬迁件（core 3 对 + util 1 头）镜像落位到 avm-worker 的 `src/` 树内，由新的本地静态库 target `avm_stitch_integration` 承载（与既有的 `avm_renderer_bridge` 平级）；`BowlParams::FromJson` 降为 worker 侧单一编译单元内的自由函数。namespace 均为扁平 `avm_stitch` 不变，include 字符串零改动。

**技术栈:** CMake ≥ 3.25 / vcpkg manifest、OpenSceneGraph、OpenCV、GTest（剥离后 avm-stitch）；nlohmann-json、hiredis、avm-renderer 私有 registry port（worker 独有）。

---

## 1. 背景与动机

avm-stitch 的定位（第三次重申）是**第三方中立拼接库**：只提供纯算法 API，结构体 / `cv::Mat` 进出。JSON 任务输入输出、backend 工厂是 avm-studio 项目特有的**集成层**，归属 avm-worker。这是继 renderer 代理搬迁（`docs/specs/strip-renderer-path.md`，同构先例，模式全部沿用）之后的第二步清理。

当前 avm-stitch 内仍有如下 JSON 足迹，构成对外契约的"集成层"部分：

- `src/core/stitch_backend.{hpp,cpp}` — JSON 进 / JSON 出的 backend 抽象 + `CreateStitchBackend` 工厂；
- `src/core/osg_backend.{hpp,cpp}` — 把 JSON 任务拆解后喂给 `StitchEngine` 的适配器（base64 图解码、错误信封构建）；
- `src/core/stitch_result.{hpp,cpp}` — 拼接结果 → `stitch_result` JSON 信封构建（内部使用 eval_metrics）；
- `src/util/camera_name.hpp` — 任务 JSON 里的相机 id/label 解析（`ResolveCameraName` / `ResolveCameraLabel`）;
- `src/geometry/bowl_model.hpp` 中混入的 `BowlParams::FromJson`（唯一 JSON 成员）；
- `src/common/base64.h`、`src/eval_metrics/eval_metrics.hpp` — 仅被上述集成层使用的工具副本。

这些代码使 avm-stitch 无法以"仅含算法依赖"的姿态发布给第三方，且 `base64.h` / `eval_metrics.hpp` 在 worker 侧存在逐字节相同的副本（已 diff 实测），是 renderer 剥离时搁置的同名头 shadowing 遗留问题。

**现状实测结论（已验证）:**

- **依赖闭包:** 搬迁件 → 保留件仅 2 条单向 include 边（`osg_backend.cpp` → `core/stitch_engine.hpp`、`geometry/bowl_model.hpp`），全仓 grep 反向零引用、无环；worker 链接 `avm_stitch` 后搬迁件的 include 全部可解析。
- **FromJson 调用点唯一:** `src/` 中 `BowlParams::FromJson` 仅被 `osg_backend.cpp` 一处调用，降为匿名 namespace 自由函数不破坏任何其他 TU。
- **`StitchEngine` 本身零 JSON:** 其接口以 `BowlParams` 结构体、intrinsics/extrinsics、`cv::Mat` 进出，不触碰 nlohmann 类型。
- **worker 消费者 include 串全为相对路径:** `renderer_backend.hpp`（继承 `StitchBackend`）、`stitch_eval.cpp`（调 `CreateStitchBackend` / `ResolveCameraLabel` / `BuildStitchResult`）、`calib_files_generator.cpp`（调 `ResolveCameraLabel`）、`renderer_backend.cpp`（调 `BuildStitchResult`），依赖方向为 bridge/worker → avm_stitch 的 PUBLIC include 根；搬迁后这些字符串原样解析到 worker 本地新副本。
- **同名副本已核实:** worker 的 `src/common/base64.h`、`src/eval_metrics/eval_metrics.hpp` 与 avm-stitch 侧逐字节相同；avm-stitch 侧引用它们的文件（`osg_backend.cpp`、`stitch_result.cpp`）全部在搬迁集合内，删除即去重。
- **测试足迹收敛:** avm-stitch 全部 9 个测试文件中，仅 `test_stitch_engine_regression.cpp` 与 `test_bowl_model.cpp` 两个引用 JSON/搬迁件，其余测试文件零 JSON 引用（已实测），原样保留。
- **下游唯一:** 除 avm-worker 外无其他消费方（用户确认）。

---

## 2. 目标 / 非目标

### 目标

1. avm-stitch 源码、构建、测试、文档中**不含任何 JSON 集成层代码与 nlohmann-json 依赖**；剥离后库只导出纯算法 API。
2. JSON 门面功能完整搬迁到 avm-worker，`queue:stitch` 各 `algorithm` 取值（osg / renderer / subprocess / 未知）的对外行为**零变化**。
3. 消除 `base64.h` / `eval_metrics.hpp` 双副本：avm-stitch 侧删除，worker 侧既有副本成为唯一事实源。
4. avm-stitch README 从中立库视角同轮改写，不再以 JSON 契约为主集成叙事。

### 非目标

- 不重构搬迁代码内部实现；搬迁即"原样搬家"（`FromJson` 由成员函数降为匿名 namespace 自由函数是唯一形态调整，解析语义逐字保留）。
- 不改变任何对外协议（redis 任务 JSON、`algorithm` 取值、`stitch_result` / `stitch_error` 信封格式）。
- 不让 `CreateStitchBackend` 工厂吸收 `"renderer"` 名字或感知 bridge——worker dispatch 特判维持现状。
- 不改变 namespace、不改 worker 测试的业务断言。
- 不为"未来 avm-stitch 提供可选 JSON 门面"预留任何接口或编译开关。

---

## 3. 目标架构

### 3.1 两仓库职责（剥离后）

```
avm-stitch（第三方中立拼接库，仅算法）
  src/core/     stitch_engine（Configure / RenderFrame, 纯结构体 API）
  src/render/   osg_scene osg_view egl_headless_context
  src/geometry/ projection bowl_model(去 FromJson) mesh_bake blend_weights
  src/util/     view_config
  依赖: opencv4, osg, gtest（vcpkg.json 移除 nlohmann-json）

avm-worker（消费者 + JSON 集成层宿主）
  src/core/     ← 迁入: stitch_backend osg_backend stitch_result
  src/util/     ← 迁入: camera_name.hpp（新建目录）
  avm_stitch_integration 静态库 = 上述迁入 .cpp 集合（osg_backend.cpp 内嵌 ParseBowlParams）
  avm_renderer_bridge     静态库（不变，renderer 代理宿主）
  src/stitch_eval/        dispatch / 流式（include 串与行为不变）
  依赖: avm_stitch_integration → avm_stitch + nlohmann-json; bridge → avm_renderer(私有 registry) ...
```

### 3.2 组件详情

#### avm_stitch 静态库（变更后）

- **职责:** 纯拼接算法。导出 `StitchEngine::Configure(intrinsics, extrinsics, BowlParams, ...)` / `RenderFrame(...)` 及 geometry / render 层工具；数据类型以结构体与 `cv::Mat` 表达，头文件树不含任何 JSON 类型。
- **模块边界:** 不再导出 `StitchBackend` / `CreateStitchBackend` / `BuildStitchResult` / `ResolveCameraLabel` / `BowlParams::FromJson`——这些符号整体消失，由 worker 本地副本提供。
- **依赖:** OpenCV、OSG、EGL、GTest；configure/构建不再需要 nlohmann-json。

#### avm_stitch_integration 静态库（avm-worker 新增）

- **职责:** 承载 JSON 集成层：`StitchBackend` 抽象与工厂、`OSGBackend`（JSON ↔ `StitchEngine` 适配）、`BuildStitchResult` 信封构建、`ResolveCameraName` / `ResolveCameraLabel`。namespace 保持 `avm_stitch` 不变。
- **对外契约:** 与搬迁前 avm-stitch 的 JSON 门面完全一致（工厂只认 `"osg"`，未知名返回 `nullptr`）。
- **依赖:** `avm_stitch`（PUBLIC，`osg_backend.cpp` 需要 `core/stitch_engine.hpp`、`geometry/bowl_model.hpp`，并传递 OpenCV/OSG）、`nlohmann_json::nlohmann_json`（**PUBLIC**——消费者头文件如 `stitch_backend.hpp` 暴露 json 类型于签名中）。
- **设计决策:** 与 `avm_renderer_bridge` **平级而非并入**——OSG 集成路径与 avm-renderer 无关，塞入 bridge 会把私有 registry 依赖强加给只想用 OSG 分支的消费者 TU，且符号边界（renderer 代理 vs 本项目自有集成层）会糊掉。

#### StitchEvalDispatch（avm-worker，零改动）

- **职责:** 算法分派。`"osg"` → `avm_stitch::CreateStitchBackend("osg")`（工厂仍在，只是宿主换为本地 integration 库）；`"renderer"` → bridge 本地直调（上次搬迁已就位）；`"subprocess"`/未知 → 本地管道。include 字符串、dispatch 逻辑、错误语义全部不动。

#### StitchEngine（avm-stitch 保留件，零改动）

- **职责:** 算法核心，接口不变；`BowlParams` 继续以结构体形式进出。它"不知道" JSON 的存在——这正是剥离的验收切面。

---

## 4. 关键设计决策

| # | 决策 | 理由 |
|---|------|------|
| D1 | `CreateStitchBackend` 工厂**原样搬迁**，不吸收 `"renderer"` 名；worker dispatch 特判维持现状 | 已拍板。工厂语义不变 = `test_stitch_backend.cpp` 的工厂/dispatch 用例断言零修改；avm-stitch→bridge 的知情方向也不回灌。行为零变化是本次搬迁的最高优先约束。 |
| D2 | `BowlParams::FromJson` **降为 worker 侧 `osg_backend.cpp` 匿名 namespace 自由函数**（如 `ParseBowlParams(const nlohmann::json&)`），`bowl_model.hpp` 保留 `Default()` / `BowlMesh` / 网格数学，删除 nlohmann include | 已拍板。全仓实测 FromJson 调用点唯一（osg_backend.cpp），成员函数形态纯属历史惯性；降级后消除"worker 侧函数 ↔ avm-stitch 头文件声明"的跨 TU 契约，bowl_model.hpp 回归纯数学头。 |
| D3 | worker 新建静态库 **`avm_stitch_integration`**（源 = 迁入的 3 个 `.cpp`），与 `avm_renderer_bridge` 平级；OSG 路不依赖 avm_renderer，**不塞进 bridge** | 已拍板。bridge 是 avm-renderer 代理宿主、硬依赖私有 registry；integration 只依赖 avm_stitch + nlohmann-json。边界对应"renderer 代理"与"本项目 JSON 集成层"两种性质。库化（而非直接列进可执行）沿用上次 D3 的理由：多个测试 target 复用同一批符号。 |
| D4 | 搬迁文件**镜像 avm-stitch 相对目录结构**落 worker 的 `src/core/`（3 对）与 `src/util/`（1 头，目录新建）；namespace 保持扁平 `avm_stitch` | 沿用上次 D1/D2 模式（同构先例已被验证）：worker 消费者与测试的 `#include "core/stitch_backend.hpp"` 等相对字符串**全部零改动**，剥离后自动解析到本地副本。 |
| D5 | `src/common/base64.h`、`src/eval_metrics/eval_metrics.hpp` 在 avm-stitch 侧**直接删除**（非"搬迁"动作） | worker 已有逐字节相同副本（diff 实测）；avm-stitch 侧引用者全在搬迁集合内。删除 = 去重，并顺手消除上次剥离搁置的 base64 同名头 shadowing 隐患（搬迁后 avm-stitch `src/` 仅剩 stitch_engine，无同名头残留）。 |
| D6 | 测试分流按**用例**粒度：avm-stitch `test_stitch_engine_regression.cpp` 收缩为 3 个中立用例薄壳（自包含检查用例需删 `stitch_backend.hpp` 与 nlohmann include）；3 个工厂/信封用例删除——worker `test_stitch_backend.cpp`（30 用例）已更强覆盖同批行为 | 用例级 grep 实测的分流边界。保留中立侧回归壳的意义在于持续验证"公开头树自包含且无 JSON"这一剥离不变量。 |
| D7 | `test_bowl_model.cpp` 的 7 个 FromJson 用例随迁 worker（并入 `test_stitch_backend.cpp` 或新文件，**具体落位 plan 阶段定**）；`Default` / `CreateBowlMesh` / 网格数学用例留 avm-stitch 并删 nlohmann include | FromJson 语义（优先级路径、数字容错、非数字回退默认不抛）是集成层契约，测试应跟着契约走；网格数学测试是算法层资产。 |
| D8 | 执行顺序：先剥 avm-stitch 工作树 → worker 落位 → 合并工作区统一构建验证 → avm-stitch 先提交、worker 后提交，均不 push | 沿用上次 D8 模式，规避过渡期双份同名符号的 ODR/重复符号风险；验证顺序不可反转（见 §7）。 |
| D9 | avm-stitch README **同轮改写**：§8 JSON 契约章节大幅收缩为"算法 API + 指向 worker 的集成说明"（核心示例改为 `StitchEngine::Configure` + `RenderFrame`，§8 已有雏形）；§1 特性表、§4 架构图（去掉 JSON 门面层）、§5 目录结构、§7 依赖（去 nlohmann）、§10 已知限制（删 eval_metrics/base64 相关条目）同步 | 文档与代码不同轮更新会造成"中立库仍以 JSON 契约示人"的叙事漂移。 |

---

## 5. avm-stitch 侧变更清单

**删除文件（9）:** `src/core/{stitch_backend,osg_backend,stitch_result}.{hpp,cpp}`、`src/util/camera_name.hpp`（共 7 文件，搬迁至 worker，见 §6）；`src/common/base64.h`、`src/eval_metrics/eval_metrics.hpp`（去重删除，D5；`common/`、`eval_metrics/` 目录随之清空移除）。

**修改:**

- `src/geometry/bowl_model.hpp`: 删 nlohmann include 与 `BowlParams::FromJson` 成员（连同其契约注释）；保留 `Default()`、`BowlMesh`、`CreateBowlMesh`、`detail::ComputeBowlNormals`。
- `CMakeLists.txt`: 删 `find_package(nlohmann_json ...)`；`avm_stitch` 源列表移除 `stitch_backend.cpp` / `osg_backend.cpp` / `stitch_result.cpp` 三项；`avm_stitch` 的 PUBLIC 链接移除 `nlohmann_json::nlohmann_json`（**切断对消费者的 JSON 传递依赖**，这是"第三方库不带 json"承诺的机制保证）；`test_bowl_model` / `test_osg_scene` / `test_stitch_engine` 三个测试 target 删 nlohmann 链接；`test_stitch_engine_regression` target 收缩为 3 用例薄壳。
- `vcpkg.json`: 移除 `nlohmann-json` 条目。
- `tests/core/test_stitch_engine_regression.cpp`: 删 `stitch_backend.hpp` 与 nlohmann include；删 3 个 `StitchBackendFactory` 用例（含 renderer-returns-nullptr 契约用例——该契约随工厂移家，断言归 worker 维护）。
- `tests/geometry/test_bowl_model.cpp`: 删 nlohmann include 与 7 个 `FromJson_*` 用例（随迁 worker，D7）。
- `README.md`: 按 D9 改写。

**保留（不动）:** `src/core/stitch_engine.{hpp,cpp}`、`src/render/` 全部、`src/geometry/` 其余件、`src/util/view_config.hpp`；其余测试文件原样保留。

剥离后库内容 = `StitchEngine`（Configure / RenderFrame）+ geometry 4 件 + render 3 件 + `view_config`——纯算法，依赖只剩 OpenCV / OSG / EGL / GTest。

---

## 6. avm-worker 侧变更清单

**落入（7 文件，按 D4 镜像路径）:** `src/core/{stitch_backend,osg_backend,stitch_result}.{hpp,cpp}`（3 对） + `src/util/camera_name.hpp`（`src/util/` 目录新建）；每文件加 provenance 头注（源仓库/源 commit/日期，沿用上次 D7 做法）。

**`src/core/osg_backend.cpp` 落地时的唯一内容调整（D2）:** 原 `BowlParams::FromJson` 逻辑改写为本文件匿名 namespace 自由函数 `ParseBowlParams(const nlohmann::json&)`，原调用点改指它；解析语义逐字保留。

**新增 `avm_stitch_integration` 静态库 target（D3）:**

- 源 = 迁入的 `stitch_backend.cpp` + `osg_backend.cpp` + `stitch_result.cpp`（`camera_name.hpp` 为 header-only，随 include 根导出）；
- PUBLIC include 根 = worker `src`（与 bridge 同构，保证镜像路径的相对 include 成立）;
- 链接 = `avm_stitch`（传递 OpenCV/OSG）+ `nlohmann_json::nlohmann_json` PUBLIC（消费者需要 json 类型）。

**修改:**

- `CMakeLists.txt`: 新增上述 `find_package` 复用与库 target；`avm-worker` 可执行与所有触达 JSON 门面的测试 target 增加链接 `avm_stitch_integration`——至少含 `test_stitch_backend`（其工厂/OSGBackend/camera_name 用例现直接引用迁入符号）、编译 `stitch_eval.cpp` / `renderer_backend.*` / `calib_files_generator.cpp` 的 target（这些源文件 include 相对字符串不变，符号改由 integration 提供）。
- `tests/avm_stitch/test_stitch_backend.cpp`: 30 用例断言零修改；接收 `test_bowl_model` 迁来的 7 个 FromJson 用例（并入本文件或新文件，plan 阶段定，D7）。
- `vcpkg.json`: 不动（nlohmann-json 本已声明）。

**零改动确认（预期即验证项）:** `renderer_backend.hpp` / `renderer_backend.cpp` / `stitch_eval.cpp` / `stitch_stream.cpp` / `calib_files_generator.cpp` 的 include 字符串与 dispatch 逻辑均不动，剥离态下编译通过即证明 D4 镜像落位成立。

---

## 7. 执行顺序与符号冲突规避

**冲突机理:** namespace 均为扁平 `avm_stitch`，过渡期内若 avm-stitch 库与 integration 库同时导出 `CreateStitchBackend` / `BuildStitchResult` 等完全同名符号，且同名头两处定义，将出现 ODR/重复符号与 include 解析二义（上次剥离 R1 的同款机理）。

**顺序（D8，单趟，无中间可构建态）:**

1. **avm-stitch 剥离:** 工作树完成 §5 全部删除/修改 → 单仓构建 + 保留测试全绿（此时 avm-stitch 自证"无 JSON 依赖"成立）。
2. **worker 落位:** 工作树完成 §6 全部落位/建库 → 在 avm-stitch 已是剥离态的工作区构建 worker + 全量 ctest。
3. **合并工作区统一验证**（§11 两仓标准均通过）后分别提交：avm-stitch commit 在前、worker commit 在后（worker commit message 记录来源），均不 push。

**验证顺序不可反转:** avm-stitch 未剥离前，worker 不得出现第二份同名符号源。

---

## 8. 数据流（重构后）

```
redis task { algorithm }
  └─ StitchEvalDispatch (worker, 零改动)
       ├─ "subprocess"(默认)/未知 → 本地管道 → stitcher 子进程 → stitch_result JSON
       ├─ "osg"     → CreateStitchBackend("osg")        ← avm_stitch_integration [D1/D3]
       │                └─ OSGBackend::Run(task_json)
       │                     ParseBowlParams(task_json) → BowlParams   [D2]
       │                     StitchEngine::Configure/RenderFrame        ← avm_stitch(纯算法)
       │                     BuildStitchResult(...) → stitch_result / stitch_error JSON
       └─ "renderer"→ bridge 本地构造 RendererBackend(RealRenderer) → avm_renderer   (不变)

avm-stitch 对外 API（第三方视角）:
   struct 进 (intrinsics / extrinsics / BowlParams) → StitchEngine → cv::Mat 出
   —— 无 JSON、无信封、无工厂
```

## 9. 错误处理

- 工厂未知名（含 `"renderer"`）→ 返回 `nullptr`——契约原样搬迁，worker dispatch 对 `nullptr` 的既有回退/报错语义不变。
- `OSGBackend::Run` 的 JSON 缺字段错误信封（如缺 `images` → `stitch_error`）随 `osg_backend.cpp` 整体搬迁，文案与时机零变化。
- `ParseBowlParams` 保持"非数字字段回退默认、绝不抛异常"语义（原 FromJson 契约逐字保留）。
- avm-stitch 侧若剥离有遗漏（残留 json include），vcpkg.json 无 nlohmann-json + 无 `find_package` 会使 configure/编译直接失败——遗漏不可能静默通过。

## 10. 测试策略

搬迁即"原样搬家"：worker 测试（30 工厂/dispatch/信封/camera_name 用例 + 迁来的 7 个 FromJson 用例）全绿即验证集成行为保真；avm-stitch 保留测试（含 3 用例回归薄壳 + 去 JSON 后的 bowl 网格用例）全绿即验证算法层无过切、公开头树无 JSON 自包含。验收基线 = 剥离前后 dispatch 行为矩阵（osg / renderer / subprocess / 未知值）输出信封一致。

## 11. 成功标准

**avm-stitch（中立库自证）:**
```bash
cmake --preset default && cmake --build build      # vcpkg.json 已无 nlohmann-json, configure 通过
ctest --test-dir build --output-on-failure          # 保留测试全绿
grep -rni "nlohmann\|json" src/ tests/ CMakeLists.txt vcpkg.json   # 无任何命中（大小写均计）
```
另加：全新干净目录 configure 验证仅公共 registry + 无 JSON 依赖即可自举。

**avm-worker（合并工作区，avm-stitch 为剥离 HEAD）:**
```bash
cmake --preset default && cmake --build build
ctest --test-dir build --output-on-failure
# 预期: 原全部测试 target + test_stitch_backend(30+7 用例) 全绿
grep -rn "avm_stitch_integration" CMakeLists.txt    # 新库 target 存在且被消费者/测试链接
```
行为级：向 `queue:stitch` 投 `algorithm:"osg"` 与 `"renderer"` 任务，结果信封与剥离前一致。

## 12. 风险与缓解

| # | 风险 | 缓解 |
|---|------|------|
| R1 | 过渡期双份同名符号（`avm_stitch` 与 integration 均导出工厂/信封符号）ODR/重复符号 | D8 顺序：先剥 avm-stitch、worker 后建库；合并工作区统一构建验证（§7） |
| R2 | worker 某消费者/测试 target 遗漏链接 `avm_stitch_integration` → 相对 include 解析失败（avm-stitch 侧已无此头） | 链接关系在剥离态工作区构建，缺链直接编译/链接报错，不会静默指回旧头；§6"零改动确认"清单逐一构建验证 |
| R3 | `nlohmann_json` 从 `avm_stitch` PUBLIC 链接整体移除，影响假想的既有下游 | 已确认下游仅 avm-worker（用户拍板）；worker 自带 nlohmann 声明，不依赖 avm_stitch 传递 |
| R4 | `ParseBowlParams` 改写引入语义漂移（优先级路径/容错行为变化） | 逐字搬迁 + 迁去的 7 个 FromJson 用例作为契约回归网（D7） |
| R5 | 同名头 base64/eval_metrics 若 avm-stitch 侧删除有漏网引用 → 构建失败 | 已实测引用者（osg_backend.cpp / stitch_result.cpp）全在搬迁集合；§11 的 grep 门槛兜底 |
| R6 | 跨仓库搬迁丢 git blame 历史 | provenance 头注 + worker commit message 记录源 commit（沿用上次 D7 实践，代价可接受） |
| R7 | README 改写与代码剥离不同步，叙事残留 JSON 契约 | D9 同轮改写；列入 avm-stitch 提交前检查项 |

## 13. 约束与假设

- **硬约束:** 两仓库并列、独立远端，跨仓库 `git mv` 不可行；namespace 扁平 `avm_stitch` 全程不变（两仓刚统一，无并行任务冲突）；redis 协议与 `algorithm` 取值集合不变；worker dispatch 逻辑零改动；两仓提交但均不 push。
- **假设:** avm-stitch 唯一下游是 avm-worker（用户确认），故 PUBLIC 链接切断 nlohmann-json 无破坏面；osg_backend.cpp 对 stitch_engine / bowl_model 的两条依赖边由 `avm_stitch_integration → avm_stitch` 链接单向满足，无反向需求（已实测零反向引用）。
- **不做:** 不合并 worker 侧既有 `ResolveCameraLabel` 调用点为任何新抽象；不为 avm-stitch 添加"可选 JSON 门面"编译开关；不改 `test_stitch_backend.cpp` 既有 30 用例的断言。
