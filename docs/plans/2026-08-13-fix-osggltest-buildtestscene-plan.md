# 修复 OSGGLTest 3 个失败测试 — 实现计划

> **设计文档:** [../specs/2026-08-13-fix-osggltest-buildtestscene.md](../specs/2026-08-13-fix-osggltest-buildtestscene.md)
> **状态:** 待执行
> **日期:** 2026-08-13
> **仓库:** avm-stitch (`/home/leehsiang/Documents/avm-studio/avm-stitch`)
> **目标文件:** `tests/core/test_stitch_engine.cpp`
> **改动面:** 仅 `BuildTestScene`（第 103–107 行）+ 一条 `MakeLookAtCameras` 前置声明。零生产代码改动，零断言改动。

---

## Phase 0: 引用验证（已执行，结果如下）

| 引用 | 验证结果 |
|------|---------|
| `tests/core/test_stitch_engine.cpp` 存在 | ✅ 731 行，已通读 |
| `BuildTestScene` 定义于第 103–107 行 | ✅ 与设计文档一致 |
| `MakeSideCameras` 定义于第 49–58 行 | ✅ 仍被 13 处直接调用（行 435/443/455/466/480/488/501/518/528/538/547/559/720） |
| `MakeCoarseBowl` 定义于第 60–65 行 | ✅ 仍被 `BuildTestSceneWithExtrinsics` 第 78 行使用 |
| `MakeLookAtCamera` 定义于第 129–146 行 | ✅ 位于 `BuildTestScene` 之后（声明顺序问题确认存在） |
| `MakeLookAtCameras` 定义于第 149–155 行 | ✅ 位于 `BuildTestScene` 之后；当前无任何前置声明 |
| `CameraExtrinsics` 类型 | ✅ 定义于 `src/geometry/projection.hpp:22`（`cv::Matx33d R; cv::Vec3d t;`），测试文件第 36 行 `using namespace avm_worker::avm_stitch;` 引入 |
| 3 个失败测试位置 | ✅ `OSGViewRenderer_InitializeDoesNotCrash`(349) / `RenderAllViews_Returns5Mats`(358) / `RenderAllViews_CorrectSize`(376) |
| build 目录存在 | ✅ `/home/leehsiang/Documents/avm-studio/avm-stitch/build` 已配置并含 `test_stitch_engine` 二进制（顶层，非 `build/tests/core/`） |
| CMake preset | ✅ `CMakePresets.json` 提供 `default`（vcpkg 工具链，binaryDir=`build`） |
| avm-worker 集成 | ✅ `avm-worker/CMakeLists.txt:22` 通过 `add_subdirectory("${AVM_STITCH_DIR}" "${CMAKE_BINARY_DIR}/avm-stitch")` 引入 |

**冲突检查:** 无冲突现有实现。`MakeLookAtCameras` 无前置声明，`BuildTestScene` 当前用 `MakeSideCameras`，直接改调用会因"先使用后定义"编译失败，必须补前置声明（见 Task 2）。

---

## Phase 1: 文件结构映射

仅修改一个文件，一个函数（+ 一条声明）：

```
tests/core/test_stitch_engine.cpp
├── MakeSideCameras (行 49–58)        [保留，不动]
├── MakeCoarseBowl (行 60–65)         [保留，不动]
├── BuildTestSceneWithExtrinsics (75–101) [保留，不动]
├── ★ 新增: MakeLookAtCameras 前置声明  (插在行 103 之前)
├── ★ BuildTestScene (103–107)        [修改: 换相机生成器]
├── MakeLookAtCamera (129–146)        [保留，不动]
├── MakeLookAtCameras (149–155)       [保留，不动]
└── 3 个 OSGGLTest 测试 (349/358/376) [保留，断言不动]
```

无新建文件，无删除文件。生产代码（`src/render/osg_view.cpp`、`src/core/stitch_engine.cpp`、`src/geometry/mesh_bake.cpp`、`src/render/osg_scene.cpp`）零改动。

---

## Phase 2: 任务分解

本计划为**测试夹具修复**：RED 状态已存在（3 个测试当前因 `has_visible_vertices == false` 失败），GREEN 是改一处夹具代码让 3 个测试转绿，REFACTOR 验证无坏味道 + 全量回归无破坏。每个任务按 TDD 循环结构展开。

---

### Task 1: 建立 RED 基线

**Files:**
- 不修改任何文件（只读操作）。

**行为描述:**
- 输入/输出: 无输入；输出为 `test_stitch_engine` 的 `OSGGLTest.*` 定向测试报告。
- 行为规则: 确保二进制与当前源码一致（增量构建），运行定向测试，捕获 3 个失败及其失败原因。
- 边界约束: 本环境 EGL 无头上下文可用，`DetectGLContext()`（行 324–343）应自动探测成功，测试**实际运行**而非 `GTEST_SKIP`。若被 skip，说明环境探测失败，须先排查 GL（但按上下文预期，不会发生）。
- 副作用: 无（不写文件、不改状态）。
- 不做: 不修改任何代码；不修复测试；不在此阶段试图让测试通过。

**测试策略（RED 判据）:**
- 输入: 运行 `OSGGLTest.*`（11 个测试）。
- 预期: `OSGViewRenderer_InitializeDoesNotCrash`、`OSGViewRenderer_RenderAllViews_Returns5Mats`、`OSGViewRenderer_RenderAllViews_CorrectSize` 3 个 **FAILED**，失败断言均为 `ASSERT_TRUE(ts.has_visible_vertices)`；其余 8 个 OSGGLTest 测试 **PASSED**（它们用 `MakeLookAtCameras` 或直接调用 `BuildTestSceneWithExtrinsics`，不受影响）。

- [ ] **Step 1: 增量构建（确保二进制反映当前源码）**
  ```bash
  cd /home/leehsiang/Documents/avm-studio/avm-stitch
  cmake --build build
  ```
  预期: 编译无错误，`build/test_stitch_engine` 为最新。
  （若 `build/` 不存在——本仓库已存在——先 `cmake --preset default` 配置再构建。）

- [ ] **Step 2: 运行定向测试，观察 RED**
  ```bash
  cd /home/leehsiang/Documents/avm-studio/avm-stitch/build
  ./test_stitch_engine --gtest_filter='OSGGLTest.*'
  ```
  预期输出（关键片段）:
  ```
  [ RUN      ] OSGGLTest.OSGViewRenderer_InitializeDoesNotCrash
  tests/core/test_stitch_engine.cpp:352: Failure
  Value of: ts.has_visible_vertices
    Actual: false
  Expected: true
  [  FAILED  ] OSGGLTest.OSGViewRenderer_InitializeDoesNotCrash

  [ RUN      ] OSGGLTest.OSGViewRenderer_RenderAllViews_Returns5Mats
  tests/core/test_stitch_engine.cpp:362: Failure
  Value of: ts.has_visible_vertices
    Actual: false
  Expected: true
  [  FAILED  ] OSGGLTest.OSGViewRenderer_RenderAllViews_Returns5Mats

  [ RUN      ] OSGGLTest.OSGViewRenderer_RenderAllViews_CorrectSize
  tests/core/test_stitch_engine.cpp:379: Failure
  Value of: ts.has_visible_vertices
    Actual: false
  Expected: true
  [  FAILED  ] OSGGLTest.OSGViewRenderer_RenderAllViews_CorrectSize

  [  PASSED  ] 8 tests.
  [  FAILED  ] 3 tests, listed below:
  [  FAILED  ] OSGGLTest.OSGViewRenderer_InitializeDoesNotCrash
  [  FAILED  ] OSGGLTest.OSGViewRenderer_RenderAllViews_Returns5Mats
  [  FAILED  ] OSGGLTest.OSGViewRenderer_RenderAllViews_CorrectSize
  ```

- [ ] **Step 3: 确认 RED 因正确原因失败**
  - 3 个失败都是 `ASSERT_TRUE(ts.has_visible_vertices)` 失败（`Actual: false`），**不是**编译错误、不是崩溃、不是断言本身写错。
  - 8 个通过测试包含 `OSGViewRenderer_RenderAllViews_DrawsSceneNotJustBackground`（它直接 `BuildTestSceneWithExtrinsics(4, MakeLookAtCameras(4))`）与 `StitchEngineGL_*` 系列 —— 证明 `MakeLookAtCameras` 路径能产生 `has_visible_vertices == true`，问题唯一出在 `BuildTestScene` 内部用了 `MakeSideCameras`。
  - 若出现 `GTEST_SKIP`（整组 skip）而非上述失败，停止并排查 GL 上下文（不在本计划范围内）。

---

### Task 2: 修改 BuildTestScene（GREEN 最小实现）

**Files:**
- Modify: `tests/core/test_stitch_engine.cpp:103-107`（`BuildTestScene` 函数体）
- Modify: `tests/core/test_stitch_engine.cpp:103` 之前插入 `MakeLookAtCameras` 前置声明

**行为描述:**
- 输入/输出: 输入 `int num_cameras`（相机数量）；输出 `TestScene`（含 `has_visible_vertices` 标志）。
- 行为规则: `BuildTestScene` 不再生成「水平侧视」相机，改为生成「俯视地面」的 look-at 相机（`MakeLookAtCameras(num_cameras)`，r=4m、h=2m、看向 z=0 地面中心），使 `BuildTestSceneWithExtrinsics` 烘培出的网格拥有可见顶点（`has_visible_vertices == true`）。
- 边界约束: `num_cameras` 由调用方传入（3 个测试均传 `4`），本函数不校验。
- 副作用: 无（纯构造返回值）。
- 不做: 不修改 `BuildTestSceneWithExtrinsics`、`MakeSideCameras`、`MakeCoarseBowl`、`MakeLookAtCamera`、`MakeLookAtCameras` 的语义；不删除 `MakeSideCameras`（仍被 13 处使用）；不改任何断言；不碰生产代码。

**声明顺序决策（最小 diff）:** 采用**前置声明**方案（设计文档 ADR-5 二选一之选项 1）。`MakeLookAtCamera`（129 行）/`MakeLookAtCameras`（149 行）位于 `BuildTestScene`（103 行）之后，C++ 要求先声明后使用，故在 `BuildTestScene` 前插入一条前置声明。不整体上移函数（上移会产生 ~26 行移动 diff，且无行为收益）。

**精确 diff:**

```diff
--- a/tests/core/test_stitch_engine.cpp
+++ b/tests/core/test_stitch_engine.cpp
@@
- TestScene BuildTestScene(int num_cameras) {
-     std::vector<double> azimuths;
-     for (int i = 0; i < num_cameras; ++i) azimuths.push_back(i * 360.0 / num_cameras);
-     return BuildTestSceneWithExtrinsics(num_cameras, MakeSideCameras(azimuths));
- }
+ // MakeLookAtCameras is defined below (after the scene builders); forward-
+ // declare it so BuildTestScene can call it.
+ std::vector<CameraExtrinsics> MakeLookAtCameras(int num_cameras);
+
+ TestScene BuildTestScene(int num_cameras) {
+     return BuildTestSceneWithExtrinsics(num_cameras, MakeLookAtCameras(num_cameras));
+ }
```

改后第 103–110 行（精确文本）:

```cpp
// MakeLookAtCameras is defined below (after the scene builders); forward-
// declare it so BuildTestScene can call it.
std::vector<CameraExtrinsics> MakeLookAtCameras(int num_cameras);

TestScene BuildTestScene(int num_cameras) {
    return BuildTestSceneWithExtrinsics(num_cameras, MakeLookAtCameras(num_cameras));
}
```

- [ ] **Step 1: 应用最小改动（GREEN 实现）**
  用编辑工具对 `tests/core/test_stitch_engine.cpp` 执行上述 diff：
  1. 将第 103–107 行原 `BuildTestScene`（含 `azimuths` 声明 + 填充循环 + `MakeSideCameras(azimuths)` 调用）替换为直接 `return BuildTestSceneWithExtrinsics(num_cameras, MakeLookAtCameras(num_cameras));`。
  2. 在 `TestScene BuildTestScene(int num_cameras) {` 之前插入前置声明 `std::vector<CameraExtrinsics> MakeLookAtCameras(int num_cameras);` 及其两行说明注释。

- [ ] **Step 2: 编译（确保声明顺序正确、无语法错误）**
  ```bash
  cd /home/leehsiang/Documents/avm-studio/avm-stitch
  cmake --build build
  ```
  预期: 编译零错误零警告（`test_stitch_engine` 重建成功）。
  - 若报「`MakeLookAtCameras` was not declared in this scope」→ 说明前置声明未生效，检查声明是否插在匿名 namespace 内（第 38–306 行）且位于 `BuildTestScene` 之前。
  - 若报重定义 → 说明声明位置错误（插到了定义之后或 namespace 外），修正位置。

---

### Task 3: 验证 GREEN

**Files:**
- 无新增/修改（验证 Task 2 的结果）。

**行为描述:**
- 输入/输出: 输入 `OSGGLTest.*` 过滤条件；输出测试报告。
- 行为规则: 3 个此前失败的测试现在应全部 PASSED；确认 11 个 OSGGLTest 测试全部通过。
- 边界约束: GL 上下文可用（同 Task 1）。若 `DetectGLContext` 自动探测失败导致 skip，用 `AVM_STITCH_TEST_GL=1` 强制运行。
- 副作用: 无。
- 不做: 不改任何代码、不改断言。

**测试策略（GREEN 判据）:**
- `OSGGLTest.OSGViewRenderer_InitializeDoesNotCrash` → PASSED（通过行 352 `has_visible_vertices` 断言后，`Initialize` 不抛异常）。
- `OSGGLTest.OSGViewRenderer_RenderAllViews_Returns5Mats` → PASSED（进入渲染，`views.size() == 5`）。
- `OSGGLTest.OSGViewRenderer_RenderAllViews_CorrectSize` → PASSED（5 个视图均为 960x640 CV_8UC3）。
- 其余 8 个 OSGGLTest → PASSED（不变）。

- [ ] **Step 1: 运行定向测试，观察 GREEN**
  ```bash
  cd /home/leehsiang/Documents/avm-studio/avm-stitch/build
  ./test_stitch_engine --gtest_filter='OSGGLTest.*'
  ```
  预期输出（关键片段）:
  ```
  [ RUN      ] OSGGLTest.OSGViewRenderer_InitializeDoesNotCrash
  [       OK ] OSGGLTest.OSGViewRenderer_InitializeDoesNotCrash
  [ RUN      ] OSGGLTest.OSGViewRenderer_RenderAllViews_Returns5Mats
  [       OK ] OSGGLTest.OSGViewRenderer_RenderAllViews_Returns5Mats
  [ RUN      ] OSGGLTest.OSGViewRenderer_RenderAllViews_CorrectSize
  [       OK ] OSGGLTest.OSGViewRenderer_RenderAllViews_CorrectSize
  ...
  [  PASSED  ] 11 tests.
  ```

- [ ] **Step 2: 验证 GREEN（确认 3 个目标转绿、无连带失败）**
  - 11 个 OSGGLTest 全部 PASSED，0 FAILED。
  - 若出现 `[  SKIPPED ]`（11 个全部 skip），说明 GL 探测失败，改用强制模式重跑:
    ```bash
    AVM_STITCH_TEST_GL=1 ./test_stitch_engine --gtest_filter='OSGGLTest.*'
    ```
  - 若仍有 FAILED → 检查是否改错位置（确认改动落在 `BuildTestScene`，不是 `BuildTestSceneWithExtrinsics`），回到 Task 2 排查。

---

### Task 4: REFACTOR 扫描 + 全量回归验证

**Files:**
- 无新增/修改（验证阶段）。

**行为描述:**
- 输入/输出: 输入全量 ctest；输出通过/失败汇总。
- 行为规则: 扫描本次改动是否引入代码坏味道；运行全量测试确认无回归。
- 边界约束: 无 GL 的测试（`StitchEngineTest`、`test_geometry`、`test_render` 等）应全部通过；GL 门控测试在本环境应运行（非 skip）。
- 副作用: 无（`DumpViews` 测试会写 `/tmp/opencode/stitch-views/*.png`，属测试固有行为，无害）。
- 不做: 不引入新重构（本改动最小，无坏味道）；不改生产代码。

- [ ] **Step 1: REFACTOR 扫描（坏味道检测）**
  - 对照 refactoring skill 坏味道表逐项检查本次改动：
    - 重复代码: 前置声明与定义（行 149）签名重复 —— 属 C++ 惯用的「前向声明 + 定义」成对形式，非可消除的语义重复，**不算坏味道**（消除它需要上移 ~26 行代码，无行为收益，反增 diff）。
    - 其余（过长方法/过大类/过长参数列表/依恋情结/switch/基本类型偏执/消息链/中间人/过度亲密/注释/嵌套过深）: 本次改动不触发任何一项。
  - **结论: `REFACTOR_SKIPPED` — 未检测到需消除的代码坏味道。** 不执行任何重构。

- [ ] **Step 2: 全量回归（avm-stitch 独立 ctest）**
  ```bash
  ctest --test-dir /home/leehsiang/Documents/avm-studio/avm-stitch/build --output-on-failure
  ```
  预期:
  ```
  100% tests passed, 0 tests failed out of 114
  ```
  （判据: `0 tests failed`；实际总数若与 114 略有出入，以 ctest 报告为准，但**不得出现 FAILED**。GL 门控测试在本环境 EGL 可用时应实际运行。）

- [ ] **Step 3: 集成回归（avm-worker 树，可选但推荐）**
  avm-worker 通过 `add_subdirectory` 引入 avm-stitch，测试在同一 ctest 树运行。在 avm-worker 根目录增量构建并定向验证 `test_stitch_engine`（若 avm-worker build 目录已存在）:
  ```bash
  cd /home/leehsiang/Documents/avm-studio/avm-worker
  cmake --build build
  ctest --test-dir build -R test_stitch_engine --output-on-failure
  ```
  预期: 该 ctest 树 299 个测试中，3 个 OSGGLTest 从红转绿，其余不受影响，0 FAILED。
  （若 avm-worker build 目录不存在，此步可跳过——Task 4 的硬性判据是 Step 2 的 avm-stitch 独立全量通过。）

- [ ] **Step 4: 保留函数完整性核验**
  ```bash
  rg -n 'MakeSideCameras|MakeCoarseBowl' /home/leehsiang/Documents/avm-studio/avm-stitch/tests/core/test_stitch_engine.cpp
  ```
  预期: `MakeSideCameras` 仍有 ≥13 处匹配（定义 1 + 调用 13），`MakeCoarseBowl` 仍有 ≥2 处匹配（定义 1 + 调用 1）。二者均未被删除，`BuildTestScene` 不再引用 `MakeSideCameras`（仅 `BuildTestSceneWithExtrinsics` 之外不再有来自 `BuildTestScene` 的调用）。

---

## Phase 3: 自检

### 1. 规范覆盖
| 设计文档章节 | 对应任务 |
|-------------|---------|
| §4.1 目标文件 | Task 2（仅改 `tests/core/test_stitch_engine.cpp`） |
| §4.2 改动 diff | Task 2（精确 diff，含前置声明） |
| §4.3 保留函数 | Task 2 不做项 + Task 4 Step 4 核验 |
| §4.4 不改动项 | Task 2 不做项（生产代码/断言不动） |
| §5 影响分析 | Task 1（8 个不受影响测试仍过）+ Task 4（全量回归） |
| §6 验证计划 | Task 1/3（定向）+ Task 4（全量 + 集成） |
| §7 回滚方案 | 见下文「回滚」 |
| ADR-5 声明顺序 | Task 2（选择前置声明，最小 diff） |

### 2. 占位符扫描
- 无 "TBD" / "TODO" / "稍后实现"。
- 每个验证步骤含精确命令 + 预期输出。
- 完整 diff 与改后代码全文均已给出，非「描述要做什么」。

### 3. 类型一致性
- 前置声明 `std::vector<CameraExtrinsics> MakeLookAtCameras(int num_cameras);` 与定义（行 149）签名**完全一致**（返回值类型、参数类型、参数名省略均合法）。
- `CameraExtrinsics` 来自 `avm_worker::avm_stitch`（`using namespace` 第 36 行），`std::vector` 来自 `<vector>`（第 34 行），前置声明处二者均在作用域内。

### 4. 自包含
- 每个任务含 Files、行为描述、测试策略、具体命令与预期，可独立执行。
- 任务间依赖：Task 2 依赖 Task 1 建立的 RED 基线；Task 3 依赖 Task 2 的编译产物；Task 4 依赖 Task 3 的 GREEN 状态。均为线性顺序依赖。

---

## 回滚方案

改动集中于 `BuildTestScene` + 一条前置声明，回滚方式（二选一）：

1. **git 回滚:** `git -C /home/leehsiang/Documents/avm-studio/avm-stitch restore tests/core/test_stitch_engine.cpp`
2. **手动还原:** 删除插入的前置声明（2 行注释 + 1 行声明），将 `BuildTestScene` 改回:
   ```cpp
   TestScene BuildTestScene(int num_cameras) {
       std::vector<double> azimuths;
       for (int i = 0; i < num_cameras; ++i) azimuths.push_back(i * 360.0 / num_cameras);
       return BuildTestSceneWithExtrinsics(num_cameras, MakeSideCameras(azimuths));
   }
   ```

回滚后测试恢复原状（3 个测试因 `has_visible_vertices == false` 失败），无其他副作用。

---

## 输出

```
计划路径: docs/plans/2026-08-13-fix-osggltest-buildtestscene-plan.md
任务数量: 4
关键依赖:
  Task 1 (RED 基线) → Task 2 (GREEN 改动) → Task 3 (GREEN 验证) → Task 4 (REFACTOR + 回归)
```
