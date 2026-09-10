# renderer 路径剥离实现计划（avm-stitch / avm-worker 双仓库重构）

> **设计文档:** `docs/specs/strip-renderer-path.md`（已批准）
> **计划生成日期:** 2026-09-09
> **源 commit（provenance 基准）:** avm-stitch `f338aec`（分支 `feature/avm-renderer-integration`，已实测确认）
> **两仓路径:** `/home/leehsiang/Documents/avm-studio/avm-stitch`（剥离对象）/ `/home/leehsiang/Documents/avm-studio/avm-worker`（接收方，分支 `main`）
> **TDD 适配声明:** 本计划为纯重构 + 2 处行为新增（D5 工厂收敛的可观测行为、D6 fail-fast）。结构性搬迁任务的"测试"= Task 0 记录的双仓基线回归对比；行为新增点走完整 RED-GREEN-REFACTOR。

---

## 0. 现状核实结论与设计偏差（计划裁决）

计划编写前已实测核实两仓库。设计与现状一致的部分不赘述（19 个搬迁文件全部存在、两份 CMakeLists 行号与设计引用一致、dispatch 位于 worker `stitch_eval.cpp:526-538`、`EnsureNoUnreplacedPlaceholders` 已存在于 avm-stitch `calib_files_generator.cpp:58-75` 且 worker subprocess 副本缺失、namespace/include 闭包与设计 §1 实测结论一致）。

以下 5 处为设计未覆盖或内部矛盾，本计划裁决如下（executor 按裁决执行，不自行发挥）：

| # | 偏差 | 裁决 |
|---|------|------|
| **D-A** | worker `tests/avm_stitch/test_renderer_backend.cpp:83-87` 的 `RoutesRendererName` 用例断言 `CreateStitchBackend("renderer")` 返回非 null `RendererBackend`，与 D5（工厂只留 "osg"）**正面冲突**；设计 §2 目标 4"断言零修改"与设计 §9"工厂对 renderer 返回 nullptr"矛盾。 | 架构决策（D5/§9）优先。该 **单个 TEST 块**（仅此一处）断言翻转为期望 `nullptr`（新断言全文见 Task 8）。其余 5 个测试文件的用例零修改承诺维持。 |
| **D-B** | worker `CMakeLists.txt:100-113` 的 `test_stitch_eval` target **也编译 `stitch_eval.cpp`**——dispatch 改本地直调后它同样引用 bridge 符号。设计 §6 只列了 `test_stitch_backend`/`test_stitch_stream`/5 个搬迁测试。 | `test_stitch_eval` 一并增加链接 `avm_renderer_bridge`（Task 7）。 |
| **D-C** | avm-stitch `src/core/stitch_backend.hpp:17-18` 工厂注释提及 `"renderer" -> RendererBackend`，设计 §5 修改清单只列了 `.cpp`。 | 顺带更新 `.hpp` 注释（纯注释，无行为影响，Task 4）。 |
| **D-D** | avm-stitch 当前分支为 `feature/avm-renderer-integration`；avm-worker 当前在 `main`。设计未指定提交分支；全局规则禁止向 main 直接推送。 | avm-stitch：在当前 feature 分支上提交（剥离 commit 在前，满足 D8）。avm-worker：先创建 `feature/strip-renderer-proxy` 分支再提交。两仓均只本地 commit、**不 push**（PR 由用户发起）。 |
| **D-E** | 两仓当前均无 `build/` 目录（glob 无 `build/CMakeCache.txt`）；计划编写工具集无法执行 `git status`。 | Task 0 强制首步为 `git status --porcelain` 检查 + 完整 configure。若 worker 因私有 registry 不可达而 configure 失败 → 记录现状，avm-stitch 侧批次照常执行，worker 侧构建验证标记 BLOCKED-on-environment（文件/CMake/代码修改仍可执行）。 |

---

## 1. 批次总览

| 批次 | 任务 | 类型 | 内容 | commit 策略 |
|------|------|------|------|------------|
| **A** | Task 0 | **契约批** | 双仓基线（git 状态、configure/build/ctest、私有 registry 可达性探测） | 基线任务不产生代码变更，无 commit |
| **B** | Task 1, 2, 3, 4 | **机械批** | avm-stitch 剥离：暂存 19 文件 → 删 23 文件 → CMake 清理 → 工厂 TDD 收敛 | 批末单 commit（avm-stitch 仓） |
| **C** | Task 5, 11 | **机械批，可并行（与 D）** | avm-stitch 构建全绿验证 + 干净自举 + README 重写（纯 avm-stitch 仓文件） | 批末单 commit（avm-stitch 仓） |
| **D** | Task 6, 7 | **契约批** | worker 落位 19 文件（provenance + 去宏）+ `avm_renderer_bridge` 库与全部 target 链接调整（产出 bridge 契约，下游批次依赖） | 批末单 commit（avm-worker 仓） |
| **E** | Task 8, 9, 10 | **机械批** | worker 行为改造：dispatch 本地直调 TDD（含 D-A 断言翻转）→ subprocess fail-fast TDD（D6）→ stream 注释 + AGENTS.md + 全量 ctest | 批末单 commit（avm-worker 仓） |
| **F** | Task 12 | **机械批** | **检查点： Task 12（合并态最终验收，设计 §11 两仓标准全跑）** + 双仓分别提交（avm-stitch 先） | 双仓各一 commit，顺序锁定 |

**依赖链:** A → B → (C ∥ D) → E → F。
C 与 D 文件集不相交（C 只动 avm-stitch 仓，D 只动 avm-worker 仓）且无逻辑依赖（C 只要求 B 完成）→ 标记可并行。D 必须在 B 之后（D8：avm-stitch 未剥离前 worker 不得出现第二份符号源）。

**批内纪律:** 逐任务完整 TDD 红绿循环（重构任务=构建+ctest 回归验证），全部验证通过后一次 commit；批内 BLOCKED 时对已完成且测试通过的任务抢救 commit。

---

## Task 0: 双仓库基线验证

**What（计划锁定，不可改）:**

**Files:** 无代码变更。产出基线记录（写入终端输出，executor 摘录进执行报告）。

**前置检查（按序，任何一步异常即停止并报告）:**

1. **git 状态检查（强制首步）:**
   ```bash
   cd /home/leehsiang/Documents/avm-studio/avm-stitch && git status --porcelain
   cd /home/leehsiang/Documents/avm-studio/avm-worker && git status --porcelain
   ```
   预期: 两仓均输出为空（干净）。非空 → **停止，报告未提交变更清单，等待用户处置**（不得自行 stash/checkout）。
   同时记录：`git rev-parse HEAD`（两仓）——avm-stitch 应为 `f338aec...`（provenance 基准；若不同，后续 provenance 注释与 worker commit message 中的源 commit 以实测值为准并报告偏差）。

2. **avm-stitch 基线:**
   ```bash
   cd /home/leehsiang/Documents/avm-studio/avm-stitch
   echo "VCPKG_ROOT=$VCPKG_ROOT"    # 预期: 非空；为空则停止，请求用户设置
   cmake --preset default
   cmake --build build -j$(nproc)
   ctest --test-dir build --output-on-failure 2>&1 | tail -5
   ```
   预期: configure 成功（CLI11/glfw3 来自公共 registry 可解析，`avm_renderer not found` STATUS 消息属预期，宏 OFF）；构建 0 error；ctest 全 PASS/SKIP（GL 用例无 EGL 环境 SKIP 属预期）。**记录: 通过/跳过的测试计数。**

3. **avm-worker 基线（含私有 registry 可达性探测，风险点 R0）:**
   ```bash
   cd /home/leehsiang/Documents/avm-studio/avm-worker
   cmake --preset default
   cmake --build build -j$(nproc)
   ctest --test-dir build --output-on-failure 2>&1 | tail -5
   ```
   - 成功 → 记录测试计数，后续全部验证可做。
   - configure 失败（avm-renderer 私有 registry 克隆失败 / SSH 不可达）→ **记录失败输出全文**，在执行报告中标注: "worker 构建验证 BLOCKED-on-environment"。此后 Task 7/10/12 中依赖 worker 构建的验证命令标记为"待环境恢复执行"，文件修改照常完成。avm-stitch 侧（批次 B/C）不受影响。

**验证命令:** 上文三组命令本身即验证；每组均有预期输出。

**How（executor 自由）:** 记录格式、日志保存位置（建议 `/tmp/strip-renderer-baseline-{stitch,worker}.log`）自定。

---

## Task 1: 19 个搬迁文件复制暂存（防丢失保险）

**What:**

**Files:**
- Create（暂存区，git 外）: `/tmp/avm-stitch-migration-staging/` 下按原相对路径镜像 19 文件

**暂存命令（可复制粘贴）:**
```bash
cd /home/leehsiang/Documents/avm-studio/avm-stitch
STAGE=/tmp/avm-stitch-migration-staging
mkdir -p $STAGE/src/core $STAGE/src/render $STAGE/src/common
for f in src/core/renderer_backend.hpp src/core/renderer_backend.cpp \
         src/core/real_renderer.hpp src/core/real_renderer.cpp \
         src/core/renderer_helpers.hpp src/core/renderer_helpers.cpp \
         src/core/calib_files_generator.hpp src/core/calib_files_generator.cpp \
         src/core/view_mapping.hpp src/core/view_mapping.cpp \
         src/core/stitch_session.hpp src/core/stitch_session.cpp \
         src/core/renderer_interface.hpp \
         src/render/egl_gles3_context.hpp src/render/egl_gles3_context.cpp \
         src/render/fbo_manager.hpp src/render/fbo_manager.cpp \
         src/common/image_converter.hpp src/common/image_converter.cpp; do
    cp "$f" "$STAGE/$f"
done
```

**验证命令:**
```bash
find /tmp/avm-stitch-migration-staging -type f | wc -l
```
→ 预期输出: `19`
```bash
# 逐文件内容一致性（暂存副本与仓库当前文件必须零差异）:
cd /home/leehsiang/Documents/avm-studio/avm-stitch
ok=1; for f in $(cd /tmp/avm-stitch-migration-staging && find src -type f); do \
  cmp -s "/tmp/avm-stitch-migration-staging/$f" "$f" || { echo "MISMATCH: $f"; ok=0; }; done
[ $ok -eq 1 ] && echo ALL-MATCH
```
→ 预期: 输出 `ALL-MATCH`（无 MISMATCH 行）。

**How:** 无自由度（命令锁定）。

---

## Task 2: avm-stitch 删除 23 个文件（19 搬迁 + 4 测试）

**What:**

**Files（删除，精确清单）:**

搬迁源文件 19 个（内容已暂存于 Task 1）:
```
src/core/renderer_backend.hpp      src/core/renderer_backend.cpp
src/core/real_renderer.hpp         src/core/real_renderer.cpp
src/core/renderer_helpers.hpp      src/core/renderer_helpers.cpp
src/core/calib_files_generator.hpp src/core/calib_files_generator.cpp
src/core/view_mapping.hpp          src/core/view_mapping.cpp
src/core/stitch_session.hpp        src/core/stitch_session.cpp
src/core/renderer_interface.hpp
src/render/egl_gles3_context.hpp   src/render/egl_gles3_context.cpp
src/render/fbo_manager.hpp         src/render/fbo_manager.cpp
src/common/image_converter.hpp     src/common/image_converter.cpp
```

对应测试 4 个（设计 §5）:
```
tests/core/test_view_mapping.cpp
tests/core/test_real_renderer.cpp
tests/render/test_fbo_manager.cpp
tests/render/test_egl_gles3_context.cpp
```

**删除命令:**
```bash
cd /home/leehsiang/Documents/avm-studio/avm-stitch
git rm src/core/renderer_backend.hpp src/core/renderer_backend.cpp \
       src/core/real_renderer.hpp src/core/real_renderer.cpp \
       src/core/renderer_helpers.hpp src/core/renderer_helpers.cpp \
       src/core/calib_files_generator.hpp src/core/calib_files_generator.cpp \
       src/core/view_mapping.hpp src/core/view_mapping.cpp \
       src/core/stitch_session.hpp src/core/stitch_session.cpp \
       src/core/renderer_interface.hpp \
       src/render/egl_gles3_context.hpp src/render/egl_gles3_context.cpp \
       src/render/fbo_manager.hpp src/render/fbo_manager.cpp \
       src/common/image_converter.hpp src/common/image_converter.cpp \
       tests/core/test_view_mapping.cpp tests/core/test_real_renderer.cpp \
       tests/render/test_fbo_manager.cpp tests/render/test_egl_gles3_context.cpp
```

**验证命令:**
```bash
git status --porcelain | grep '^D' | wc -l
```
→ 预期: `23`
```bash
ls src/core/ src/render/ src/common/
```
→ 预期: `src/core/` 剩 `stitch_backend.{hpp,cpp} osg_backend.{hpp,cpp} stitch_engine.{hpp,cpp} stitch_result.{hpp,cpp}`；`src/render/` 剩 `egl_headless_context.{hpp,cpp} osg_scene.{hpp,cpp} osg_view.{hpp,cpp}`；`src/common/` 剩 `base64.h`。

**说明:** 本任务后仓库暂不可构建（CMake 仍引用已删文件）——属预期中间态，Task 3/4 修复，批次 B 末恢复绿色（D8 允许批内无中间可构建态）。

**How:** 无自由度。

---

## Task 3: avm-stitch CMakeLists.txt 清理

**What:**

**Files:**
- Modify: `/home/leehsiang/Documents/avm-studio/avm-stitch/CMakeLists.txt`

**修改锚点表（当前行号 + 锚点内容双重定位；行号随删改漂移时以内容锚定）:**

| # | 当前行号 | 锚点内容 | 操作 |
|---|---------|---------|------|
| 1 | L10 | `find_package(OpenGL REQUIRED COMPONENTS EGL GLES3)` | 改为 `find_package(OpenGL REQUIRED COMPONENTS EGL)`（GLES3 仅为已搬走的 `egl_gles3_context`/`fbo_manager` 服务；保留 EGL 供 `egl_headless_context`） |
| 2 | L13-29 | 自 `# -- avm-renderer (private registry, optional for renderer backend) --` 起，至 `endif()`（含 `find_package(CLI11/glfw3/avm_renderer)` 与 `AVMRENDERER_AVAILABLE` 设置块） | 整块删除 |
| 3 | L48-67 | `add_library(avm_stitch STATIC ...)` 源列表 | 删除列表中 9 个搬迁 `.cpp`：`src/render/egl_gles3_context.cpp`（L51）、`src/core/renderer_backend.cpp`（L59）、`src/core/real_renderer.cpp`（L60）、`src/core/calib_files_generator.cpp`（L61）、`src/common/image_converter.cpp`（L62）、`src/core/view_mapping.cpp`（L63）、`src/render/fbo_manager.cpp`（L64）、`src/core/renderer_helpers.cpp`（L65）、`src/core/stitch_session.cpp`（L66）。保留 9 个：blend_weights / egl_headless_context / mesh_bake / osg_scene / osg_view / stitch_engine / stitch_backend / osg_backend / stitch_result |
| 4 | L76 | `    OpenGL::EGL OpenGL::GLES3` | 改为 `    OpenGL::EGL` |
| 5 | L82-89 | `# -- avm-renderer (private registry, optional for renderer backend) --` 第二块（`if(AVMRENDERER_AVAILABLE)` 条件链接）至 `endif()` | 整块删除 |
| 6 | L216-227 | `# -- test_fbo_manager ...` 至 `gtest_discover_tests(test_fbo_manager)` | 整块删除 |
| 7 | L229-239 | `# -- test_egl_gles3_context ...` 至 `gtest_discover_tests(test_egl_gles3_context)` | 整块删除 |
| 8 | L241-250 | `# -- test_view_mapping ...` 至 `gtest_discover_tests(test_view_mapping)` | 整块删除 |
| 9 | L252-262 | `# -- test_real_renderer ...` 至 `gtest_discover_tests(test_real_renderer)` | 整块删除 |

**修改后 `avm_stitch` 源列表与链接应为（声明式产物，逐字对照）:**
```cmake
add_library(avm_stitch STATIC
    src/geometry/blend_weights.cpp
    src/render/egl_headless_context.cpp
    src/geometry/mesh_bake.cpp
    src/render/osg_scene.cpp
    src/render/osg_view.cpp
    src/core/stitch_engine.cpp
    src/core/stitch_backend.cpp
    src/core/osg_backend.cpp
    src/core/stitch_result.cpp
)
```
链接块中 OpenGL 行: `    OpenGL::EGL`（其余 nlohmann_json/OpenCV/osg 各行不动）。

**验证命令:**
```bash
grep -n "avm_renderer\|AVMRENDERER\|CLI11\|glfw\|renderer_backend\|real_renderer\|egl_gles3\|fbo_manager\|image_converter\|view_mapping\|stitch_session\|calib_files" CMakeLists.txt
```
→ 预期: 无任何输出。
```bash
grep -c "add_executable" CMakeLists.txt
```
→ 预期: `9`（bowl_model/projection/blend_weights/mesh_bake/view_config/osg_scene/stitch_engine/egl_headless_context/stitch_engine_regression）。

**How:** 无自由度（锚点表锁定）。

---

## Task 4: stitch_backend 工厂 TDD 收敛（D5）+ `.hpp` 注释（D-C）

**What:**

**Files:**
- Modify: `tests/core/test_stitch_engine_regression.cpp`（新增 1 个 TEST 块 + `#include "core/stitch_backend.hpp"`）
- Modify: `src/core/stitch_backend.cpp`（目标形态全文见下）
- Modify: `src/core/stitch_backend.hpp:17-18`（注释）

**测试规格（RED 先行，断言锁定）:**

在 `tests/core/test_stitch_engine_regression.cpp` 头部 include 区（`#include "core/stitch_engine.hpp"` 之后）加:
```cpp
#include "core/stitch_backend.hpp"
```
文件末尾追加:
```cpp
TEST(StitchBackendFactory, RendererNameReturnsNullptrAfterMigration) {
    // The renderer proxy moved to avm-worker (avm_renderer_bridge); this
    // library's factory no longer knows that name (D5).
    EXPECT_EQ(CreateStitchBackend("renderer"), nullptr);
}
```

**RED 验证命令:**
```bash
cmake --build build --target test_stitch_engine_regression -j$(nproc) && \
./build/test_stitch_engine_regression --gtest_filter=StitchBackendFactory.RendererNameReturnsNullptrAfterMigration
```
→ 预期: **FAIL**。失败形式: `Expected equality of these values: CreateStitchBackend("renderer") (非空指针) vs nullptr`——当前工厂（宏 OFF 环境）返回 `make_unique<RendererBackend>(nullptr)` 非 null。（若环境恰好宏 ON 返回 RealRenderer 包装，同样非 null，RED 仍成立。）若失败原因是**编译错误**而非断言失败 → 测试代码有误，修正测试再跑。

**目标形态（GREEN 产物，`src/core/stitch_backend.cpp` 全文）:**
```cpp
#include "core/stitch_backend.hpp"

#include "core/osg_backend.hpp"

namespace avm_worker { namespace avm_stitch {

std::unique_ptr<StitchBackend> CreateStitchBackend(const std::string& name) {
    if (name == "osg") {
        return std::make_unique<OSGBackend>();
    }
    return nullptr;
}

}} // namespace avm_worker::avm_stitch
```

`src/core/stitch_backend.hpp` L17-18 注释替换为:
```cpp
// Factory: task["algorithm"] == "osg" -> OSGBackend; any other name
// (including "renderer", whose proxy moved to avm-worker) -> nullptr.
```

**GREEN 验证命令:**
```bash
cmake --build build --target test_stitch_engine_regression -j$(nproc) && \
./build/test_stitch_engine_regression
```
→ 预期: 全部 PASS（含既有 3 个 RegressionTest 用例）。

**How（executor 自由）:** 无实现自由度（目标形态为逐字产物）；执行顺序（先测试后实现）不可颠倒。

---

## Task 5: avm-stitch 剥离态全量验证 + 干净自举（设计 §11 上半）

**What:**

**验证命令（按序，全部锁定）:**

1. **全量构建+测试:**
   ```bash
   cd /home/leehsiang/Documents/avm-studio/avm-stitch
   cmake --preset default && cmake --build build -j$(nproc)
   ctest --test-dir build --output-on-failure 2>&1 | tail -5
   ```
   → 预期: 9 个测试 target 全 PASS/SKIP，0 FAIL。SKIP 仅限 GL 门控用例（与 Task 0 基线中 SKIP 集合一致）。

2. **残留 grep（设计 §11）:**
   ```bash
   grep -rn "avm_renderer\|AVMRENDERER\|CLI11\|glfw\|real_renderer\|renderer_backend" \
        CMakeLists.txt src tests
   ```
   → 预期: **无任何输出**。有输出 → 回到 Task 2-4 补删，重跑本步。

3. **干净目录自举（独立目录，验证仅公共 registry 可配置）:**
   ```bash
   cd /home/leehsiang/Documents/avm-studio/avm-stitch
   rm -rf /tmp/avm-stitch-bootstrap
   mkdir -p /tmp/avm-stitch-bootstrap
   cp -r CMakeLists.txt CMakePresets.json vcpkg.json src tests /tmp/avm-stitch-bootstrap/
   cd /tmp/avm-stitch-bootstrap && cmake --preset default && cmake --build build -j$(nproc) && \
     ctest --test-dir build 2>&1 | tail -3
   ```
   → 预期: configure 成功且**不出现任何私有 registry 访问**；构建测试全 PASS/SKIP。（vcpkg 二进制缓存复用属正常，不违背"无 avm-renderer 依赖"验证目的——关键断言是 `find_package` 集合里已无 CLI11/glfw3/avm_renderer，grep 已在步骤 2 锁定。）
   完成后: `rm -rf /tmp/avm-stitch-bootstrap`。

**回归对比:** ctest 通过数 ≥ 基线（Task 0 步骤 2）中非搬迁测试的通过数（搬迁的 4 个测试 target 已删除，其余不可有任何回归）。

**How:** 无自由度。

---

## Task 6: worker 落位 19 文件（D1 镜像 + D7 provenance + D4 去宏）

**What:**

**Files（Create，全部来自 `/tmp/avm-stitch-migration-staging/`，目标路径 = worker `src/` + 原相对路径）:**
```
src/core/renderer_backend.{hpp,cpp}      src/core/real_renderer.{hpp,cpp}
src/core/renderer_helpers.{hpp,cpp}      src/core/calib_files_generator.{hpp,cpp}
src/core/view_mapping.{hpp,cpp}          src/core/stitch_session.{hpp,cpp}
src/core/renderer_interface.hpp
src/render/egl_gles3_context.{hpp,cpp}   src/render/fbo_manager.{hpp,cpp}
src/common/image_converter.{hpp,cpp}
```
（worker `src/core/`、`src/render/` 目录当前不存在，需创建；`src/common/` 已存在。）

**操作规格:**

1. 逐文件从暂存区复制到目标路径（保留原内容，`#include` 零改动——D1）。

2. **每文件头部**（`#pragma once` / 首个 `#include` 之前）插入 provenance 注释（D7，19 文件统一模板，逐字使用）:
   ```cpp
   // Migrated from avm-stitch (personal GitHub repo) @ f338aec
   // (feature/avm-renderer-integration) on 2026-09-09. Relocated verbatim to
   // avm-worker as part of the renderer-proxy strip; namespace
   // (avm_worker::avm_stitch) and relative include paths unchanged.
   // Design: avm-worker docs/specs/strip-renderer-path.md (D1/D7).
   ```
   （若 Task 0 实测 HEAD ≠ f338aec，以实测 SHA 替换并报告。）

3. **去宏（D4，仅 `real_renderer.cpp` + `real_renderer.hpp` 注释）:**
   - `real_renderer.cpp`: 删除 L10 `#ifdef AVMRENDERER_AVAILABLE`、L113-123 整个 `#else ... #endif` stub 分支、L124 `#endif`；保留 L11-111 真实现为无条件编译体。L1-7 头注释中 "AVMRENDERER_AVAILABLE branch" 表述改为无条件依赖表述。
   - `real_renderer.hpp` L12-14 注释中 "even when AVMRENDERER_AVAILABLE is off" 措辞更新（纯注释）。

**验证命令:**
```bash
cd /home/leehsiang/Documents/avm-studio/avm-worker
find src/core src/render src/common -type f | sort   # 19 个新文件就位（加上 worker 原有 src/common/base64.h）
grep -rn "AVMRENDERER" src/                            # 预期: 无输出
# provenance 覆盖检查（列出缺少 provenance 头注的文件; 预期无输出）:
for f in src/core/renderer_backend.hpp src/core/renderer_backend.cpp \
         src/core/real_renderer.hpp src/core/real_renderer.cpp \
         src/core/renderer_helpers.hpp src/core/renderer_helpers.cpp \
         src/core/calib_files_generator.hpp src/core/calib_files_generator.cpp \
         src/core/view_mapping.hpp src/core/view_mapping.cpp \
         src/core/stitch_session.hpp src/core/stitch_session.cpp \
         src/core/renderer_interface.hpp \
         src/render/egl_gles3_context.hpp src/render/egl_gles3_context.cpp \
         src/render/fbo_manager.hpp src/render/fbo_manager.cpp \
         src/common/image_converter.hpp src/common/image_converter.cpp; do \
  grep -q "Migrated from avm-stitch" "$f" || echo "NO-PROVENANCE: $f"; done
```
内容一致性核验（除允许差异外必须零差异; 允许差异 = 19 文件统一的 provenance 头注 + real_renderer.{cpp,hpp} 的去宏改动）:
```bash
for f in $(cd /tmp/avm-stitch-migration-staging && find src -type f); do \
  echo "=== $f ==="; diff -u "/tmp/avm-stitch-migration-staging/$f" "$f" | head -40; done
```
→ 预期: 17 个无去宏文件的 diff 仅头部 5 行 provenance 新增；`real_renderer.cpp` 另含 `#ifdef/#else/#endif` 删除（L10/L113-124 区域）；`real_renderer.hpp` 另含注释措辞差异。任何其他 hunks = 未按"原样搬家"执行，修正。

**说明:** 本任务后 worker 暂不可构建（新文件未进任何 target）——预期中间态，Task 7 接通。

**How:** 文件复制机械操作无自由度；逐文件 diff 的具体执行方式自定。

---

## Task 7: worker CMakeLists —— avm_renderer_bridge 库 + 全部链接调整（D3/D4 + D-B）

**What:**

**Files:**
- Modify: `/home/leehsiang/Documents/avm-studio/avm-worker/CMakeLists.txt`

**修改锚点表（当前行号 + 内容锚定）:**

| # | 当前行号 | 锚点 | 操作 |
|---|---------|------|------|
| 1 | L25 之后（`add_subdirectory(avm-stitch)` 的 `endif()` 之后、`add_executable(avm-worker` 之前） | 插入 bridge 块（全文见下） | 插入 |
| 2 | L39-44 | `target_link_libraries(avm-worker PRIVATE ... avm_stitch )` | 列表追加 `avm_renderer_bridge`（`avm_stitch` 保留——可执行仍直接用 OSG 工厂与保留头） |
| 3 | L105-111 | `target_link_libraries(test_stitch_eval PRIVATE ... avm_stitch )` | 追加 `avm_renderer_bridge`（D-B：该 target 编译 stitch_eval.cpp） |
| 4 | L197-201 | `target_link_libraries(test_stitch_backend PRIVATE ... avm_stitch )` | 追加 `avm_renderer_bridge`（该 target 编译 stitch_eval.cpp，Task 8 后引用 bridge 符号） |
| 5 | L210-213 | `test_renderer_backend` 链接 `avm_stitch` | 改为 `avm_renderer_bridge avm_stitch`（引用迁入头 + 保留头 stitch_backend.hpp，双链） |
| 6 | L222-225 | `test_image_converter` 链接 `avm_stitch` | 改为 `avm_renderer_bridge avm_stitch`（迁入头 image_converter.hpp + 保留头 base64.h） |
| 7 | L234-237 | `test_renderer_helpers` 链接 `avm_stitch` | 改为 `avm_renderer_bridge avm_stitch`（迁入头 + 保留头 base64.h） |
| 8 | L248-251 | `test_calib_files_generator` 链接 `avm_stitch` | 改为 `avm_renderer_bridge avm_stitch`（迁入头 + 保留头 camera_name.hpp）；**`WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}` 保持不动（R5）** |
| 9 | L260-263 | `test_stitch_session` 链接 `avm_stitch` | 改为 `avm_renderer_bridge`（纯迁入头） |
| 10 | L273-278 | `test_stitch_stream` 链接 `avm_stitch` | 改为 `avm_renderer_bridge`（编译 stitch_stream.cpp，include 迁入头；nlohmann_json/hiredis 保留） |

**插入块全文（声明式产物，逐字使用；放在锚点 1 位置）:**
```cmake
# -- avm_renderer_bridge: renderer proxy code migrated from avm-stitch --
# Source: avm-stitch @ f338aec (2026-09-09), see provenance headers.
# avm_renderer is a HARD dependency here (D4): worker's vcpkg environment
# always has the private registry; the AVMRENDERER_AVAILABLE escape hatch
# was removed together with the migration.
# avm_renderer's exported targets reference CLI11::CLI11 and a bare `glfw`
# in their link interface but the config file does not find_dependency
# them; find both explicitly before avm_renderer's targets load.
find_package(CLI11 CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS EGL GLES3)
find_package(avm_renderer CONFIG REQUIRED)

add_library(avm_renderer_bridge STATIC
    src/core/renderer_backend.cpp
    src/core/real_renderer.cpp
    src/core/renderer_helpers.cpp
    src/core/calib_files_generator.cpp
    src/core/view_mapping.cpp
    src/core/stitch_session.cpp
    src/render/egl_gles3_context.cpp
    src/render/fbo_manager.cpp
    src/common/image_converter.cpp
)
# PUBLIC include root mirrors avm_stitch's (D1): "core/..." "render/..."
# "common/..." include strings resolve unchanged.
target_include_directories(avm_renderer_bridge PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(avm_renderer_bridge PUBLIC
    avm_renderer::avm_renderer_lib
    avm_renderer::avm_fbs
    OpenGL::EGL OpenGL::GLES3
    nlohmann_json::nlohmann_json
    ${OpenCV_LIBS}
    avm_stitch
)
```

**验证命令（若 Task 0 判定 worker 环境 BLOCKED-on-environment，本验证标记待执行，但仍完成 grep 静态核验）:**
```bash
cd /home/leehsiang/Documents/avm-studio/avm-worker
grep -n "avm_renderer_bridge" CMakeLists.txt
```
→ 预期: **13 行命中**（注释 1 + add_library 1 + target_include_directories 1 + target_link_libraries 1 + 使用方 9: avm-worker 可执行 / test_stitch_eval / test_stitch_backend / test_renderer_backend / test_image_converter / test_renderer_helpers / test_calib_files_generator / test_stitch_session / test_stitch_stream）。
```bash
grep -n "AVMRENDERER" CMakeLists.txt    # 预期: 无输出（worker 侧从未有过，确认未引入）
cmake --preset default && cmake --build build -j$(nproc) 2>&1 | tail -3
```
→ 预期: configure 通过（avm_renderer FOUND，REQUIRED 生效）；构建 0 error。若链接期报 `CLI11::CLI11` / `glfw` 未定义 → 检查插入块的 find_package 是否位于 `find_package(avm_renderer ...)` 之前（顺序锁定）。
```bash
ctest --test-dir build --output-on-failure 2>&1 | tail -5
```
→ 预期: **全 PASS/SKIP**——此刻 dispatch 仍走工厂（avm-stitch 已剥离 → `"renderer"` 工厂返回 nullptr → 回退 subprocess），`test_renderer_backend` 中既有用例除 `RoutesRendererName`（D-A，Task 8 处理）外应全绿；`RoutesRendererName` 在本步**预期 FAIL**（工厂已不认识 "renderer"），属已知中间态，Task 8 修复。其余 target（含 5 个搬迁对象测试，已链 bridge）全绿 = 搬迁落位初步保真。

**How:** 插入位置若有微调（如放在文件更靠后的 find_package 集中区），保持两个不变式: (a) 在所有使用 bridge 的 target 之前; (b) `find_package(CLI11/glfw3)` 在 `find_package(avm_renderer)` 之前。

---

## Task 8: dispatch 本地直调 TDD（D5 worker 侧 + D-A 断言翻转）

**What:**

**Files:**
- Modify: `tests/avm_stitch/test_stitch_backend.cpp`（新增 1 个 dispatch 用例）
- Modify: `tests/avm_stitch/test_renderer_backend.cpp:82-87`（D-A：翻转 `RoutesRendererName` 断言）
- Modify: `src/stitch_eval/stitch_eval.cpp`（dispatch 改造 + include + 注释，目标形态见下）
- Modify: `src/stitch_eval/stitch_eval.hpp:14-16`（doc 注释同步）

**测试规格（RED 先行，断言锁定）:**

(a) `test_stitch_backend.cpp` 末尾追加:
```cpp
// ── 6. Facade dispatch: renderer (local bridge call, D5) ──
// The renderer branch now constructs RendererBackend locally via
// avm_renderer_bridge; a task without images must come back as a
// stitch_error JSON (RendererBackend contract), NOT as the subprocess
// path's thrown "Missing 'images' in task" exception.
TEST(StitchEvalDispatch, RendererMissingImagesReturnsStitchErrorNotThrow) {
    nlohmann::json task = {
        {"task_id", "dispatch-renderer-1"},
        {"project_id", "test-proj"},
        {"algorithm", "renderer"}
    };
    nlohmann::json result;
    ASSERT_NO_THROW(result = StitchEvalDispatch(task));
    EXPECT_EQ(result["type"], "stitch_error");
    EXPECT_FALSE(result.value("error", "").empty());
}
```

(b) `test_renderer_backend.cpp:82-87` 的 TEST 块整体替换为（D-A 裁决，仅此一块）:
```cpp
// ── 1. Factory no longer routes "renderer" (proxy migrated here) ──
TEST(RendererBackendFactory, RendererNameReturnsNullptrAfterMigration) {
    // avm-stitch's factory only knows "osg" (D5); the renderer proxy lives
    // in this repo (avm_renderer_bridge) and is constructed locally by
    // StitchEvalDispatch instead of via the factory.
    EXPECT_EQ(CreateStitchBackend("renderer"), nullptr);
}
```

**RED 验证命令:**
```bash
cmake --build build --target test_stitch_backend test_renderer_backend -j$(nproc)
./build/test_stitch_backend --gtest_filter='StitchEvalDispatch.RendererMissingImages*'
./build/test_renderer_backend --gtest_filter='RendererBackendFactory.*'
```
（两个测试二进制独立运行——(a) 预期非零退出，`&&` 串联会吞掉 (b)。）
→ 预期: **双双 FAIL**。(a) 失败形式: `StitchEvalDispatch` 抛 `std::runtime_error: Missing 'images' in task` → `ASSERT_NO_THROW` 失败（现状: 工厂返回 nullptr → 回退 subprocess → StitchEval 前置校验抛异常）。(b) 失败形式: `CreateStitchBackend("renderer")` 返回 `nullptr`... 注意 (b) 在 avm-stitch 剥离后**已经 PASS**（工厂不认识 renderer）——(b) 无 RED 阶段，它是把"必挂的旧断言"翻转为"新契约断言"，GREEN 由本任务最终验证覆盖。若 (a) 的失败不是 throw 而是 PASS → 检查 avm-stitch 是否真为剥离 HEAD（D8 顺序被破坏），停止排查。

**目标形态（GREEN 产物）—— `stitch_eval.cpp` 修改点:**

include 区（L3 `#include "core/stitch_backend.hpp"` 附近）新增两行:
```cpp
#include "core/renderer_backend.hpp"
#include "core/real_renderer.hpp"
```

`StitchEvalDispatch`（L526-538）整体替换为:
```cpp
nlohmann::json StitchEvalDispatch(const nlohmann::json& task) {
    std::string algo = task.value("algorithm", "subprocess");
    // "renderer": the proxy lives in this repo (avm_renderer_bridge,
    // migrated from avm-stitch); construct locally, NOT via the factory
    // (avm-stitch's factory only knows "osg").
    if (algo == "renderer") {
        avm_stitch::RendererBackend backend(
            std::make_unique<avm_stitch::RealRenderer>());
        return backend.Run(task);
    }
    // "osg": avm-stitch's in-process OSG backend via the factory.
    if (algo == "osg") {
        auto backend = avm_stitch::CreateStitchBackend(algo);
        if (backend) {
            return backend->Run(task);
        }
    }
    // Default/"subprocess"/unknown: this worker's own StitchEval pipeline
    // (model_creator + stitcher subprocesses).
    return StitchEval(task);
}
```

`stitch_eval.hpp` L14-16 doc 注释替换为:
```cpp
/// Unified stitch facade: reads task["algorithm"]. "renderer" constructs the
/// local avm_renderer_bridge backend in-process; "osg" dispatches to
/// avm-stitch's factory; the default and any unknown name fall back to this
/// worker's own subprocess pipeline (StitchEval, backward compatible).
```

**GREEN 验证命令:**
```bash
cmake --build build --target test_stitch_backend test_renderer_backend -j$(nproc) && \
./build/test_stitch_backend && ./build/test_renderer_backend
```
→ 预期: 两个 target 全部 PASS（含既有 dispatch 矩阵用例: subprocess/osg/unknown/default——回归确认未破坏）。
```bash
grep -n "renderer" src/stitch_eval/stitch_eval.cpp | head -20
```
→ 预期: dispatch 内仅本地直调路径（无 `CreateStitchBackend(algo)` 用于 "renderer" 的写法；`CreateStitchBackend` 仅出现在 "osg" 分支）。

**How:** 实现本体已在目标形态锁定；执行顺序（先测试后实现）不可颠倒。

---

## Task 9: subprocess 副本 fail-fast TDD（D6，补 0c7fc4d 等价修复）

**What:**

**Files:**
- Modify: `tests/avm_stitch/test_stitch_backend.cpp`（新增 1 个 fail-fast 用例，追加于 Task 8 用例之后）
- Modify: `src/stitch_eval/stitch_eval.cpp`（新增 static 函数 + 1 处调用）

**测试规格（RED 先行，断言锁定）:**

`test_stitch_backend.cpp` 追加:
```cpp
// ── 7. Subprocess calib copy fails fast on unreplaced placeholders (D6) ──
// A camera whose label matches no template role ("top") leaves {{...}}
// placeholders in the rendered XML. GenCalibXml must throw (naming the
// leftover), mirroring avm_renderer_bridge's CalibFilesGenerator — never
// emit XML that model_creator would parse as DBL_MAX black frames.
TEST(StitchSubprocessCalibXml, ThrowsWhenPlaceholdersRemainUnreplaced) {
    fs::remove_all("/tmp/stitch-debug/calib-xml-failfast-1");
    fs::remove_all("/tmp/stitch/calib-xml-failfast-1");

    std::string b64 = valid_jpeg_base64();
    nlohmann::json task = subprocess_labeled_task("calib-xml-failfast-1");
    // Camera 0: non-role id + non-role label -> ResolveCameraLabel -> "top"
    // (role ids like "front" would win resolution and mask the label).
    task["calibration"]["cameras"][0]["id"] = "cam_0";
    task["calibration"]["cameras"][0]["label"] = "top";

    try {
        StitchEval(task);
        FAIL() << "StitchEval must throw when placeholders remain unreplaced";
    } catch (const std::exception& e) {
        EXPECT_NE(std::string(e.what()).find("Unreplaced template placeholders"),
                  std::string::npos)
            << "error must name the fail-fast cause, got: " << e.what();
    }
    fs::remove_all("/tmp/stitch/calib-xml-failfast-1");
}
```

**RED 验证命令:**
```bash
cmake --build build --target test_stitch_backend -j$(nproc) && \
./build/test_stitch_backend --gtest_filter='StitchSubprocessCalibXml.ThrowsWhenPlaceholdersRemainUnreplaced'
```
→ 预期: **FAIL**，失败信息 `StitchEval must throw when placeholders remain unreplaced`（现状: 无 fail-fast，渲染残留占位符照写文件，流程走到 model_creator 失败返回 stitch_error JSON，未抛）。若失败为编译错误 → 修正测试。

**目标形态（GREEN 产物）—— `stitch_eval.cpp` 修改点（逐字产物）:**

在 `render_template` 函数（L58-77）之后插入（与搬迁副本 `calib_files_generator.cpp:58-75` 等价，static 化）:
```cpp
// Fail fast when any {{...}} placeholder survived rendering (0c7fc4d
// parity with avm_renderer_bridge's CalibFilesGenerator): model_creator
// parses unreplaced literals as DBL_MAX and still exits 0, silently
// producing black frames. Throwing here — naming the output file and the
// leftover placeholders (camera role info) — turns the mismatch into a
// visible error instead.
static void EnsureNoUnreplacedPlaceholders(const std::string& content,
                                           const std::string& out_path) {
    size_t pos = content.find("{{");
    if (pos == std::string::npos) return;

    std::string leftovers;
    for (int shown = 0; pos != std::string::npos && shown < 4; ++shown) {
        size_t end = content.find("}}", pos);
        size_t len = (end == std::string::npos) ? std::string::npos
                                                : end + 2 - pos;
        leftovers += (shown ? ", " : "") + content.substr(pos, len);
        pos = (end == std::string::npos) ? std::string::npos : end + 2;
    }
    throw std::runtime_error(
        "Unreplaced template placeholders [" + leftovers +
        "] in generated calib XML: " + out_path +
        " (camera labels do not match the template's camera roles)");
}
```

`GenCalibXml`（L88-103）中，在 `std::string xml = render_template(template_name, vars);`（L99）之后、`std::ofstream f(out_path);`（L100）之前插入:
```cpp
    EnsureNoUnreplacedPlaceholders(xml, out_path);  // before any disk write
```

**范围锁定:** `gen_models_config` **不加** fail-fast（搬迁副本 `calib_files_generator.cpp:181-188` 的 models_config 部分同样没有——两副本行为对齐是 D6 的"等价修复"边界）。

**GREEN 验证命令:**
```bash
cmake --build build --target test_stitch_backend -j$(nproc) && \
./build/test_stitch_backend
```
→ 预期: 全 PASS（既有 `StitchSubprocessCalibXml.NoLiteralPlaceholdersInIntrinsicsXml` 正路径用例不回归——合法标定无残留，fail-fast 不触发）。

**How:** 无实现自由度（逐字产物）；执行顺序不可颠倒。

---

## Task 10: stitch_stream 注释改述 + worker AGENTS.md 边界说明 + worker 全量验证

**What:**

**Files:**
- Modify: `src/stitch_eval/stitch_stream.cpp:4-6`（仅注释，代码零改动）
- Modify: `src/AGENTS.md`（追加一段）

**`stitch_stream.cpp` L4-6 注释替换为（include 字符串本身不动——D1 下解析到 worker 本地新路径）:**
```cpp
#include "core/real_renderer.hpp"        // MakeDefaultSession → RealRenderer (avm_renderer_bridge, migrated from avm-stitch)
#include "core/renderer_helpers.hpp"     // MakeStitchError (BuildStreamErrorEnvelope) (avm_renderer_bridge)
#include "core/stitch_session.hpp"       // (avm_renderer_bridge)
```

**`src/AGENTS.md` 追加段落（放在模块概览表之后）:**
```markdown
## Renderer 代理代码（avm_renderer_bridge, 自 avm-stitch 迁入）

`src/core/` 下的 renderer_backend / real_renderer / renderer_helpers /
calib_files_generator / view_mapping / stitch_session / renderer_interface.hpp，
`src/render/` 下的 egl_gles3_context / fbo_manager，`src/common/` 下的
image_converter——共 19 文件，2026-09-09 自 avm-stitch @ f338aec 原样迁入
（"搬迁非删除"，见 docs/specs/strip-renderer-path.md D1/D7）。namespace 保持
`avm_worker::avm_stitch`、相对 include 路径不变；由静态库 target
`avm_renderer_bridge` 承载，avm-renderer 为硬依赖。avm-stitch 仓已剥离该批
代码，本仓是唯一宿主。判断"这份在哪个仓"：凡 renderer 代理路径的文件均在
本仓；OSG 自研引擎仍在 ../avm-stitch。
```

**worker 全量验证命令:**
```bash
cd /home/leehsiang/Documents/avm-studio/avm-worker
cmake --build build -j$(nproc) && ctest --test-dir build --output-on-failure 2>&1 | tail -5
```
→ 预期: **全 PASS/SKIP，0 FAIL**。测试集合 = 原全部 target + 5 个搬迁对象测试（断言除 D-A 一处翻转外零修改）+ Task 8/9 新用例。GL 门控用例 SKIP 与 Task 0 基线一致。
```bash
grep -rn "avm-stitch" src/stitch_eval/stitch_stream.cpp   # 预期: 仅注释内 provenance 表述, 无 include 改动
git diff --stat -- src/stitch_eval/stitch_stream.cpp       # 预期: 仅 3 行注释变更
```

**How:** 注释措辞可微调，信息点（来源仓、commit、bridge、硬依赖、19 文件）不可缺。

---

## Task 11: avm-stitch README 重写（单路径叙事）

**What:**

**Files:**
- Modify: `/home/leehsiang/Documents/avm-studio/avm-stitch/README.md`

**段落级修改锚点（当前行号）:**

| 当前行号 | 现状 | 改为 |
|---------|------|------|
| L14（§1）| "库内含两套渲染路径:本库自研的 OSG 引擎…以及对闭源 `avm-renderer` 库的**代理后端**" | 单路径叙事: 本库是自研 OSG 拼接引擎；曾内置的闭源 avm-renderer 代理路径已整体迁移至其唯一消费者 avm-worker（并列关系的算法实现，二者共用同一任务 JSON 契约） |
| L187-193（§4 路径表）| 三行: OSG 引擎 / Renderer 代理 / 流式会话 | 只留 OSG 引擎行；删 Renderer 代理行与流式会话行（StitchSession 已迁移），工厂叙述改"`CreateStitchBackend(name)` 只识别 `\"osg\"`，未知名返回 nullptr" |
| L198-238（§5 目录树）| 含 9+1 搬迁文件行、4 个测试 | 删除 `renderer_backend`/`renderer_interface`/`real_renderer`/`stitch_session`/`calib_files_generator`/`renderer_helpers`/`view_mapping`（core）、`egl_gles3_context`/`fbo_manager`（render）、`image_converter`（common）各行及对应说明；测试树只留 9 个 target（用例数删去具体数字或以实测为准） |
| L258（§7 构建依赖段）| "可选依赖:`avm_renderer`（私有 registry）…当前 CMakeLists.txt 无条件 find_package(CLI11…)…后续可改进" | 整段替换为: 依赖仅 `vcpkg.json` 所列公共 registry 包（nlohmann-json、gtest、opencv4、osg）+ 系统 EGL（`find_package(OpenGL COMPONENTS EGL)`）；无任何私有 registry / avm-renderer / CLI11 / glfw3 依赖，干净环境可直接自举。"剥离过渡"占位语句（如有）删除 |
| L311（§8 algorithm 取值）| "`\"osg\"` → OSGBackend,`\"renderer\"` → RendererBackend,未知值工厂返回 nullptr" | "`\"osg\"` → OSGBackend；`\"renderer\"` 与未知值 → 返回 nullptr（renderer 代理路径已迁移至 avm-worker，由 worker 本地构造）" |
| L389-390（§10 限制）| "renderer 路径需要闭源库…" / "renderer 路径固定 4 相机…" 两条 | 删除（该路径不在本库） |
| L340（§8 失败示例前导）| "（`RendererBackend` 部分视图失败会附带…）" | 措辞改为 `OSGBackend` 场景或删除括注 |

**验证命令:**
```bash
cd /home/leehsiang/Documents/avm-studio/avm-stitch
grep -n "avm-renderer\|avm_renderer\|AVMRENDERER\|CLI11\|glfw\|RendererBackend\|StitchSession\|RealRenderer\|renderer_backend\|calib_files\|view_mapping\|fbo_manager\|egl_gles3\|image_converter" README.md
```
→ 预期: 仅允许两类命中——(a) 迁移叙事句中的 "avm-renderer"（L14 新句、§8 取值说明）；(b) 无。其余任何命中（目录树残留、限制条款残留、依赖段残留）→ 未清干净，继续删。

**How:** 具体行文措辞自由；上表的信息点（已迁移、唯一消费者、单路径、自举能力）不可缺，且不得改动 renderer 叙事以外的内容（§2/§3/§9 算法讲解等原样保留）。

---

## Task 12: 合并态最终验收 + 双仓提交（检查点任务）

**What:**

**前置:** 批次 B/C/D/E 全部完成（两仓工作树为剥离+接收的合并态，D8 步骤 3）。

**验收命令（设计 §11 全量，按序锁定）:**

1. **avm-stitch（剥离自足 + 自举）:**
   ```bash
   cd /home/leehsiang/Documents/avm-studio/avm-stitch
   git status --porcelain | head -30        # 预期: 23 个 D + stitch_backend.{cpp,hpp} M + CMakeLists M + README M + 新测试 M, 无意外文件
   cmake --preset default && cmake --build build -j$(nproc) && \
     ctest --test-dir build --output-on-failure 2>&1 | tail -3
   # 预期: 0 tests failed
   grep -rn "avm_renderer\|AVMRENDERER\|CLI11\|glfw\|real_renderer\|renderer_backend" \
        CMakeLists.txt src tests
   # 预期: 无输出
   ```

2. **avm-worker（合并工作区，avm-stitch 为剥离 HEAD）:**
   ```bash
   cd /home/leehsiang/Documents/avm-studio/avm-worker
   cmake --preset default && cmake --build build -j$(nproc) && \
     ctest --test-dir build --output-on-failure 2>&1 | tail -3
   # 预期: 0 tests failed（含 5 个搬迁测试 + dispatch/fail-fast 新用例）
   grep -n "renderer" src/stitch_eval/stitch_eval.cpp | grep -i "factory\|CreateStitchBackend"
   # 预期: 仅 "osg" 分支的 CreateStitchBackend 行命中; renderer 分支无工厂调用
   grep -rn "AVMRENDERER" src CMakeLists.txt
   # 预期: 无输出（D4 双侧确认）
   ```

3. **回归对比:** 两仓测试集合均 ≥ Task 0 基线（扣除刻意移除的 4 个 avm-stitch 搬迁测试 target；worker 侧新增 3 个用例、翻转 1 个用例断言）。

**提交（顺序锁定，avm-stitch 先；均不 push）:**

4. **avm-stitch commit（先）:** 当前分支 `feature/avm-renderer-integration`（D-D）:
   ```bash
   cd /home/leehsiang/Documents/avm-studio/avm-stitch
   git add -A
   git commit   # message 见下
   ```
   commit message 要点（英文/中文均可，why 导向）: 剥离 avm-renderer 代理路径——19 个 renderer 代理文件整体迁移至 avm-worker（镜像路径 + provenance 注释 + `avm_renderer_bridge` 静态库承载，见 avm-worker 仓对应 commit），本库收敛为纯自研 OSG 拼接引擎; 删除 `AVMRENDERER_AVAILABLE` 降级机制与 CLI11/glfw3/avm_renderer 私有依赖，公共 registry 即可自举; `CreateStitchBackend` 只识别 "osg"，"renderer" 返回 nullptr（源 commit f338aec）。

5. **avm-worker commit（后，新建分支，不推 main）:**
   ```bash
   cd /home/leehsiang/Documents/avm-studio/avm-worker
   git checkout -b feature/strip-renderer-proxy
   git add -A
   git commit   # message 见下
   ```
   commit message 要点（D7）: 接收自 avm-stitch @ f338aec（feature/avm-renderer-integration）的 renderer 代理代码——**搬迁非删除**，19 文件镜像落位 `src/{core,render,common}/`（namespace 与 include 路径零改动），新静态库 `avm_renderer_bridge` 承载、avm-renderer 改硬依赖（删 AVMRENDERER_AVAILABLE）; `StitchEvalDispatch` "renderer" 分支本地直调; subprocess 双份标定副本补 0c7fc4d 等价占位符 fail-fast + 新增 dispatch/fail-fast 测试用例; 1 处工厂断言翻转（RoutesRendererName → nullptr，设计 §9 契约）。

**BLOCKED-on-environment 分支:** 若 Task 0 判定 worker 环境不可构建——步骤 2 的构建/ctest 标记"待环境恢复执行"并**暂缓两仓 commit**（无合并态验证不提交，D8 红线），avm-stitch 侧验证照常完成；报告用户等待环境。

**How:** commit message 具体措辞自由，要点不可缺。

---

## 2. 风险与红线（executor 必读）

1. **D8 顺序不可反转:** 批次 D/E 必须在批次 B 之后。avm-stitch 未剥离前，worker 出现 bridge = 同 namespace 双份符号 = 链接必炸（R1）。
2. **禁用 `git checkout` 恢复代码**（全局规则）；中间态修复靠 `git diff` 定位 + 手工编辑。
3. **禁删 build 目录**——增量编译；Task 5 的干净自举在 `/tmp` 独立目录做，不动主 build。
4. **`WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}`**（`test_calib_files_generator` 等模板读取 target）保持不动（R5）。
5. **avm-stitch 侧不引入任何 renderer 接口占位**（非目标红线）；worker 侧不改 redis 协议/`algorithm` 取值集合/测试业务断言（除 D-A 一处）。
6. **base64.h 双副本 shadowing（R2）:** 现状 worker target 的 include 顺序（本地 `src` 优先于 `avm_stitch` 传递路径）不因本计划改变——bridge 的 PUBLIC include 同样排在本地 `PRIVATE src` 之后。不处理，仅确认不加剧。
7. **OpenGL::GLES3 从 avm-stitch 移除（Task 3 锚点 1/4）若导致 OSG 无头路径链接失败**（R6，理论不可能——`egl_headless_context` 仅用 EGL）: 回退为保留 `GLES3` 组件，并在报告中记录实测证据。

---

## 3. 自检记录（Phase 3）

- **规范覆盖:** §5（Task 2/3/4/11）、§6（Task 6/7/8/9/10 + D-B 补 test_stitch_eval）、§7 顺序（批次依赖链 A→B→(C∥D)→E→F，无中间可构建态承诺仅在批间层面——批内 Task 2→3 间的短暂不可构建已显式声明）、§9 错误处理（Task 8 目标形态保留 RendererBackend 错误信封语义）、§10 测试策略（Task 8/9 新用例 + 全量回归 + 基线对比）、§11 成功标准（Task 5/12 命令逐条落地）、D1-D8 全部有落点（D1: Task 6、D2: 全程不改 namespace、D3/D4: Task 6/7、D5: Task 4/8、D6: Task 9、D7: Task 6/12、D8: 批次链）。
- **占位符扫描:** WHAT 部分（文件路径、锚点表、CMake/代码逐字产物、测试断言全文、验证命令+预期输出）无 TBD/TODO/"适当处理"类模式；HOW 部分仅声明执行顺序与措辞自由度。
- **类型一致性:** `avm_renderer_bridge` 命名贯穿 Task 7-12; `avm_stitch::RendererBackend`/`RealRenderer` 构造签名与 `stitch_backend.cpp`/`stitch_stream.cpp` 现状一致; 19 文件清单在 Task 1/2/6 三处完全一致。
- **自包含:** 每任务自带锚点行号+内容、命令、预期；Task 8/9 的 RED 前置条件（avm-stitch 已剥离、bridge 已落位）在批次依赖链中保证。
