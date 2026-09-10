# renderer 路径剥离（avm-stitch / avm-worker 双仓库重构）设计规范

> **状态:** 草稿

**目标:** 将 avm-stitch 中所有 avm-renderer 代理代码（renderer backend 路径）**搬迁**（非删除）到其唯一消费者 avm-worker，使 avm-stitch 成为独立自研 OSG 拼接库（与闭源 avm-renderer 并列的算法实现），同时在 avm-worker 中保住 "renderer" 算法分支与流式（stitch-stream）功能。

**架构:** avm-stitch 收敛为单路径（OSG）；renderer 代理代码镜像落位到 avm-worker 的 `src/` 树内，由一个新的本地静态库 target 承载，worker 侧 "renderer" 分支改为本地直调。namespace 保持 `avm_worker::avm_stitch` 不变，搬迁文件的相对目录结构不变，从而 include 字符串零改动。

**技术栈:** CMake ≥ 3.25 / vcpkg manifest、OpenSceneGraph、OpenCV、nlohmann-json、GTest、hiredis（worker）、avm-renderer 私有 registry port（worker 独有）。

---

## 1. 背景与动机（时间线证据）

avm-stitch 于 2026-08-11 从 avm-worker 拆出（`fa1c79b`，Initial import），定位是"自研 OSG 拼接引擎"。拆出后两天（2026-08-13，`4a58531` "avm_renderer backend + OSGGLTest fix"），avm-renderer 的**代理封装**（RendererBackend/RealRenderer/StitchSession 等）被引入 avm-stitch。这批代码：

- 不包含算法，只是对闭源 avm-renderer 的调用编排；
- 把私有依赖（avm-renderer + 其传导依赖 CLI11、glfw3）带进了 avm-stitch 的构建图，使 avm-stitch 在没有公司私有 registry 的环境下无法配置构建；
- 唯一消费者是 avm-worker（`StitchEvalDispatch` 的 `"renderer"` 分支、`stitch_stream.cpp` 的流式会话管理、`tests/avm_stitch/` 5 个测试）。

后续修复留在 avm-stitch 侧：2026-08-20 `0c7fc4d`（normalize camera label + `EnsureNoUnreplacedPlaceholders` 占位符 fail-fast，防 DBL_MAX 黑帧事故）改动了 `calib_files_generator.cpp` 与 `util/camera_name.hpp`。而 worker `stitch_eval.cpp` 中 subprocess 路径的标定生成代码是**双份副本**（与 avm-stitch 的 calib_files_generator 同构），**未同步** 0c7fc4d 修复——这是本次重构承诺一并消除的隐患。

**现状实测结论（已验证）:**

- avm-stitch 的 OSG 路径对 renderer 路径文件**零依赖**；整个 `src/` 中唯一引用点是 `src/core/stitch_backend.cpp` 的 include 与 `CreateStitchBackend` 的 `"renderer"` 分支。`AVMRENDERER_AVAILABLE` 宏只出现在 `real_renderer.{hpp,cpp}` 和 `stitch_backend.cpp` 三处。
- worker 的 5 个搬迁对象测试（`tests/avm_stitch/`）全部以**相对路径字符串**（如 `#include "core/renderer_backend.hpp"`）include avm-stitch 头文件，靠链接 `avm_stitch` 库获得的 PUBLIC `src/` include 目录解析；测试 target 是**链接整库**而非逐文件列源。`test_calib_files_generator` 以 `WORKING_DIRECTORY=仓库根` 读取 `tools/` 真模板。
- worker `stitch_stream.cpp` include `core/real_renderer.hpp`、`core/renderer_helpers.hpp`、`core/stitch_session.hpp` 三个 avm-stitch 头；`MakeDefaultSession` 直接构造 `StitchSession(RealRenderer)`。
- avm-stitch `vcpkg.json` 已不含 avm-renderer/私有 registry 条目（依赖仅 nlohmann-json、gtest、opencv4、osg）——剥离 CMake 引用后即可自举。
- worker `vcpkg.json` 声明 `avm-renderer >= 0.1.15`（私有 registry）+ `overlay_ports/avm-renderer` 补丁——**保持不动**。

---

## 2. 目标 / 非目标

### 目标

1. avm-stitch 源码、构建、测试、文档中**不含任何 avm-renderer 代理代码与依赖**；干净环境（仅公共 vcpkg registry）可完成配置、构建、测试。
2. renderer 代理代码完整搬迁到 avm-worker，worker 的 `"renderer"` 一次性拼接分支与 `queue:stitch-stream` 流式功能行为不变。
3. 随搬迁将 0c7fc4d 的占位符 fail-fast 修复落到 worker（搬迁副本自带修复 + **同步补进** `stitch_eval.cpp` subprocess 双份副本）。
4. worker 侧 5 个相关测试的断言与期望**零修改**（仅链接关系调整）。

### 非目标

- 不重构 renderer 代理代码内部实现；搬迁即"原样搬家 + 去除可选编译宏"。
- 不改变任何对外协议（redis 任务 JSON、`algorithm` 取值、结果/错误信封格式）。
- 不改变 namespace、不改 avm-worker 测试的业务断言。
- 不在 avm-stitch 中为"未来自研 renderer 等价物"预留任何接口。
- 不处理 worker/avm-stitch 之间 `common/base64.h` 同名头 shadowing 的历史问题（见风险 R2，本次仅确认搬迁不加剧它）。

---

## 3. 目标架构

### 3.1 两仓库职责（剥离后）

```
avm-stitch（独立自研 OSG 库，个人 GitHub）
  src/core/    stitch_backend.{hpp,cpp}(仅"osg"分支) osg_backend stitch_engine stitch_result
  src/render/  osg_scene osg_view egl_headless_context(桌面 GL 3.3)
  src/geometry/ src/util/camera_name.hpp src/common/base64.h src/eval_metrics/
  依赖: nlohmann-json, opencv4, osg, gtest（全公共 registry）

avm-worker（消费者 + renderer 代理宿主，公司 GitLab）
  src/core/    ← 迁入: renderer_backend real_renderer renderer_helpers
                calib_files_generator view_mapping stitch_session renderer_interface.hpp
  src/render/  ← 迁入: egl_gles3_context fbo_manager
  src/common/  ← 迁入: image_converter
  bridge 静态库(建议名 avm_renderer_bridge) = 上述迁入 .cpp 集合
  src/stitch_eval/stitch_eval.cpp   "renderer" → 本地直调 bridge
  src/stitch_eval/stitch_stream.cpp 流式会话（不变）
  依赖: avm-stitch(add_subdirectory, OSG) + avm-renderer(私有 registry, 硬依赖) + hiredis ...
```

### 3.2 组件详情

#### avm_stitch 静态库（变更后）
- **职责:** 自研 OSG 拼接算法库。对外接口 `CreateStitchBackend(name)` 只识别 `"osg"`；`"renderer"` 传入返回 `nullptr`（与未知名字行为一致）。
- **错误处理:** 不再有 `AVMRENDERER_AVAILABLE` 条件编译与"production 返回错误的 renderer 桩"——该降级语义整体消失，由 worker 的硬依赖取代。

#### avm_renderer_bridge 静态库（avm-worker 新增）
- **职责:** 承载全部 avm-renderer 代理代码（`namespace avm_worker::avm_stitch` 不变），对 worker 内部提供 `RendererBackend`/`RealRenderer`/`StitchSession`/`CalibFilesGenerator`/`ImageConverter`/`ViewMapping`/`FboManager`/`EglGles3Context`，并导出 PUBLIC include 根 `src`。
- **依赖:** avm_renderer 导出 target、OpenGL（EGL + GLES3）、OpenCV、nlohmann-json、`avm_stitch`（复用共用的 `stitch_result`、header-only 的 `util/camera_name.hpp`、`common/base64.h`）。
- **设计决策:** avm-renderer 在 bridge 中是**硬依赖**，删除 `AVMRENDERER_AVAILABLE` 条件编译——worker 环境私有 registry 恒可用，可选构建变体没有消费者。

#### StitchEvalDispatch（avm-worker，变更）
- **职责:** 算法分派。`"osg"` → `avm_stitch::CreateStitchBackend("osg")`（工厂仍保留）；`"renderer"` → 在 stitch_eval.cpp 内**本地直接构造** `RendererBackend(std::make_unique<RealRenderer>())`，不再经工厂；`"subprocess"`（默认）与未知值 → 本地 subprocess 管道。行为契约不变：backend 构造失败时同样回退/报错语义与现状一致。

#### StitchSessionManager（avm-worker，基本不变）
- **职责:** `queue:stitch-stream` 会话缓存。代码不动；其 `real_renderer/renderer_helpers/stitch_session` 三个 include 字符串因镜像落位（见 D1）解析到 worker 本地新路径，链接关系由 `avm_stitch` 改/增为 bridge。

---

## 4. 关键设计决策

| # | 决策 | 理由 |
|---|------|------|
| D1 | 搬迁文件**镜像 avm-stitch 相对目录结构**落到 worker 的 `src/core/`、`src/render/`、`src/common/`（19 个文件：core 12 + `renderer_interface.hpp` 1 + render 4 + common 2） | 搬迁对象与保留对象无同名 basename，镜像后 worker include 根同为 `src`，`#include "core/xxx.hpp"` 等字符串**全部零改动**；worker 测试 include 方式（相对路径 + 链接库）已实测如此。代价：worker `src/` 下新增 `core/`、`render/` 目录。 |
| D2 | namespace 保持 `avm_worker::avm_stitch` 不变 | 测试代码零改动；worker 侧对 avm-stitch OSG 头本就使用此命名空间。 |
| D3 | 搬迁代码由独立静态库 target（建议 `avm_renderer_bridge`）承载，而非直接列进 avm-worker 可执行 | worker 有 6 个测试 target 需要复用这批符号，逐 target 列源是重复维护；与 `avm_stitch` 对称，边界清晰。 |
| D4 | worker 侧删除 `AVMRENDERER_AVAILABLE` 宏与可选链接，avm-renderer 为硬依赖 | 该宏机制在 avm-stitch 存在是为了"私有 registry 不可用时降级"；worker 的 vcpkg 环境本就管理私有 registry，降级路径无消费者。 |
| D5 | `CreateStitchBackend` 工厂只保留 `"osg"`；`"renderer"` 由 worker 本地直调 | 工厂属于 avm-stitch，不应知道已搬走的实现；同时兑现"avm-stitch 无 renderer 代理代码"。 |
| D6 | 0c7fc4d 修复双落地：搬迁的 `calib_files_generator.cpp` 自带修复；同时在 `stitch_eval.cpp` subprocess 双份副本补等价 fail-fast | A1 方案承诺消除双副本隐患分叉；本次不合并双份代码（非目标），只同步修复。 |
| D7 | 跨仓库历史：每个搬迁文件头部加 provenance 注释（源仓库 = avm-stitch、源 commit = `f338aec`、日期），worker 侧提交信息记录来源与"搬迁非删除"性质；不做 `git filter-repo` 保史合并 | 跨仓库 `git mv` 无法保史；provenance 注释 + commit message 的性价比高于重写历史。 |
| D8 | 执行顺序 = **先剥 avm-stitch（工作树），再进 worker（工作树），以合并工作区状态统一构建验证，两仓分别提交（avm-stitch 在前）**；过程中不产生、不 tag 任何"中间可构建"状态 | 规避 ODR/重复符号（见 §7）；两仓库在同一工作区，跨仓 PR 原子性只能靠顺序承诺，无 CI 中间态需求。 |

---

## 5. avm-stitch 侧变更清单

**删除文件（19）:** `src/core/{renderer_backend,real_renderer,renderer_helpers,calib_files_generator,view_mapping,stitch_session}.{hpp,cpp}`、`src/core/renderer_interface.hpp`、`src/render/{egl_gles3_context,fbo_manager}.{hpp,cpp}`、`src/common/image_converter.{hpp,cpp}`。

**删除测试（4）:** `tests/core/test_view_mapping.cpp`、`tests/core/test_real_renderer.cpp`、`tests/render/test_fbo_manager.cpp`、`tests/render/test_egl_gles3_context.cpp`（对应 avm-stitch CMakeLists 中 4 个测试 target 一并移除）。

**修改:**
- `src/core/stitch_backend.cpp`:删 `renderer_backend.hpp`/`real_renderer.hpp` include 与 `AVMRENDERER_AVAILABLE` 条件，删 `"renderer"` 分支；保留 `"osg"` 分支与未知名返回 `nullptr`。
- `CMakeLists.txt`:删 `find_package(CLI11/glfw3/avm_renderer)` workaround 块及 `AVMRENDERER_AVAILABLE` 条件链接块；从 `avm_stitch` 源列表移除全部 9 个搬迁 `.cpp`；删除 4 个迁移测试 target 及 `test_fbo_manager`/`test_egl_gles3_context` 内联列出的搬迁源引用；`OpenGL` 组件需求收紧到 OSG 无头路径实际所需（GLES3 组件仅为已搬走的 `egl_gles3_context` 服务，预期移除——由计划阶段以实测链接结果定夺）。
- `vcpkg.json`:**不改**（已无 avm-renderer/私有 registry 条目）。
- `README.md`:双路径叙事改单路径（自研 OSG vs 闭源 avm-renderer 并列关系）；删 CLI11/glfw3/avm-renderer 相关段落；更新目录结构与 `algorithm` 取值说明（avm-stitch 侧仅 `"osg"`）；删"剥离过渡"占位语句。

**保留（两路共用，不动）:** `stitch_backend.hpp`（接口）、`stitch_result.*`、`util/camera_name.hpp`、`common/base64.h`、`render/egl_headless_context`。

---

## 6. avm-worker 侧变更清单

**落入（19 文件，按 D1 镜像路径）+ 每文件 provenance 头注（D7）。**

**新增 bridge 静态库 target（D3/D4）**：源 = 9 个迁入 `.cpp`；PUBLIC 依赖 = avm_renderer 导出 target、OpenGL（EGL/GLES3）、OpenCV、nlohmann-json、avm_stitch；`find_package(CLI11/glfw3)` 的 target 解析 workaround 原样从 avm-stitch CMake 搬来放在 `find_package(avm_renderer CONFIG REQUIRED)` 之前（avm-renderer 改为 REQUIRED，不再有 `AVMRENDERER_AVAILABLE` 宏）。迁入文件内部的 `#ifdef AVMRENDERER_AVAILABLE` 条件删去、无条件编译。

**修改:**
- `src/stitch_eval/stitch_eval.cpp`（按 D5/D6）:`StitchEvalDispatch` 拆分——`"osg"` 走工厂，`"renderer"` include bridge 头并本地直调构造；注释同步（现注释声称 "avm-stitch provides only the in-process osg/renderer backends" 需改写）。subprocess 双份标定生成副本补 `EnsureNoUnreplacedPlaceholders` 等价 fail-fast。
- `src/stitch_eval/stitch_stream.cpp`:代码零改动（include 字符串经 D1 仍有效）；仅 include 行的用途注释从"avm-stitch 头"改述为"worker 本地 bridge 头"。
- `CMakeLists.txt`:`avm-worker` 可执行与相关测试 target 增加链接 bridge。测试落位：`tests/avm_stitch/` 的 5 个测试**文件原位不动、内容不改**，其 target 从"仅链 avm_stitch"改为"链 bridge"（对仍引用 avm-stitch 保留头的 target 保持同时链 avm_stitch）；`WORKING_DIRECTORY=仓库根`（模板读取）保持不变；`test_stitch_backend.cpp`（编译 stitch_eval.cpp）增加链 bridge；`test_stitch_stream`（编译 stitch_stream.cpp）同样增加 bridge。`tests/avm_stitch/` 目录名与"avm_stitch"字样保留（历史落位，非本次议题）。
- `src/AGENTS.md` 或新增说明：`src/core/`、`src/render/` 下 renderer 代理代码的来源与边界（一段即可）。
- `vcpkg.json` / `overlay_ports/`: 不动。

---

## 7. 执行顺序与符号冲突规避

**冲突机理:** namespace 不变（D2）意味着过渡期内 `avm_stitch` 库与 bridge/本地副本会导出**同一批完全同名符号**，且同名头存在两处定义——若先给 worker 添加 bridge 而复用未剥离的 avm-stitch，链接必然重复符号，同名头也可能被 include 顺序解析到不同副本。

**顺序（D8，单趟，无中间可构建态）:**

1. **avm-stitch 剥离**:在其工作树完成 §5 全部删除/修改 → 构建 + 跑其 OSG 测试（此时 avm-stitch 单仓自足）。
2. **worker 搬迁**:在其工作树完成 §6 全部落位/改桥 → 在 avm-stitch 已是剥离态的工作区构建 worker + 全量 ctest。
3. **合并工作区状态统一验证**（§9 两仓标准均跑一遍通过）后才分别提交：avm-stitch commit 在前、worker commit 在后，worker commit message 记录来源 commit（D7）。不引入"过渡 tag"、不做 revert 分支。

**验证顺序不可反转:** avm-stitch 未剥离前，worker 不得出现第二份符号源。

---

## 8. 数据流（重构后）

```
redis task { algorithm }
  └─ StitchEvalDispatch
       ├─ "subprocess"(默认)/未知 → StitchEval 本地管道(双份标定生成副本 + fail-fast[D6]) → stitcher 子进程 → stitch_result
       ├─ "osg"     → CreateStitchBackend("osg") → avm_stitch(OSGBackend) → stitch_result
       └─ "renderer"→ bridge 本地构造 RendererBackend(RealRenderer) → avm_renderer → stitch_result   ← 直调，不经工厂[D5]

queue:stitch-stream
  └─ HandleStitchStream → StitchSessionManager → MakeDefaultSession
       = bridge.StitchSession(bridge.RealRenderer) → 会话缓存/帧转发（语义不变）
```

## 9. 错误处理

- `algorithm:"renderer"` 但 bridge 构造失败/异常 → `RendererBackend` 现有错误信封语义（`MakeStitchError`）不变；不再存在"编译期未开 avm-renderer → 运行期返回错误桩"的降级态（D4）。
- `CreateStitchBackend("renderer")` 在 avm-stitch 侧返回 `nullptr`；worker dispatch 已不经该路径，行为不可达外部。
- 占位符未替换 → 两处标定生成路径（搬迁副本 + subprocess 双份副本）均 fail-fast 抛错，杜绝 DBL_MAX 黑帧输出（0c7fc4d 语义）。
- stitch-stream 帧级错误：错误信封字段与现状一致（`session_id/frame_index/stream`）。

## 10. 测试策略

搬迁即"原样搬家"，测试期望零改动：worker 5 个 `tests/avm_stitch/` 测试全绿，即验证搬迁保真；avm-stitch 保留 OSG 套件全绿，即验证剥离无过切。额外回归维度：dispatch 行为矩阵（subprocess/osg/renderer/未知值 各跑 `test_stitch_backend` 既有用例）、subprocess 副本新增 fail-fast 用例（补 0c7fc4d 对应测试到 worker 侧）、真实 env 的流式冒烟（首帧建会话/多帧复用/超时 sweep/错误信封）。

## 11. 成功标准

**avm-stitch（剥离自足 + 自举）:**
```bash
cmake --preset default              # 干净环境: 无私有 registry、无 avm-renderer, configure 通过
cmake --build build
ctest --test-dir build --output-on-failure   # 全部保留测试通过
grep -rn "avm_renderer\|AVMRENDERER\|CLI11\|glfw\|real_renderer\|renderer_backend" \
     CMakeLists.txt src tests       # 无任何命中
```
另加全新目录 configure（清 vcpkg 缓存或新 clone）验证仅公共 registry 即可自举。

**avm-worker（合并工作区，avm-stitch 为剥离 HEAD）:**
```bash
cmake --preset default && cmake --build build
ctest --test-dir build --output-on-failure
# 预期: 原全部测试 target + 迁移的 5 个 avm_stitch 测试 + subprocess fail-fast 新用例, 全绿
grep -n "renderer" src/stitch_eval/stitch_eval.cpp   # dispatch 仅剩本地直调路径
```
部署级冒烟（需 redis/stitcher 环境，验收标准非自动化门槛）:
- `queue:stitch` 投 `algorithm:"renderer"` 任务 → 合法 `stitch_result` images；
- `queue:stitch-stream` 连续多帧 → 会话复用、`stream:true` 信封；构造非法帧 → `stitch_error` 信封带 `session_id/frame_index`。

## 12. 风险与缓解

| # | 风险 | 缓解 |
|---|------|------|
| R1 | 过渡期双份符号（同 namespace）ODR/重复符号 | D8 顺序：先剥 avm-stitch、worker 后落 bridge；合并工作区统一构建验证 |
| R2 | worker `src/` 与 avm-stitch `src/` 同 basename 头互相 shadow（现状已有 `common/base64.h`；GTest include 顺序决定解析结果） | 已核对：搬迁集合与 avm-stitch 保留集合**无同名 basename**，本次不新增碰撞；bridge PUBLIC 传递 avm_stitch include，行为与现状等价。历史 base64.h 双副本问题范围外，另案 |
| R3 | include 解析仍"意外"指向 avm-stitch 旧头 → 掩盖搬迁文件缺失 | 剥离后 avm-stitch 树内已无这些头（D8 第一步完成即不可能），冲突在剥离前不可能存在（bridge 尚未添加） |
| R4 | worker CI 单独 checkout 时找不到 avm-stitch 或拿到**未剥离**旧 HEAD | 保持现有 `AVM_STITCH_DIR` 缓存变量与双路径 fallback 机制；两 commit 顺序提交（avm-stitch 先），worker CI pin 需同步更新由计划阶段落实 |
| R5 | `test_calib_files_generator` 等以 CWD 读 `tools/` 真模板，搬迁链接方式改变可能扰动工作目录假设 | target 的 `WORKING_DIRECTORY=${CMAKE_SOURCE_DIR}` 保持不动；搬迁不改测试文件内容 |
| R6 | avm-stitch 的 OpenGL 组件收紧（去 GLES3）误伤 OSG 无头路径 | 以构建+`test_egl_headless_context`/`test_stitch_engine` 实测为准；若 OSG 路径传递依赖 GLES3 则保留组件声明（计划阶段定夺，本设计只锁"去 renderer 依赖"意图） |
| R7 | 跨仓库搬迁丢失 git  blame 历史 | D7 provenance 头注 + worker commit message 记录源 commit，代价可接受 |
| R8 | D6 补齐 subprocess 副本 fail-fast → 原本"带病成功"产出黑帧的任务改为报错 | 这是修复意图本身；在冒烟验收中确认对合法标定无影响 |
| R9 | 镜像落位使 worker `src/core`、`src/render` 与 avm-stitch 目录同名，日后阅读混淆（"这份在哪个仓?"） | provenance 头注 + `src/AGENTS.md` 边界说明（§6 末项） |

## 13. 约束与假设

- **硬约束:** 两仓库并列、独立远端（个人 GitHub / 公司 GitLab），跨仓库 `git mv` 不可行；namespace `avm_worker::avm_stitch` 全程不变；redis 协议与 `algorithm` 取值集合不变。
- **假设:** worker 构建环境恒定可达私有 registry（avm-renderer 硬依赖成立）；avm-stitch 剥离后的 OSG 路径不依赖任何待迁文件（已全仓 grep 实测，唯一耦合点只有 stitch_backend.cpp）；同一工作区双仓库路径 `../avm-stitch` 关系保持。
- **不做:** 不合并 worker 内 subprocess 双份标定代码与搬迁的 CalibFilesGenerator（只同步 fail-fast 修复）；不为 avm-stitch 引入任何 renderer 相关接口占位；不改 avm-stitch 除 renderer 叙事以外的 README 内容。
