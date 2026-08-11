# Stage 2 目录重构 — 实现计划

> **设计文档:** `docs/specs/reorganize-directories.md`
> **前置:** Stage 1 已完成（文件已按功能域分类到 `src/avm_stitch/{geometry,render,core,util}/` 和 `tests/avm_stitch/{geometry,render,core,util}/`）
> **目标:** 去掉 `avm_stitch/` 中间层，将 `common/` 和 `eval_metrics/` 移入 `src/`，统一 include 根为 `${CMAKE_CURRENT_SOURCE_DIR}/src`

---

## TDD 适配说明

本计划是纯重构（文件移动 + 文本替换），不改变任何代码逻辑。TDD RED-GREEN-REFACTOR 结构适配为：

| 阶段 | 含义 |
|------|------|
| **RED** | 验证变更前的**当前状态**（确认旧路径存在、旧 include 模式存在、当前构建通过） |
| **Verify RED** | 确认 RED 验证通过（观察预期输出） |
| **GREEN** | 执行变更操作（git mv、sed 替换、CMake 编辑） |
| **Verify GREEN** | 验证变更后状态（确认新路径存在、旧模式已清除、构建通过） |
| **REFACTOR** | 清理空目录、删除残留 |

---

## Phase 0: 引用验证

验证日期: 2026-08-11

### 0.1 文件路径验证

所有路径已通过 `ls` / `glob` / `read` 确认存在：

**当前状态（Stage 1 完成后）:**

```
src/avm_stitch/
├── geometry/       (6 文件: blend_weights.{hpp,cpp}, bowl_model.hpp, mesh_bake.{hpp,cpp}, projection.hpp)
├── render/         (6 文件: egl_headless_context.{hpp,cpp}, osg_scene.{hpp,cpp}, osg_view.{hpp,cpp})
├── core/           (10 文件: osg_backend.{hpp,cpp}, stitch_backend.{hpp,cpp}, stitch_engine.{hpp,cpp},
│                        stitch_result.{hpp,cpp}, subprocess_backend.{hpp,cpp})
└── util/           (2 文件: camera_name.hpp, view_config.hpp)

tests/avm_stitch/
├── geometry/       (4 文件: test_blend_weights.cpp, test_bowl_model.cpp, test_mesh_bake.cpp, test_projection.cpp)
├── render/         (2 文件: test_egl_headless_context.cpp, test_osg_scene.cpp)
├── core/           (2 文件: test_stitch_engine.cpp, test_stitch_engine_regression.cpp)
└── util/           (1 文件: test_view_config.cpp)

common/base64.h
eval_metrics/eval_metrics.hpp
```

- [x] `src/avm_stitch/` 含 `geometry`, `render`, `core`, `util` 4 个子目录（共 24 文件）
- [x] `tests/avm_stitch/` 含 `geometry`, `render`, `core`, `util` 4 个子目录（共 9 文件）
- [x] `common/base64.h` 存在于仓库根
- [x] `eval_metrics/eval_metrics.hpp` 存在于仓库根

### 0.2 include 模式验证

```
rg '#include "avm_stitch/' src/ → 33 处匹配（17 个文件）
rg '#include "avm_stitch/' tests/ → 23 处匹配（9 个文件）
总计: 56 处（26 个文件）
```

- [x] src/ 和 tests/ 目录下的 `#include "avm_stitch/` 模式存在且数量吻合设计文档
- [x] `#include "common/base64.h"` 存在于 `src/avm_stitch/core/osg_backend.cpp:7`（1 处，文本不变）
- [x] `#include "eval_metrics/eval_metrics.hpp"` 存在于 `src/avm_stitch/core/stitch_result.cpp:3`（1 处，文本不变）

### 0.3 冲突检查

- [x] `src/geometry/`、`src/render/`、`src/core/`、`src/util/`、`src/common/`、`src/eval_metrics/` 均**不存在**（无冲突）
- [x] `tests/geometry/`、`tests/render/`、`tests/core/`、`tests/util/` 均**不存在**（无冲突）

---

## Phase 1: 文件结构映射

### 移动到 src/（去掉中间层 + 整合）

| # | 从 | 到 |
|---|-----|-----|
| 1 | `common/base64.h` | `src/common/base64.h` |
| 2 | `eval_metrics/eval_metrics.hpp` | `src/eval_metrics/eval_metrics.hpp` |
| 3 | `src/avm_stitch/geometry/` (6 文件) | `src/geometry/` |
| 4 | `src/avm_stitch/render/` (6 文件) | `src/render/` |
| 5 | `src/avm_stitch/core/` (10 文件) | `src/core/` |
| 6 | `src/avm_stitch/util/` (2 文件) | `src/util/` |

### 移动到 tests/（去掉中间层）

| # | 从 | 到 |
|---|-----|-----|
| 7 | `tests/avm_stitch/geometry/` (4 文件) | `tests/geometry/` |
| 8 | `tests/avm_stitch/render/` (2 文件) | `tests/render/` |
| 9 | `tests/avm_stitch/core/` (2 文件) | `tests/core/` |
| 10 | `tests/avm_stitch/util/` (1 文件) | `tests/util/` |

### 删除的旧目录

| # | 目录 |
|---|------|
| 11 | `src/avm_stitch/`（git mv 后自动为空；若残留则 rmdir） |
| 12 | `tests/avm_stitch/`（同上） |
| 13 | `common/`（仓库根；git mv 后自动消失） |
| 14 | `eval_metrics/`（仓库根；git mv 后自动消失） |

---

## Phase 2: 任务分解

---

### Task 0: 基线 — 构建当前 Stage 1 状态并运行全量测试

**行为描述:**
- 从当前 Stage 1 代码构建项目，确保所有测试通过，作为 Stage 2 变更前的安全基线。
- 无副作用（构建产物写入 `build/` 目录）。

**测试策略:**
- 构建成功 → 预期: cmake 配置零错误，编译零错误
- 测试运行 → 预期: 所有 9 个测试全部 PASSED

- [ ] **Step 1: 写入基线检查（RED）**
  确认 build 目录不存在（或可选择删除旧 build 以从干净状态开始）:
  ```bash
  ls build/ 2>&1 || echo "build directory absent — will build from scratch"
  ```

- [ ] **Step 2: 验证基线检查（Verify RED）**
  预期输出: build 目录不存在（或如存在则确认是 Stage 1 构建产物）

- [ ] **Step 3: 构建 + 测试（GREEN）**
  ```bash
  cmake --preset default && cmake --build build -j$(nproc) && ctest --test-dir build --output-on-failure
  ```

- [ ] **Step 4: 验证构建和测试通过（Verify GREEN）**
  预期: cmake 配置成功（无错误），编译成功（零错误），ctest 全部 PASSED（9/9 tests passed）

- [ ] **Step 5: 重构（REFACTOR）**
  REFACTOR_SKIPPED: 基线验证，无代码坏味道。

---

### Task 1: 移动 `common/` 和 `eval_metrics/` 到 `src/` 下

**Files:**
- Move: `common/base64.h` → `src/common/base64.h`
- Move: `eval_metrics/eval_metrics.hpp` → `src/eval_metrics/eval_metrics.hpp`

**行为描述:**
- 使用 `git mv` 将仓库根下的 `common/` 和 `eval_metrics/` 两个目录整体移入 `src/`。
- 移动后，旧路径 `common/base64.h` 和 `eval_metrics/eval_metrics.hpp` 不再存在于仓库根。
- 副作用: git 索引更新，工作树文件位置变更。

- [ ] **Step 1: 确认旧路径存在（RED）**
  ```bash
  ls common/base64.h eval_metrics/eval_metrics.hpp
  ```
  预期: 两个文件路径都显示文件存在（无 "No such file" 错误）

- [ ] **Step 2: 验证 RED（Verify RED）**
  预期输出: `common/base64.h` `eval_metrics/eval_metrics.hpp`（两行文件名输出）

- [ ] **Step 3: 执行 git mv（GREEN）**
  ```bash
  git mv common src/common
  git mv eval_metrics src/eval_metrics
  ```

- [ ] **Step 4: 验证移动成功（Verify GREEN）**
  ```bash
  ls src/common/base64.h src/eval_metrics/eval_metrics.hpp
  ```
  预期: 两文件在新路径下存在
  ```bash
  ls common/ 2>&1 || echo "common/ 已不存在于仓库根"
  ls eval_metrics/ 2>&1 || echo "eval_metrics/ 已不存在于仓库根"
  ```
  预期: 仓库根 `common/` 和 `eval_metrics/` 目录已消失

- [ ] **Step 5: 查看 git 状态确认**
  ```bash
  git status --short | head -20
  ```
  预期: 看到 `R common/base64.h -> src/common/base64.h` 和 `R eval_metrics/eval_metrics.hpp -> src/eval_metrics/eval_metrics.hpp`（git 识别为重命名）

- [ ] **Step 6: 重构（REFACTOR）**
  REFACTOR_SKIPPED: 纯文件移动，无代码坏味道。

---

### Task 2: 去掉 `src/avm_stitch/` 中间层 — 移动 4 个子目录到 `src/`

**Files:**
- Move: `src/avm_stitch/geometry/` (6 文件) → `src/geometry/`
- Move: `src/avm_stitch/render/` (6 文件) → `src/render/`
- Move: `src/avm_stitch/core/` (10 文件) → `src/core/`
- Move: `src/avm_stitch/util/` (2 文件) → `src/util/`

**行为描述:**
- 使用 `git mv` 将 `src/avm_stitch/` 下的 4 个子目录整体提升一级到 `src/` 下。
- 副作用: git 索引更新，`src/avm_stitch/` 成为空目录。

- [ ] **Step 1: 确认旧路径存在（RED）**
  ```bash
  ls src/avm_stitch/geometry/blend_weights.cpp
  ls src/avm_stitch/render/osg_scene.cpp
  ls src/avm_stitch/core/stitch_engine.cpp
  ls src/avm_stitch/util/view_config.hpp
  ```
  预期: 4 个文件都显示路径存在

- [ ] **Step 2: 确认新路径不存在（RED）**
  ```bash
  ls src/geometry/ 2>&1 || echo "NOT EXIST — expected"
  ls src/render/ 2>&1 || echo "NOT EXIST — expected"
  ls src/core/ 2>&1 || echo "NOT EXIST — expected"
  ls src/util/ 2>&1 || echo "NOT EXIST — expected"
  ```
  预期: 4 个目录均不存在

- [ ] **Step 3: 验证 RED（Verify RED）**
  预期: Step 1 全部成功（旧路径存在），Step 2 全部显示 "NOT EXIST"

- [ ] **Step 4: 执行 git mv（GREEN）**
  ```bash
  git mv src/avm_stitch/geometry src/geometry
  git mv src/avm_stitch/render src/render
  git mv src/avm_stitch/core src/core
  git mv src/avm_stitch/util src/util
  ```

- [ ] **Step 5: 验证新路径存在（Verify GREEN）**
  ```bash
  ls src/geometry/blend_weights.cpp src/render/osg_scene.cpp src/core/stitch_engine.cpp src/util/view_config.hpp
  ```
  预期: 4 个文件在新路径下存在

- [ ] **Step 6: 验证旧路径消失（Verify GREEN）**
  ```bash
  ls src/avm_stitch/ 2>&1 || echo "src/avm_stitch/ is empty or removed"
  ```
  预期: `src/avm_stitch/` 目录为空（git mv 后子目录已移除，父目录可能残留为空）

- [ ] **Step 7: 重构（REFACTOR）**
  REFACTOR_SKIPPED: 纯文件移动，无代码坏味道。（空目录清除在 Task 4 中统一处理）

---

### Task 3: 去掉 `tests/avm_stitch/` 中间层 — 移动 4 个子目录到 `tests/`

**Files:**
- Move: `tests/avm_stitch/geometry/` (4 文件) → `tests/geometry/`
- Move: `tests/avm_stitch/render/` (2 文件) → `tests/render/`
- Move: `tests/avm_stitch/core/` (2 文件) → `tests/core/`
- Move: `tests/avm_stitch/util/` (1 文件) → `tests/util/`

**行为描述:**
- 使用 `git mv` 将 `tests/avm_stitch/` 下的 4 个子目录整体提升一级到 `tests/` 下。
- 副作用: git 索引更新，`tests/avm_stitch/` 成为空目录。

- [ ] **Step 1: 确认旧路径存在（RED）**
  ```bash
  ls tests/avm_stitch/geometry/test_bowl_model.cpp
  ls tests/avm_stitch/render/test_osg_scene.cpp
  ls tests/avm_stitch/core/test_stitch_engine.cpp
  ls tests/avm_stitch/util/test_view_config.cpp
  ```
  预期: 4 个文件都显示路径存在

- [ ] **Step 2: 确认新路径不存在（RED）**
  ```bash
  ls tests/geometry/ 2>&1 || echo "NOT EXIST — expected"
  ls tests/render/ 2>&1 || echo "NOT EXIST — expected"
  ls tests/core/ 2>&1 || echo "NOT EXIST — expected"
  ls tests/util/ 2>&1 || echo "NOT EXIST — expected"
  ```
  预期: 4 个目录均不存在

- [ ] **Step 3: 验证 RED（Verify RED）**
  预期: Step 1 全部成功（旧路径存在），Step 2 全部显示 "NOT EXIST"

- [ ] **Step 4: 执行 git mv（GREEN）**
  ```bash
  git mv tests/avm_stitch/geometry tests/geometry
  git mv tests/avm_stitch/render tests/render
  git mv tests/avm_stitch/core tests/core
  git mv tests/avm_stitch/util tests/util
  ```

- [ ] **Step 5: 验证新路径存在（Verify GREEN）**
  ```bash
  ls tests/geometry/test_bowl_model.cpp tests/render/test_osg_scene.cpp tests/core/test_stitch_engine.cpp tests/util/test_view_config.cpp
  ```
  预期: 4 个文件在新路径下存在

- [ ] **Step 6: 验证旧路径消失（Verify GREEN）**
  ```bash
  ls tests/avm_stitch/ 2>&1 || echo "tests/avm_stitch/ is empty or removed"
  ```
  预期: `tests/avm_stitch/` 目录为空

- [ ] **Step 7: 重构（REFACTOR）**
  REFACTOR_SKIPPED: 纯文件移动，无代码坏味道。

---

### Task 4: 清理空的中间目录

**Files:**
- Remove: `src/avm_stitch/`（若仍为空的目录条目）
- Remove: `tests/avm_stitch/`（若仍为空的目录条目）

**行为描述:**
- 确认 `src/avm_stitch/` 和 `tests/avm_stitch/` 已空后，用 `rmdir` 删除空目录。git 不跟踪空目录，删除后工作树清洁。
- 副作用: 无（git 不跟踪空目录，不影响索引）

- [ ] **Step 1: 确认目录为空（RED）**
  ```bash
  ls -la src/avm_stitch/ 2>&1
  ls -la tests/avm_stitch/ 2>&1
  ```
  预期: 两个目录若存在则只含 `.` 和 `..`（即空目录）；若不存在则表示 git mv 已自动清除

- [ ] **Step 2: 验证 RED（Verify RED）**
  预期: 确认目录状态 — 空/不存在均可进入下一步

- [ ] **Step 3: 删除空目录（GREEN）**
  ```bash
  rmdir src/avm_stitch/ 2>/dev/null || echo "src/avm_stitch/ already gone"
  rmdir tests/avm_stitch/ 2>/dev/null || echo "tests/avm_stitch/ already gone"
  ```

- [ ] **Step 4: 验证删除（Verify GREEN）**
  ```bash
  ls src/avm_stitch/ 2>&1 || echo "OK: src/avm_stitch/ removed"
  ls tests/avm_stitch/ 2>&1 || echo "OK: tests/avm_stitch/ removed"
  ```
  预期: 两个目录均报 "No such file or directory"

- [ ] **Step 5: 重构（REFACTOR）**
  REFACTOR_SKIPPED: 清理操作，无代码坏味道。

---

### Task 5: 批量更新 #include 指令 — 去掉 `avm_stitch/` 前缀

**Files (26 个待修改文件，56 处待替换):**

**src/ 下 17 个文件（33 处）:**
| 文件 (移动后路径) | #include 行数 |
|---|---|
| `src/render/osg_view.cpp` | 2 |
| `src/core/osg_backend.cpp` | 5 |
| `src/geometry/mesh_bake.cpp` | 2 |
| `src/geometry/blend_weights.hpp` | 1 |
| `src/core/stitch_result.cpp` | 1 |
| `src/geometry/blend_weights.cpp` | 1 |
| `src/core/subprocess_backend.cpp` | 1 |
| `src/core/stitch_engine.hpp` | 2 |
| `src/render/osg_view.hpp` | 1 |
| `src/core/stitch_engine.cpp` | 6 |
| `src/core/osg_backend.hpp` | 1 |
| `src/geometry/mesh_bake.hpp` | 2 |
| `src/core/stitch_backend.cpp` | 3 |
| `src/render/egl_headless_context.cpp` | 1 |
| `src/render/osg_scene.hpp` | 2 |
| `src/core/subprocess_backend.hpp` | 1 |
| `src/render/osg_scene.cpp` | 1 |

**tests/ 下 9 个文件（23 处）:**
| 文件 (移动后路径) | #include 行数 |
|---|---|
| `tests/geometry/test_projection.cpp` | 1 |
| `tests/util/test_view_config.cpp` | 1 |
| `tests/geometry/test_mesh_bake.cpp` | 3 |
| `tests/render/test_egl_headless_context.cpp` | 1 |
| `tests/geometry/test_blend_weights.cpp` | 2 |
| `tests/core/test_stitch_engine_regression.cpp` | 4 |
| `tests/render/test_osg_scene.cpp` | 3 |
| `tests/geometry/test_bowl_model.cpp` | 1 |
| `tests/core/test_stitch_engine.cpp` | 7 |

**行为描述:**
- 将 `#include "avm_stitch/<subdir>/xxx.hpp"` 改为 `#include "<subdir>/xxx.hpp"`（去掉 `avm_stitch/` 前缀）。
- `#include "common/base64.h"` 文本不变（1 处，`src/core/osg_backend.cpp`）。
- `#include "eval_metrics/eval_metrics.hpp"` 文本不变（1 处，`src/core/stitch_result.cpp`）。
- 使用 `sed` 批量替换，一次处理所有文件。
- 副作用: 修改 26 个文件内容。

- [ ] **Step 1: 记录替换前 include 数量（RED）**
  ```bash
  rg '#include "avm_stitch/' src/ tests/ --count
  ```
  预期: 输出 26 个文件，每行显示匹配次数，总和 56

- [ ] **Step 2: 验证 RED（Verify RED）**
  预期: 56 处待替换的 include 确认存在

- [ ] **Step 3: 执行 sed 批量替换（GREEN）**
  使用 find + sed 对所有 .hpp、.cpp、.h 文件执行替换：
  ```bash
  find src/ tests/ -type f \( -name "*.hpp" -o -name "*.cpp" -o -name "*.h" \) \
    -exec sed -i 's|#include "avm_stitch/|#include "|g' {} +
  ```

- [ ] **Step 4: 验证替换完成 — 无旧模式残留（Verify GREEN）**
  ```bash
  rg '#include "avm_stitch/' src/ tests/
  ```
  预期: **零匹配**（exit code 1）

- [ ] **Step 5: 验证 common/ 和 eval_metrics/ include 未被误改（Verify GREEN）**
  ```bash
  rg '#include "common/base64.h"' src/
  rg '#include "eval_metrics/eval_metrics.hpp"' src/
  ```
  预期: 各匹配 1 次（`src/core/osg_backend.cpp` 和 `src/core/stitch_result.cpp`）

- [ ] **Step 6: 抽样验证替换后的 include 行格式（Verify GREEN）**
  ```bash
  head -1 src/geometry/blend_weights.cpp
  head -1 src/core/stitch_engine.cpp
  head -8 tests/core/test_stitch_engine.cpp
  ```
  预期: 示例 include 行为 `#include "geometry/blend_weights.hpp"`、`#include "core/stitch_engine.hpp"`、`#include "geometry/bowl_model.hpp"` 等（不含 `avm_stitch/` 前缀）

- [ ] **Step 7: 重构（REFACTOR）**
  REFACTOR_SKIPPED: sed 替换是机械操作，无代码坏味道。

---

### Task 6: 更新 CMakeLists.txt

**Files:**
- Modify: `CMakeLists.txt`（32 处路径替换 + 1 处 include 目录行删除）

**行为描述:**
1. 删除 `target_include_directories(avm_stitch PUBLIC ...)` 中的 `${CMAKE_CURRENT_SOURCE_DIR}` 行（第 45 行），只保留 `${CMAKE_CURRENT_SOURCE_DIR}/src` 行（第 44 行）。
2. 将所有 `src/avm_stitch/` 替换为 `src/`（23 处）。
3. 将所有 `tests/avm_stitch/` 替换为 `tests/`（9 处）。
4. 保留所有其他内容不变（库目标名 `avm_stitch`、`project(avm-stitch)`、链接库、测试配置等）。

**受影响的代码区域:**

```cmake
# 变更前 (行 43-46):
target_include_directories(avm_stitch PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 变更后:
target_include_directories(avm_stitch PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
```

**受影响的路径（32 处）:**
| 模式 | 出现次数 | 示例 |
|------|---------|------|
| `src/avm_stitch/` → `src/` | 23 | `src/avm_stitch/geometry/blend_weights.cpp` → `src/geometry/blend_weights.cpp` |
| `tests/avm_stitch/` → `tests/` | 9 | `tests/avm_stitch/geometry/test_bowl_model.cpp` → `tests/geometry/test_bowl_model.cpp` |

- [ ] **Step 1: 确认 CMakeLists.txt 当前状态（RED）**
  ```bash
  rg 'src/avm_stitch/' CMakeLists.txt --count
  rg 'tests/avm_stitch/' CMakeLists.txt --count
  rg 'CMAKE_CURRENT_SOURCE_DIR}' CMakeLists.txt
  ```
  预期: 23 处 `src/avm_stitch/`，9 处 `tests/avm_stitch/`，2 处 `CMAKE_CURRENT_SOURCE_DIR}` 匹配（第 44 行 `${CMAKE_CURRENT_SOURCE_DIR}/src` 和第 45 行 `${CMAKE_CURRENT_SOURCE_DIR}`）

- [ ] **Step 2: 验证 RED（Verify RED）**
  预期: 确认当前 CMakeLists.txt 含 32 处旧路径 + 2 行 include 目录

- [ ] **Step 3: 执行替换 — 源文件路径（GREEN）**
  ```bash
  sed -i 's|src/avm_stitch/|src/|g' CMakeLists.txt
  ```

- [ ] **Step 4: 执行替换 — 测试文件路径（GREEN）**
  ```bash
  sed -i 's|tests/avm_stitch/|tests/|g' CMakeLists.txt
  ```

- [ ] **Step 5: 删除 `${CMAKE_CURRENT_SOURCE_DIR}` 行（GREEN）**
  使用精确行匹配删除。当前变更后该行内容为:
  ```
      ${CMAKE_CURRENT_SOURCE_DIR}
  ```
  在 `target_include_directories(avm_stitch PUBLIC` 块内。执行:
  ```bash
  sed -i '/^    \${CMAKE_CURRENT_SOURCE_DIR}$/d' CMakeLists.txt
  ```
  注意: 此 sed 删除所有**恰好匹配** `    ${CMAKE_CURRENT_SOURCE_DIR}` 的行。在 CMakeLists.txt 中不会误删其他内容（只有一处匹配）。

- [ ] **Step 6: 验证路径替换完成（Verify GREEN）**
  ```bash
  rg 'src/avm_stitch/' CMakeLists.txt
  rg 'tests/avm_stitch/' CMakeLists.txt
  ```
  预期: **零匹配**（exit code 1 两次）

- [ ] **Step 7: 验证新路径正确（Verify GREEN）**
  ```bash
  rg 'src/geometry/blend_weights.cpp' CMakeLists.txt
  rg 'src/render/egl_headless_context.cpp' CMakeLists.txt
  rg 'src/core/stitch_engine.cpp' CMakeLists.txt
  rg 'tests/geometry/test_bowl_model.cpp' CMakeLists.txt
  rg 'tests/core/test_stitch_engine.cpp' CMakeLists.txt
  ```
  预期: 各至少 1 次匹配

- [ ] **Step 8: 验证 include 目录变更（Verify GREEN）**
  ```bash
  rg 'CMAKE_CURRENT_SOURCE_DIR' CMakeLists.txt
  ```
  预期: 输出只显示 `${CMAKE_CURRENT_SOURCE_DIR}/src` 的匹配（不再有单独的 `${CMAKE_CURRENT_SOURCE_DIR}`），都是 `PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src` 或 `PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src` 形式。

- [ ] **Step 9: 验证关键不变项（Verify GREEN）**
  ```bash
  rg 'add_library\(avm_stitch' CMakeLists.txt
  rg 'project\(avm-stitch' CMakeLists.txt
  rg 'vcpkg' CMakeLists.txt
  ```
  预期: 库目标名 `avm_stitch`、项目名 `avm-stitch` 不变；`find_package` 调用不变。

- [ ] **Step 10: 重构（REFACTOR）**
  REFACTOR_SKIPPED: sed 替换是机械操作，无代码坏味道。

---

### Task 7: 完整构建验证

**行为描述:**
- 删除旧 `build/` 目录（确保从零开始重新配置，避免 Stage 1 缓存干扰）。
- 运行 `cmake --preset default` 重新配置。
- 运行 `cmake --build build` 编译。
- 所有目标必须成功编译，零错误零警告（或仅允许 Stage 1 即存在的警告）。

**前置条件:** Task 1-6 全部完成。

- [ ] **Step 1: 删除旧 build 目录（RED — 确认需要从头构建）**
  ```bash
  rm -rf build
  ls build/ 2>&1 || echo "build directory removed"
  ```
  预期: build 目录已删除

- [ ] **Step 2: 验证 RED（Verify RED）**
  预期: "build directory removed" 消息

- [ ] **Step 3: cmake 配置（GREEN）**
  ```bash
  cmake --preset default
  ```
  预期: 配置成功，无 "Cannot find source file" 或 "No such file or directory" 错误

- [ ] **Step 4: 编译（GREEN）**
  ```bash
  cmake --build build -j$(nproc)
  ```
  预期: 编译成功，所有 10 个 .cpp 文件（库） + 9 个测试文件编译通过，零错误

- [ ] **Step 5: 验证编译成功（Verify GREEN）**
  检查 cmake --build 的退出码为 0，输出末尾无 "Error" 字样。

- [ ] **Step 6: 重构（REFACTOR）**
  REFACTOR_SKIPPED: 构建验证，无代码坏味道。

---

### Task 8: 全量测试验证

**行为描述:**
- 运行 `ctest --test-dir build` 执行全部 9 个单元测试。
- 全部测试必须通过（PASSED）。

**前置条件:** Task 7 构建成功。

- [ ] **Step 1: RED — 不适用（测试不存在失败预期，此为回归验证）**
  跳过 RED 步骤。此任务是纯验证：确认重构后测试行为不变。

- [ ] **Step 2: 运行 ctest（GREEN）**
  ```bash
  ctest --test-dir build --output-on-failure
  ```

- [ ] **Step 3: 验证测试结果（Verify GREEN）**
  预期输出:
  ```
  100% tests passed, 0 tests failed out of 9
  ```
  所有 9 个测试名称全部显示 "Passed"。

- [ ] **Step 4: 重构（REFACTOR）**
  REFACTOR_SKIPPED: 测试验证，无代码坏味道。

---

### Task 9: 最终静态验证清单

**行为描述:**
- 逐一核对目标目录结构和残留检查清单。
- 确认没有任何遗漏的旧路径、旧 include 模式。

- [ ] **Step 1: 验证 src/ 目录结构（GREEN）**
  ```bash
  ls src/common/base64.h
  ls src/eval_metrics/eval_metrics.hpp
  ls src/geometry/blend_weights.cpp src/geometry/bowl_model.hpp src/geometry/mesh_bake.cpp src/geometry/projection.hpp
  ls src/render/egl_headless_context.cpp src/render/osg_scene.cpp src/render/osg_view.cpp
  ls src/core/stitch_engine.cpp src/core/stitch_backend.cpp src/core/osg_backend.cpp src/core/subprocess_backend.cpp src/core/stitch_result.cpp
  ls src/util/camera_name.hpp src/util/view_config.hpp
  ```
  预期: 全部路径存在，无错误

- [ ] **Step 2: 验证 tests/ 目录结构（GREEN）**
  ```bash
  ls tests/geometry/test_bowl_model.cpp tests/geometry/test_projection.cpp tests/geometry/test_mesh_bake.cpp tests/geometry/test_blend_weights.cpp
  ls tests/render/test_osg_scene.cpp tests/render/test_egl_headless_context.cpp
  ls tests/core/test_stitch_engine.cpp tests/core/test_stitch_engine_regression.cpp
  ls tests/util/test_view_config.cpp
  ```
  预期: 全部 9 个测试文件路径存在，无错误

- [ ] **Step 3: 确认旧目录已删除（GREEN）**
  ```bash
  for dir in src/avm_stitch tests/avm_stitch common eval_metrics; do
    ls "$dir" 2>&1 || echo "OK: $dir removed"
  done
  ```
  预期: 4 个旧目录全部报 "No such file or directory"

- [ ] **Step 4: 验证无残留 `avm_stitch/` include（GREEN）**
  ```bash
  rg '#include "avm_stitch/' src/ tests/
  ```
  预期: **零匹配**（exit code 1）

- [ ] **Step 5: 验证无残留旧路径于 CMakeLists.txt（GREEN）**
  ```bash
  rg 'src/avm_stitch/|tests/avm_stitch/' CMakeLists.txt
  ```
  预期: **零匹配**（exit code 1）

- [ ] **Step 6: 验证 CMakeLists.txt include 目录配置（GREEN）**
  ```bash
  rg 'CMAKE_CURRENT_SOURCE_DIR' CMakeLists.txt
  ```
  预期: 所有匹配均为 `${CMAKE_CURRENT_SOURCE_DIR}/src` 形式（包含尾部 `/src`），不再有单独的 `${CMAKE_CURRENT_SOURCE_DIR}` 行。

- [ ] **Step 7: 验证 git 状态清洁（GREEN）**
  ```bash
  git status --short
  ```
  预期: 仅显示预期变更 — 文件重命名（R）、修改（M 的 CMakeLists.txt 和 include 更新文件）。无意外的新增/删除文件。

- [ ] **Step 8: 重构（REFACTOR）**
  REFACTOR_SKIPPED: 最终验证，无代码坏味道。

---

## Phase 3: 自检

### 3.1 规范覆盖

| 设计文档章节 | 对应任务 |
|-------------|---------|
| 目标: common/ → src/common/ | Task 1 |
| 目标: eval_metrics/ → src/eval_metrics/ | Task 1 |
| 目标: src/avm_stitch/ → src/ | Task 2 |
| 目标: tests/avm_stitch/ → tests/ | Task 3 |
| 目标: 删除空目录 | Task 4 |
| 目标: #include 更新 | Task 5 |
| 目标: CMakeLists.txt 更新 | Task 6 |
| 目标: cmake 重新配置 + 编译 | Task 7 |
| 目标: 全部单元测试通过 | Task 8 |
| 成功标准 #1-#12 | Task 9 |

### 3.2 占位符扫描

- [x] 无 "TBD"、"TODO"、"稍后实现"
- [x] 无 "添加适当的错误处理"、"添加验证"、"处理边缘情况"
- [x] 无 "为上述写测试"（未提供实际测试代码）
- [x] 无 "类似任务 N"（每个任务完整独立）
- [x] 所有步骤含精确命令和预期输出
- [x] 所有文件路径已验证存在

### 3.3 类型一致性

- [x] 所有任务中的文件路径、sed 模式、rg 搜索模式一致
- [x] CMakeLists.txt 变更与文件移动目标一致
- [x] 库目标名 `avm_stitch` 在 CMakeLists.txt 中保持不变（Task 6 Step 9 验证）
- [x] `#include "common/base64.h"` 和 `#include "eval_metrics/eval_metrics.hpp"` 文本不变（Task 5 Step 5 验证）

### 3.4 自包含性

- [x] 每个任务包含所有必要的命令，不依赖"理解上下文"
- [x] 每个任务的 Step 1（RED）独立验证该任务的前置状态
- [x] 任务间依赖通过"前置条件"明确标注（Task 7 依赖 Task 1-6，Task 8 依赖 Task 7）

---

## 红旗检查

- [x] 无任何形式的 "TBD"、"TODO"
- [x] 无步骤描述缺少行为描述或测试策略
- [x] 所有验证步骤有精确命令和预期输出
- [x] 引用的所有文件路径已通过 Phase 0 验证
- [x] 计划文件输出到 `docs/plans/` 目录（与设计文档 `docs/specs/` 分离）
- [x] tdd skill 和 refactoring skill 已加载并在计划头部引述

---

## 输出

```
计划路径: docs/plans/reorganize-directories-stage2.md
任务数量: 10 (Task 0-9)
关键依赖:
  Task 0 (基线) → 独立
  Task 1 (移动 common/ eval_metrics/) → 依赖 Task 0
  Task 2 (src/ 去中间层) → 依赖 Task 1
  Task 3 (tests/ 去中间层) → 依赖 Task 2
  Task 4 (清理空目录) → 依赖 Task 2, Task 3
  Task 5 (更新 include) → 依赖 Task 2, Task 3（文件已在新位置）
  Task 6 (更新 CMakeLists.txt) → 依赖 Task 2, Task 3（路径基准已变更）
  Task 7 (构建验证) → 依赖 Task 1-6 全部
  Task 8 (测试验证) → 依赖 Task 7
  Task 9 (最终验证) → 依赖 Task 8
```

### 任务摘要

| Task | 操作 | 影响文件数 | 验证方式 |
|------|------|-----------|---------|
| 0 | 基线构建+测试 | 0 | cmake + ctest 全通过 |
| 1 | git mv common/ eval_metrics/ → src/ | 2 文件 | ls 新路径存在，旧路径消失 |
| 2 | git mv src/avm_stitch/{geometry,render,core,util}/ → src/ | 24 文件 | ls 新路径存在，旧路径消失 |
| 3 | git mv tests/avm_stitch/{geometry,render,core,util}/ → tests/ | 9 文件 | ls 新路径存在，旧路径消失 |
| 4 | rmdir 空目录 | 0 (清理) | ls 旧目录报不存在 |
| 5 | sed 替换 include 前缀 `avm_stitch/` → `` | 26 文件，56 行 | rg 零匹配 `#include "avm_stitch/` |
| 6 | sed + 精确行删除更新 CMakeLists.txt | 1 文件，33 处 | rg 零匹配旧路径 |
| 7 | 干净构建 | 0 (build/) | cmake 配置+编译零错误 |
| 8 | 运行测试 | 0 | ctest 9/9 PASSED |
| 9 | 最终验证清单 | 0 | 7 项 rg/ls 检查全部通过 |
