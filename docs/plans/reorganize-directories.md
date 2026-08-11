# Plan: 代码文件分类到子目录 — 实现计划

> **设计文档:** `docs/specs/reorganize-directories.md`
> **日期:** 2026-08-11
> **类型:** 纯重构（文件移动 + include 路径 + CMake 路径，不改变代码逻辑）

---

## 前置条件: 记录基线

**必须在任何文件变更前完成。**

---

### Task 0: Baseline — 记录当前测试基线

**行为描述:**
- 运行 ctest 确认当前所有测试通过
- 记录通过的测试数量和名称
- 如果当前有任何测试失败，**停止**，先修复失败的测试再继续

**验证:**
```bash
# 运行: ctest --test-dir build
# 预期: All tests passed (100% tests passed)
```

- [ ] **Step 1: 运行基线 ctest**
  运行: `ctest --test-dir build`
  预期: `100% tests passed, 0 tests failed out of 9`

- [ ] **Step 2: 记录基线结果**
  将 ctest 输出保存为参考，用于重构后对比：
  ```bash
  ctest --test-dir build --output-on-failure > /tmp/baseline_ctest.log 2>&1
  cat /tmp/baseline_ctest.log
  ```

**预期测试列表（9个）:** test_bowl_model, test_projection, test_blend_weights, test_mesh_bake, test_view_config, test_osg_scene, test_stitch_engine, test_egl_headless_context, test_stitch_engine_regression

---

## Phase 1: 创建子目录并移动文件

---

### Task 1: 创建 src/ 下的 4 个子目录

**行为描述:**
- 在 `src/avm_stitch/` 下创建 `geometry/`, `render/`, `core/`, `util/`

**验证:**
```bash
# 运行: ls -d src/avm_stitch/geometry src/avm_stitch/render src/avm_stitch/core src/avm_stitch/util
# 预期: 四个目录都存在
```

- [ ] **Step 1: 创建目录**
  ```bash
  mkdir -p src/avm_stitch/geometry src/avm_stitch/render src/avm_stitch/core src/avm_stitch/util
  ```

- [ ] **Step 2: 验证目录已创建**
  运行: `ls -d src/avm_stitch/{geometry,render,core,util}`
  预期: 输出了 4 个已存在的目录路径

---

### Task 2: 创建 tests/ 下的 4 个子目录

**行为描述:**
- 在 `tests/avm_stitch/` 下创建 `geometry/`, `render/`, `core/`, `util/`

**验证:**
```bash
# 运行: ls -d tests/avm_stitch/geometry tests/avm_stitch/render tests/avm_stitch/core tests/avm_stitch/util
# 预期: 四个目录都存在
```

- [ ] **Step 1: 创建目录**
  ```bash
  mkdir -p tests/avm_stitch/geometry tests/avm_stitch/render tests/avm_stitch/core tests/avm_stitch/util
  ```

- [ ] **Step 2: 验证目录已创建**
  运行: `ls -d tests/avm_stitch/{geometry,render,core,util}`
  预期: 输出了 4 个已存在的目录路径

---

### Task 3: 移动 src/avm_stitch/ 的 geometry 文件到 geometry/

**行为描述:**
- 使用 `git mv` 移动 6 个 geometry 文件以保留 git 历史
- blend_weights.{hpp,cpp}, bowl_model.hpp, mesh_bake.{hpp,cpp}, projection.hpp

**验证:**
```bash
# 运行: ls src/avm_stitch/geometry/
# 预期: 列出 blend_weights.cpp blend_weights.hpp bowl_model.hpp mesh_bake.cpp mesh_bake.hpp projection.hpp
```

- [ ] **Step 1: 移动 geometry 文件**
  ```bash
  git mv src/avm_stitch/blend_weights.cpp src/avm_stitch/geometry/blend_weights.cpp
  git mv src/avm_stitch/blend_weights.hpp src/avm_stitch/geometry/blend_weights.hpp
  git mv src/avm_stitch/bowl_model.hpp   src/avm_stitch/geometry/bowl_model.hpp
  git mv src/avm_stitch/mesh_bake.cpp    src/avm_stitch/geometry/mesh_bake.cpp
  git mv src/avm_stitch/mesh_bake.hpp    src/avm_stitch/geometry/mesh_bake.hpp
  git mv src/avm_stitch/projection.hpp   src/avm_stitch/geometry/projection.hpp
  ```

- [ ] **Step 2: 验证文件已移动**
  运行: `ls src/avm_stitch/geometry/`
  预期: 输出 `blend_weights.cpp  blend_weights.hpp  bowl_model.hpp  mesh_bake.cpp  mesh_bake.hpp  projection.hpp`

- [ ] **Step 3: 确认原位置文件已移除**
  运行: `ls src/avm_stitch/blend_weights.cpp src/avm_stitch/blend_weights.hpp src/avm_stitch/bowl_model.hpp src/avm_stitch/mesh_bake.cpp src/avm_stitch/mesh_bake.hpp src/avm_stitch/projection.hpp 2>&1`
  预期: `No such file or directory` 错误 — 全部 6 个文件已移走

---

### Task 4: 移动 src/avm_stitch/ 的 render 文件到 render/

**行为描述:**
- 使用 `git mv` 移动 6 个 render 文件
- egl_headless_context.{hpp,cpp}, osg_scene.{hpp,cpp}, osg_view.{hpp,cpp}

**验证:**
```bash
# 运行: ls src/avm_stitch/render/
# 预期: 列出 egl_headless_context.cpp egl_headless_context.hpp osg_scene.cpp osg_scene.hpp osg_view.cpp osg_view.hpp
```

- [ ] **Step 1: 移动 render 文件**
  ```bash
  git mv src/avm_stitch/egl_headless_context.cpp src/avm_stitch/render/egl_headless_context.cpp
  git mv src/avm_stitch/egl_headless_context.hpp src/avm_stitch/render/egl_headless_context.hpp
  git mv src/avm_stitch/osg_scene.cpp            src/avm_stitch/render/osg_scene.cpp
  git mv src/avm_stitch/osg_scene.hpp            src/avm_stitch/render/osg_scene.hpp
  git mv src/avm_stitch/osg_view.cpp             src/avm_stitch/render/osg_view.cpp
  git mv src/avm_stitch/osg_view.hpp             src/avm_stitch/render/osg_view.hpp
  ```

- [ ] **Step 2: 验证文件已移动**
  运行: `ls src/avm_stitch/render/`
  预期: 输出 `egl_headless_context.cpp  egl_headless_context.hpp  osg_scene.cpp  osg_scene.hpp  osg_view.cpp  osg_view.hpp`

- [ ] **Step 3: 确认原位置文件已移除**
  运行: `ls src/avm_stitch/egl_headless_context.cpp src/avm_stitch/egl_headless_context.hpp src/avm_stitch/osg_scene.cpp src/avm_stitch/osg_scene.hpp src/avm_stitch/osg_view.cpp src/avm_stitch/osg_view.hpp 2>&1`
  预期: 全部 `No such file or directory`

---

### Task 5: 移动 src/avm_stitch/ 的 core 文件到 core/

**行为描述:**
- 使用 `git mv` 移动 10 个 core 文件
- stitch_engine.{hpp,cpp}, stitch_backend.{hpp,cpp}, stitch_result.{hpp,cpp}, osg_backend.{hpp,cpp}, subprocess_backend.{hpp,cpp}

**验证:**
```bash
# 运行: ls src/avm_stitch/core/
# 预期: 列出所有 10 个 core 文件
```

- [ ] **Step 1: 移动 core 文件**
  ```bash
  git mv src/avm_stitch/stitch_engine.cpp      src/avm_stitch/core/stitch_engine.cpp
  git mv src/avm_stitch/stitch_engine.hpp      src/avm_stitch/core/stitch_engine.hpp
  git mv src/avm_stitch/stitch_backend.cpp     src/avm_stitch/core/stitch_backend.cpp
  git mv src/avm_stitch/stitch_backend.hpp     src/avm_stitch/core/stitch_backend.hpp
  git mv src/avm_stitch/stitch_result.cpp      src/avm_stitch/core/stitch_result.cpp
  git mv src/avm_stitch/stitch_result.hpp      src/avm_stitch/core/stitch_result.hpp
  git mv src/avm_stitch/osg_backend.cpp        src/avm_stitch/core/osg_backend.cpp
  git mv src/avm_stitch/osg_backend.hpp        src/avm_stitch/core/osg_backend.hpp
  git mv src/avm_stitch/subprocess_backend.cpp src/avm_stitch/core/subprocess_backend.cpp
  git mv src/avm_stitch/subprocess_backend.hpp src/avm_stitch/core/subprocess_backend.hpp
  ```

- [ ] **Step 2: 验证文件已移动**
  运行: `ls src/avm_stitch/core/`
  预期: 输出 `osg_backend.cpp  osg_backend.hpp  stitch_backend.cpp  stitch_backend.hpp  stitch_engine.cpp  stitch_engine.hpp  stitch_result.cpp  stitch_result.hpp  subprocess_backend.cpp  subprocess_backend.hpp`

- [ ] **Step 3: 确认原位置文件已移除**
  运行: `ls src/avm_stitch/stitch_engine.{cpp,hpp} src/avm_stitch/stitch_backend.{cpp,hpp} src/avm_stitch/stitch_result.{cpp,hpp} src/avm_stitch/osg_backend.{cpp,hpp} src/avm_stitch/subprocess_backend.{cpp,hpp} 2>&1`
  预期: 全部 `No such file or directory`

---

### Task 6: 移动 src/avm_stitch/ 的 util 文件到 util/

**行为描述:**
- 使用 `git mv` 移动 2 个 util 文件（纯头文件）
- view_config.hpp, camera_name.hpp

**验证:**
```bash
# 运行: ls src/avm_stitch/util/
# 预期: 列出 camera_name.hpp view_config.hpp
```

- [ ] **Step 1: 移动 util 文件**
  ```bash
  git mv src/avm_stitch/view_config.hpp src/avm_stitch/util/view_config.hpp
  git mv src/avm_stitch/camera_name.hpp src/avm_stitch/util/camera_name.hpp
  ```

- [ ] **Step 2: 验证文件已移动**
  运行: `ls src/avm_stitch/util/`
  预期: 输出 `camera_name.hpp  view_config.hpp`

- [ ] **Step 3: 确认原位置 src/avm_stitch/ 已空（仅剩子目录）**
  运行: `find src/avm_stitch/ -maxdepth 1 -type f`
  预期: 无输出（0 个平铺文件）

---

### Task 7: 移动 tests/avm_stitch/ 的 geometry 测试到 geometry/

**行为描述:**
- 使用 `git mv` 移动 4 个 geometry 测试文件
- test_bowl_model.cpp, test_projection.cpp, test_mesh_bake.cpp, test_blend_weights.cpp

- [ ] **Step 1: 移动 geometry 测试文件**
  ```bash
  git mv tests/avm_stitch/test_bowl_model.cpp    tests/avm_stitch/geometry/test_bowl_model.cpp
  git mv tests/avm_stitch/test_projection.cpp    tests/avm_stitch/geometry/test_projection.cpp
  git mv tests/avm_stitch/test_mesh_bake.cpp     tests/avm_stitch/geometry/test_mesh_bake.cpp
  git mv tests/avm_stitch/test_blend_weights.cpp tests/avm_stitch/geometry/test_blend_weights.cpp
  ```

- [ ] **Step 2: 验证文件已移动**
  运行: `ls tests/avm_stitch/geometry/`
  预期: 输出 `test_blend_weights.cpp  test_bowl_model.cpp  test_mesh_bake.cpp  test_projection.cpp`

---

### Task 8: 移动 tests/avm_stitch/ 的 render 测试到 render/

**行为描述:**
- 使用 `git mv` 移动 2 个 render 测试文件
- test_osg_scene.cpp, test_egl_headless_context.cpp

- [ ] **Step 1: 移动 render 测试文件**
  ```bash
  git mv tests/avm_stitch/test_osg_scene.cpp             tests/avm_stitch/render/test_osg_scene.cpp
  git mv tests/avm_stitch/test_egl_headless_context.cpp  tests/avm_stitch/render/test_egl_headless_context.cpp
  ```

- [ ] **Step 2: 验证文件已移动**
  运行: `ls tests/avm_stitch/render/`
  预期: 输出 `test_egl_headless_context.cpp  test_osg_scene.cpp`

---

### Task 9: 移动 tests/avm_stitch/ 的 core 测试到 core/

**行为描述:**
- 使用 `git mv` 移动 2 个 core 测试文件
- test_stitch_engine.cpp, test_stitch_engine_regression.cpp

- [ ] **Step 1: 移动 core 测试文件**
  ```bash
  git mv tests/avm_stitch/test_stitch_engine.cpp            tests/avm_stitch/core/test_stitch_engine.cpp
  git mv tests/avm_stitch/test_stitch_engine_regression.cpp tests/avm_stitch/core/test_stitch_engine_regression.cpp
  ```

- [ ] **Step 2: 验证文件已移动**
  运行: `ls tests/avm_stitch/core/`
  预期: 输出 `test_stitch_engine.cpp  test_stitch_engine_regression.cpp`

---

### Task 10: 移动 tests/avm_stitch/ 的 util 测试到 util/

**行为描述:**
- 使用 `git mv` 移动 1 个 util 测试文件
- test_view_config.cpp

- [ ] **Step 1: 移动 util 测试文件**
  ```bash
  git mv tests/avm_stitch/test_view_config.cpp tests/avm_stitch/util/test_view_config.cpp
  ```

- [ ] **Step 2: 验证文件已移动**
  运行: `ls tests/avm_stitch/util/`
  预期: 输出 `test_view_config.cpp`

- [ ] **Step 3: 确认原位置 tests/avm_stitch/ 已空（仅剩子目录）**
  运行: `find tests/avm_stitch/ -maxdepth 1 -type f`
  预期: 无输出（0 个平铺测试文件）

---

### Task 11: 验证全部文件移动完成（33 个文件）

**行为描述:**
- 跨 src 和 tests 统计所有文件是否已正确放置到新子目录
- 确认原平铺位置无残留文件

- [ ] **Step 1: 统计 src 子目录文件总数**
  运行: `find src/avm_stitch/geometry src/avm_stitch/render src/avm_stitch/core src/avm_stitch/util -type f | wc -l`
  预期: `24`

- [ ] **Step 2: 统计 tests 子目录文件总数**
  运行: `find tests/avm_stitch/geometry tests/avm_stitch/render tests/avm_stitch/core tests/avm_stitch/util -type f | wc -l`
  预期: `9`

- [ ] **Step 3: 确认平铺位置无残留**
  运行: `find src/avm_stitch/ -maxdepth 1 -type f; find tests/avm_stitch/ -maxdepth 1 -type f`
  预期: 无输出（两个命令均无输出）

- [ ] **Step 4: 用 git status 查看暂存区变更**
  运行: `git status --short`
  预期: 所有变更显示为 `R`（rename）类型，共 33 行

---

## Phase 2: 更新 #include 指令

**规则:** 所有 `#include "avm_stitch/xxx.hpp"` 改为 `#include "avm_stitch/<subdir>/xxx.hpp"`，其中 `<subdir>` 是目标文件所在的子目录。

---

### Task 12: 更新 src/avm_stitch/geometry/ 下文件的 #include

**涉及文件（6个）:**
- `src/avm_stitch/geometry/blend_weights.cpp` — 1 处修改
- `src/avm_stitch/geometry/blend_weights.hpp` — 1 处修改
- `src/avm_stitch/geometry/mesh_bake.cpp` — 2 处修改
- `src/avm_stitch/geometry/mesh_bake.hpp` — 2 处修改
- `src/avm_stitch/geometry/bowl_model.hpp` — 0 处（纯定义，无自身 include）
- `src/avm_stitch/geometry/projection.hpp` — 0 处（纯定义，无自身 include）

**行为描述:**
- 修改每个文件中 `#include "avm_stitch/xxx.hpp"` 为包含 `geometry/` 前缀的路径
- 不修改标准库/第三方库 include

**验证:**
```bash
# 运行: rg '#include "avm_stitch/' src/avm_stitch/geometry/blend_weights.cpp src/avm_stitch/geometry/blend_weights.hpp src/avm_stitch/geometry/mesh_bake.cpp src/avm_stitch/geometry/mesh_bake.hpp
# 预期: 所有 avm_stitch/ include 现在包含 geometry/ 前缀
```

- [ ] **Step 1: blend_weights.cpp — 1 处修改**

  将第 1 行:
  ```cpp
  #include "avm_stitch/blend_weights.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/blend_weights.hpp"
  ```

- [ ] **Step 2: blend_weights.hpp — 1 处修改**

  将第 6 行:
  ```cpp
  #include "avm_stitch/projection.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/projection.hpp"
  ```

- [ ] **Step 3: mesh_bake.cpp — 2 处修改**

  将第 1 行:
  ```cpp
  #include "avm_stitch/mesh_bake.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/mesh_bake.hpp"
  ```

  将第 3 行:
  ```cpp
  #include "avm_stitch/blend_weights.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/blend_weights.hpp"
  ```

- [ ] **Step 4: mesh_bake.hpp — 2 处修改**

  将第 6 行:
  ```cpp
  #include "avm_stitch/bowl_model.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/bowl_model.hpp"
  ```

  将第 7 行:
  ```cpp
  #include "avm_stitch/projection.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/projection.hpp"
  ```

- [ ] **Step 5: 验证 geometry/ 目录下无旧路径残留**
  运行: `rg '#include "avm_stitch/(blend_weights|bowl_model|mesh_bake|projection)\.hpp"' src/avm_stitch/geometry/`
  预期: 无匹配（所有旧路径已被替换为新路径含 `geometry/`）

---

### Task 13: 更新 src/avm_stitch/render/ 下文件的 #include

**涉及文件（5个）:**
- `src/avm_stitch/render/egl_headless_context.cpp` — 1 处修改
- `src/avm_stitch/render/osg_scene.cpp` — 1 处修改
- `src/avm_stitch/render/osg_scene.hpp` — 2 处修改
- `src/avm_stitch/render/osg_view.cpp` — 2 处修改
- `src/avm_stitch/render/osg_view.hpp` — 1 处修改

- [ ] **Step 1: egl_headless_context.cpp — 1 处修改**

  将第 1 行:
  ```cpp
  #include "avm_stitch/egl_headless_context.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/render/egl_headless_context.hpp"
  ```

- [ ] **Step 2: osg_scene.cpp — 1 处修改**

  将第 1 行:
  ```cpp
  #include "avm_stitch/osg_scene.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/render/osg_scene.hpp"
  ```

- [ ] **Step 3: osg_scene.hpp — 2 处修改**

  将第 11 行:
  ```cpp
  #include "avm_stitch/bowl_model.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/bowl_model.hpp"
  ```

  将第 12 行:
  ```cpp
  #include "avm_stitch/mesh_bake.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/mesh_bake.hpp"
  ```

- [ ] **Step 4: osg_view.cpp — 2 处修改**

  将第 1 行:
  ```cpp
  #include "avm_stitch/osg_view.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/render/osg_view.hpp"
  ```

  将第 3 行:
  ```cpp
  #include "avm_stitch/egl_headless_context.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/render/egl_headless_context.hpp"
  ```

- [ ] **Step 5: osg_view.hpp — 1 处修改**

  将第 13 行:
  ```cpp
  #include "avm_stitch/view_config.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/util/view_config.hpp"
  ```

- [ ] **Step 6: 验证 render/ 目录下无旧路径残留**
  运行: `rg '#include "avm_stitch/(egl_headless_context|osg_scene|osg_view)\.hpp"' src/avm_stitch/render/`
  预期: 无匹配（所有自身引用已更新为含 `render/`）

---

### Task 14: 更新 src/avm_stitch/core/ 下文件的 #include

**涉及文件（9个）:**
- `src/avm_stitch/core/stitch_engine.cpp` — 6 处修改
- `src/avm_stitch/core/stitch_engine.hpp` — 2 处修改
- `src/avm_stitch/core/stitch_backend.cpp` — 3 处修改
- `src/avm_stitch/core/osg_backend.cpp` — 5 处修改
- `src/avm_stitch/core/osg_backend.hpp` — 1 处修改
- `src/avm_stitch/core/subprocess_backend.cpp` — 1 处修改
- `src/avm_stitch/core/subprocess_backend.hpp` — 1 处修改
- `src/avm_stitch/core/stitch_result.cpp` — 1 处修改
- `src/avm_stitch/core/stitch_backend.hpp` — 0 处（纯接口，无 include）
- `src/avm_stitch/core/stitch_result.hpp` — 0 处（纯定义，无 include）

- [ ] **Step 1: stitch_engine.cpp — 6 处修改**

  将第 1 行:
  ```cpp
  #include "avm_stitch/stitch_engine.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/core/stitch_engine.hpp"
  ```

  将第 3 行:
  ```cpp
  #include "avm_stitch/blend_weights.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/blend_weights.hpp"
  ```

  将第 4 行:
  ```cpp
  #include "avm_stitch/mesh_bake.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/mesh_bake.hpp"
  ```

  将第 5 行:
  ```cpp
  #include "avm_stitch/osg_scene.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/render/osg_scene.hpp"
  ```

  将第 6 行:
  ```cpp
  #include "avm_stitch/osg_view.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/render/osg_view.hpp"
  ```

  将第 7 行:
  ```cpp
  #include "avm_stitch/view_config.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/util/view_config.hpp"
  ```

- [ ] **Step 2: stitch_engine.hpp — 2 处修改**

  将第 12 行:
  ```cpp
  #include "avm_stitch/bowl_model.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/bowl_model.hpp"
  ```

  将第 13 行:
  ```cpp
  #include "avm_stitch/projection.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/projection.hpp"
  ```

- [ ] **Step 3: stitch_backend.cpp — 3 处修改**

  将第 1 行:
  ```cpp
  #include "avm_stitch/stitch_backend.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/core/stitch_backend.hpp"
  ```

  将第 3 行:
  ```cpp
  #include "avm_stitch/osg_backend.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/core/osg_backend.hpp"
  ```

  将第 4 行:
  ```cpp
  #include "avm_stitch/subprocess_backend.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/core/subprocess_backend.hpp"
  ```

- [ ] **Step 4: osg_backend.cpp — 5 处修改**

  将第 1 行:
  ```cpp
  #include "avm_stitch/osg_backend.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/core/osg_backend.hpp"
  ```

  将第 3 行:
  ```cpp
  #include "avm_stitch/bowl_model.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/bowl_model.hpp"
  ```

  将第 4 行:
  ```cpp
  #include "avm_stitch/camera_name.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/util/camera_name.hpp"
  ```

  将第 5 行:
  ```cpp
  #include "avm_stitch/stitch_engine.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/core/stitch_engine.hpp"
  ```

  将第 6 行:
  ```cpp
  #include "avm_stitch/stitch_result.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/core/stitch_result.hpp"
  ```

- [ ] **Step 5: osg_backend.hpp — 1 处修改**

  将第 3 行:
  ```cpp
  #include "avm_stitch/stitch_backend.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/core/stitch_backend.hpp"
  ```

- [ ] **Step 6: subprocess_backend.cpp — 1 处修改**

  将第 1 行:
  ```cpp
  #include "avm_stitch/subprocess_backend.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/core/subprocess_backend.hpp"
  ```

- [ ] **Step 7: subprocess_backend.hpp — 1 处修改**

  将第 3 行:
  ```cpp
  #include "avm_stitch/stitch_backend.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/core/stitch_backend.hpp"
  ```

- [ ] **Step 8: stitch_result.cpp — 1 处修改**

  将第 1 行:
  ```cpp
  #include "avm_stitch/stitch_result.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/core/stitch_result.hpp"
  ```

- [ ] **Step 9: 验证 core/ 目录下无旧路径残留**
  运行: `rg '#include "avm_stitch/(stitch_engine|stitch_backend|stitch_result|osg_backend|subprocess_backend)\.hpp"' src/avm_stitch/core/`
  预期: 无匹配（所有自身引用已更新为含 `core/`）

---

### Task 15: 更新 tests/avm_stitch/geometry/ 下测试文件的 #include

**涉及文件（4个）:**
- `tests/avm_stitch/geometry/test_bowl_model.cpp` — 1 处修改
- `tests/avm_stitch/geometry/test_projection.cpp` — 1 处修改
- `tests/avm_stitch/geometry/test_mesh_bake.cpp` — 3 处修改
- `tests/avm_stitch/geometry/test_blend_weights.cpp` — 2 处修改

- [ ] **Step 1: test_bowl_model.cpp — 1 处修改**

  将第 8 行:
  ```cpp
  #include "avm_stitch/bowl_model.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/bowl_model.hpp"
  ```

- [ ] **Step 2: test_projection.cpp — 1 处修改**

  将第 8 行:
  ```cpp
  #include "avm_stitch/projection.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/projection.hpp"
  ```

- [ ] **Step 3: test_mesh_bake.cpp — 3 处修改**

  将第 6 行:
  ```cpp
  #include "avm_stitch/bowl_model.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/bowl_model.hpp"
  ```

  将第 7 行:
  ```cpp
  #include "avm_stitch/projection.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/projection.hpp"
  ```

  将第 8 行:
  ```cpp
  #include "avm_stitch/mesh_bake.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/mesh_bake.hpp"
  ```

- [ ] **Step 4: test_blend_weights.cpp — 2 处修改**

  将第 6 行:
  ```cpp
  #include "avm_stitch/blend_weights.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/blend_weights.hpp"
  ```

  将第 7 行:
  ```cpp
  #include "avm_stitch/projection.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/projection.hpp"
  ```

- [ ] **Step 5: 验证 tests/geometry/ 下无旧路径残留**
  运行: `rg '"avm_stitch/(blend_weights|bowl_model|mesh_bake|projection)\.hpp"' tests/avm_stitch/geometry/`
  预期: 无匹配（所有路径已包含 `geometry/`）

---

### Task 16: 更新 tests/avm_stitch/render/ 下测试文件的 #include

**涉及文件（2个）:**
- `tests/avm_stitch/render/test_osg_scene.cpp` — 3 处修改
- `tests/avm_stitch/render/test_egl_headless_context.cpp` — 1 处修改

- [ ] **Step 1: test_osg_scene.cpp — 3 处修改**

  将第 6 行:
  ```cpp
  #include "avm_stitch/bowl_model.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/bowl_model.hpp"
  ```

  将第 7 行:
  ```cpp
  #include "avm_stitch/mesh_bake.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/mesh_bake.hpp"
  ```

  将第 8 行:
  ```cpp
  #include "avm_stitch/osg_scene.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/render/osg_scene.hpp"
  ```

- [ ] **Step 2: test_egl_headless_context.cpp — 1 处修改**

  将第 14 行:
  ```cpp
  #include "avm_stitch/egl_headless_context.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/render/egl_headless_context.hpp"
  ```

- [ ] **Step 3: 验证 tests/render/ 下无旧路径残留**
  运行: `rg '"avm_stitch/(osg_scene|egl_headless_context)\.hpp"' tests/avm_stitch/render/`
  预期: 无匹配

---

### Task 17: 更新 tests/avm_stitch/core/ 下测试文件的 #include

**涉及文件（2个）:**
- `tests/avm_stitch/core/test_stitch_engine.cpp` — 7 处修改
- `tests/avm_stitch/core/test_stitch_engine_regression.cpp` — 4 处修改

- [ ] **Step 1: test_stitch_engine.cpp — 7 处修改**

  修改第 7 行:
  ```cpp
  #include "avm_stitch/bowl_model.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/bowl_model.hpp"
  ```

  修改第 8 行:
  ```cpp
  #include "avm_stitch/egl_headless_context.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/render/egl_headless_context.hpp"
  ```

  修改第 9 行:
  ```cpp
  #include "avm_stitch/mesh_bake.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/mesh_bake.hpp"
  ```

  修改第 10 行:
  ```cpp
  #include "avm_stitch/osg_scene.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/render/osg_scene.hpp"
  ```

  修改第 11 行:
  ```cpp
  #include "avm_stitch/osg_view.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/render/osg_view.hpp"
  ```

  修改第 12 行:
  ```cpp
  #include "avm_stitch/stitch_engine.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/core/stitch_engine.hpp"
  ```

  修改第 13 行:
  ```cpp
  #include "avm_stitch/view_config.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/util/view_config.hpp"
  ```

- [ ] **Step 2: test_stitch_engine_regression.cpp — 4 处修改**

  修改第 6 行:
  ```cpp
  #include "avm_stitch/bowl_model.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/bowl_model.hpp"
  ```

  修改第 7 行:
  ```cpp
  #include "avm_stitch/projection.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/geometry/projection.hpp"
  ```

  修改第 8 行:
  ```cpp
  #include "avm_stitch/view_config.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/util/view_config.hpp"
  ```

  修改第 9 行:
  ```cpp
  #include "avm_stitch/stitch_engine.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/core/stitch_engine.hpp"
  ```

- [ ] **Step 3: 验证 tests/core/ 下无旧路径残留**
  运行: `rg '"avm_stitch/(stitch_engine|stitch_backend)\.hpp"' tests/avm_stitch/core/`
  预期: 无匹配

---

### Task 18: 更新 tests/avm_stitch/util/ 下测试文件的 #include

**涉及文件（1个）:**
- `tests/avm_stitch/util/test_view_config.cpp` — 1 处修改

- [ ] **Step 1: test_view_config.cpp — 1 处修改**

  修改第 6 行:
  ```cpp
  #include "avm_stitch/view_config.hpp"
  ```
  改为:
  ```cpp
  #include "avm_stitch/util/view_config.hpp"
  ```

- [ ] **Step 2: 验证 tests/util/ 下无旧路径残留**
  运行: `rg '"avm_stitch/view_config\.hpp"' tests/avm_stitch/util/`
  预期: 无匹配

---

### Task 19: 全量验证 — 项目范围内无平铺 include 残留

**行为描述:**
- 扫描整个 src/ 和 tests/ 目录，确认不包含任何 `#include "avm_stitch/xxx.hpp"`（不含子目录的旧格式）
- 只检查以 `#include "avm_stitch/` 开头且后面紧跟文件名（而非子目录）的模式

- [ ] **Step 1: 执行全局 grep 验证（关键检查）**
  运行: `rg '#include "avm_stitch/[a-z_]+\.hpp"' src/ tests/`
  预期: **无匹配** — 所有 `avm_stitch/` include 现在都包含子目录前缀

- [ ] **Step 2: 确认新格式 include 数量正确**
  运行: `rg -c '#include "avm_stitch/' src/ | awk -F: '{sum+=$2} END{print sum}'`
  预期: `33`（src 中 33 处 avm_stitch include）

  运行: `rg -c '#include "avm_stitch/' tests/ | awk -F: '{sum+=$2} END{print sum}'`
  预期: `23`（tests 中 23 处 avm_stitch include）

- [ ] **Step 3: 确认 common/ 和 eval_metrics/ 未受影响**
  运行: `rg '#include' common/ eval_metrics/`
  预期: 输出不变 — `common/base64.h` 的 `#include` 不含 `avm_stitch/` 前缀；`eval_metrics/eval_metrics.hpp` 路径已含 `eval_metrics/` 子目录

---

## Phase 3: 更新 CMakeLists.txt

---

### Task 20: 更新 avm_stitch 库的源文件路径

**行为描述:**
- 修改 `CMakeLists.txt` 中 `add_library(avm_stitch STATIC ...)` 块里的所有 10 个源文件路径
- 从 `src/avm_stitch/xxx.cpp` 改为 `src/avm_stitch/<subdir>/xxx.cpp`

**修改内容:**

- [ ] **Step 1: 修改 CMakeLists.txt 第 31-40 行**

  将:
  ```cmake
  add_library(avm_stitch STATIC
      src/avm_stitch/blend_weights.cpp
      src/avm_stitch/egl_headless_context.cpp
      src/avm_stitch/mesh_bake.cpp
      src/avm_stitch/osg_scene.cpp
      src/avm_stitch/osg_view.cpp
      src/avm_stitch/stitch_engine.cpp
      src/avm_stitch/stitch_backend.cpp
      src/avm_stitch/osg_backend.cpp
      src/avm_stitch/subprocess_backend.cpp
      src/avm_stitch/stitch_result.cpp
  )
  ```

  改为:
  ```cmake
  add_library(avm_stitch STATIC
      src/avm_stitch/geometry/blend_weights.cpp
      src/avm_stitch/render/egl_headless_context.cpp
      src/avm_stitch/geometry/mesh_bake.cpp
      src/avm_stitch/render/osg_scene.cpp
      src/avm_stitch/render/osg_view.cpp
      src/avm_stitch/core/stitch_engine.cpp
      src/avm_stitch/core/stitch_backend.cpp
      src/avm_stitch/core/osg_backend.cpp
      src/avm_stitch/core/subprocess_backend.cpp
      src/avm_stitch/core/stitch_result.cpp
  )
  ```

- [ ] **Step 2: 验证 update 后的 CMakeLists.txt 无旧路径**
  运行: `rg 'src/avm_stitch/[a-z_]+\.cpp' CMakeLists.txt`
  预期: **无匹配** — 不存在 `src/avm_stitch/xxx.cpp`（不含子目录的旧路径）

- [ ] **Step 3: 验证包含新子目录路径**
  运行: `rg 'src/avm_stitch/' CMakeLists.txt`
  预期: 所有匹配都包含子目录前缀（geometry/、render/、core/、util/）

---

### Task 21: 更新测试可执行文件的源文件路径

**行为描述:**
- 修改 `CMakeLists.txt` 中所有 9 个 `add_executable(test_*)` 块中的源文件路径
- 每个测试的路径更新一次（测试文件路径 + 依赖的 src 文件路径）

**test_bowl_model (第 62 行) — 1 处修改：**

- [ ] **Step 1: test_bowl_model**
  将:
  ```cmake
      tests/avm_stitch/test_bowl_model.cpp
  ```
  改为:
  ```cmake
      tests/avm_stitch/geometry/test_bowl_model.cpp
  ```

**test_projection (第 74 行) — 1 处修改：**

- [ ] **Step 2: test_projection**
  将:
  ```cmake
      tests/avm_stitch/test_projection.cpp
  ```
  改为:
  ```cmake
      tests/avm_stitch/geometry/test_projection.cpp
  ```

**test_blend_weights (第 85-86 行) — 2 处修改：**

- [ ] **Step 3: test_blend_weights**
  将:
  ```cmake
      tests/avm_stitch/test_blend_weights.cpp
      src/avm_stitch/blend_weights.cpp
  ```
  改为:
  ```cmake
      tests/avm_stitch/geometry/test_blend_weights.cpp
      src/avm_stitch/geometry/blend_weights.cpp
  ```

**test_mesh_bake (第 97-99 行) — 3 处修改：**

- [ ] **Step 4: test_mesh_bake**
  将:
  ```cmake
      tests/avm_stitch/test_mesh_bake.cpp
      src/avm_stitch/blend_weights.cpp
      src/avm_stitch/mesh_bake.cpp
  ```
  改为:
  ```cmake
      tests/avm_stitch/geometry/test_mesh_bake.cpp
      src/avm_stitch/geometry/blend_weights.cpp
      src/avm_stitch/geometry/mesh_bake.cpp
  ```

**test_view_config (第 110 行) — 1 处修改：**

- [ ] **Step 5: test_view_config**
  将:
  ```cmake
      tests/avm_stitch/test_view_config.cpp
  ```
  改为:
  ```cmake
      tests/avm_stitch/util/test_view_config.cpp
  ```

**test_osg_scene (第 121-124 行) — 4 处修改：**

- [ ] **Step 6: test_osg_scene**
  将:
  ```cmake
      tests/avm_stitch/test_osg_scene.cpp
      src/avm_stitch/blend_weights.cpp
      src/avm_stitch/mesh_bake.cpp
      src/avm_stitch/osg_scene.cpp
  ```
  改为:
  ```cmake
      tests/avm_stitch/render/test_osg_scene.cpp
      src/avm_stitch/geometry/blend_weights.cpp
      src/avm_stitch/geometry/mesh_bake.cpp
      src/avm_stitch/render/osg_scene.cpp
  ```

**test_stitch_engine (第 137-143 行) — 7 处修改：**

- [ ] **Step 7: test_stitch_engine**
  将:
  ```cmake
      tests/avm_stitch/test_stitch_engine.cpp
      src/avm_stitch/blend_weights.cpp
      src/avm_stitch/egl_headless_context.cpp
      src/avm_stitch/mesh_bake.cpp
      src/avm_stitch/osg_scene.cpp
      src/avm_stitch/osg_view.cpp
      src/avm_stitch/stitch_engine.cpp
  ```
  改为:
  ```cmake
      tests/avm_stitch/core/test_stitch_engine.cpp
      src/avm_stitch/geometry/blend_weights.cpp
      src/avm_stitch/render/egl_headless_context.cpp
      src/avm_stitch/geometry/mesh_bake.cpp
      src/avm_stitch/render/osg_scene.cpp
      src/avm_stitch/render/osg_view.cpp
      src/avm_stitch/core/stitch_engine.cpp
  ```

**test_egl_headless_context (第 158-159 行) — 2 处修改：**

- [ ] **Step 8: test_egl_headless_context**
  将:
  ```cmake
      tests/avm_stitch/test_egl_headless_context.cpp
      src/avm_stitch/egl_headless_context.cpp
  ```
  改为:
  ```cmake
      tests/avm_stitch/render/test_egl_headless_context.cpp
      src/avm_stitch/render/egl_headless_context.cpp
  ```

**test_stitch_engine_regression (第 173 行) — 1 处修改：**

- [ ] **Step 9: test_stitch_engine_regression**
  将:
  ```cmake
      tests/avm_stitch/test_stitch_engine_regression.cpp
  ```
  改为:
  ```cmake
      tests/avm_stitch/core/test_stitch_engine_regression.cpp
  ```

- [ ] **Step 10: 验证 CMakeLists.txt 中所有测试路径已更新**
  运行: `rg 'tests/avm_stitch/[a-z_]+\.cpp' CMakeLists.txt`
  预期: **无匹配** — 不存在 `tests/avm_stitch/test_xxx.cpp`（不含子目录的旧路径）

---

## Phase 4: 构建与验证

---

### Task 22: 构建验证 — 删除 build 产物 + 重新配置 + 编译

**行为描述:**
- 删除旧的 build 产物（因为包含旧路径的编译依赖文件）
- 使用 cmake presets 重新配置
- 编译整个项目

- [ ] **Step 1: 删除旧 build 目录**
  运行: `rm -rf build`

- [ ] **Step 2: 重新配置 cmake**
  运行: `cmake --preset default`
  预期: 配置成功，无错误 — 输出 `-- Build files have been written to: .../build`

- [ ] **Step 3: 编译**
  运行: `cmake --build build -j$(nproc)`
  预期: 编译成功，零错误 — 所有编译单元通过，链接成功

---

### Task 23: 运行 ctest 验证回归

**行为描述:**
- 运行 ctest 确保所有 9 个测试仍然通过
- 与 Task 0 基线对比

- [ ] **Step 1: 运行 ctest**
  运行: `ctest --test-dir build --output-on-failure`
  预期: `100% tests passed, 0 tests failed out of 9`

- [ ] **Step 2: 与基线对比**
  运行: `diff <(ctest --test-dir build -N | grep 'Test #') <(grep 'Test #' /tmp/baseline_ctest.log)`
  预期: 无差异（测试数量和名称完全一致）

---

## Phase 5: 最终验证清单

---

### Task 24: 最终完整性验证

- [ ] **Step 1: 目录结构验证 — src/**
  运行: `find src/avm_stitch/ -maxdepth 1 -type d | sort`
  预期: 输出 `src/avm_stitch/core`, `src/avm_stitch/geometry`, `src/avm_stitch/render`, `src/avm_stitch/util`（仅子目录，无平铺文件）

- [ ] **Step 2: 目录结构验证 — tests/**
  运行: `find tests/avm_stitch/ -maxdepth 1 -type d | sort`
  预期: 输出 `tests/avm_stitch/core`, `tests/avm_stitch/geometry`, `tests/avm_stitch/render`, `tests/avm_stitch/util`（仅子目录，无平铺文件）

- [ ] **Step 3: 文件总数验证**
  运行: `find src/avm_stitch/ -type f -name '*.cpp' -o -name '*.hpp' | wc -l`
  预期: `24`

  运行: `find tests/avm_stitch/ -type f -name '*.cpp' | wc -l`
  预期: `9`

- [ ] **Step 4: 零平铺 include 残留**
  运行: `rg '#include "avm_stitch/[a-z_]+\.hpp"' src/ tests/`
  预期: **无匹配** — 所有 `avm_stitch/` include 都包含子目录前缀

- [ ] **Step 5: common/ 和 eval_metrics/ 未被修改**
  运行: `git diff --name-only HEAD -- common/ eval_metrics/`
  预期: 无输出（这两个目录无任何变更）

- [ ] **Step 6: git 历史未丢失**
  运行: `git log --follow --oneline src/avm_stitch/geometry/blend_weights.cpp | head -1`
  预期: 显示该文件在移动前的历史提交（`--follow` 能追踪 rename 后的文件）

  运行: `git log --follow --oneline src/avm_stitch/core/stitch_engine.cpp | head -1`
  预期: 显示该文件在移动前的历史提交

- [ ] **Step 7: CMake 配置 + 编译 + 测试全部通过**
  运行: `cmake --preset default && cmake --build build -j$(nproc) && ctest --test-dir build --output-on-failure`
  预期: cmake 配置成功、编译零错误、ctest 9/9 全部通过

---

## 任务依赖图

```
Task 0 (基线 ctest)  — 前置条件，阻塞所有后续任务
    │
Task 1 (src 子目录)   ──┐
Task 2 (tests 子目录) ──┤  并行执行（无依赖）
    │                   │
    ├─── Task 3  (geometry src)  ──┐
    ├─── Task 4  (render src)    ──┤
    ├─── Task 5  (core src)      ──┤  Task 3-10 依赖 Task 1/2
    ├─── Task 6  (util src)      ──┤  可并行执行（不同目录）
    ├─── Task 7  (geometry tests)──┤
    ├─── Task 8  (render tests)  ──┤
    ├─── Task 9  (core tests)    ──┤
    └─── Task 10 (util tests)    ──┘
            │
    Task 11 (验证移动完成)
            │
   ┌────────┼────────┬────────┐
   │        │        │        │
 Task 12  Task 13  Task 14  Task 15-18  (include 更新，按目录独立)
 (geom)   (render) (core)   (tests)
   │        │        │        │
   └────────┴────────┴────────┘
            │
    Task 19 (全量验证 include)
            │
   ┌────────┴────────┐
   │                  │
 Task 20 (CMake lib) Task 21 (CMake tests)
   └────────┬────────┘
            │
    Task 22 (删除 build + 配置 + 编译)
            │
    Task 23 (ctest 回归验证)
            │
    Task 24 (最终完整性验证)
```

## 摘要

| 指标 | 数值 |
|------|------|
| 移动文件数 | 33（24 src + 9 tests） |
| #include 变更数 | 56（33 src + 23 tests） |
| CMakeLists.txt 变更行数 | ~50 行 |
| 预估总耗时 | 30-45 分钟 |
| 成功标准数量 | 9 项（见设计文档） |
