# 代码文件分类到子目录 — 设计规范

> **状态:** 草稿

**目标:** Stage 1（已完成）将平铺文件按功能域分入子目录；Stage 2（本阶段）去掉 `avm_stitch/` 中间层，将 `common/` 和 `eval_metrics/` 移入 `src/`，统一 include 根目录。

**架构:** 源码全部放在 `src/{common,eval_metrics,geometry,render,core,util}/` 下，测试镜像放在 `tests/{geometry,render,core,util}/` 下。编译器 include 根为 `${CMAKE_CURRENT_SOURCE_DIR}/src`。

**技术栈:** C++20, CMake 3.25+, vcpkg (nlohmann_json, OpenCV, unofficial-osg, OpenGL::EGL, GTest)

---

## 背景

Stage 1（已执行）将原本平铺在 `src/avm_stitch/` 的 24 个文件和 `tests/avm_stitch/` 的 9 个测试文件按功能域分到了 4 个子目录：

```
src/avm_stitch/{geometry,render,core,util}/
tests/avm_stitch/{geometry,render,core,util}/
```

所有 #include 更新为 `"avm_stitch/<subdir>/xxx.hpp"` 格式，CMakeLists.txt 同步更新。

Stage 2（本阶段）在此基础上做两项简化：

1. **`common/` 和 `eval_metrics/` 移入 `src/` 下** — 将它们从仓库根挪到 `src/common/base64.h` 和 `src/eval_metrics/eval_metrics.hpp`，使所有源码集中在 `src/` 内。

2. **去掉 `src/` 和 `tests/` 下的 `avm_stitch/` 中间层** — `src/avm_stitch/{geometry,render,core,util}/` → `src/{geometry,render,core,util}/`，`tests/avm_stitch/{geometry,render,core,util}/` → `tests/{geometry,render,core,util}/`。include 路径同步去掉 `avm_stitch/` 前缀。

---

## 目标与非目标

### 目标

- 将 `common/base64.h` 移动到 `src/common/base64.h`
- 将 `eval_metrics/eval_metrics.hpp` 移动到 `src/eval_metrics/eval_metrics.hpp`
- 去掉 `src/avm_stitch/` → `src/` 中间层（4 个子目录中 24 个文件重定位）
- 去掉 `tests/avm_stitch/` → `tests/` 中间层（4 个子目录中 9 个文件重定位）
- 更新所有 #include 指令：去掉 `avm_stitch/` 前缀
- 简化 `target_include_directories` 为单一 `${CMAKE_CURRENT_SOURCE_DIR}/src`
- 更新 CMakeLists.txt 中所有源文件路径
- 删除空的旧目录（`src/avm_stitch/`、`tests/avm_stitch/`、`common/`、`eval_metrics/`）
- cmake 重新配置 + 编译通过
- 全部单元测试通过 (ctest)

### 非目标

- 不修改任何 .cpp/.hpp 的逻辑内容（仅修改 #include 路径）
- 不改变类的接口、行为或 API
- 不修改 vcpkg.json 或其他构建配置文件
- 不拆分或合并任何源文件

---

## 分类方案

子目录不变，最终位置从 Stage 1 的 `src/avm_stitch/<subdir>/` 变为 `src/<subdir>/`。另外 `common/` 和 `eval_metrics/` 加入 `src/`。

### 功能域定义

| 子目录 | 功能域 | 文件 |
|--------|--------|------|
| `common/` | 通用工具 | base64.h |
| `eval_metrics/` | 评估指标 | eval_metrics.hpp |
| `geometry/` | 几何模型、投影、网格生成、融合权重 | bowl_model.hpp, projection.hpp, mesh_bake.{hpp,cpp}, blend_weights.{hpp,cpp} |
| `render/` | OSG 渲染、视口、EGL 无头上下文 | osg_scene.{hpp,cpp}, osg_view.{hpp,cpp}, egl_headless_context.{hpp,cpp} |
| `core/` | 引擎核心、后端抽象/实现、结果容器 | stitch_engine.{hpp,cpp}, stitch_backend.{hpp,cpp}, stitch_result.{hpp,cpp}, subprocess_backend.{hpp,cpp}, osg_backend.{hpp,cpp} |
| `util/` | 配置、工具类 | view_config.hpp, camera_name.hpp |

---

## 最终目录结构

### src/ — 26 个文件（15 .hpp + 10 .cpp + 1 .h）

```
src/
├── common/
│   └── base64.h                       ← 从仓库根 common/ 移入
├── eval_metrics/
│   └── eval_metrics.hpp               ← 从仓库根 eval_metrics/ 移入
├── geometry/
│   ├── blend_weights.cpp
│   ├── blend_weights.hpp
│   ├── bowl_model.hpp
│   ├── mesh_bake.cpp
│   ├── mesh_bake.hpp
│   └── projection.hpp
├── render/
│   ├── egl_headless_context.cpp
│   ├── egl_headless_context.hpp
│   ├── osg_scene.cpp
│   ├── osg_scene.hpp
│   ├── osg_view.cpp
│   └── osg_view.hpp
├── core/
│   ├── osg_backend.cpp
│   ├── osg_backend.hpp
│   ├── stitch_backend.cpp
│   ├── stitch_backend.hpp
│   ├── stitch_engine.cpp
│   ├── stitch_engine.hpp
│   ├── stitch_result.cpp
│   ├── stitch_result.hpp
│   ├── subprocess_backend.cpp
│   └── subprocess_backend.hpp
└── util/
    ├── camera_name.hpp
    └── view_config.hpp
```

### tests/ — 9 个文件

```
tests/
├── geometry/
│   ├── test_blend_weights.cpp
│   ├── test_bowl_model.cpp
│   ├── test_mesh_bake.cpp
│   └── test_projection.cpp
├── render/
│   ├── test_egl_headless_context.cpp
│   └── test_osg_scene.cpp
├── core/
│   ├── test_stitch_engine.cpp
│   └── test_stitch_engine_regression.cpp
└── util/
    └── test_view_config.cpp
```

### 删除的旧目录

```
src/avm_stitch/         — 所有内容已迁移到 src/{geometry,render,core,util}/
tests/avm_stitch/       — 所有内容已迁移到 tests/{geometry,render,core,util}/
common/                 — base64.h 已迁移到 src/common/
eval_metrics/           — eval_metrics.hpp 已迁移到 src/eval_metrics/
```

---

## #include 路径变更规则

### 规则

所有 `#include "avm_stitch/<subdir>/xxx.hpp"` 改为 `#include "<subdir>/xxx.hpp"`（去掉 `avm_stitch/` 前缀）。

`#include "common/base64.h"` 和 `#include "eval_metrics/eval_metrics.hpp"` 文本不变 — 它们当前就不含 `avm_stitch/` 前缀，移动后由 `${CMAKE_CURRENT_SOURCE_DIR}/src` 解析。

### include 根目录

Stage 1 有两个 include 根：
- `${CMAKE_CURRENT_SOURCE_DIR}/src` — 解析 `avm_stitch/xxx`
- `${CMAKE_CURRENT_SOURCE_DIR}` — 解析 `common/` 和 `eval_metrics/`

Stage 2 简化为单一根 `${CMAKE_CURRENT_SOURCE_DIR}/src`，因为 `common/` 和 `eval_metrics/` 移入 `src/` 后，不再需要仓库根作为 include 根。

---

## #include 变更映射表

### avm_stitch/ → 去掉前缀

#### geometry/ 内的文件

| 原 include 路径 (Stage 1 状态) | 新 include 路径 (Stage 2) |
|------------------------------|--------------------------|
| `avm_stitch/geometry/blend_weights.hpp` | `geometry/blend_weights.hpp` |
| `avm_stitch/geometry/bowl_model.hpp` | `geometry/bowl_model.hpp` |
| `avm_stitch/geometry/mesh_bake.hpp` | `geometry/mesh_bake.hpp` |
| `avm_stitch/geometry/projection.hpp` | `geometry/projection.hpp` |

#### render/ 内的文件

| 原 include 路径 (Stage 1 状态) | 新 include 路径 (Stage 2) |
|------------------------------|--------------------------|
| `avm_stitch/render/egl_headless_context.hpp` | `render/egl_headless_context.hpp` |
| `avm_stitch/render/osg_scene.hpp` | `render/osg_scene.hpp` |
| `avm_stitch/render/osg_view.hpp` | `render/osg_view.hpp` |

#### core/ 内的文件

| 原 include 路径 (Stage 1 状态) | 新 include 路径 (Stage 2) |
|------------------------------|--------------------------|
| `avm_stitch/core/osg_backend.hpp` | `core/osg_backend.hpp` |
| `avm_stitch/core/stitch_backend.hpp` | `core/stitch_backend.hpp` |
| `avm_stitch/core/stitch_engine.hpp` | `core/stitch_engine.hpp` |
| `avm_stitch/core/stitch_result.hpp` | `core/stitch_result.hpp` |
| `avm_stitch/core/subprocess_backend.hpp` | `core/subprocess_backend.hpp` |

#### util/ 内的文件

| 原 include 路径 (Stage 1 状态) | 新 include 路径 (Stage 2) |
|------------------------------|--------------------------|
| `avm_stitch/util/camera_name.hpp` | `util/camera_name.hpp` |
| `avm_stitch/util/view_config.hpp` | `util/view_config.hpp` |

### common/ 和 eval_metrics/ — 文本不变

| include 文本 | 说明 |
|-------------|------|
| `common/base64.h` | 文本不变，文件从仓库根移到 src/common/ |
| `eval_metrics/eval_metrics.hpp` | 文本不变，文件从仓库根移到 src/eval_metrics/ |

---

## 受影响的文件清单

### 需要移动的文件（35 个）

#### 从仓库根移入 src/ (2 个)

| 文件 | 从 | 到 |
|------|----|----|
| base64.h | common/ | src/common/ |
| eval_metrics.hpp | eval_metrics/ | src/eval_metrics/ |

#### 从 src/avm_stitch/ 去掉中间层 (24 个)

每个文件从 `src/avm_stitch/<subdir>/xxx` → `src/<subdir>/xxx`。详见目录结构。

#### 从 tests/avm_stitch/ 去掉中间层 (9 个)

每个文件从 `tests/avm_stitch/<subdir>/xxx` → `tests/<subdir>/xxx`。详见目录结构。

### 需要修改 #include 的文件

共 33 个文件（24 src + 9 tests），总计约 56 处 #include 需要修改（每处去掉 `avm_stitch/` 前缀）。

两个含 `common/` 和 `eval_metrics/` include 的文件（`osg_backend.cpp`、`stitch_result.cpp`）因为文本不变，不需要修改 include 行 — 只需确认 CMake include 根目录正确即可。

### 需要修改 CMakeLists.txt 的位置

所有 `add_library` 和 `add_executable` 中的源文件路径，以及 `target_include_directories`（简化根目录）。

---

## #include 更新详细列表

以下列出每个文件中需要修改的 #include 行。每行格式为 `原 → 新`（均相对于 Stage 1 状态）。

### src/geometry/ （原 src/avm_stitch/geometry/ 内文件）

**blend_weights.cpp:**
```
"avm_stitch/geometry/blend_weights.hpp" → "geometry/blend_weights.hpp"
```

**blend_weights.hpp:**
```
"avm_stitch/geometry/projection.hpp" → "geometry/projection.hpp"
```

**mesh_bake.cpp:**
```
"avm_stitch/geometry/mesh_bake.hpp" → "geometry/mesh_bake.hpp"
"avm_stitch/geometry/blend_weights.hpp" → "geometry/blend_weights.hpp"
```

**mesh_bake.hpp:**
```
"avm_stitch/geometry/bowl_model.hpp" → "geometry/bowl_model.hpp"
"avm_stitch/geometry/projection.hpp" → "geometry/projection.hpp"
```

### src/render/ （原 src/avm_stitch/render/ 内文件）

**egl_headless_context.cpp:**
```
"avm_stitch/render/egl_headless_context.hpp" → "render/egl_headless_context.hpp"
```

**osg_scene.cpp:**
```
"avm_stitch/render/osg_scene.hpp" → "render/osg_scene.hpp"
```

**osg_scene.hpp:**
```
"avm_stitch/geometry/bowl_model.hpp" → "geometry/bowl_model.hpp"
"avm_stitch/geometry/mesh_bake.hpp" → "geometry/mesh_bake.hpp"
```

**osg_view.cpp:**
```
"avm_stitch/render/osg_view.hpp" → "render/osg_view.hpp"
"avm_stitch/render/egl_headless_context.hpp" → "render/egl_headless_context.hpp"
```

**osg_view.hpp:**
```
"avm_stitch/util/view_config.hpp" → "util/view_config.hpp"
```

### src/core/ （原 src/avm_stitch/core/ 内文件）

**stitch_engine.cpp:**
```
"avm_stitch/core/stitch_engine.hpp" → "core/stitch_engine.hpp"
"avm_stitch/geometry/blend_weights.hpp" → "geometry/blend_weights.hpp"
"avm_stitch/geometry/mesh_bake.hpp" → "geometry/mesh_bake.hpp"
"avm_stitch/render/osg_scene.hpp" → "render/osg_scene.hpp"
"avm_stitch/render/osg_view.hpp" → "render/osg_view.hpp"
"avm_stitch/util/view_config.hpp" → "util/view_config.hpp"
```

**stitch_engine.hpp:**
```
"avm_stitch/geometry/bowl_model.hpp" → "geometry/bowl_model.hpp"
"avm_stitch/geometry/projection.hpp" → "geometry/projection.hpp"
```

**stitch_backend.cpp:**
```
"avm_stitch/core/stitch_backend.hpp" → "core/stitch_backend.hpp"
"avm_stitch/core/osg_backend.hpp" → "core/osg_backend.hpp"
"avm_stitch/core/subprocess_backend.hpp" → "core/subprocess_backend.hpp"
```

**osg_backend.cpp:**
```
"avm_stitch/core/osg_backend.hpp" → "core/osg_backend.hpp"
"avm_stitch/geometry/bowl_model.hpp" → "geometry/bowl_model.hpp"
"avm_stitch/util/camera_name.hpp" → "util/camera_name.hpp"
"avm_stitch/core/stitch_engine.hpp" → "core/stitch_engine.hpp"
"avm_stitch/core/stitch_result.hpp" → "core/stitch_result.hpp"
```
（`"common/base64.h"` 文本不变）

**osg_backend.hpp:**
```
"avm_stitch/core/stitch_backend.hpp" → "core/stitch_backend.hpp"
```

**subprocess_backend.cpp:**
```
"avm_stitch/core/subprocess_backend.hpp" → "core/subprocess_backend.hpp"
```

**subprocess_backend.hpp:**
```
"avm_stitch/core/stitch_backend.hpp" → "core/stitch_backend.hpp"
```

**stitch_result.cpp:**
```
"avm_stitch/core/stitch_result.hpp" → "core/stitch_result.hpp"
```
（`"eval_metrics/eval_metrics.hpp"` 文本不变）

### tests/ （原 tests/avm_stitch/ 内文件）

**tests/geometry/test_bowl_model.cpp:**
```
"avm_stitch/geometry/bowl_model.hpp" → "geometry/bowl_model.hpp"
```

**tests/geometry/test_projection.cpp:**
```
"avm_stitch/geometry/projection.hpp" → "geometry/projection.hpp"
```

**tests/geometry/test_mesh_bake.cpp:**
```
"avm_stitch/geometry/bowl_model.hpp" → "geometry/bowl_model.hpp"
"avm_stitch/geometry/projection.hpp" → "geometry/projection.hpp"
"avm_stitch/geometry/mesh_bake.hpp" → "geometry/mesh_bake.hpp"
```

**tests/geometry/test_blend_weights.cpp:**
```
"avm_stitch/geometry/blend_weights.hpp" → "geometry/blend_weights.hpp"
"avm_stitch/geometry/projection.hpp" → "geometry/projection.hpp"
```

**tests/render/test_osg_scene.cpp:**
```
"avm_stitch/geometry/bowl_model.hpp" → "geometry/bowl_model.hpp"
"avm_stitch/geometry/mesh_bake.hpp" → "geometry/mesh_bake.hpp"
"avm_stitch/render/osg_scene.hpp" → "render/osg_scene.hpp"
```

**tests/render/test_egl_headless_context.cpp:**
```
"avm_stitch/render/egl_headless_context.hpp" → "render/egl_headless_context.hpp"
```

**tests/core/test_stitch_engine.cpp:**
```
"avm_stitch/geometry/bowl_model.hpp" → "geometry/bowl_model.hpp"
"avm_stitch/render/egl_headless_context.hpp" → "render/egl_headless_context.hpp"
"avm_stitch/geometry/mesh_bake.hpp" → "geometry/mesh_bake.hpp"
"avm_stitch/render/osg_scene.hpp" → "render/osg_scene.hpp"
"avm_stitch/render/osg_view.hpp" → "render/osg_view.hpp"
"avm_stitch/core/stitch_engine.hpp" → "core/stitch_engine.hpp"
"avm_stitch/util/view_config.hpp" → "util/view_config.hpp"
```

**tests/core/test_stitch_engine_regression.cpp:**
```
"avm_stitch/geometry/bowl_model.hpp" → "geometry/bowl_model.hpp"
"avm_stitch/geometry/projection.hpp" → "geometry/projection.hpp"
"avm_stitch/util/view_config.hpp" → "util/view_config.hpp"
"avm_stitch/core/stitch_engine.hpp" → "core/stitch_engine.hpp"
```

**tests/util/test_view_config.cpp:**
```
"avm_stitch/util/view_config.hpp" → "util/view_config.hpp"
```

---

## 跨子目录的 include 依赖

分类后（去掉 `avm_stitch/` 前缀），不同子目录之间的 include 依赖关系不变，仅路径前缀变化：

| 所在子目录 | 文件 | include 的文件 | 所在子目录 |
|-----------|------|---------------|-----------|
| geometry/ | mesh_bake.cpp | blend_weights.hpp | geometry/（同子目录） |
| geometry/ | mesh_bake.hpp | bowl_model.hpp, projection.hpp | geometry/（同子目录） |
| geometry/ | blend_weights.hpp | projection.hpp | geometry/（同子目录） |
| render/ | osg_scene.hpp | bowl_model.hpp | **geometry/** |
| render/ | osg_scene.hpp | mesh_bake.hpp | **geometry/** |
| render/ | osg_view.hpp | view_config.hpp | **util/** |
| render/ | osg_view.cpp | egl_headless_context.hpp | render/（同子目录） |
| core/ | stitch_engine.hpp | bowl_model.hpp | **geometry/** |
| core/ | stitch_engine.hpp | projection.hpp | **geometry/** |
| core/ | stitch_engine.cpp | blend_weights.hpp | **geometry/** |
| core/ | stitch_engine.cpp | mesh_bake.hpp | **geometry/** |
| core/ | stitch_engine.cpp | osg_scene.hpp | **render/** |
| core/ | stitch_engine.cpp | osg_view.hpp | **render/** |
| core/ | stitch_engine.cpp | view_config.hpp | **util/** |
| core/ | osg_backend.hpp | stitch_backend.hpp | core/（同子目录） |
| core/ | osg_backend.cpp | bowl_model.hpp | **geometry/** |
| core/ | osg_backend.cpp | camera_name.hpp | **util/** |
| core/ | osg_backend.cpp | stitch_engine.hpp | core/（同子目录） |
| core/ | osg_backend.cpp | stitch_result.hpp | core/（同子目录） |
| core/ | osg_backend.cpp | base64.h | **common/** |
| core/ | subprocess_backend.hpp | stitch_backend.hpp | core/（同子目录） |
| core/ | stitch_backend.cpp | osg_backend.hpp | core/（同子目录） |
| core/ | stitch_backend.cpp | subprocess_backend.hpp | core/（同子目录） |
| core/ | stitch_result.cpp | eval_metrics.hpp | **eval_metrics/** |

依赖关系不变，架构合理性判断不变：
- **geometry/** 是底层模块，被 render/ 和 core/ 依赖
- **render/** 依赖 geometry/ 和 util/
- **core/** 依赖所有其他子目录（包括 common/ 和 eval_metrics/）
- **util/** 独立，仅被 render/ 和 core/ 依赖
- **common/** 和 **eval_metrics/** 仅被 core/ 依赖

---

## CMakeLists.txt 变更清单

### 1. target_include_directories 简化

**Stage 1 状态:**
```cmake
target_include_directories(avm_stitch PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)
```

**Stage 2 变更 — 只保留 `${CMAKE_CURRENT_SOURCE_DIR}/src`:**
```cmake
target_include_directories(avm_stitch PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
```

删除 `${CMAKE_CURRENT_SOURCE_DIR}`（仓库根）是因为 `common/` 和 `eval_metrics/` 已移入 `src/`，不再需要从仓库根解析。

所有测试的 `target_include_directories(... PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)` 保持不变。

### 2. avm_stitch 库源文件路径

所有路径去掉 `avm_stitch/` 中间层：

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
    src/core/subprocess_backend.cpp
    src/core/stitch_result.cpp
)
```

### 3. 测试可执行文件路径

每个测试的源文件路径去掉 `avm_stitch/` 中间层。变更总览：

| 测试目标 | 原路径 (Stage 1) | 新路径 (Stage 2) |
|---------|-----------------|-----------------|
| test_bowl_model | tests/avm_stitch/geometry/test_bowl_model.cpp | tests/geometry/test_bowl_model.cpp |
| test_projection | tests/avm_stitch/geometry/test_projection.cpp | tests/geometry/test_projection.cpp |
| test_blend_weights | tests/avm_stitch/geometry/test_blend_weights.cpp | tests/geometry/test_blend_weights.cpp |
| | src/avm_stitch/geometry/blend_weights.cpp | src/geometry/blend_weights.cpp |
| test_mesh_bake | tests/avm_stitch/geometry/test_mesh_bake.cpp | tests/geometry/test_mesh_bake.cpp |
| | src/avm_stitch/geometry/blend_weights.cpp | src/geometry/blend_weights.cpp |
| | src/avm_stitch/geometry/mesh_bake.cpp | src/geometry/mesh_bake.cpp |
| test_view_config | tests/avm_stitch/util/test_view_config.cpp | tests/util/test_view_config.cpp |
| test_osg_scene | tests/avm_stitch/render/test_osg_scene.cpp | tests/render/test_osg_scene.cpp |
| | src/avm_stitch/geometry/blend_weights.cpp | src/geometry/blend_weights.cpp |
| | src/avm_stitch/geometry/mesh_bake.cpp | src/geometry/mesh_bake.cpp |
| | src/avm_stitch/render/osg_scene.cpp | src/render/osg_scene.cpp |
| test_stitch_engine | tests/avm_stitch/core/test_stitch_engine.cpp | tests/core/test_stitch_engine.cpp |
| | src/avm_stitch/geometry/blend_weights.cpp | src/geometry/blend_weights.cpp |
| | src/avm_stitch/render/egl_headless_context.cpp | src/render/egl_headless_context.cpp |
| | src/avm_stitch/geometry/mesh_bake.cpp | src/geometry/mesh_bake.cpp |
| | src/avm_stitch/render/osg_scene.cpp | src/render/osg_scene.cpp |
| | src/avm_stitch/render/osg_view.cpp | src/render/osg_view.cpp |
| | src/avm_stitch/core/stitch_engine.cpp | src/core/stitch_engine.cpp |
| test_egl_headless_context | tests/avm_stitch/render/test_egl_headless_context.cpp | tests/render/test_egl_headless_context.cpp |
| | src/avm_stitch/render/egl_headless_context.cpp | src/render/egl_headless_context.cpp |
| test_stitch_engine_regression | tests/avm_stitch/core/test_stitch_engine_regression.cpp | tests/core/test_stitch_engine_regression.cpp |

所有测试的 `target_include_directories(... PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)` 保持不变。

---

## 执行顺序

1. 在 `src/` 下创建 `common/` 和 `eval_metrics/` 目录（若不存在）
2. 将 `common/base64.h` → `src/common/base64.h`（`git mv`）
3. 将 `eval_metrics/eval_metrics.hpp` → `src/eval_metrics/eval_metrics.hpp`（`git mv`）
4. 将 `src/avm_stitch/{geometry,render,core,util}/` 下的全部文件移到 `src/{geometry,render,core,util}/`（`git mv`，每文件）
5. 将 `tests/avm_stitch/{geometry,render,core,util}/` 下的全部文件移到 `tests/{geometry,render,core,util}/`（`git mv`，每文件）
6. 删除空目录: `src/avm_stitch/`、`tests/avm_stitch/`、`common/`（仓库根）、`eval_metrics/`（仓库根）
7. 更新所有 .hpp/.cpp 中的 `#include "avm_stitch/xxx"` → `#include "xxx"`（去掉前缀）
8. 更新 CMakeLists.txt:
   a. `target_include_directories` 删掉 `${CMAKE_CURRENT_SOURCE_DIR}`
   b. 所有 `src/avm_stitch/` → `src/`
   c. 所有 `tests/avm_stitch/` → `tests/`
9. 运行 cmake 重新配置: `cmake --preset default`
10. 编译: `cmake --build build`
11. 运行 ctest: `ctest --test-dir build`

---

## 成功标准

| # | 标准 | 验证方法 |
|---|------|---------|
| 1 | common/base64.h 在 src/common/ 下 | `ls src/common/base64.h` |
| 2 | eval_metrics/eval_metrics.hpp 在 src/eval_metrics/ 下 | `ls src/eval_metrics/eval_metrics.hpp` |
| 3 | 仓库根 common/ 和 eval_metrics/ 已删除 | `ls common/ eval_metrics/` 报错 |
| 4 | src/avm_stitch/ 目录为空且可删除 | `ls src/avm_stitch/` 仅含空子目录或无内容 |
| 5 | tests/avm_stitch/ 目录为空且可删除 | `ls tests/avm_stitch/` 仅含空子目录或无内容 |
| 6 | 全部 24 个 src 文件在正确子目录 | `ls src/{geometry,render,core,util}/` |
| 7 | 全部 9 个测试文件在正确子目录 | `ls tests/{geometry,render,core,util}/` |
| 8 | 所有 #include 不含 avm_stitch/ 前缀 | `rg '#include "avm_stitch/' src/ tests/` 无匹配 |
| 9 | cmake 配置成功 | `cmake --preset default` 无错误 |
| 10 | 编译成功 | `cmake --build build` 零错误 |
| 11 | 全部单元测试通过 | `ctest --test-dir build` 全部 PASSED |
| 12 | target_include_directories 不含仓库根 | `grep 'CMAKE_CURRENT_SOURCE_DIR}' CMakeLists.txt` 仅匹配 src |

---

## 风险与缓解

| 风险 | 可能性 | 缓解措施 |
|------|--------|---------|
| include 路径遗漏 | 中 | 全量 grep 扫描 `#include "avm_stitch/`；编译会暴露遗漏 |
| CMakeLists.txt 路径写错 | 中 | cmake 重新配置会检测到缺失源文件，报明确错误 |
| git mv 未正确保留历史 | 低 | 使用 `git mv` 而非 `mv`；`git log --follow` 验证 |
| 仓库根 include 删除后 common/ 或 eval_metrics/ 引用失败 | 低 | `rg '#include "(common/|eval_metrics/)' src/` 确认仍在 src/ 内；正则已验证只有 `osg_backend.cpp` 和 `stitch_result.cpp` 含这两个 include |
| 被忽略的构建产物引用旧路径 | 低 | 删除 build 目录，从干净状态重新配置 |
| 跨平台编译问题（QNX 等） | 低 | 重构不改变任何代码逻辑，#include 路径在预处理阶段解析，不引入平台相关问题 |
