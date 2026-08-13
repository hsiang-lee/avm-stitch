# 修复 OSGGLTest 3 个失败测试 — 设计规范

> **状态:** 草稿

**目标:** 修复 `tests/core/test_stitch_engine.cpp` 中 `OSGGLTest` fixture 下 3 个在进入渲染管线前被哨兵断言拦截而失败的测试。

**架构:** 不修改任何渲染生产代码，仅修改测试夹具 `BuildTestScene`，将其中生成相机的函数从「水平侧视」的 `MakeSideCameras` 替换为「俯视地面」的 `MakeLookAtCameras`，使烘培后的网格具有可见顶点，从而通过前置条件断言。

**技术栈:** C++20, CMake 3.25+, vcpkg (OpenCV, unofficial-osg, GTest), OpenGL EGL 无头上下文

---

## 1. 背景与现状

### 1.1 测试文件与 fixture

所有相关代码位于 `tests/core/test_stitch_engine.cpp`（共 731 行）。GL 依赖测试由 `OSGGLTest` fixture 门控（第 311–344 行）：无 OpenGL 上下文时整个套件 `GTEST_SKIP`。

测试夹具通过三个辅助函数搭建场景：

| 函数 | 定义位置 | 作用 |
|------|---------|------|
| `MakeSideCameras` | 第 49–58 行 | 生成 N 个 identity 旋转、水平朝里看、半径 3m、高度 1m 的相机 |
| `MakeCoarseBowl` | 第 60–65 行 | 生成粗碗网格（`step=1.0`，`r_outter` 可调） |
| `BuildTestSceneWithExtrinsics` | 第 75–101 行 | 烘培网格（`BakeMesh`）、计算可见顶点权重、构建 OSG 场景并取出纹理数组 |
| `BuildTestScene` | 第 103–107 行 | 便捷封装：用 `MakeSideCameras` 生成相机后调用上面的函数 |
| `MakeLookAtCamera` / `MakeLookAtCameras` | 第 129–155 行 | 生成「俯视地面」的 look-at 相机（r=4m, h=2m, 看向 z=0 的地面中心） |

### 1.2 失败的测试

3 个失败测试全部属于 `OSGGLTest` fixture：

| 测试 | 定义行 | 失败断言行 | 断言 |
|------|--------|-----------|------|
| `OSGViewRenderer_InitializeDoesNotCrash` | 349 | 352 | `ASSERT_TRUE(ts.has_visible_vertices)` |
| `OSGViewRenderer_RenderAllViews_Returns5Mats` | 358 | 362 | `ASSERT_TRUE(ts.has_visible_vertices)` |
| `OSGViewRenderer_RenderAllViews_CorrectSize` | 376 | 379 | `ASSERT_TRUE(ts.has_visible_vertices)` |

三者失败原因相同：`ts.has_visible_vertices` 实际值为 `false`，哨兵断言在进入渲染管线前拦截。

---

## 2. 问题根因

### 2.1 调用链

```
BuildTestScene(4)                                   [行 103–107]
  └─ MakeSideCameras({0, 90, 180, 270})             [行 49–58]
       └─ 每个相机: R = identity, pos = (r·cosθ, r·sinθ, 1.0), r = 3.0
  └─ BuildTestSceneWithExtrinsics(4, extrs)          [行 75–101]
       ├─ MakeCoarseBowl(12.6)                       [行 78] → r_outter=12.6, step=1.0
       ├─ BakeMesh(mesh, intrs, extrs)               [行 80]
       └─ 遍历 baked.vertices 累加 weight[j]         [行 82–86]
            → 权重和全为 0 → has_visible_vertices = false
```

### 2.2 几何原因

`MakeSideCameras` 生成的相机是 identity 旋转（朝 -Z 或 +X 方向水平看），相机位置 z = 1.0m。粗碗网格 `r_outter=12.6, step=1.0` 下，碗面在相机高度 1m 以下的顶点几乎全部背对相机或投影不可见，只有 z > 1 的碗沿（rim）顶点 forward-facing。`BakeMesh` 计算出的每顶点融合权重和为 0（`sum > 1e-9` 从未成立，见第 82–86 行），于是 `has_visible_vertices` 保持 `false`。

因此失败**不在渲染器**，而在 `BuildTestScene` 的相机配置不满足「网格必须存在可见顶点」的前置条件。

### 2.3 对照证据（问题定位依据）

- 同 fixture 的 `OSGViewRenderer_RenderAllViews_DrawsSceneNotJustBackground`（第 393 行）直接调用 `BuildTestSceneWithExtrinsics(4, MakeLookAtCameras(4))`，绕过 `BuildTestScene`，其 `ASSERT_TRUE(ts.has_visible_vertices)`（第 401 行）通过。
- `StitchEngineGL_*` 系列（第 569–688 行）全部用 `MakeLookAtCameras`（r=4m, h=2m，向下看地面），全部通过。
- 第 393–397 行的注释已明确记录：旧的 side-camera 布局只对 z>1 的碗沿贴纹理，look-at 相机则覆盖平坦的碗中心。

结论：生产代码（`osg_view.cpp`、`stitch_engine.cpp`、`mesh_bake.cpp` 等）无缺陷，问题在于 `BuildTestScene` 这个测试便捷封装内部用了错误的相机生成器。

---

## 3. 方案对比

针对「3 个测试因 `has_visible_vertices == false` 被拦截」这一问题，共考虑 4 种修复方案：

| # | 方案 | 说明 | 评估 |
|---|------|------|------|
| A | **改夹具**：`BuildTestScene` 改用 `MakeLookAtCameras` | 使便捷封装与其余通过测试一致 | ✅ **选定**：改动最小、语义正确、不触碰被测代码 |
| B | 改断言：删除/弱化 3 个 `ASSERT_TRUE(ts.has_visible_vertices)` | 让测试「不检查前置条件」继续跑 | ❌ 拒绝：删断言让测试在「证明不了任何东西」的状态下运行，违背断言注释（第 352 行 `// otherwise the test proves nothing`）；且这 3 个测试确实需要真实纹理工作才有意义 |
| C | 改网格：调大 `r_outter` / 调小 `step` 使 side 相机能看到顶点 | 改变 `MakeCoarseBowl` 或 `BuildTestScene` 传入参数 | ❌ 拒绝：side 相机水平朝里，本质上看不到平坦碗心，任何网格参数都无法让平坦区域 forward-facing；加细网格仅放大测试成本，不解决几何本质 |
| D | 改生产代码：让 `BakeMesh`/`BuildOSGScene` 容忍无可见顶点 | 修改 `mesh_bake.cpp` / `osg_scene.cpp` 的可见性判断 | ❌ 拒绝：生产代码行为正确——无可见顶点时渲染确实无意义；这是测试夹具前置条件不满足，不是生产缺陷。改生产代码会掩盖真实场景中的配置错误 |

### 选择方案 A 的理由

1. **语义正确**：`BuildTestScene` 的意图是「搭建一个可用于 OSG 渲染的场景」。look-at 相机（俯视地面）是真实 AVM 相机布局，能保证平坦碗心可见，符合 3 个测试「渲染出 5 个视图 / 尺寸正确」的验证目标。
2. **与既有通过测试对齐**：`OSGViewRenderer_RenderAllViews_DrawsSceneNotJustBackground` 和第 569–688 行的 `StitchEngineGL_*` 系列已经用 `MakeLookAtCameras` 并通过，方案 A 只是把 `BuildTestScene` 对齐到同一约定。
3. **改动面最小**：仅动 `BuildTestScene` 函数体（第 103–107 行），不触碰被测生产代码、不改断言、不改网格。

---

## 4. 详细设计（具体改动）

### 4.1 目标文件

`tests/core/test_stitch_engine.cpp`，仅修改 `BuildTestScene`（第 103–107 行）。

### 4.2 改动 diff

**改前（第 103–107 行）：**

```cpp
TestScene BuildTestScene(int num_cameras) {
    std::vector<double> azimuths;
    for (int i = 0; i < num_cameras; ++i) azimuths.push_back(i * 360.0 / num_cameras);
    return BuildTestSceneWithExtrinsics(num_cameras, MakeSideCameras(azimuths));
}
```

**改后：**

```cpp
TestScene BuildTestScene(int num_cameras) {
    return BuildTestSceneWithExtrinsics(num_cameras, MakeLookAtCameras(num_cameras));
}
```

改动要点：
- 删除 `azimuths` 向量及其填充循环（第 104–105 行）。
- 将 `MakeSideCameras(azimuths)` 替换为 `MakeLookAtCameras(num_cameras)`。

> **声明顺序（编译前提）**：`MakeLookAtCamera` 定义在第 129–146 行、`MakeLookAtCameras` 定义在第 149–155 行，二者都位于 `BuildTestScene`（第 103–107 行）**之后**。C++ 要求先声明后使用，因此 `BuildTestScene` 直接调用 `MakeLookAtCameras` 会编译失败。实现时必须二选一：
>
> 1. 在 `BuildTestScene`（第 103 行）之前插入 `MakeLookAtCameras` 的前置声明 `std::vector<CameraExtrinsics> MakeLookAtCameras(int num_cameras);`；或
> 2. 将 `MakeLookAtCamera`/`MakeLookAtCameras` 两个函数定义整体上移到 `BuildTestScene` 之前。
>
> 两种方式语义等价，仅影响 diff 形态；本文档不指定具体做法，交由实现者选择最小 diff。

### 4.3 保留的函数（不得删除）

| 函数 | 定义行 | 剩余调用点 | 删除后果 |
|------|--------|-----------|---------|
| `MakeSideCameras` | 49–58 | 13 处直接调用（行 435, 443, 455, 466, 480, 488, 501, 518, 528, 538, 547, 559, 720），覆盖 `StitchEngineTest` 配置校验测试（第 435–559 行）与 `DumpViews`（第 720 行） | 删除会导致 13 处编译错误 |
| `MakeCoarseBowl` | 60–65 | 1 处：`BuildTestSceneWithExtrinsics` 第 78 行 | 删除会导致编译错误 |

结论：两个函数都保留原样，仅 `BuildTestScene` 停止使用 `MakeSideCameras`。

### 4.4 不改动项

- 生产代码：`osg_view.cpp`、`stitch_engine.cpp`、`mesh_bake.cpp`、`osg_scene.cpp` 等一律不动。
- 3 个失败测试的断言本身：`ASSERT_TRUE(ts.root.valid())`、`ASSERT_TRUE(ts.texture.valid())`、`ASSERT_TRUE(ts.has_visible_vertices)`、`ASSERT_EQ(5u, views.size())`、尺寸/类型 `EXPECT_EQ` 全部保留。

---

## 5. 影响分析

### 5.1 直接受影响的测试（3 个，预期转绿）

| 测试 | 定义行 | 影响 |
|------|--------|------|
| `OSGViewRenderer_InitializeDoesNotCrash` | 349 | `BuildTestScene` 改用 look-at 相机后，`has_visible_vertices` 为 true，通过第 352 行断言 |
| `OSGViewRenderer_RenderAllViews_Returns5Mats` | 358 | 通过第 362 行断言后进入渲染，验证返回 5 个视图 |
| `OSGViewRenderer_RenderAllViews_CorrectSize` | 376 | 通过第 379 行断言后进入渲染，验证 960x640 CV_8UC3 |

### 5.2 不受影响的测试

- `OSGViewRenderer_RenderAllViews_DrawsSceneNotJustBackground`（第 393 行）：不调用 `BuildTestScene`，直接调用 `BuildTestSceneWithExtrinsics(4, MakeLookAtCameras(4))`，行为不变。
- `StitchEngineGL_*` 系列（第 569–688 行）：用 `MakeLookAtCameras`/`MakeLookAtCamera`，行为不变。
- `StitchEngineTest` 配置校验系列（第 427–564 行）：用 `MakeSideCameras`，函数保留，行为不变。
- `StitchEngineGL_DumpViewsForInspection`（第 688 行）：第 720 行仍用 `MakeSideCameras`，行为不变。
- 其他测试文件（`tests/geometry/*`、`tests/render/*`、`tests/util/*`、`tests/core/test_stitch_engine_regression.cpp`、`tests/core/test_view_mapping.cpp`）不受影响。

### 5.3 影响范围总结

- 修改文件：仅 `tests/core/test_stitch_engine.cpp`。
- 修改函数：仅 `BuildTestScene`（可能附带 `MakeLookAtCameras` 前置声明/上移，见 4.2）。
- 生产代码零改动；`MakeSideCameras`、`MakeCoarseBowl` 语义与调用点零改动。

---

## 6. 验证计划

### 6.1 构建

在 `avm-stitch` 仓库内独立构建（`CMakePresets.json` 提供 `default` preset，使用 vcpkg 工具链）：

```bash
cd /home/leehsiang/Documents/avm-studio/avm-stitch
cmake --preset default
cmake --build build
```

### 6.2 运行测试

全部测试：

```bash
ctest --test-dir build --output-on-failure
```

定向复现修复目标（仅 `OSGGLTest`）：

```bash
./build/tests/core/test_stitch_engine --gtest_filter='OSGGLTest.*'
```

> 说明：GL 测试需 OpenGL 上下文。若无头 EGL 上下文可用（`DetectGLContext` 探测通过）则自动运行；否则需 `AVM_STITCH_TEST_GL=1` 强制，或套件整体 `GTEST_SKIP`（跳过不算失败）。

### 6.3 通过判据

| # | 判据 | 验证方法 |
|---|------|---------|
| 1 | 3 个目标测试转绿 | `--gtest_filter='OSGGLTest.OSGViewRenderer_*'` 全部 PASSED |
| 2 | 无回归 | `ctest --test-dir build` 全部 PASSED（或仅有预期的 GL 跳过） |
| 3 | 编译零错误 | `cmake --build build` 无错误 |
| 4 | `MakeSideCameras`/`MakeCoarseBowl` 仍被引用 | `rg 'MakeSideCameras|MakeCoarseBowl' tests/core/test_stitch_engine.cpp` 仍有匹配 |

### 6.4 经 avm-worker 验证（可选）

由于 `avm-stitch` 通过 `add_subdirectory` 被 `avm-worker` 消费、测试在同一 ctest 树运行，可额外在 avm-worker 根目录执行 `cmake --build build && ctest --test-dir build -R test_stitch_engine --output-on-failure` 确认集成无回归。

---

## 7. 回滚方案

改动集中于单一函数 `BuildTestScene`，回滚方式：

1. **git 回滚**：`git restore tests/core/test_stitch_engine.cpp`（仅撤销该文件）。
2. **手动还原**：将 `BuildTestScene` 改回原始 4 行实现（`azimuths` 循环 + `MakeSideCameras(azimuths)`），并删除为调用 `MakeLookAtCameras` 而添加的前置声明/上移。

回滚后测试恢复原状（3 个测试因 `has_visible_vertices == false` 失败），无其他副作用。回滚风险低，因为本方案不触碰生产代码与网格参数。

---

## 8. 关键决策记录（ADR 风格）

### ADR-1：修复位置定在测试夹具而非生产代码

- **状态**：接受
- **决策**：在 `BuildTestScene` 内替换相机生成器，不改 `osg_view.cpp` / `stitch_engine.cpp` / `mesh_bake.cpp`。
- **理由**：`has_visible_vertices == false` 是测试场景不满足前置条件的表现，生产代码的可见性判断行为正确，无缺陷。

### ADR-2：夹具改用 `MakeLookAtCameras` 而非调整网格参数

- **状态**：接受
- **决策**：`BuildTestScene` 调用 `MakeLookAtCameras(num_cameras)`（r=4m, h=2m 向下看地面）。
- **理由**：side 相机水平朝里，几何上永远看不到平坦碗心，调网格参数无法解决；look-at 布局与既有的通过测试（`DrawsSceneNotJustBackground`、`StitchEngineGL_*`）一致。

### ADR-3：保留 `MakeSideCameras` 与 `MakeCoarseBowl`

- **状态**：接受
- **决策**：两个辅助函数原样保留。
- **理由**：`MakeSideCameras` 仍被 13 处直接调用（`StitchEngineTest` 配置校验 + `DumpViews`），`MakeCoarseBowl` 仍被 `BuildTestSceneWithExtrinsics`（第 78 行）使用；删除会导致编译失败。

### ADR-4：不改动 3 个测试的断言

- **状态**：接受
- **决策**：`valid` / `texture.valid` / `has_visible_vertices` / 5 视图 / 尺寸 断言全部保留。
- **理由**：这些断言是正确的前置条件与输出契约，问题只在夹具不满足前置条件，而非断言错误。

### ADR-5：声明顺序处理交由实现者选择最小 diff

- **状态**：接受
- **决策**：若 `MakeLookAtCameras`（第 149 行）位于 `BuildTestScene`（第 103 行）之后导致未声明，通过在 `BuildTestScene` 前加前置声明或将两个 look-at 函数上移解决，二选一。
- **理由**：两种方式语义等价，仅影响 diff 形态，不改变设计意图。
