# 碗模型黑色楔形缝修复 — 实现计划

**计划锁定的阶段: WHAT ONLY. 每个任务的 HOW 部分（实现本体）由 executor 在测试反馈下书写。**

**设计文档:** `docs/specs/2026-09-11-black-seam-fix-design.md`
**5 项设计决策:** D1(car_center) / D2(光轴扇区) / D3(烘焙兜底) / D4(手写投影) / D5(数据驱动)

---

## Phase 0: 验证引用

```
已验证:
  src/geometry/projection.hpp              ✓ 存在（第 41-66 行 ProjectWorldToFishEye）
  src/geometry/bowl_model.hpp               ✓ 存在（第 27-39 行 BowlParams）
  src/geometry/bowl_model.cpp               ✓ 存在（第 281-337 行 BakeVertices）
  src/geometry/blend_weights.hpp            ✓ 存在（第 26-28 行 ComputeBlendWeights）
  src/geometry/blend_weights.cpp            ✓ 存在（第 21-24 行 AzimuthDeg）
  examples/data/cameras.json                ✓ 存在（四相机 T_wc 外参）
  examples/avm_viewer.cpp                   ✓ 存在（第 144 行 BowlParams 默认构造）
  tests/geometry/test_projection.cpp        ✓ 存在
  tests/geometry/test_blend_weights.cpp     ✓ 存在（第 18-27 行 MakeNCameras 辅助函数）
  tests/geometry/test_bowl_model.cpp        ✓ 存在
  tests/test_support/stitch_test_helpers.hpp ✓ 存在（第 36-45 行 MakeSideCameras）
```

**现有实现冲突检查:**
- `AzimuthDeg` 当前使用 `atan2(t[1], t[0])`（位置方位角）→ 需改为 `atan2(R(1,2), R(0,2))`（光轴地面投影方位）
- `ProjectWorldToFishEye` 当前使用 `cv::fisheye::projectPoints` → 需替换为手写等距模型
- `BakeVertices` 当前权重归一化无兜底分支 → 需增加 owner 不可见时的重归一化
- `BowlParams` 无 `car_center` 字段 → 需增加
- `cameras.json` 无 `car` 块 → 需增加

---

## 文件映射

| 文件 | 修改类型 | 涉及决策 |
|------|----------|----------|
| `src/geometry/projection.hpp` | 修改（ProjectWorldToFishEye 实现替换 + z≤0 移除；IsForwardFacing 加注释） | D4 |
| `src/geometry/blend_weights.hpp` | 修改（ComputeBlendWeights 签名增加 pole 参数） | D2 |
| `src/geometry/blend_weights.cpp` | 修改（AzimuthDeg 实现更换；ComputeBlendWeights 内部增加极点偏移） | D2 |
| `src/geometry/bowl_model.hpp` | 修改（BowlParams 增加 car_center；BakeVertices 签名增加 car_center 参数） | D1 |
| `src/geometry/bowl_model.cpp` | 修改（ComputeRd/ComputeRdBowlBoat/Generate*Mesh 基准偏移；BakeVertices 传递 car_center + D3 兜底 + 移除 IsForwardFacing 门控） | D1/D3 |
| `tests/geometry/test_projection.cpp` | 修改（容差收紧 0.01→1e-6；增加 z<0 测试；增加 z>0 逐值一致性测试） | D4 |
| `tests/geometry/test_blend_weights.cpp` | 修改（MakeNCameras 增加旋转；增加光轴方位角测试；测试 Assertion 更新） | D2 |
| `tests/geometry/test_bowl_model.cpp` | 修改（更新 DefaultIntrinsics/DefaultBakeCameras；增加 car_center 对称性测试；增加 D3 兜底测试） | D1/D2/D3 |
| `tests/test_support/stitch_test_helpers.hpp` | 修改（MakeSideCameras 增加旋转，使其光学轴在地面投影与方位角一致） | D2 |
| `examples/data/cameras.json` | 修改（增加顶层 `car` 块） | D5 |
| `examples/avm_viewer.cpp` | 修改（读取 car 字段并填充 BowlParams） | D5 |
| `tests/geometry/test_black_seam_e2e.cpp` | **新建**（端到端验收：真实数据构建 BowlModel → 统计零权重顶点） | 验收 |
| `CMakeLists.txt` | 修改（新增 test_black_seam_e2e target + gtest_discover_tests） | 验收 |

---

## 任务分解

---

### Batch A: D4 — 手写等距鱼眼投影 [机械批, 可并行]

本批产出被后续批次依赖的**正确投影行为**（z<0 不再返回镜像），但后续批次不依赖本批接口签名。

---

### Task 1: 替换 ProjectWorldToFishEye 为手写等距模型

**What（计划锁定，不可改）:**

**Files:**
- Modify: `src/geometry/projection.hpp:41-66`
- Test: `tests/geometry/test_projection.cpp`

**接口签名（维持不变，实现替换）:**
```cpp
// 签名不变，实现替换。移除 P_cam[2] ≤ kEps 的早期返回（z≤0 不再返回 (-1,-1)）。
// u, v 对 z 任意值有效（θ=atan2(r_xy, z) ∈ [0,π]）。
inline void ProjectWorldToFishEye(const cv::Vec3d& P_world,
                                   const CameraIntrinsics& intr,
                                   const CameraExtrinsics& extr,
                                   double& u, double& v);
```

**测试规格:**

1. **z>0 与 cv::fisheye::projectPoints 逐值一致**（覆盖光轴上、离轴、有畸变、多畸变系数组合）:
   - 输入: `intr={1920,1536,441.45,441.45,952.14,758.27, 0.215, -0.05, 0.003, 0.0}`, `extr=MakeLookAtRotation({3.84,-0.016,0.66}, {0,0,0.2})`, 测试点集 `[{0,0,0.2}, {1,0,0.2}, {0,1,0.2}, {2,2,0.5}, {-2,-1,0.8}, {3,0,0.6}]`
   - 预期: 每个点的 `(u,v)` 与 `cv::fisheye::projectPoints` 输出的差值 < `1e-6` px
   - 验证命令: `cmake --build build --target test_projection && ctest --test-dir build -R ProjectionTest.MatchesOpenCV` → PASS

2. **z<0 输入不返回 (-1,-1)**, 输出落在正确半平面（后半球的点投影应关于主点对侧翻转，uv 坐标不是镜像）:
   - 输入: `intr={640,480,300,300,320,240,0,0,0,0}`, `extr={identity, (0,0,0)}`, `P=(0,0,-1)`（严格在相机后方）
   - 预期: `u != -1.0 && v != -1.0`（不再被拒绝）；`u == 320.0 && v == 240.0`（光轴反方向，r_xy=0 → 退化分支输出主点）
   - 输入: `P=(1, 0, -1)`（后方偏右）
   - 预期: `θ = atan2(1, -1) = 3π/4`, uv 应在 `(cx + fx * θ_d/1 * 1, cy)` ≈ `(320+300*θ, 240)`，即 `u > 320`（前端点 `(1,0,1)` 也在右侧，所以正确半平面意味着 uv 坐标的 x 方向与 (x, y) 同号）

3. **r_xy < eps 退化分支:**
   - 输入: `intr={640,480,300,300,320,240,0,0,0,0}`, `extr={identity, (0,0,0)}`, `P=(0,0,1)`
   - 预期: `u == 320.0 && v == 240.0`

4. **IsForwardFacing 注释更新:**
   - 在 `IsForwardFacing` 函数前添加注释: `// DEPRECATED: z>0 only semantic, DO NOT use for visibility gating. Circle check replaces FOV boundary.`

**声明式产物（新投影公式全文）:**
```cpp
cv::Vec3d P_cam = TransformWorldToCamera(P_world, extr);
const double x = P_cam[0], y = P_cam[1], z = P_cam[2];
const double r_xy = std::sqrt(x*x + y*y);
const double& fx = intr.fx, &fy = intr.fy, &cx = intr.cx, &cy = intr.cy;
const double& k1 = intr.k1, &k2 = intr.k2, &k3 = intr.k3, &k4 = intr.k4;

if (r_xy < kEps) {
    u = cx;
    v = cy;
    return;
}

double theta = std::atan2(r_xy, z);       // ∈ [0, π]
double theta2 = theta * theta;
double theta4 = theta2 * theta2;
double theta6 = theta4 * theta2;
double theta8 = theta4 * theta4;
double theta_d = theta * (1.0 + k1*theta2 + k2*theta4 + k3*theta6 + k4*theta8);
double factor = theta_d / r_xy;
u = cx + fx * factor * x;
v = cy + fy * factor * y;
```

**验证命令:**
- RED 验证: `cmake --build build --target test_projection 2>&1 && ctest --test-dir build -R ProjectionTest` → FAIL with [旧 cv::fisheye 实现被替换后，MatchesOpenCV 容差 0.01 仍宽松，z<0 测试不存在故 RED 仅因功能缺失（无 z<0 断言）]。RED 因新测试添加、旧测试行为改变而失败。
- GREEN 验证: `cmake --build build --target test_projection && ctest --test-dir build -R ProjectionTest` → ALL PASS

**How（executor 在 HOW 内自由, plan 有意留白）:**
- 实现替换: 将第 44-66 行的 `cv::fisheye::projectPoints` 调用替换为上述公式。移除 z≤kEps 早期返回。
- 测试更新: MatchesOpenCV 容差从 `0.01` 收紧到 `1e-6`；新增 z<0 测试（PointBehindCamera_ReturnsValidUV 和 退化分支测试）；新增注释到 IsForwardFacing

---

### Batch B: D2 — 扇区模型修正 [契约批]

本批产出 `ComputeBlendWeights` 新签名（含 pole 参数），被 Batch C 依赖。

---

### Task 2: 提取光轴地面方位角工具函数

**What（计划锁定，不可改）:**

**Files:**
- Modify: `src/geometry/blend_weights.cpp:21-24`（AzimuthDeg 实现替换）
- Test: `tests/geometry/test_blend_weights.cpp`

**接口签名:**
```cpp
// 在匿名命名空间中，将现有 AzimuthDeg 替换为基于光轴方向的版本:
// 从 R_wc 第三列（相机 Z 轴在世界系中的像）取地面投影 (R(0,2), R(1,2))，
// 方位 = atan2(axis.y, axis.x)。
// 旧: atan2(e.t[1], e.t[0]) — 相机位置方位角
// 新: atan2(e.R(1,2), e.R(0,2)) — 光轴地面投影方位角
double AzimuthDeg(const CameraExtrinsics& e);
```
（匿名命名空间，签名不变，实现语义变更）

**测试规格:**

1. **真实数据前后相机光轴方位验证:**
   - 使用 `examples/data/cameras.json` 中 front 相机外参（`R={...}, t={4.1167, -0.0534, 0.6874}`）
   - front 光轴地面方位 ≈ `atan2(0.0319, 0.9623)` ≈ 2°（即 `NormalizeAngleDeg(2°)` ≈ 2°）

2. **真实数据左右相机光轴方位验证:**
   - left 相机（`t={2.462, 1.096, 1.231}`, R column 2 ≈ `(-0.0259, 0.9244, -0.3806)`）:
   - left 光轴地面方位 = `atan2(0.9244, -0.0259)` ≈ 92°（`NormalizeAngleDeg(92°)` ≈ 92°）
   - 预期: `≈ 92°`（±1°）
   - right 相机（`t={2.474, -0.942, 1.132}`, R column 2 ≈ `(-0.0102, -0.9613, -0.2753)`）
   - 预期: `≈ -91°`（±1°）

3. **旧位置方位角不再影响扇区划分:**
   - 构造相机 `t={3,0,1}` 但 R 使光轴指向 +Y（而非 +X）:
   - `R_wc = [[0,0,0],[0,0,1],[1,-1,0]]` 这种无效旋转的检查... 实际用有效旋转:
   - 构造身份旋转 → 光轴向上 (0,0,1) → 地面投影 (0,0) → atan2(0,0) = 0°（所有相机都 0°，警告: 退化）
   - 测试验证: 身份旋转相机的 AzimuthDeg 返回 0°（不返回位置方位角）

**验证命令:**
- RED 验证: `cmake --build build --target test_blend_weights && ctest --test-dir build -R BlendWeightsTest` → FAIL with [现有测试使用 MakeNCameras(identity rotation) 导致所有光轴方位角=0，断言位置方位角的旧测试失败]
- GREEN 验证: `cmake --build build --target test_blend_weights && ctest --test-dir build -R BlendWeightsTest` → PASS（但需要先完成 Task 3 更新 MakeNCameras）

**How（executor 在 HOW 内自由, plan 有意留白）:**
- 将 `AzimuthDeg` 函数体从 `NormalizeAngleDeg(std::atan2(e.t[1], e.t[0]) * kRadToDeg)` 改为 `NormalizeAngleDeg(std::atan2(e.R(1,2), e.R(0,2)) * kRadToDeg)`
- 注意 R(0,2) 和 R(1,2) 同时接近 0 时的退化情况（光轴垂直向上），atan2(0,0) 返回 0.0（这是有意为之 — 水平布局的 AVM 系统无此退化；存在时属异常标定数据）

---

### Task 3: ComputeBlendWeights 增加极点参数 + 点方位相对极点计算

**What（计划锁定，不可改）:**

**Files:**
- Modify: `src/geometry/blend_weights.hpp:26-28`（签名增加 pole 参数）
- Modify: `src/geometry/blend_weights.cpp:73-81`（ComputeBlendWeights 实现：点的方位改为相对 pole 计算）

**接口签名（破坏性变更 — 增加默认参数，向后兼容源码级）:**
```cpp
// blend_weights.hpp
std::vector<double> ComputeBlendWeights(
    const cv::Vec3d& P_world,
    const std::vector<CameraExtrinsics>& extrinsics,
    const cv::Vec2d& pole = cv::Vec2d(0.0, 0.0));   // 新增: 扇区极点，默认 (0,0) 保持向后兼容
```

**实现变更（仅在 blend_weights.cpp ComputeBlendWeights 函数体内）:**
- 第 80-81 行: `const double theta = NormalizeAngleDeg(std::atan2(P_world[1], P_world[0]) * kRadToDeg);`
- 改为: `const double theta = NormalizeAngleDeg(std::atan2(P_world[1] - pole[1], P_world[0] - pole[0]) * kRadToDeg);`

**测试规格:**
1. **默认 pole=(0,0) 行为不变:**
   - 输入: `ComputeBlendWeights({5,0,0}, MakeNCameras({0,180,90,-90}))` — 不传 pole
   - 预期: 结果与旧实现一致（因 Task 2 的 AzimuthDeg 变了，但点方位仍在原点计算-> 方位角 = atan2(0,5) = 0° → sector 0 dominant，这要求 MakeNCameras 返回的相机光轴方位角与预期方位一致）

2. **非默认 pole 偏移极点:**
   - 输入: `pole=(2,0)`, `P=(7,0,0)` → `theta = atan2(0, 5) = 0°`（相对 pole 的方位仍为 0°），但 `P=(3,0,0)` → `theta = atan2(0, 1) = 0°`，与无极点时 `P=(3,0,0)` → `theta = 0°` 一致
   - 输入: `pole=(2,0)`, `P=(2, 1, 0)` → `theta = atan2(1, 0) = 90°`

**验证命令:**
- RED 验证: `cmake --build build --target test_blend_weights && ctest --test-dir build -R BlendWeightsTest` → FAIL [新签名编译通过但测试因旧辅助函数不匹配而失败]
- GREEN 验证: `cmake --build build --target test_blend_weights && ctest --test-dir build -R BlendWeightsTest` → ALL PASS（需要 Task 4 更新测试后）

**How（executor 在 HOW 内自由, plan 有意留白）:**
- .hpp: 在 ComputeBlendWeights 声明末尾加 `const cv::Vec2d& pole = cv::Vec2d(0.0, 0.0)`
- .cpp: 在函数体内将 `atan2(P_world[1], P_world[0])` 改为 `atan2(P_world[1] - pole[1], P_world[0] - pole[0])`

---

### Task 4: 更新 BlendWeights 测试 — 辅助函数 MakeNCameras 增加旋转

**What（计划锁定，不可改）:**

**Files:**
- Modify: `tests/geometry/test_blend_weights.cpp:17-27`（MakeNCameras 增加旋转，使光轴地面投影方位与位置方位一致）
- Modify: 测试断言验证光轴方位而非位置方位

**MakeNCameras 接口（改造后）:**
```cpp
// N 个相机，每个相机位置在方位角 azimuths_deg[i] 方向、半径 3m、高度 1m。
// 旋转矩阵 R_wc 使光轴地面投影指向方位角 azimuths_deg[i] 方向:
//   R_wc column 2 (forward) = (cos(az), sin(az), 0)   // 光轴水平指向前方
//   R_wc column 1 (down)    = (0, 0, -1)               // 相机 Y 轴指向下方
//   R_wc column 0 (right)   = column1 × column2        // 右手系
std::vector<CameraExtrinsics> MakeNCameras(
    const std::vector<double>& azimuths_deg);
```

**测试变更:**
1. 保留所有现有测试函数名和大部分断言逻辑，仅更新因光轴方位变化而变的数值:
   - `N4Standard_Front`: 相机 0 光轴方位 = 0°（仍指向 +X），点 {5,0,0} 方位 = 0° → front 权重 > 0.9 ✓
   - `N4Standard_SectorBoundaries`: 边界 = 45° / 135° / 225° / 315°（与旧测试逻辑一致，因光轴方位 = 位置方位，新旧边界相同）✓
   - `N4_CosSmoothAtBoundary`: 点 {5,5,0} 方位 45° = 前/左边界 → 权重 0.5/0.5 ✓

2. **新增光轴方位验证测试:**
   - `OpticalAxisAzimuth_IdentityRotationReturnsZero`: 身份旋转相机（R=identity）→ 光轴向上 → 地面投影 (0,0) → atan2(0,0)=0°。构造 CameraExtrinsics{R=identity, t=(3,0,1)}，调用 `AzimuthDeg`（通过匿名命名空间内部的函数——但 AzimuthDeg 是匿名空间的，测不了... 可以改暴露或通过 ComputeSectorBoundaries 间接测）。

   改为: 通过 `ComputeSectorBoundaries` 间接验证: 一台 identity 旋转相机 → 扇区数量 = 1，边界 = 0°（或扇区数量=1时边界返回 []，不返回值）。简化: N=1 时直接测 `ComputeBlendWeights` 权重 = 1.0 给唯一相机（已在 N1_SingleCamera 覆盖），不需要光轴方位验算。

   简化: 不测 identity 旋转的退化情况（生产数据不会有），保留现有测试结构。

3. **测试顶点方位从原点相对改为 pole 相对:** 不在本任务内（Task 3 已加入 ComputeBlendWeights 的 pole 参数，但现有测试不传 pole 即默认 (0,0)，行为不变）。

**验证命令:**
- GREEN 验证: `cmake --build build --target test_blend_weights && ctest --test-dir build -R BlendWeightsTest` → ALL PASS

**How（executor 在 HOW 内自由, plan 有意留白）:**
- 更新 MakeNCameras: 对每个 azimuth_deg 构建旋转矩阵:
  ```
  double rad = az * M_PI / 180.0;
  cv::Vec3d forward(std::cos(rad), std::sin(rad), 0.0);  // column 2
  cv::Vec3d down(0.0, 0.0, -1.0);                        // column 1
  cv::Vec3d right = down.cross(forward);                  // column 0 = Y × Z
  // R_wc = [right | down | forward]
  cv::Matx33d R(right[0], down[0], forward[0],
                right[1], down[1], forward[1],
                right[2], down[2], forward[2]);
  ```
- 保留现有测试函数名和断言值（因光轴方位现在与位置方位一致，扇区边界值不变）

---

### Task 5: 更新 MakeSideCameras（测试辅助函数）增加旋转

**What（计划锁定，不可改）:**

**Files:**
- Modify: `tests/test_support/stitch_test_helpers.hpp:36-45`（MakeSideCameras 增加旋转）

**MakeSideCameras 改造后签名（不变）:**
```cpp
// N 个身份旋转相机 → 光轴垂直向上。
// 改造后: 每个相机光轴地面投影指向方位角 azimuths_deg[i]，旋转构造同 MakeNCameras。
// 位置在 radius=3m, height=1m（设计不变）。
std::vector<CameraExtrinsics> MakeSideCameras(
    const std::vector<double>& azimuths_deg);
```

**影响范围（调用方均自动收到新旋转，无需逐处改断言）:**
- `test_bowl_model.cpp`: DefaultBakeCameras() = MakeSideCameras({0, 180, 90, -90})
  - 影响测试: N4_NumCameras / N4_WeightsSumToOne_WhenVisible / N4_UVInNormalizedRange / N4_InvisibleCameraHasZeroWeight / N4_AllVerticesHaveAtLeastOneVisible
  - 这些测试的断言是"权重归一化"或"可见性计数"，不依赖特定相机方位值 → 自动兼容 ✓
- `test_stitch_engine.cpp`: 使用 MakeSideCameras 的验证测试（如 `ConfigureN0Throws`、`ConfigureN9Throws`）不检查渲染输出 → 自动兼容 ✓
- GL 测试使用 `MakeLookAtCameras`（已有正确旋转）→ 不受影响 ✓

**验证命令:**
- RED 验证: `cmake --build build && ctest --test-dir build -R "BowlModelBakeTest|BowlModelTest"` → 可能 FAIL（因 MakeSideCameras 旋转改变后，某些顶点可见性发生变化，导致旧计数断言失效——需要检查并调整断言值）
- GREEN 验证: `cmake --build build && ctest --test-dir build -R "BowlModelBakeTest|BowlModelTest"` → ALL PASS

**How（executor 在 HOW 内自由, plan 有意留白）:**
- 在 MakeSideCameras 函数体内，为每台相机构建同 Task 4 的旋转矩阵（forward 指向方位角方向，down = (0,0,-1)）
- 更新 test_bowl_model.cpp 中可能因可见性变化而需要调整数值的断言（如 `N4_AllVerticesHaveAtLeastOneVisible` 的 visible/invisible 计数）

---

### Batch C: D1 — BowlParams car_center [契约批]

本批产出 `BowlParams.car_center` 字段，被 Batch D 依赖。

---

### Task 6: BowlParams 增加 car_center + 网格生成基准偏移

**What（计划锁定，不可改）:**

**Files:**
- Modify: `src/geometry/bowl_model.hpp:27-39`（BowlParams 增加 car_center）
- Modify: `src/geometry/bowl_model.cpp:35-92`（ComputeRd / ComputeRdBowlBoat / GenerateBlockBoatMesh / GenerateBowlBoatMesh 改为相对 car_center 计算）

**接口签名变更:**
```cpp
// bowl_model.hpp
struct BowlParams {
    // ... 现有字段不变 ...
    cv::Vec2d car_center{0.0, 0.0};   // 新增: 车体中心在数据坐标系中的位置 (m)
                                       // 默认 (0,0) = 底部中心系，向后兼容
                                       // 后轴系: 调用方设 (axis_x/2, 0)
    static BowlParams Default() { return BowlParams{}; }
};
```

**实现变更:**

1. **ComputeRd** (bowl_model.cpp:35-42):
   - 增加 `car_center` 参数（或直接读取 params.car_center）
   - `double dx = std::max(0.0, std::abs(x - car_center[0]) - (car_l_half + params.round_l));`
   - `double dy = std::max(0.0, std::abs(y - car_center[1]) - (car_w_half + params.round_l));`
   - **签名变更:** `static double ComputeRd(double x, double y, const BowlParams& params);` → 内部从 params 读 car_center，签名不变（因为 BowlParams 已包含 car_center）

2. **ComputeRdBowlBoat** (bowl_model.cpp:48-92):
   - 同样改为相对于 car_center

3. **GenerateBlockBoatMesh** (bowl_model.cpp:97-145):
   - 网格生成点 `(x,y)` 不变（网格以数据原点为中心），但 ComputeRd 内部的偏移使平底矩形区域实际以 car_center 为中心

4. **GenerateBowlBoatMesh** (bowl_model.cpp:150-236):
   - 同上，ComputeRdBowlBoat 内部已处理偏移

**测试规格:**

1. **car_center=(0,0) 向后兼容（默认值）:**
   - `BowlParams::Default().car_center == cv::Vec2d(0.0, 0.0)`
   - `BowlParams::Default()` 构造的碗网格与旧版顶点数量一致（构造后 vertex_count 不变）

2. **非零 car_center 网格对称性:**
   - 输入: `params.car_center = (1.55, 0.0)`, `params.car_length=5.6`, `params.car_width=2.2`
   - 构造 BowlModel 后，取 `x∈[-1, 1]` 且 `y∈[-1, 1]` 范围内的顶点中，计算质心
   - 预期: 平底区域以 `(1.55, 0.0)` 为中心（旧代码以 `(0,0)` 为中心）。验证方式: 计算 `x > car_center[0]` 和 `x < car_center[0]` 的平底顶点数量应大致相等（对称性）

3. **ComputeRd 偏移验证:**
   - 输入: `params={car_length=2.0, car_width=1.0, round_l=0.5, car_center=(2.0, 0.0)}`, `(x,y)=(3.0, 0.0)`
   - `x - car_center[0] = 1.0` → `|1.0| - (1.0 + 0.5) = -0.5` → `dx=0.0` → `rd = 0.0`（点在平底区域内）
   - 旧代码 `x=3.0` → `|3.0| - (1.0 + 0.5) = 1.5` → `dx=1.5` → `rd > 0`（点在平底区域外）
   - 预期: car_center 偏移后平底区域向右移动了 2.0m

**验证命令:**
- RED 验证: `cmake --build build --target test_bowl_model && ctest --test-dir build -R BowlModelTest` → FAIL [新字段 car_center 尚未使用，现有测试因 car_center 默认值保持不变而不应失败——实际上应 PASS。RED 来自新增的对称性测试找不到旧行为]
- GREEN 验证: `cmake --build build --target test_bowl_model && ctest --test-dir build -R BowlModelTest` → ALL PASS

**How（executor 在 HOW 内自由, plan 有意留白）:**
- bowl_model.hpp: BowlParams 结构体末尾加 `cv::Vec2d car_center{0.0, 0.0};`
- bowl_model.cpp: ComputeRd 中 dx/dy 计算改为相对 params.car_center 偏移；ComputeRdBowlBoat 同理；Generate*Mesh 内部调用 ComputeRd/RdBowlBoat 时参数不变（函数内部已加偏移）

---

### Task 7: BakeVertices 传递 car_center 到 ComputeBlendWeights

**What（计划锁定，不可改）:**

**Files:**
- Modify: `src/geometry/bowl_model.hpp:138`（BakeVertices 签名增加 car_center 参数）
- Modify: `src/geometry/bowl_model.cpp:281-284`（BakeVertices 调用 ComputeBlendWeights 时传入 car_center）
- Modify: `src/geometry/bowl_model.cpp:13-30`（两个构造函数调用 BakeVertices 时传入 car_center）

**接口变更:**

```cpp
// bowl_model.hpp — BakeVertices 签名
void BakeVertices(const std::vector<CameraIntrinsics>& intrinsics,
                  const std::vector<CameraExtrinsics>& extrinsics,
                  const cv::Vec2d& car_center = cv::Vec2d(0.0, 0.0));
```

```cpp
// bowl_model.cpp — 主构造函数（第 13-22 行）
BowlModel::BowlModel(const BowlParams& params, ...) {
    // ... mesh gen ...
    BakeVertices(intrinsics, extrinsics, params.car_center);
}

// wrap 构造函数（第 24-30 行）
BowlModel::BowlModel(std::vector<cv::Vec3d> vertices, std::vector<cv::Vec3d> normals,
                     const std::vector<CameraIntrinsics>& intrinsics,
                     const std::vector<CameraExtrinsics>& extrinsics,
                     const cv::Vec2d& car_center = cv::Vec2d(0.0, 0.0))  // 新增参数
    : vertices_(std::move(vertices)), normals_(std::move(normals)) {
    BakeVertices(intrinsics, extrinsics, car_center);
}
```

**实现变更:**
- `BakeVertices` 第 291 行: `const auto raw_weights = ComputeBlendWeights(P, extrinsics);`
- 改为: `const auto raw_weights = ComputeBlendWeights(P, extrinsics, car_center);`

**测试规格:**
1. wrap 构造函数默认 car_center=(0,0):
   - 现有测试 `IndexAccess_WrappingConstructorCounts` 不传 car_center → 默认 (0,0) → 行为不变 ✓
2. BakeVertices 传递正确:
   - 通过构造 BowlModel 并验证权重归一化不变（car_center 默认值时与旧行为一致）来间接验证

**验证命令:**
- GREEN 验证: `cmake --build build --target test_bowl_model && ctest --test-dir build -R BowlModelBakeTest` → ALL PASS

**How（executor 在 HOW 内自由, plan 有意留白）:**
- BakeVertices 签增加 `const cv::Vec2d& car_center` 参数（默认 (0,0)）
- 第 291 行: `ComputeBlendWeights(P, extrinsics, car_center)`
- 主构造函数传入 `params.car_center`
- wrap 构造函数签名增加 `car_center` 参数（默认 (0,0)）

---

### Batch D: D3 + D5 + 验收 [机械批]

---

### Task 8: BakeVertices 可见相机兜底归一化（D3）

**What（计划锁定，不可改）:**

**Files:**
- Modify: `src/geometry/bowl_model.cpp:296-336`（BakeVertices 增加兜底分支）
- Modify: `src/geometry/bowl_model.cpp:307-310`（移除 IsForwardFacing 可见性门控——圆检查接管 FOV 边界判定）

**实现变更:**

1. **可见性判定（第 307-310 行）**:
   - 旧: `visible[j] = IsForwardFacing(P, extrinsics[j]) && u_px >= 0.0 && u_px < intr.width && v_px >= 0.0 && v_px < intr.height && IsInsideFisheyeCircle(...)`
   - 新: `visible[j] = u_px >= 0.0 && u_px < intr.width && v_px >= 0.0 && v_px < intr.height && IsInsideFisheyeCircle(...)`
   - （移除 `IsForwardFacing` — 圆检查 + 手写投影对 z<0 的正确处理自然接管 FOV 边界）

2. **兜底分支（在第 329-336 行 `if (weight_sum > 1e-9)` 之后增加 `else` 分支）**:
```cpp
if (weight_sum > 1e-9) {
    // 现有归一化代码不变
    ...
} else {
    // D3: owner 不可见但存在可见相机 → 可见集合内重归一化
    double visible_raw_sum = 0.0;
    for (int j = 0; j < N; ++j) {
        if (visible[j] && raw_weights[j] > 1e-9 && edge_factor[j] > 0.0) {
            visible_raw_sum += raw_weights[j] * edge_factor[j];
        }
    }
    if (visible_raw_sum > 1e-9) {
        for (int j = 0; j < N; ++j) {
            if (visible[j] && raw_weights[j] > 1e-9 && edge_factor[j] > 0.0) {
                bv.weight[j] = static_cast<float>(
                    raw_weights[j] * edge_factor[j] / visible_raw_sum);
            }
        }
    }
    // 无任何可见相机: 所有权重保持 0（中心车底物理盲区，属预期行为）
}
```

**测试规格:**

1. **Owner 不可见但邻相机可见 → 非零权重:**
   - 构造场景: 相机 0 位于 `(3, 0, 1)`（identity 旋转），相机 1 也位于 `(3, 0, 1)` 但 R 使光轴朝 +X 看向车中心（使用 MakeLookAtCamera 风格的旋转）
   - 顶点 `P=(2, 0, 1.05)`（z=1.05 刚好在相机 0 的可见门限之上——但相机 0 身份旋转光轴朝上，`P_cam=(2-3, 0-0, 1.05-1)=(-1, 0, 0.05)` → z=0.05 > 0 → 正向... 需要更精确的设计）

   简化: 使用 `MakeSideCameras({0, 180, 90, -90})`（更新后的版本，光轴水平指向方位角）。取顶点 `P=(5, 0, 0.5)`（z=0.5 < 相机高度 1.0 → 所有相机从上方看这个点 → P_cam_z = 1.0 - 0.5 = 0.5 > 0 → 正向... 但低于相机高度的点其实可见）

   更好: 用 `MakeLookAtCamera` 构造一台上方向下看的相机 + 一台侧向相机。前相机看不到车中心区域的某个点（因该点在前相机视野之外或 FOV 边界），但侧相机能看到。

   最干净: 构造两个相机:
   - 相机 0: 位置 `(0, 0, 5)`，身份旋转（光轴朝 +Z/向上），顶点 `P=(0, 0, 0.2)` → `P_cam = (0, 0, 0.2-5) = (0, 0, -4.8)` → z<0 → invisible。（使用手写投影后，uv 坐标会在正确半平面但超出 fisheye circle）
   - 相机 1: `MakeLookAtCamera(0, 4, 2, 0)`（在 +X 方向 4m 高 2m 处看向地面原点），顶点 `P=(0, 0, 0.2)` → visible
   - 预期: 顶点权重不在相机 0，全在相机 1，且归一化和为 1.0

2. **无任何可见相机 → 权重保持零（物理盲区）:**
   - 构造相机: `{(identity, (0,0,5))}`（z=5m 高，光轴朝上）
   - 顶点: `P=(0, 0, 0.2)` → 所有相机不可见（z<0，uv 在圆外）
   - 预期: `weight[0] == 0.0`

3. **owner 可见 → 与旧行为一致:**
   - 现有 `N4_WeightsSumToOne_WhenVisible` 测试应继续 PASS（car_center 默认 (0,0) + 可见性正常）

4. **IsForwardFacing 不再门控:**
   - 构造一个 P_cam z<0 但在圆内的点... 实际上 z<0 意味着 θ > π/2，即鱼眼视角 > 90°。对于典型鱼眼，θ_d > θ > π/2，投影半径 = fx * θ_d > fx * π/2 ≈ 471（对于 fx=300），远大于 fisheye circle radius (240) → 被圆检查自然拒绝。
   - 验证: 构造 `P=(0, 1, 0)`（在相机正下方 1m），相机 `{identity, (0,0,0)}` → `P_cam=(0,1,0)` → `z=0, r_xy=1` → `θ = atan2(1, 0) = π/2`, `θ_d = θ`, `u = 320 + 300 * (π/2/1) * 0 = 320`, `v = 240 + 300 * (π/2/1) * 1 = 240 + 471.2 = 711.2` → 远 > height → 不在矩形内 → `visible = false` ✓

**验证命令:**
- RED 验证: `cmake --build build --target test_bowl_model && ctest --test-dir build -R BowlModelBakeTest` → FAIL [新增 D3 测试因功能缺失而失败]
- GREEN 验证: 同上 → ALL PASS

**How（executor 在 HOW 内自由, plan 有意留白）:**
- 在 BakeVertices 第 307 行移除 `IsForwardFacing` 调用和 `&&` 连接
- 在 `if (weight_sum > 1e-9)` 块之后增加 `else` 分支实现兜底归一化
- D3 测试: 使用 `MakeLookAtCamera` 构造可见/不可见场景

---

### Task 9: cameras.json 增加 car 字段 + avm_viewer 读取填充

**What（计划锁定，不可改）:**

**Files:**
- Modify: `examples/data/cameras.json:1`（增加顶层 car 对象）
- Modify: `examples/avm_viewer.cpp:144-145`（读取 car 字段并填 BowlParams）

**声明式产物（cameras.json 新增字段全文）:**

```json
{
  "car": {
    "length": 5.27,
    "width": 2.17,
    "axis_x": 3.1,
    "origin_convention": "rear_axle"
  },
  "cameras": [
    // ... 现有四相机数据不变 ...
  ]
}
```

**avm_viewer 变更:**

```cpp
// 在 LoadCameraConfig 之后，构造 BowlParams 之前，读取 car 字段:
// 读取 car 块（可选，缺失时用 BowlParams 默认值）
cv::Vec2d car_center(0.0, 0.0);
double car_length = 5.6;
double car_width = 2.2;
cv::FileNode car_node = fs["car"];
if (!car_node.empty()) {
    car_length = static_cast<double>(car_node["length"]);
    car_width = static_cast<double>(car_node["width"]);
    double axis_x = static_cast<double>(car_node["axis_x"]);
    car_center = cv::Vec2d(axis_x / 2.0, 0.0);
    std::string convention = static_cast<std::string>(car_node["origin_convention"]);
    std::cout << "Car origin: " << convention << " (axis_x=" << axis_x
              << " → car_center=(" << car_center[0] << ", " << car_center[1] << "))\n";
} else {
    std::cout << "No 'car' field in cameras.json, using BowlParams defaults\n";
}

BowlParams bp = BowlParams::Default();
bp.car_length = car_length;
bp.car_width = car_width;
bp.car_center = car_center;
```

**测试规格:**
1. **cameras.json 含 car 字段时 avm_viewer 正确读取:**
   - 人工验证: `./build/examples/avm_viewer 2>&1 | head -5` → 输出包含 "Car origin: rear_axle (axis_x=3.1 → car_center=(1.55, 0.0))"
   - 或在测试中读取新 cameras.json 并用 FileStorage 解析验证: 将 avm_viewer 的 JSON 解析逻辑封装为可引用函数（可选，非强制）

**验证命令:**
- 编译: `cmake --build build --target avm_viewer` → 无编译错误
- 运行: `cd build && ./examples/avm_viewer 2>&1` → 控制台输出 car_center 信息

**How（executor 在 HOW 内自由, plan 有意留白）:**
- cameras.json: 在顶层 `{` 后、`"cameras"` 前增加 `"car": { ... },` 块
- avm_viewer.cpp: 在第 102 行 `LoadCameraConfig` 之后增加读取 car 块的代码，使用现有的 `cv::FileStorage` 和 `fs["car"]` API

---

### Task 10: 端到端验收 — 黑楔计数检查点

**What（计划锁定，不可改）:**

**Files:**
- Create: `tests/geometry/test_black_seam_e2e.cpp`
- Modify: `CMakeLists.txt`（新增 test_black_seam_e2e target）

**测试规格:**

1. **使用 cameras.json 真实数据构建 BowlModel，零权重顶点数 < 50:**
   - 读取 `examples/data/cameras.json` 中的相机参数和 `car` 字段
   - `car_center = (car.axis_x / 2, 0)`
   - `car_length = car.length`, `car_width = car.width`
   - 构造 `BowlParams` 并创建 `BowlModel`
   - 遍历所有顶点，统计 `weight_sum == 0` 的顶点数
   - 预期: `zero_weight_count < 50`
   - 报告: `std::cout << "Zero-weight vertices: " << zero_weight_count << " / " << model.vertex_count()`

2. **现有 113 个测试全部通过:**
   - `ctest --test-dir build` → 100% PASS

**验证命令:**
- 编译: `cmake --build build --target test_black_seam_e2e` → 无编译错误
- 运行: `ctest --test-dir build -R BlackSeamE2E` → PASS
- 全量回归: `cmake --build build && ctest --test-dir build` → ALL PASS（113+1=114 个测试）

**CMakeLists.txt 新增 target 规格:**
```cmake
add_executable(test_black_seam_e2e
    tests/geometry/test_black_seam_e2e.cpp
    src/geometry/bowl_model.cpp
    src/geometry/blend_weights.cpp
)
target_include_directories(test_black_seam_e2e PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
)
target_link_libraries(test_black_seam_e2e PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_black_seam_e2e)
```

**How（executor 在 HOW 内自由, plan 有意留白）:**
- 在 test_black_seam_e2e.cpp 中实现:
  - 用 `cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON` 打开 cameras.json
  - 读取四相机 intrinsics/extrinsics（复用 avm_viewer.cpp 中类似解析逻辑）
  - 读取 car 块中的 length/width/axis_x
  - 构造 BowlParams 并创建 BowlModel
  - 遍历 `baked_vertex(i)` 统计零权重点
  - ASSERT_LT(zero_weight_count, 50)

---

## 批次总览

| 批次 | 任务 | 类型 | 可并行 | 依赖 | Checkpoint |
|------|------|------|--------|------|------------|
| A | Task 1 (D4 投影) | 机械批 | ✅ 可并行 | 无 | 否 |
| B | Task 2–4 (D2 扇区) | 契约批 | ❌ 串行 | 无（光轴提取独立） | 否 |
| C | Task 5–6 (D1 几何) + Task 7 (D1 Bake传递) | 契约批 | ❌ 串行 | 依赖 Batch B（ComputeBlendWeights pole 参数） | 否 |
| D | Task 8 (D3 兜底) + Task 9 (D5 接线) + Task 10 (验收) | 机械批 | ❌ 串行 | 依赖 Batch C（car_center + BakeVertices） | **检查点： Task 10（全量测试验收 + 黑楔计数）** |

**批次单 commit 策略:**
- 每批内按任务顺序完整 TDD（RED → GREEN → REFACTOR），全部 GREEN 后一次 commit
- 批内 BLOCKED 时对已完成且测试通过的任务抢救 commit，标注部分完成

**依赖关系标注:**
- Batch A（D4 投影）与 Batch B（D2 扇区）在文件级别**不相交**（projection.hpp vs blend_weights.cpp），且无接口依赖，可**并行**执行（分别在不同分支或在同一工作目录交错进行，推荐分批次串行执行以避免冲突）
- Batch C（D1）必须等待 Batch B 完成后才能开始（依赖 ComputeBlendWeights 的新签名）
- Batch D（D3/D5/验收）必须等待 Batch C 完成后才能开始（依赖 BakeVertices 的新调用）

---

## 关键依赖

| 依赖 | 描述 | 提供方 |
|------|------|--------|
| D4（手写投影）→ D3（兜底） | D4 使 z<0 投影正确，D3 才能安全移除 IsForwardFacing | Task 1 |
| D2（扇区极点）→ D1（BakeVertices 传递） | ComputeBlendWeights 新签名含 pole 参数 | Task 3 → Task 7 |
| D1（car_center）→ D3（兜底） | BakeVertices 需要 car_center 传 ComputeBlendWeights | Task 6+7 → Task 8 |
| D1 + D5（数据驱动）→ 验收 | 真实 cameras.json + BowlParams 正确配置 | Task 6+9 → Task 10 |

---

## 自检清单

- ✅ 设计文档 5 项决策（D1-D5）均有对应任务
- ✅ WHAT 部分无占位符（TBD/TODO/省略）
- ✅ 每个任务包含具体文件路径、接口签名、测试规格、验证命令
- ✅ HOW 部分留白（实现本体不写）
- ✅ 接口签名在计划中锁死
- ✅ 批次划分标注了类型（契约批/机械批）和检查点
- ✅ 检查点任务（Task 10）显式标记
- ✅ 引用的文件路径存在（Phase 0 已验证）
- ✅ 类型一致性：后面的任务使用的类型/签名与前面的任务一致
- ✅ 端到端验证可复现（C++ 测试 + cmake/ctest 命令）