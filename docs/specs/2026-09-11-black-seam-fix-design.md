# 碗模型黑色楔形缝修复 设计规范

> **状态:** 草稿

**目标:** 消除 avm_viewer 渲染贴图碗模型时出现的 4 条从中心放射到碗边缘的黑色楔形缝，将零权重（全黑）顶点从实证的 344/12741 降到 <50/12741（残余仅为中心车底物理盲区）。

**架构:** 五项修复全部实施：(1) `BowlParams` 增加显式车中心偏移 `car_center`，碗网格几何以数据坐标系中的车体位置为基准生成；(2) `BlendWeights` 扇区划分从"相机位置方位角"改为"光轴方向地面投影方位角"，扇区极点改为 `car_center`；(3) `BakeVertices` 增加可见相机兜底归一化，owner 不可见时不再输出硬黑；(4) `ProjectWorldToFishEye` 用手写等距鱼眼投影替换 `cv::fisheye::projectPoints`，消除 z<0 输入的镜像像素雷；(5) 车尺寸与原点约定由 cameras.json 顶层 `car` 字段数据驱动，avm_viewer 读取填充。

**技术栈:** C++、OpenCV（`cv::Vec2d`、`cv::Mat`，投影路径去 `cv::fisheye` 依赖）、手写等距鱼眼投影公式、GTest（RED-GREEN）、OSG（avm_viewer，渲染层不改）。

---

## 1. 背景与问题

avm_viewer 渲染贴图碗模型出现 4 条黑色楔形缝，从中心放射到碗边缘。数值探针 + 跨仓库调查（avm-worker / avm_renderer 参考实现）已实锤根因链，共 5 条，每项修复设计（第 3 节 D1–D5）与之一一对应。

### 根因分析（已实证，证据随附）

**R1: 坐标系原点错位。** `examples/data/cameras.json` 是**后轴中心原点**约定数据。铁证：front 相机 x=4.12，按底部中心约定相机悬空于车头前 1.48m，物理不可能；按后轴系则前保险杠 ≈ x=4.1，完全自洽；且与 avm_renderer 参考文件 `extrinsics_params_black.xml` 的原点模式吻合。而 avm-stitch 的 `BowlModel` 碗网格以数据原点为车中心生成 → 网格整体错位 axis_x/2 ≈ 1.55m。对照：avm_renderer 在消费端注入 `bev_offset_x` 补偿（其 `AVMMapper.cpp:787-792`），avm-stitch 无此补偿机制。

**R2: 扇区划分用相机位置方位角而非光轴方向。** `blend_weights.cpp` 中 `AzimuthDeg = atan2(t_y, t_x)`（t 为相机平移量）。左右后视镜相机的**位置**方位角实测 ±24°/-21°，但其**光轴**实际指向 ±90°（从 R_wc 第三列实测四相机光轴地面方位：front≈2°、left≈92°、right≈-91°、rear≈180°）。用位置方位角算出的扇区边界射线（实测 349.2° / 11.6° / 101.2° / 258.8°）与黑楔出现位置完全重合——黑楔即"无 owner 相机可见"的错划扇区。

**R3: 无可见性兜底。** `BakeVertices` 中 owner 相机对顶点不可见时 `weight_sum=0`，权重归一化失败 → 输出全黑。实测 344 个零权重顶点中有 95 个存在几何上可见的其他相机——即近 1/3 的黑点本可由邻相机接管，属纯软件缺陷。

**R4: `cv::fisheye::projectPoints` 镜像雷。** 该函数对 P_cam.z<0（离轴 >90°）的输入静默返回关于主点对侧的镜像像素（实测 θ=106.4° 时输出距中心 193px 的镜像 uv，能通过 in-circle 检查被误判为可见）。当前数据因畸变系数把有效 FOV 压到 ±79.5° 而**尚未触发**；一旦换用低畸变标定即爆雷。属隐患修复，非当前黑缝直接成因。

**R5: 车尺寸硬编码。** `BowlParams` 默认 `car_length=5.6 / car_width=2.2`，真实车为 5.27×2.17（models_config.json 实测），碗底平底矩形每侧偏大 0.165m(x) / 0.115m(y)——车体边缘顶点落在"车内"被所有相机判不可见，贡献边缘黑区。

---

## 2. 目标 / 非目标

### 目标

- 碗网格几何、扇区划分、可见性判定三者与数据坐标系和真实标定几何一致（R1/R2/R5）。
- owner 不可见但存在可见相机时不再输出硬黑（R3）。
- 鱼眼投影对 z<0 输入行为正确（无镜像），低畸变标定下不爆雷（R4）。
- 车尺寸与原点约定由数据显式声明，avm-stitch 不猜测原点约定。

### 非目标

- **fisheye_radius 自动检测**: 当前 k1–k4 是否为占位假值需真实标定确认，记录为**数据侧 follow-up**，本次不做。
- **车角锚定融合线**: avm_renderer 参考实现针对 4 相机专用 rig 的调优项，记录为未来精化项，本次 out of scope（见 D2）。
- **avm-worker 仓库改动**: worker 侧 `ParseBowlParams` 从 `calibration.car` 派生默认值仅作为 follow-up 记录（见 D5），本次不改 worker 仓库。
- 渲染层改动：StitchScene、OSGViewRenderer、shader 均不动。

---

## 3. 设计决策

### D1: BowlParams 增加车中心偏移（对应 R1）

新增字段，设计原文：

```cpp
cv::Vec2d car_center{0.0, 0.0};  // 车体中心在数据坐标系中的位置
```

语义约定：

- 默认 `(0,0)` = 数据原点即车中心，对应**底部中心系**数据的既有语义，向后兼容。
- **后轴系**数据由调用方显式设 `car_center = (axis_x/2, 0)`。
- `GenerateBlockBoatMesh` / `GenerateBowlBoatMesh` 中的网格范围、`ComputeRd`、`ComputeRdBowlBoat` 的 (x,y) 全部改为**相对 `car_center`** 计算。

明确记录的设计立场：avm-stitch **不猜测原点约定**，`car_center` 是"车体在数据系中的位置"的显式声明。投影数学（`TransformWorldToCamera`）本身与原点无关，**不需要改**——本次仅修几何生成与扇区极点的基准点。

### D2: BlendWeights 扇区模型修正（对应 R2）

三处语义变更：

1. **相机方位来源**: 由相机位置方位角改为**光轴方向**——取 R_wc 第三列（相机 z 轴在世界系的像）的地面投影，方位 = `atan2(axis.y, axis.x)`。
2. **扇区极点**: 由数据原点改为 `car_center`。API 增加极点参数，默认 `(0,0)` 保持向后兼容语义。
3. **扇区边界**: 相邻相机光轴方位的**角平分线**，保持现有 52/N 混合带公式不变。

参考实现的"车角锚定融合线"是 4 相机专用 rig 的调优，明确 out of scope，记录为未来精化项。

### D3: 可见相机兜底归一化（对应 R3）

`BakeVertices` 权重烘焙增加兜底分支：

- 触发条件：owner 加权 `weight_sum ≤ 1e-9` **且**存在几何可见的其他相机。
- 处理：权重 = raw_weights 在**可见相机集合内重新归一化**——保留扇区偏好（可见集合内的相对权重仍来自扇区加权），仅消除因 owner 不可见导致的硬黑。
- 无任何可见相机的顶点（中心车底物理盲区）仍允许零权重，属预期行为。
- 边缘 feather 逻辑**不变**。

### D4: 手写等距鱼眼投影（对应 R4）

`ProjectWorldToFishEye` 内部替换 `cv::fisheye::projectPoints` 为手写等距模型，公式为设计原文：

```
θ   = atan2(r_xy, z) ∈ [0, π]
θ_d = θ · (1 + k1·θ² + k2·θ⁴ + k3·θ⁶ + k4·θ⁸)
u   = cx + fx · (θ_d / r_xy) · x
v   = cy + fy · (θ_d / r_xy) · y
r_xy < eps 时: u = cx, v = cy（光轴正向退化分支）
```

行为要求（构成 TDD 验收）：

- **z>0 输入**: 与 `cv::fisheye::projectPoints` 逐值一致，容差 1e-6 px——保证现有标定数据下投影结果不变，修复不引入回归。
- **z<0 输入**: 输出落在**正确半平面**（镜像消除）——`θ ∈ [0,π]` 配合 `θ_d/r_xy` 恒正的缩放使 uv 方向跟随 (x,y) 实际方向，超过有效 FOV 的输入被圆检查自然拒绝。

配套变更：`IsForwardFacing` 从 BakeVertices 可见性链**移除**（圆检查接管 FOV 边界判定）；函数本身保留，但标注"仅 z>0 语义，勿用于可见性门控"。

### D5: 车尺寸数据驱动（对应 R5）

cameras.json 增加顶层 `car` 对象（值取自真实车辆数据，非本次规定），含四个字段：

- `length` / `width`：车长 / 车宽（本车实测 5.27×2.17，models_config.json）
- `axis_x`：后轴距坐标值，用于换算 car_center
- `origin_convention`：原点约定字符串，本数据为 `"rear_axle"`

- avm_viewer 读取 `car` 并填 `BowlParams`：`car_length` / `car_width` 直接取 length/width，`car_center = (axis_x/2, 0)`。
- worker 侧 follow-up（本次不实施）：avm-worker `ParseBowlParams` 无 bowl 覆盖参数时应从 `calibration.car` 派生同值。

---

## 4. 数据流

```
cameras.json（含 car 块: length/width/axis_x/origin_convention）
    │  avm_viewer 读取
    ▼
BowlParams { car_length, car_width, car_center=(axis_x/2, 0) }
    │
    ├─► BowlModel 网格生成：范围 / ComputeRd / ComputeRdBowlBoat 以 car_center 为车中心基准 (D1)
    │
    ├─► ComputeBlendWeights：极点 = car_center，方位 = 光轴地面投影，边界 = 角平分线 + 52/N 混合带 (D2)
    │
    ▼
BakeVertices：逐顶点 → ProjectWorldToFishEye（手写等距投影, D4）→ in-circle 可见性
    │            ├─ owner 加权 weight_sum > 1e-9 → 正常归一化
    │            └─ owner 不可见 + 存在可见相机 → 可见集合内重归一化 (D3)
    ▼
贴图碗模型 → OSG 渲染（渲染层不改）
```

关键不变量：`TransformWorldToCamera` 投影数学与原点无关，全程未改；改动集中在"以哪个点为车中心"（几何 + 极点）与"以哪个方向划扇区"（光轴）。

---

## 5. 错误处理

| 场景 | 策略 |
|------|------|
| owner 相机不可见，存在其他可见相机 | D3 兜底：raw_weights 在可见集合内重归一化，消除硬黑 |
| 无任何可见相机（中心车底物理盲区） | 允许零权重输出黑色，属预期，统计目标 <50/12741 |
| 世界点与光轴重合（r_xy < eps） | D4 退化分支：输出主点 (cx, cy) |
| z<0 / 离轴超有效 FOV 的投影输入 | D4 手写投影落正确半平面，由圆检查统一拒绝，无镜像误判 |
| cameras.json 缺 `car` 字段 | avm_viewer 沿用 BowlParams 默认值（car_center=(0,0)，即按底部中心系解读）；`car` 字段为首选配置来源 |
| 原点约定未知 | 不存在隐式猜测：`car_center` 与 `origin_convention` 均为数据显式声明，缺省即声明"数据原点=车中心" |

---

## 6. 测试策略

每项设计决策对应 RED-GREEN 测试，端到端有量化验收：

- **D1**: car_center 网格对称性测试——设非零 car_center 后网格关于车中心对称。
- **D2**: 扇区归属测试——断言左右后视镜相机光轴地面方位 ≈ ±90°（非位置方位 ±24°/-21°），边界射线为角平分线。
- **D3**: 兜底归一化测试——构造 owner 不可见但邻相机可见的场景，断言权重非零且归一化。
- **D4**: 投影等值性测试——z>0 输入与 `cv::fisheye::projectPoints` 逐值一致（容差 1e-6 px）；z<0 输入断言输出在正确半平面（镜像消除）。
- **端到端验收**: avm_viewer 渲染后跑黑楔像素统计（复用数值探针脚本），零权重顶点从 344/12741 降至 <50/12741，残余均为中心车底物理盲区。
- **回归**: 现有 113 个测试全过；API 变更处（D1/D2）同步更新受影响测试。

---

## 7. 影响面与兼容性

**公共 API 变更（3 处）:**

1. `BowlParams` 增加 `car_center` 字段（默认值保持旧语义）。
2. `ComputeBlendWeights` 增加极点参数（默认 `(0,0)` 保持向后兼容语义）。
3. `AzimuthDeg` 语义变更：相机位置方位 → 光轴地面投影方位（同名不同义，为破坏性语义变更，调用方需确认预期）。

**受影响调用方:** StitchEngine、avm_viewer、现有测试。

**明确不改:** StitchScene、OSGViewRenderer、shader、avm-worker 仓库。

---

## 8. 约束与假设

**硬约束:**

- D4 在 z>0 区间的输出必须与 `cv::fisheye::projectPoints` 在 1e-6 px 容差内一致——现有标定数据下渲染结果不得因替换投影实现而变化。
- 52/N 混合带公式、边缘 feather 逻辑保持原样。
- 投影数学（`TransformWorldToCamera`）不改。
- 本次不触碰 avm-worker 仓库。

**假设:**

- `cameras.json` 采用后轴系约定，通过新增 `car` 块显式声明；axis_x 具体值以数据文件实际内容为准（R1 证据为错位 axis_x/2≈1.55m）。
- 相机光轴方向可从 R_wc 第三列正确取得（外参约定为 R_wc：世界 → 相机旋转）。
- 四相机为前/后/左/右环绕布局，角平分线扇区模型适用。

**记录的 follow-up（不属于本次实施）:**

- 数据侧：用真实标定确认 k1–k4 是否为占位假值，决定是否需要 fisheye_radius 自动检测。
- 渲染精化：车角锚定融合线（参考 avm_renderer 4 相机 rig 调优）。
- worker 侧：`ParseBowlParams` 无 bowl 覆盖时从 `calibration.car` 派生。
