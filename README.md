# avm-stitch

C++20 AVM(Around View Monitor,环视/360 鸟瞰)拼接静态库。

- **纯算法 API**:结构体 / `cv::Mat` 进出(`Configure` + `RenderFrame`),无 JSON、无协议依赖
- **Headless EGL 渲染**:EGL surfaceless 离屏上下文,无需 X server / 显示器,可直接跑在 Docker/CI
- **可嵌入**:纯静态库,无 main,命名空间 `avm_stitch`,CMake + vcpkg 管理
- **N 路相机**:架构上限 8 路,`Configure` 接受 N∈[1,8];典型配置为 4 路(前/后/左/右)

---

## 1. 项目简介

avm-stitch 把 N 路鱼眼摄像头画面实时拼接成车辆周围的"鸟瞰"视图。本库是**自研 OSG 拼接引擎**(碗形模型 + 方位角融合,GLSL 330 着色器)的**第三方中立算法库**:只暴露纯算法 API(结构体 / `cv::Mat` 进出),不感知任何任务协议。曾内置的 JSON 集成层(任务 JSON 契约、后端工厂、结果信封)与闭源 `avm-renderer` 代理路径均已整体迁移至其唯一消费者 avm-worker(由 worker 侧的 `avm_stitch_integration` / `avm_renderer_bridge` 库承载并统一分发)。C++ 算法 API 见[第 8 节](#8-集成方式)。

核心技术决策:

| 决策 | 位置 |
|------|------|
| 几何/融合全部 CPU 预计算(烘焙到顶点),GPU 只做查表混合 | `src/geometry/` |
| 融合算法与相机数量 N 无关,4 路和 8 路同一套代码 | `src/geometry/blend_weights.cpp` |
| GPU 侧固定 8 槽顶点属性布局,少于 8 路时空槽权重为 0 | `src/render/stitch_scene.cpp`(`StitchScene`) |
| 库内零 JSON 依赖:`StitchEngine` 以结构体与 `cv::Mat` 进出,错误以异常表达 | `src/core/stitch_engine.cpp` |

## 2. 什么是 AVM 拼接

AVM(环视,亦称 Bird's Eye View / 360 全景)系统在车辆前后左右安装 **N 路鱼眼摄像头**(典型 4 路,本库架构上限 8 路)。每路相机视场角(FOV)通常超过 180°,相邻视野在地面重叠:

```
                    [Front]
                      F
            +---------+---------+          F/R/L = 鱼眼相机
            | overlap | overlap |          (FOV > 180 deg)
            |         |         |
       L    |    +---------+    |    R
       e ---+    |         |    +--- i
       f    |    |   CAR   |    |    g
       t    |    +---------+    |    h
            |         |         |
            | overlap | overlap |
            +---------+---------+
                      R
                   [Rear]

   相邻视野在地面上重叠 -> 接缝处必须做加权融合,
   否则同一块地面会被两台相机渲染出两层错位的图像。
```

**为什么需要碗形模型?** 拼接的目标是把地面"摊平"成一张俯视图。真实相机成像满足透视规律:同样大小的地面块,离相机越远,在图像中占据的像素越少。如果把 4 路图像直接按方位角贴到一张平面上,接缝两侧的"拉伸比例"不一致,会拼出扭曲的地面。

碗形(bowl)模型是行业通行解法:把车辆周围的世界建模成一只**中间平坦、四周缓缓升起的碗**——车底是平地(z=0),从车身外围开始地面沿 `z = k_slop · rd^z_exp` 抬升。渲染时虚拟相机看向这只碗,碗沿自然地"压缩"了远处地面,使 4 路图像在碗面上的投影恰好能平滑衔接。详见[3.1](#31-碗形模型-srcgeometrybowl_modelhpp)。

## 3. 核心原理

五段算法讲解,每段给出直觉解释 + 对应代码位置。

### 3.1 碗形模型(`src/geometry/bowl_model.hpp` / `.cpp`)

hpp/cpp 拆分,纯数学,无 GL 依赖。`BowlModel model(params, intrinsics, extrinsics)`(构造函数,取代最初的工厂函数)一步生成碗面三角网格并烘焙,数据经细粒度索引访问读取(`vertex_count()` / `vertex(i)` / `normal_count()` / `normal(i)` / `triangle_count()` / `triangle(i)`,另有网格规格查询 `rows()` / `cols()`)——不暴露内部容器,内部布局(将来可换 SoA 等)是纯实现细节:

```
 z ^      rim:  z = k_slop * rd^z_exp   (max_z 封顶)
   |    ___                          ___
   |   /    \___                ___/    \
   |  /         \___      ___/           \
   | /             \____/                 \
   |/                                     \
 0 +----|---------平地 z=0------|-----------\----> rd
        | 车底平坦区             |          r_outter
     rounded-rect               |         = 12.6 m
     (车长/2+round_l,           curved slope
      车宽/2+round_l)
```

`BowlParams` 8 个参数(默认值来自 `BowlParams::Default()`):

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `car_length` / `car_width` | 5.6 / 2.2 | 车长/车宽(m),决定平坦区大小 |
| `round_l` | 2.0 | 圆角外扩量:平坦区是以车为基准的圆角矩形 |
| `step` | 0.1 | 网格步长(m),网格数 = `r_outter/step × 2` |
| `r_outter` | 12.6 | 碗外半径(m),即拼接可见范围 |
| `k_slop` / `z_exp` | 0.09 / 2.2 | 坡度系数与幂次,控制碗沿抬升快慢 |
| `max_z` | 8.6 | 碗沿最大高度,超出部分的顶点被裁剪 |

`rd` 是顶点到圆角矩形(平坦区)边缘的最近距离:平坦区内 `rd=0, z=0`;平坦区外按幂函数抬升。法线用有限差分估计(`ComputeBowlNormals`)。`BowlParams` 以纯结构体进出(默认值来自 `BowlParams::Default()`);从任务 JSON 解析碗参数的集成层逻辑已迁至 avm-worker。

### 3.2 鱼眼投影(`src/geometry/projection.hpp`)

header-only,把 3D 世界点投影到鱼眼像素。核心委托 `cv::fisheye::projectPoints`(OpenCV 等距/Kannala-Brandt 模型):

- **`CameraIntrinsics`**:`{width, height, fx, fy, cx, cy, k1..k4, fisheye_radius}`,其中 `fisheye_radius=0` 表示自动取图像内切圆半径(`FisheyeCircleRadius`)
- **`CameraExtrinsics`**:T_wc 约定(camera → world),`R` 为 3×3 行主序,`t` 为平移(m);OpenCV 惯例相机 y 轴朝下
- **`ProjectWorldToFishEye(P_world, intr, extr, u, v)`**:世界点 → 像素;点在相机背后时返回 `(-1,-1)`
- **`IsForwardFacing`**:世界点是否在该相机前方(Z_cam > 0)
- **`IsInsideFisheyeCircle`**:投影像素是否落在鱼眼有效圆内——像素可能在图像**矩形**内、却在镜头圆盘**外**(四角是黑区),采样黑区会把接缝染黑
- **`FisheyeEdgeFeather`**(默认羽化 16px):靠近有效圆边缘的像素权重线性衰减到 0,防止线性纹理过滤把黑区混进接缝

### 3.3 BowlModel 烘焙(构造函数,`src/geometry/bowl_model.cpp`)

碗网格生成与烘焙由同一个类 `BowlModel` 承载,并**并入构造函数**:`BowlModel model(params, intrinsics, extrinsics)` 一步生成网格 + 逐顶点把"看什么、怎么混"预计算进顶点属性——这是整个设计的关键:**构造即就绪的构造不变量**(不存在"未烘焙"半状态,重烘 bug 的土壤根除)+ **运行时 GPU 零几何计算**。烘焙结果经细粒度索引访问(`baked_vertex(i)`)与语义查询(`baked_num_cameras()` / `HasVisibleVertices()`:是否存在任一槽权重 > 1e-9 的顶点)读取,不暴露内部容器。

每个碗面顶点得到:

- `uv[8][2]`:该顶点在每路相机图像上的归一化 UV(像素坐标 ÷ 图像宽高),不可见的相机槽位保持 `(0,0)`
- `weight[8]`:该顶点在每路相机上的融合权重,**可见相机集合内归一化,和为 1**;不可见相机权重为 0

`BakedVertex`(`src/geometry/bowl_model.hpp`)的 8 槽是为 8 路架构上限预留的:配置 N < 8 路时,空槽的 UV 和权重全为 0,着色器按 `uNumCameras` 只激活前 N 路。

顶点级处理流程:对每路相机计算方位角融合原始权重(`ComputeBlendWeights`)→ 判定可见性(前向 + 图像矩形内 + 鱼眼有效圆内)→ 可见者乘上边缘羽化系数 → 在"可见且有权重"的相机集合内归一化。注意 v 坐标**不需要翻转**——T_wc/OpenCV 惯例下 `projectPoints` 输出的 v 已经朝图像下方增长(见 `BowlModel` 构造函数内烘焙逻辑的注释)。

### 3.4 融合权重(`src/geometry/blend_weights.cpp`)

**方位角扇区算法,与相机数量 N 无关**——4 路和 8 路用同一套代码:

1. 每台相机按安装位置的方位角 `atan2(t_y, t_x)` 排序(`SortCamerasByAzimuth`)
2. `ComputeSectorBoundaries`:相邻相机方位角的中点即扇区边界,共 N 条
3. `ComputeBlendHalfAngle(N)` = `52.0 / N` 度(公式 `13° × (360/N) / 90`),即过渡带半角——N 越大相机越密,过渡带越窄
4. `ComputeBlendWeights(P_world, extrinsics)`:
   - 地面点的方位角落在某边界 ±52°/N 的过渡带内 → 该边界两侧的两台相机按**余弦 smoothstep** 混合,边界处各 0.5,离开边界向近侧相机单调升至 1.0
   - 过渡带之外 → 最近的相机独占,权重 1.0
   - 返回权重和恒为 1.0

```
 weight
 1.0 |________                          ________
             \                        /        A owns (nearest)
 0.5 |         \                     /
              |                     |     <- boundary: A = B = 0.5
 0.0 |          \__________________/          B owns
     +----------+--------+---------+---------> azimuth
               b-h      b       b+h
              blend band:  h = 52/N deg  (N=4 -> 13 deg)
        near weight = 0.5 * (1 - cos(t*pi))   cosine smoothstep
```

扇区之外,`BowlModel` 构造时的烘焙还会叠加鱼眼边缘羽化(3.2)后再归一化,保证黑区不进接缝。

### 3.5 GPU 渲染(`src/render/`)

OSG(OpenSceneGraph)场景图 + GLSL 330 着色器,全离屏:

- **`stitch_scene.cpp`**:`StitchScene` 类拥有整个场景图(构造即就绪,无需 GL),把碗网格 + 烘焙数据装配成 `osg::Geometry`,顶点属性布局(为 8 路固定打包):

```
 location 0   vec3  position   (VCS -> OpenGL: x,y,z -> x,z,-y)
 location 1   vec3  normal     (同上)
 location 2   vec4  uvCoord0 = (u0, v0, u1, v1)  --+
 location 3   vec4  uvCoord1 = (u2, v2, u3, v3)    |
 location 4   vec4  uvCoord2 = (u4, v4, u5, v5)    +-- 8 cameras,
 location 5   vec4  uvCoord3 = (u6, v6, u7, v7)  --+   4 x vec4 packed
 location 6   vec4  weights0  = (w0, w1, w2, w3)  --+
 location 7   vec4  weights1  = (w4, w5, w6, w7)  --+  2 x vec4 packed
```

- **着色器**(GLSL 330 源码是 `StitchScene` 内部实现细节,装配进 stateset 的 program):片元着色器把 8 路相机**手动展开**(避免动态采样器数组索引),每路 `if (fragNumCameras > k && w_k > 0.001) result += texture(uCameraTextures, vec3(uv_k, k)).rgb * w_k;`——最终颜色就是"查表加权求和"
- **`StitchScene::UploadCameraImages`**:N 路相机帧作为 `Texture2DArray` 的 N 个图层上传(BGR→BGRA,自持拷贝;禁用非 2 次幂重缩放)
- **`osg_view_renderer.cpp`**:`OSGViewRenderer` 为 5 个视图位姿(`DefaultViewPoses`,定义于 `src/render/osg_view_renderer.hpp`)各建一个 FBO 相机,`RenderAllViews()` 一次渲染全部视图并回读为 BGR `cv::Mat`(默认 960×640,清屏色 RGB(30,36,50),近/远裁剪面 0.1/100,回读后垂直翻转补偿 GL 左下角原点)。`ViewPose` 结构体与 `DefaultViewPoses()`(topdown + 前后左右 4 个 3D 视图,fovy 60°)也定义在该头文件
- **`egl_headless_context.cpp`**:`EGLHeadlessContext` 实现 EGL surfaceless(Mesa)桌面 GL 3.3 兼容 Profile 的 `osgViewer::GraphicsWindow`,16×16 pbuffer 只负责持有上下文,真正的绘制全部进 FBO——无窗口、无 X server

## 4. 架构与数据流

库的核心是 `StitchEngine` 两阶段 API(`src/core/stitch_engine.cpp`):`Configure`(一次性建碗→烘焙→建场景)+ `RenderFrame`(每帧渲染)。文字版数据流:

```
 Configure(intrinsics, extrinsics, BowlParams)      一次性
     |
     | bowl model + baked uv[8] / weight[8]  (CPU 预计算, src/geometry/)
     v
 osg::Geometry + GLSL 330                  (src/render/stitch_scene.cpp)
     ^
     | upload                              每帧
 RenderFrame(N x BGR cv::Mat) ------------> Texture2DArray (N layers)
     |
     v
 5 FBO cameras (one per ViewPose)          (src/render/osg_view_renderer.cpp)
     |
     v
 {"topdown": Mat, "3d_front": Mat, ...}    map<视图名, cv::Mat>
```

JSON 任务契约(`queue:stitch` 任务解析、`algorithm` 分发、base64 图解码、`stitch_result` / `stitch_error` 信封、后端工厂 `CreateStitchBackend`)已整体迁至 avm-worker(集成层库 `avm_stitch_integration`),worker 消费本库的纯算法 API 完成协议适配。

## 5. 目录结构

```
src/
├── geometry/                      纯数学,CPU 预计算
│   ├── projection.hpp             鱼眼投影: 世界点 -> 像素, 有效圆/羽化
│   ├── bowl_model.hpp / .cpp      碗形参数 + 碗网格生成 + 构造即烘焙(UV[8] + weight[8])
│   └── blend_weights.hpp / .cpp   方位角扇区融合权重 (与 N 无关)
├── render/                        GPU / OSG
│   ├── stitch_scene.hpp / .cpp       StitchScene: 场景图拥有者, GLSL 330 着色器, 纹理数组
│   ├── osg_view_renderer.hpp / .cpp        离屏 5 视图渲染 (FBO 相机 + 回读); ViewPose / DefaultViewPoses 5 个默认视图位姿 (header-only)
│   └── egl_headless_context.hpp / .cpp   EGL surfaceless 桌面 GL 3.3 (OSG 路径)
└── core/
    └── stitch_engine.hpp / .cpp   两阶段引擎: Configure + RenderFrame (PIMPL)

tests/                             9 个 gtest 目标
├── test_support/  stitch_test_helpers.hpp / gl_fixture.hpp (共享测试基建, header-only)
├── geometry/   test_projection / test_bowl_model (含烘焙) / test_blend_weights
├── render/     test_stitch_scene / test_osg_view_renderer / test_egl_headless_context / test_view_poses
└── core/       test_stitch_engine / test_stitch_engine_regression
```

JSON 集成层(`stitch_backend` / `osg_backend` / `stitch_result` / `camera_name` / `base64` / `eval_metrics`)已随 JSON 剥离迁移至 avm-worker。

## 6. ⭐ 代码阅读路径

推荐由简到繁,每步都能独立理解、独立测试:

| # | 读什么 | 能理解什么 | 配套测试(用法示例) |
|---|--------|-----------|---------------------|
| 1 | `src/geometry/projection.hpp` | 鱼眼投影基础:内参 k1-k4、T_wc 约定、有效圆与羽化。全部 inline,单文件可读 | `tests/geometry/test_projection.cpp` |
| 2 | `src/geometry/bowl_model.hpp` | 碗为什么长这样:8 参数含义、网格/法线生成 | `tests/geometry/test_bowl_model.cpp` |
| 3 | `src/geometry/blend_weights.cpp` → `src/geometry/bowl_model.cpp`(构造烘焙) | 接缝怎么融合:扇区边界、52°/N 过渡带、余弦 smoothstep;再看权重如何叠加可见性/羽化烘焙进顶点 | `tests/geometry/test_blend_weights.cpp`、`tests/geometry/test_bowl_model.cpp`(BowlModelBake 套件) |
| 4 | `src/render/stitch_scene.cpp`(`StitchScene`) | 烘焙数据怎么上 GPU:属性布局(location 0-7)、着色器源码、纹理数组。纯结构装配,不需要 GL;构造即就绪 | `tests/render/test_stitch_scene.cpp` |
| 5 | `src/render/osg_view_renderer.cpp` + `src/render/egl_headless_context.cpp` | 怎么无窗口渲染:每个视图一个 FBO 相机、MVP 手动传 uniform 的原因(vcpkg OSG GL2 构建怪癖)、回读翻转 | `tests/render/test_osg_view_renderer.cpp`、`tests/render/test_egl_headless_context.cpp` |
| 6 | `src/core/stitch_engine.cpp` | 一切怎么串起来:`Configure`(建碗→烘焙→建 `StitchScene` 场景,不做 GL 绘制,但会尝试创建离屏 EGL 上下文,创建失败不抛异常)与 `RenderFrame`(传纹理→渲 5 视图)两阶段 | `tests/core/test_stitch_engine.cpp`(校验 + GL 端到端) |

辅助阅读:`src/render/osg_view_renderer.hpp`(5 视图位姿的具体系数,`ViewPose` / `DefaultViewPoses`)。JSON 门面(后端工厂、任务契约、错误信封)与视图名→AVMDisplayMode 映射(view_mapping)、相机 id/label 解析(camera_name)均已在 avm-worker。

## 7. 构建与测试

依赖仅 `vcpkg.json` 所列公共 registry 包(gtest、opencv4、osg)+ 系统 EGL(`find_package(OpenGL COMPONENTS EGL)`)。无任何 JSON 依赖、私有 registry / avm-renderer / CLI11 / glfw3 依赖,干净环境可直接自举。

```bash
export VCPKG_ROOT=/path/to/vcpkg      # CMakePresets.json 的 default preset 依赖它
cmake --preset default                # 配置 (Unix Makefiles, build/)
cmake --build build                   # 增量编译, 不要删 build 目录
ctest --test-dir build                # 10 个目标
```

说明:`tests/core/test_stitch_engine.cpp`、`tests/render/test_osg_view_renderer.cpp` 与 `tests/render/test_egl_headless_context.cpp` 的 GL 用例自动探测 headless EGL 上下文,无 GL 环境(纯 CI 容器)会 **SKIP** 而非失败;`AVM_STITCH_TEST_GL=1` 可强制开启。

## 8. 集成方式

本库是纯算法库,以 C++ 结构体 / `cv::Mat` 集成。最小用法:

```cpp
using namespace avm_stitch;

StitchEngine engine;
engine.Configure(intrinsics, extrinsics, BowlParams::Default());  // 一次性
auto views = engine.RenderFrame(bgr_images);   // 每帧: map<视图名, cv::Mat>
```

- `intrinsics` / `extrinsics`:`std::vector<CameraIntrinsics>` / `std::vector<CameraExtrinsics>`(`src/geometry/projection.hpp`,T_wc 约定)
- `BowlParams`:碗形参数结构体(`src/geometry/bowl_model.hpp`,默认值 `BowlParams::Default()`)
- `RenderFrame` 输入 N 路 BGR `cv::Mat`,输出 5 个视图(`topdown` / `3d_front` / `3d_rear` / `3d_left` / `3d_right`)
- 错误以异常表达(如 `Configure` 对 N∈[1,8] 之外的输入抛 `std::invalid_argument`)

**任务 JSON 契约(`queue:stitch` 任务、base64 图解码、`stitch_result` / `stitch_error` 信封、`CreateStitchBackend` 工厂、`ParseBowlParams`、流式会话接口)已迁至 avm-worker**(其 `avm_stitch_integration` / `avm_renderer_bridge` 库),由 worker 统一完成 redis 协议适配与算法分发;本库不再涉及任何 JSON 形态。

## 9. 扩展性:如何支持更多相机

当前 8 路上限的瓶颈(全部在 GPU 侧布局,几何/融合算法本身与 N 无关):

| 瓶颈 | 位置 | 现状 |
|------|------|------|
| 顶点属性槽位 | `src/geometry/bowl_model.hpp` `BakedVertex` | 固定 `uv[8][2]` + `weight[8]` |
| GLSL 属性布局 | `src/render/stitch_scene.cpp:18-22` | UV 打包 4×vec4(location 2-5),权重打包 2×vec4(location 6-7),共 8 location,恰好覆盖 8 路 |
| 片元着色器展开 | `src/render/stitch_scene.cpp` 匿名 ns `FragmentShaderSource()` | 手动展开 8 路 `if (fragNumCameras > k ...)` 分支 |
| Configure 校验 | `src/core/stitch_engine.cpp` `ValidateConfigureArgs` | `N ∈ [1,8]`,超出抛 `std::invalid_argument` |

扩到 16 路需要动:

1. `BakedVertex` 槽位 8 → 16(`uv[16][2]`、`weight[16]`)
2. `StitchScene` 内的几何构建重新打包:UV 需 8×vec4(location 2-9),权重需 4×vec4(location 10-13)——注意 GLSL 330 保证的顶点属性数至少 16,layout 0-13 仍在安全范围,但需在目标 GPU 上验证
3. 片元着色器展开 16 路分支;`WEIGHT_MIN` 阈值机制可沿用
4. `Configure` 校验上限放到 16
5. **不需要动**:`ComputeBlendWeights`(N 无关,过渡半角自动变为 52/16 = 3.25°)、`BowlModel` 构造、`StitchScene::UploadCameraImages`(纹理数组层数即 N)

## 10. 已知限制

- **JSON 集成层已迁至 avm-worker**:任务 JSON 契约、后端工厂、结果信封、相机名解析等集成层代码由 avm-worker 的 `avm_stitch_integration` 库承载;本库仅保留纯算法 API,不为"可选 JSON 门面"预留编译开关
- **GL 测试依赖环境**:`tests/core/test_stitch_engine.cpp`、`tests/render/test_osg_view_renderer.cpp` 与 `tests/render/test_egl_headless_context.cpp` 的 GL 用例靠 EGL surfaceless 探测自动门控——无 GL 环境时 SKIP(`AVM_STITCH_TEST_GL=1` 强制)
- **vcpkg OSG 构建怪癖**:`osg_ModelViewProjectionMatrix` 不会被自动更新,GL uniform 必须用 `osg::Matrixf`(Matrixd 会触发 GL_INVALID_OPERATION),见 `src/render/osg_view_renderer.cpp:73-78` 注释——升级 OSG 构建时需复测
