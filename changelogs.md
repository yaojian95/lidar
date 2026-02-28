## 2026-02-28

### 代码重构与复用 (Code Refactoring & Reuse)
- **合并平面对齐逻辑**: 发现 `alignToGround` (基于 RANSAC 动态提取) 与 `alignToGroundWithPlane` (基于缓存参数) 的点云旋转矩阵推导及 Z 轴翻转代码高度重合且存在重复计算。现已将二者的变换算法完全并轨——`alignToGround` 在执行完 RANSAC 提取出内点方程后，将直接全权委托调用统一函数执行单向数据流对齐，消除了庞大冗余的重复逻辑和潜在的中间态资源消耗。

### 性能优化 (Performance Optimization)
- **移除冗余的点云旋转**: 在 `OreAnalyzer::alignToGroundWithPlane` 方法中，移除了一次针对临时变量 `rotated_cloud` 的无用运算 (`pcl::transformPointCloud`)。原因为平移量其实可以通过被缓存的平面方程直接代数求出，而第二次变换操作已在一个 `transform` 中同时整合了旋转和平移作用于原点云，故第一次旋转纯属冗余且会徒增内存和算力开销。

## 2026-02-27

### UI 智能感知绘图 (Smart UI Labeling Rendering)
- **直接渲染至厚度图**: 增强了 `saveThicknessMapToImage` 函数的表现力。如果用户在 `config.yaml` 中关闭了任何融合图层渲染 (`fuse: false`)，则 `app_pipeline` 截留在主流程中的矿石清单会被直接下放传递给厚度图保存器中；保存器会自动将原本单通道灰度的像素深度图升级为三通道彩色，并将所有矿石自身的物理尺寸坐标系全部逆运算并锁定映射至倒转后的厚度图空间点位，为其直接绘上清晰的绿色标识文字 (Labeling IDs)。如此以来，无论用户使用花里胡哨的色彩融合，或是极简的黑白厚度输出，都有标签可看。

### 混合架构聚类分割 (Hybrid Clustering Segmentation)
- **新功能**: 为了解决原本纯粹的欧氏聚类会导致物理贴合的非单一矿石被错误合并的问题，实现了基于多指标联合诊断的“混合架构聚类”。
- **三种诊断策略**: 在 `config.yaml` 中新增 `cluster_strategy` 控制诊断逻辑。
  - **策略 `1` (宽高比检测)**: 计算粗聚类簇在 XY 平面的长细比 (`aspect_ratio_threshold`)，长条形连续粘合物将被拦截。
  - **策略 `2` (下凹密实度检测)**: 评估点云密度占其包围盒面积的比例 (`density_threshold`)，葫芦状或多球贴合等高度不规则形状将被拦截。
  - **策略 `3` (多焦点 Z 图检测)**: 生成局部二维高度图扫描山峰，拥有超过 1 个显著局部极大值的粘合簇将被拦截。
- **动态局部分割 (Region Growing)**: 任何触发了上述警报策略的簇，系统将立刻计算点云的表面三维法线特征，并调用 `pcl::RegionGrowing` 沿表面法线折角 (`rg_smoothness`) 缝隙进行二次精密切断。

## 2026-02-26

### X-ray 融合可视化体验提升 (X-ray Fusion Visualization Enhancement)
- **Jet 色阶热力图映射**: 全面升级了 `fuseThicknessWithXray` 融合方法。去掉了原来直接极其暴力地加在红色通道上的方式；现在的厚度地图会在裁剪后自动映射出类似热力地形图的绚丽 Jet ColorMap 色阶（从薄如蝉翼的深蓝，平滑过渡到厚重凸起的深红），呈现出极其清晰的厚度层次感。
- **抗噪统计对比度拉伸**: 弃用了容易被雷达反光尖刺拉爆色域的绝对极小极大值算子；转而通过统计学求取全体视场的均值 (Mean) 和标准差 (StdDev)，对色盘阈值上下限使用了极其激进的切割（`Mean + 0.8 * StdDev` 等），将真正的矿石主体的微小落差在彩色视觉上强制放大。
- **标签位置优化**: 矿石 ID 原先画在正中心容易挡住最高峰的色值，现已全面固定向右侧平移 `40` 个绝对像素，让中心区域暴露无遗。

### Pipeline 架构提取解耦 (Pipeline Extraction)
- **提取重构 Pipeline 方法**: 新建了独立的核心流程控制大模块 `app_pipeline.h` 和 `src/app_pipeline.cpp`。将原先堆积堵塞在 `main.cpp` 里的极其冗长复杂的各种操作流封装重构到了 `AppPipeline` 命名空间下。
- **极简化主存根**: 原本近 400 行庞大的 `main.cpp` 在经历了剔骨之后，现在变成了一个清爽简明的如同顶级目录的 50 行编排脚本，易读性呈现指数级上升。
- **构建系统跟进**: 在 `CMakeLists.txt` 的 `add_executable` 目标中准入并链接了新诞生的 `src/app_pipeline.cpp` 源文件。

### 配置中心化 (Configuration Centralization)
- **新建强类型 `AppConfig` 结构体**: 在 `utils.h` 中新设了该结构体以集中规范化原本那些散落在各处的、使用魔法字符串读取的无强类型的弱关联变量。
- **分离重构读取逻辑**: 将 `main.cpp` 里的那一大串 `Config::get<T>` 提取解组，收拢打包进了 `utils.cpp` 中的 `Config::parseAppConfig` 里面去，实现读用分离。
- **四步走流水线**: `main()` 大改后，现明确严格遵循四段论：
  - `loadPointCloud(...)`，负责 IO
  - `processCalibrationAndGround(...)`，负责对齐与筛除去噪
  - `executeDetectionAndFusion(...)`，负责特征发现截取和视觉融合
  - `visualizeResults(...)`，负责拉起最终 3D GUI 引擎。

## 2026-02-24
### X-ray 几何校正 (X-ray Geometry Correction)
- **新功能**: 在 `OreAnalyzer` 中实现了由于点光源 X射线探测器引起图像畸变的几何校正能力。
- **集成优化**:
  - 原本作为独立的预处理步骤，现在被直接内联集成到了 `fuseThicknessWithXray` 流程内。
  - **处理顺序调整**: 现在算法会首先从原始输入的 X 射线灰度图中根据配置裁剪低能部分 (`cut_left/right`) 和相应的提取 ROI (`crop_*`)，然后再仅仅针对有效图像区域进行畸变校正 (`1/M` X轴缩放)。这样保证了裁剪操作的精度基于原始像素坐标。
  - **动态开关**: 在 `config.yaml` 中新增了 `enable_xray_geometry_correction` (true/false) 参数。用户可以随时选择在融合流程中开启或剥离该步骤。
  - **代码模块化与去冗余**: 废弃了基于文件读写路径直接套用的 `correctXrayGeometry`，将其重构成直接处理系统内存 OpenCV `cv::Mat` 矩阵的核心逻辑，并彻底移至了通用工具模块 `Utils::correctXrayGeometry`。这样可以保证其他外部模块直接重用核心像素处理代码，避免了反复的文件落盘和读取瓶颈。
- **实现细节**:
  - 利用已知参数：sod (光源到物体距离) 和 sdd (光源到探测器距离)。
  - 根据公式 `M = sdd / sod` 计算放大倍数。
  - 由于皮带的运动特性，畸变只发生在跨皮带方向（宽度/X轴），因此程序仅在图像 X 轴维度上缩放，沿皮带方向保持原始比例。

## 2026-02-06 (Update 2)

### X-ray 融合功能 (X-ray Fusion)
- **新功能**: 实现了 `fuseThicknessWithXray` 函数，用于将 LiDAR 厚度图与 X 光透射图像（低能部分）进行融合。
- **流程**:
  - 读取单通道 X 光灰度图像。
  - **Flip**: 水平翻转图像（因为原始图像是左右反转的）。
  - 将图像分为左（低能）右（高能）两半。
  - 根据 `xray_cut_left` 和 `xray_cut_right` 对低能图像进行初步裁剪（去除边缘）。
  - **New**: 应用 `xray_crop_*` 参数对裁剪后的图像进行 ROI 选择，去除无关背景。
  - 将厚度图调整为最终裁剪后的 X 光图像大小，并将其叠加到红色通道中。
- **配置更新**:
  - 新增 `xray_path` 指定输入的 X 光图像路径。
  - 新增 `xray_cut_left` 和 `xray_cut_right` 定义低能图像的有效区域。
  - **New**: 新增 `xray_crop_up/down/left/right`，实现对 X 光低能图像的精细 ROI 控制。
  - **Refactor**: 将 `fuse_rgb` 和 `fuse_xray` 合并为单个 `fuse` 参数，可选值为 `"rgb"`, `"xray"`, 或 `"false"`。

## 2026-02-06

### 工具更新 (Tools Update)
- **新增 PLY 裁剪工具**: 创建 `py_src/crop_ply_x.py`。
  - 功能：读取 PLY 文件，去除 X 轴方向前 1/3 和后 1/3 的数据，仅保留中间 1/3。
  - 用法：`python py_src/crop_ply_x.py <input_file>`
- **输出格式调整**: `stack_tif_files` 输出文件格式由 `.tif` 更改为 `.png`。
- **配置更新**: 在 `config.yaml` 中新增 `fuse` 参数，用于控制是否执行厚度图与 RGB 图像的融合。
- **新增 TIF 堆叠功能**: 在 `py_src/simple_stack.py` 中添加了 `stack_tif_files` 函数。
  - 功能：支持读取指定文件夹下的所有 `.tif` 文件，并将其按垂直方向拼接。
  - 输出：拼接后的图像以文件夹命名，保存在该文件夹的上一级目录中。

## 2026-01-30

### 融合可视化增强 (Fusion Visualization Enhancement)
- **双源 ROI 对齐 (Dual-Source ROI Alignment)**:
  - 实现了基于 ROI (Region of Interest) 的对齐机制，允许分别为 RGB 图像和 LiDAR 厚度图定义裁剪区域 (`rgb_crop_*` 和 `lidar_crop_*`)。
  - 通过定义两个来源的物理视场 (Field of View)，确保了不同分辨率和视角的传感器数据能够精确对齐。
  - 最终以此裁剪后的 RGB 图像为底图，进行融合和标注。

- **标签优化 (Labeling Optimization)**:
  - 增加了标签字体的大小和粗细，以适应高分辨率 (4096x7200) 的输出图像。
  - 标签坐标现在根据 ROI 裁剪和缩放自动转换，确保准确标注在矿石中心。
  - The thickness values are normalized and injected into the Red channel (default) of the RGB image.

- **配置重构 (Configuration Refactoring)**:
  - 废弃了简单的 `fusion_x/y_offset`。
  - 引入了 8 个独立的裁剪参数：
    - `rgb_crop_up/down/left/right`: 定义 RGB 图像的有效区域。
    - `lidar_crop_up/down/left/right`: 定义 LiDAR 厚度图的有效区域。

### 构建修复 (Build Fixes)
- **Release 构建**: 配置了 `windows-release` 预设，解决了 OpenCV 链接时的 `opencv_core_parallel_openmp` 缺失警告，确保了更快的运行速度。

## 2026-01-28

### 全局厚度图生成 (Global Thickness Map)
- **功能新增**: 实现了 `generateGlobalThicknessMap` 函数，能够将所有检测到的矿石投影到一个二维矩阵中。
- **坐标缩放修复**: 修复了分辨率计算问题。现在使用 `resolution / unit_scale` 计算像素分辨率，确保生成的图片像素代表真实的物理尺寸（例如 1像素=1厘米）。
- **边界控制**:
  - **X轴**: 根据检测到的矿石动态计算，避免因远处噪声导致图片无限宽。
  - **Y轴**: 优先使用配置中的皮带边界 (`belt_min_y`, `belt_max_y`)，确保图片高度与皮带宽度严格对应。
- **内存安全**: 增加了最大尺寸检查 (20000x20000)，防止内存溢出。

### 图像保存 (Image Saving)
- **OpenCV 集成**: 引入 OpenCV 库替代了手写的 PGM 保存代码。
  - 修改 `CMakeLists.txt` 链接 OpenCV。
  - 重写 `saveThicknessMapToImage` 使用 `cv::Mat` 和 `cv::imwrite`。
  - 支持保存为 `.png` 格式（无损压缩）。
- **图像转置**: 在保存前增加了 `cv::transpose`，使得输出图片的行列与 Python 后处理脚本 (`stack_png.py`) 的预期方向一致。

### 代码重构 (Refactoring)
- **地面过滤**: 将 `filterGroundPoints` 逻辑封装到 `OreAnalyzer` 类中。

## 2026-01-27

### 配置更新 (Configuration Updates)
- **更新 `unit_scale`**: 在 `config.yaml` 中将校准值设置为 `0.003375`，以启用正确的物理距离测量（米）。
- **新增欧几里得聚类参数**: `cluster_tolerance`、`min_cluster_size` 和 `max_cluster_size` 现在可在 `config.yaml` 中配置，用于调整矿石检测的灵敏度。
- **改进配置保存**: 修复了 `utils.cpp`，使其能正确保存 `unit_scale`，并在更新配置文件时保留现有注释。

### 代码重构 (Code Refactoring)
- **重构地面过滤逻辑**: 将地面点移除逻辑从 `detectByLidar` 移动到 `OreAnalyzer` 类中专用的 `filterGroundPoints` 方法。这简化了检测逻辑，并避免了 `main.cpp` 中可视化过程中的重复过滤。

### 构建系统 (Build System)
- **优化编译**: 在 `CMakePresets.json` 中添加了 `windows-vcpkg-build` 预设，设置 `jobs: 2` 以限制并行编译任务，防止“堆空间不足 (out of heap space)”错误。
