
## [2026-04-08]
### 完善 by_mask 模式下中间结果生成全过程文档 (Enhanced Documentation for by_mask Mode Processing)
- **文档增强**: 在 `code_explanation.md` 中新增第 7.4 节，详细解析了 `05-08` 图片的生成流水线。
- **坐标映射说明**: 明确了点云物理坐标与图像像素坐标之间的映射关系、转置逻辑以及分步过滤/Rescale 的技术细节。
- **明确各阶段导出逻辑**:
  - `05_raw`: 原始转置物理底图。
  - `06_rescaled`: 全采样 1:1 分辨率对齐图。
  - `07_ores`: 视觉掩码过滤后的纯净矿石深度图。
  - `08_fused`: 最终热力融合图。

## [2026-03-31]
### 高低能 XRT 几何畸变校正 (XRT High/Low Energy Distortion Correction)
- **几何畸变补偿**: 针对高能和低能闪烁体探测器高度不同导致的扇形投影畸变，引入了 `xray_high_energy_correction_factor` 参数。
- **对称对齐逻辑**: 在 `AppPipeline` 中实现了对高能图像的横向缩放校正。根据畸变呈“向外扩散”的特点，校正采用了**对称补全（Symmetric Padding）**或裁剪（Cropping）机制，确保校正后的高能图像与低能图像在中心对齐且尺寸一致。
- **Python 自动校正工具增强**: 升级了 `py_src/auto_xrt_calibrate.py`。现在支持同时输入多个图片，基于第一张图自动计算系数并应用到后续图片中。增加了**亮度曲线分析 (Intensity Profile)** 以精确验证边缘对齐，并支持自动保存可视化结果图（Overlay、Diff、Profile）到 `results/` 目录下。
- **配置项**: 在 `config.yaml` 的 `X-ray Geometric Correction` 栏目下新增了该配置项。

## 2026-03-30
### 体积对比脚本配置固定化与多文件对比支持 (Fixed Path & Multi-File Support for Volume Comparison)
- **配置固定化与全局化**: 在 `py_src/compare_volumes.py` 中引入了全局变量 `REF_FILE` 和 `GEN_FILES`。用户现在可以在文件顶层直接指定文件路径，极大方便了调试与多文件对比。
- **多文件对比能力**: 更新了对比逻辑，支持同时输入多个生成的体积结果文件（CSV 或 XLSX）。脚本会自动匹配 ID 并汇总横向对比数据。
- **可视化布局优化**: 
  - **合并横轴**: 实现了子图 X 轴合并 (`sharex=True`)，上下子图共用底部的 Ore ID 坐标轴，消除冗余标签，使得界面更加整洁。
  - **自动色彩分配**: 使用 `tab10` 配色方案自动为不同的 LiDAR 结果系列分配醒目的颜色。
- **可视化增强**: 改进了绘图样式，使用不同颜色区分不同的 LiDAR 结果系列，同时保留单一参考线（Reference）作为基准。

## 2026-03-27
### 显式分辨率配置与体积计算优化 (Explicit Resolution & Volume Refinement)
- **分辨率参数 (dx_mm / dy_mm)**: 废弃了基于皮带速度和激光频率的自动推算逻辑。现在在 `config.yaml` 中直接指定 `dx_mm` (纵向步进) 和 `dy_mm` (横向间距)。这对于离线处理和预裁剪的点云数据集提供了更高的稳定性和精确度。
- **数据导出格式优化 (.csv)**: 根据用户反馈，将分析结果导出格式由 `.xlsx` 调整为标准的 **`.csv` (Comma Separated Values)**。
- **体积对比可视化增强 (Line Charts)**: 更新了 `py_src/compare_volumes.py` 脚本。对比图表已从柱状图切换为更为直观的 **折线图 (Line Charts)**，并禁用了图表的自动保存功能。

### 点云分辨率参数测算与工具集成 (LiDAR Resolution Analysis & Integration)
- **每一帧点数**: 通过自相关算法推算得出该点云每一扫描帧（沿 Y 轴方向）包含 **1262** 个有效点。
- **物理分辨率 (dx/dy)**:
  - **横向间距 (dy)**: 约为 **0.755 mm** (基于 Y 轴 952.24mm 范围覆盖)。
  - **纵向步进 (dx)**: 约为 **0.586 mm** (基于 X 轴 1200mm 范围与约 2049 帧扫描线推算)。
- **分析工具自动化**: 将上述自相关分析与分辨率测算逻辑集成到了 `py_src/show_ply.py` 脚本中，使其能够自动识别任意 PLY 文件的线扫周期与物理精度。

- **动态结果路径可视化增强**: 实现了 `results_dir` 的自动化动态命名逻辑。系统现在会自动提取 `thickness_map_resolution` (将 `.` 转换为 `p`) 和 `detection_mode` 加载到后缀中（如 `results_0309_0p002_by_mask`），实现实验版本的自动隔离与清晰追踪。
- **数据导出格式优化 (.csv)**: 根据用户反馈，将分析结果导出格式由 `.xlsx` 调整为标准的 **`.csv` (Comma Separated Values)**。这不仅提升了通用性，也确保了在各种数据处理工具中的原生支持。
- **体积对比可视化增强 (Line Charts)**: 更新了 `py_src/compare_volumes.py` 脚本。对比图表已从柱状图切换为更为直观的 **折线图 (Line Charts)**，便于观察不同矿块间的体积波动趋势。同时响应需求，禁用了图表的自动保存功能，改为由用户交互式查看。
- **终端静默模式**: 禁用了矿石详情的 `std::cout` 打印，仅保留保存成功的提示，提升了运行时的界面净度。
- **配置系统增强**: 同步更新了 `utils.h/cpp` 以支持新参数的解析与评论保留保存。

## 2026-03-23
### 补全激光线扫硬件原理文档 (Added Laser Line Scanner Hardware Principle Documentation)
- **原理详解**: 在 `code_explanation.md` 中新增了第 8 节（硬件原理），详细阐述了“线激光+双目相机”的底层工作机理。明确了激光投射、双目捕捉、条纹提取、立体匹配到深度计算的全流程。
- **视角辩析**: 补充了“硬件视角 (Depth -> Point Cloud)”与“软件视角 (Point Cloud -> Depth Map)”的对比。解答了为何不同语境下三维数据的生成先后顺序会有所不同，明确了点云投影生成深度图是应用层常用的技术手段。

### 增强厚度图坐标映射文档 (Enhanced Thickness Map Coordinate Mapping Documentation)
- **文档补充**: 在 `code_explanation.md` 中新增了第 5.5 节，详细解释了 `thickness_map` 的生成、保存及轴向转置逻辑。明确了地面对齐、网格化采样（Max-Pooling）以及为了对齐视觉图像所必须进行的坐标轴转置渲染过程。
- 之所以在代码中依然保留“考虑多个点落入同一个像素槽位”的逻辑（即 if (thick > map.data[index])），主要有两个原因：
  - 浮点数偏移与抗锯齿：由于地面对齐涉及旋转矩阵（点云 $X, Y$ 坐标会从规整的采样步长变成任意浮点数），经过旋转后的坐标投影到像素网格时，不可避免地会产生微小的偏移。即使原始物理间距是 1:1，旋转后的点也可能因为舍入误差导致两个物理相邻的点落入同一个像素索引中（static_cast<int> 舍入），而相邻的另一个像素索引却变空（产生所谓的“莫尔条纹”或“黑线”）。目前的逻辑（Max-Pooling）可以稳健地处理这种浮点数边界情况，确保厚度值不会被覆盖。
  - 多帧/扫描重叠的稳健性：在实际场景中，由于扫描频率和皮带速度的波动，或者在累加多帧数据时，局部区域可能会存在点云重叠。取 max() 可以确保我们始终保留最顶端的表面厚度，过滤掉可能重复扫描产生的冗余。

- **逻辑合规性校验**: 确认了当前系统在投影时采取的“点云 X -> 图像 Row”以及“点云 Y -> 图像 Col”的映射关系，符合带式输送机工业场景的视觉习惯。

## 2026-03-16
### 支持 LiDAR 模式下动态矿石掩码生成与多模态切片保存 (Dynamic Ore Mask Generation and Multi-Modal Patch Saving in LiDAR Mode)
- **动态生成二值掩码**: 解决了当 `detection_mode: "lidar"`（纯点云聚类模式）时，因缺乏 2D 图像对应的 `ore_masks` 导致无法导出单矿石图像切片的问题。现在系统会在聚类提取出 3D 矿石点云后，自动将每个独立矿石的三维坐标系根据分辨率精确反投射(Project)回统一的 2D `Thickness Map` 像素坐标系中。
- **形态学连通性修复**: 针对激光雷达点云稀疏投影到高分辨率 2D 图像时产生的“椒盐状”离散噪点问题，新引入了基于椭圆核的数学形态学闭运算 (`cv::morphologyEx` with `cv::MORPH_CLOSE`)，将稀疏点强行融合成连通块，随后提取外轮廓并使用 `cv::FILLED` 参数进行实心填充，确保每块矿石的掩码完整无空洞。
- **打通数据导出流水线**: 构建出的虚拟 `ore_masks` 完美“骗过”了下游的图像保存模块。现在即使用户在配置文件中选用纯点云探测 (`lidar`)，程序依然能够像视觉分割模式 (`mask`/`roi`) 那样，在 `results/ore_patches` 文件夹下乖乖输出每一块矿石完美的独立包裹切片（包括 `.tif` 深度图、`_mask.png` 掩码图和双能 XRT 局部子图）。

### 矿石空间排序与输出增强 (Unified Spatial Sorting and Output Enhancement)
- **通用空间排序工具**: 在 `OreAnalyzer` 中新增静态方法 `sortOresSpatially`，统一了 LiDAR 聚类和图像分割的排序逻辑。
- **LiDAR 模式对齐修复**: 修正了 LiDAR 模式下轮廓叠加图偏移和方向不对的问题。通过将 `ore_masks` 的生成时机延后到 `thickness_map` 对齐（旋转与缩放）之后，确保了掩码坐标与 XRT 图像坐标系完美契合。
- **LiDAR 模式补全轮廓叠加图**: 现在 LiDAR 模式也会导出带编号的轮廓覆盖图，方便可视化校验。
- **轮廓颜色统一**: 将低能 (Low) 叠加图中的矿石轮廓颜色从绿色统一修改为红色，与高能 (High) 叠加图保持一致。

## 2026-03-13
### 全局厚度图分辨率解耦 (Global Thickness Map Resolution Decoupling)
- **参数体系重构**: 在config当中引入独立配置项 `thickness_map_resolution` 以定义 2D 厚度地图的输出空间分辨率（米）。该重构彻底解绑了全局厚度图分辨率与点云底层物理坐标缩放系数 (`unit_scale`) 之间的隐式计算依赖，避免了因调整光栅化输出分辨率而连带破坏核心点云物理尺度转换逻辑的隐患。相关调用（如 `generateGlobalThicknessMap`）均已适配更新。

### 矿石厚度统计特征扩展与单次遍历优化 (Ore Thickness Statistical Expansion and Single-Pass Optimization)
- **拓展统计描述特征**: 增强了单目标矿石的三维几何特征提取，在终端输出模型中补充了**最小厚度极值 (Min)** 与**厚度标准差 (Std)** 指标，为量化描述矿石表面的宏观离散度与凹凸平滑度提供了更完备的统计学参考。
- **算法复杂度优化**: 针对厚度均值与标准差的计算实现，引入了 **Welford 算法 (Welford's online algorithm)** 进行重构。该算法保证了在 O(N) 的时间复杂度内，通过单次数据流扫描 (`single-pass`) 即可稳定、高精度地并发计算出均值与方差。此优化有效消除了传统统计公式所需的二次遍历，显著降低了算法的内存 Cache 指令开销，进而提升了针对大规模点云簇的计算效能。

## 2026-03-12
### 图像输出添加序号前缀 (Output Image Sequence Prefixing)
- **为所有中间与最终输出图像添加序号前缀**：为了展示算法处理的先后顺序，在 `app_pipeline.cpp` 和 `ore_analysis.cpp` 中将保存到 `results/` 下的文件重命名。顺序如下：
  1. `01/02_cropped_xray_for_fusion_low/high.jpg`
  2. `03/04_extracted_contours_overlay_low/high.jpg`
  3. `05_raw_thickness_map.tif`
  4. `06_rescaled_thickness_map.tif`
  5. `07_ores_thickness_map.tif`
  6. `08_fused_thickness_xray...`

### 厚度图流水线三阶段重构 (Thickness Map 3-Stage Pipeline Restructure)
- **raw**: `05_raw_thickness_map.tif` — 原始网格分辨率、全部高于地面阈值的点、float32 TIFF、已 transpose 对齐 XRT 方向。
- **rescaled**: `06_rescaled_thickness_map.tif` — raw resize 到 XRT 图像尺寸、float32 TIFF。
- **ores**: `07_ores_thickness_map.tif` — rescaled 经 XRT 矿石轮廓 mask 过滤，仅保留矿石区域像素、float32 TIFF。
- **删除冗余的第二次 `generateGlobalThicknessMap` 调用**：ores map 现在直接由 rescaled map + 合并轮廓掩码派生，不再重新生成。
- **移除冗余 `pt.z > ground_threshold_` 检查**：`filterGroundPoints()` 已在早期阶段彻底删除地面点，后续循环无需再判断。
- **厚度单位统一为毫米 (mm)**：所有 `pt.z * unit_scale_` 计算后乘以 `1000.0f`，终端输出添加 `mm` 标注。
- **矿石排序**：mask 模式下按（从上到下、从左到右）行分组排序，使用中位数矿石高度作为行容差，重绘带编号的轮廓叠加图。


## 2026-03-09
### 单矿石多模态 Patch 导出 (Per-Ore Multi-Modal Patch Export)
- **新增 `saveOrePatches` 功能**: 在 `app_pipeline.cpp` 中实现了逐矿石导出 bounding box 级别的多模态数据切片。对每一块被检测到的矿石，系统会自动裁切并保存至 `results/ore_patches/` 目录下的 4 个同尺寸文件：
  - `ore_N_thickness.tif`：**32-bit float TIFF**，保留完整浮点精度的厚度值，mask 外区域置零。
  - `ore_N_mask.png`：**8-bit 二值掩码**，标识 bounding box 内哪些像素属于矿石本体。
  - `ore_N_xrt_low.png`：低能 X 射线图像对应区域裁切。
  - `ore_N_xrt_high.png`：高能 X 射线图像对应区域裁切。
- **流程集成**: 在 pipeline 的 mask 检测循环中新增并行的 `ore_masks` 向量，精确追踪每块矿石与其图像轮廓掩码的对应关系。输出目录由 `std::filesystem::create_directories` 自动创建。

### 修复厚度图 X 轴范围被隐式裁切至矿石区域 (Fix X-axis Implicit Crop to Ore Region)
- **自动锚定 X 轴完整扫描范围**: 在 `filterGroundPoints` 执行地面点移除之前，自动调用 `getMinMax3D` 捕获并缓存完整点云的 X 轴极值至内部成员 `belt_min_x_` / `belt_max_x_`。修复了因地面过滤后 `aligned_cloud_` 仅剩矿石点，导致 `computeCropBounds` 中 X 方向的基底范围被隐式收缩至矿石覆盖区域的问题。
- **`lidar_crop_up/down` 语义修正**: 现在 `lidar_crop_up` / `lidar_crop_down` 参数从**完整点云扫描长度**的两端向内裁切，而非从仅有矿石存在的狭小区间裁切，彻底恢复了用户对 X 轴裁切范围的直觉控制。无需在 `config.yaml` 中额外手动配置 X 轴边界。

## 2026-03-05
### 修复 `computeCropBounds` 裁切单位不匹配 Bug (Fix Crop Unit Mismatch)
- **Bug 修复**: `computeCropBounds` 中 `lidar_crop_*` 值直接以原生点云单位参与运算，不再做 `unit_scale` 换算，修复了之前因 `unit_scale` 乘法导致裁切效果被缩小约 1000 倍的问题。
- **接口简化**: 从 `computeCropBounds` 的函数签名中彻底移除了现在已不再使用的 `unit_scale` 参数（包括 `ore_analysis.h` 声明、`ore_analysis.cpp` 定义及全部 3 处调用点）。
- **配置还原**: `config.yaml` 中 `lidar_crop_left/right` 从测试用 `150000` 恢复为合理的 `150` (mm)。

### 点云物理尺度系重构与边缘裁切精度修正 (Point Cloud Physical Scaling & Crop Refactoring)
- **精准的物理单位尺度对齐 (`unit_scale` 语义明确化)**: 彻底梳理了 `unit_scale` 的数学语义。修复了由于 `unit_scale: 1.0` 导致将点云设备输出的“毫米级”物理坐标强制误认为“米”，从而引发 `belt_min_y` (如 -478) 产生近一公里宽计算范围放大的严重逻辑偏移。明确规定：若原始点云单位为毫米(mm)，在系统内必须强制配置 `unit_scale: 0.001` 以统一向核心算法层的国际标准单位(米)对齐换算。
- **裁切参数降维回归物理空间 (`lidar_crop` 物理化重构)**: 重构了 `OreAnalyzer::computeCropBounds` 的实现逻辑。现在配置文件中的 `lidar_crop_*` (如左/右/上/下) 参数不再被粗略等效于“生成的二维厚度图的像素网格数量 (`resolution_raw`)”；而是**被直接以原生点云坐标系单位回归为其在点云空间内的绝对物理数量差值**。例如：若原始点云单位为 mm，设置 `lidar_crop_left: 150` 即精确裁去左侧 150mm 的物理空间，彻底保证参数填写的直觉性与尺度鲁棒性。

## 2026-03-04
### 补全双能 X-ray (高能与低能) 同步坐标对齐及图像保存
- **提取并保存高能 X 射线图像**: 完善了 X-ray 模式下的双能图片读取。现在引擎不仅从源图像的左半部分切割出低能图像，同时也从右半部分精准裁剪出高能图像。高能图像会同步经历翻转 (`flip`)、去废边 (`cut`)、ROI 裁切 (`crop`) 以及畸变矫正 (`correctXrayGeometry`)，最终被保存为 `results/cropped_xray_high_for_fusion.jpg` 供融合观测参考。
- **高能图像掩码穿透映射 (`extracted_contours_overlay_high`)**: 在使用低能 X 射线图像完成边缘检测并提取出 `valid_contours` 后，系统现在会将这些精细的掩码轮廓同时无缝挂载映射到高清的“高能图像”之上，并专门保存一份由红线圈出的高能重叠对比图 (`extracted_contours_overlay_high.jpg`)。

## 2026-03-03
### 修复坐标轴映射倒置导致提取框失效的严重 Bug 
- **重构轮廓逆推寻点逻辑 (Method 2)**: 彻底弃用了原有“将图像里的轮廓逐个逆推算其物理包围盒坐标，然后再硬套入雷达空间求点”的复杂且极易因 Y 轴上下颠倒而翻车的设计。
- **全局 Thickness Map 前置生成**: 重构了 `app_pipeline.cpp` 的核心流程。现在即使是图像或选区检测模式，也会**率先强行利用所有点生成一份 2D 全局厚度底图 (`Thickness Map`)**。
- **底层深度图自适应扩充 (`alignThicknessMapToImage`)**: 响应需求，现在当图片生成后，不再是将高分辨率 RGB/X-Ray 掩码缩小以适应低分辨率雷达深度图；而是强行**将点云深度图 (Thickness Map) 直接拉伸扩充到图像的分辨率级别**！不仅统一了所有坐标系统，也让融合算法 (`fuseThicknessWithImage/Xray`) 直接进入了 1:1 的完美像素重叠通道，彻底砍掉了融合器里冗杂的转置、计算 Crop 放缩代码。
- **在原始图像栅格中实现完美套准 (`detectByMask`)**: 依靠拉伸后的底图，雷达找点时不再比对固定的物理坐标，而是直接拿着自己的点云坐标算作在裁切区域的 `[0, 1]` 相对位置，然后按比例投射到对应的原生高清 `Mat mask` 像素坐标中看像素值。确立了以高分辨率 2D 图像作主导的全新架构。
- **独立纯净版矿石深度图 (`ores_thickness_map`)**: 流程现在会额外生成并保存一张与原图等大、但**仅包含被成功检测出的 Ores 内部点**的深度图。且现在与 RGB/X-ray 图像进行的视觉融合，都会优先采用这张剔除了背景杂点的纯净深度图，使得重叠效果更加清爽。

## 2026-03-02

### 扩充矿石检测模式 (Expand Ore Detection Mode)
- **新增由图像提取矿石轮廓**: 在 `OreAnalyzer` 类中实现了 `extractOresFromImage` 方法，利用 OpenCV (如 `cv::cvtColor`, `cv::threshold`, `cv::findContours`) 将输入的 RGB或 XRT 图像进行二值化并提取轮廓，输出带有物理坐标及掩码信息的自定义结构体 (`ImageContour`)。提取时会将找到的轮廓描边叠加回原图并保存至 `results/` 目录下以便调试。
- **精准物理坐标映射复用**: 提取并封装了用于推算图像真实物理边界的独立函数 `computeCropBounds`（内聚在 `OreAnalyzer` 类中）。该计算囊括了点云原始极值、皮带限定宽度，以及 `lidar_crop_*` 设置项在物理层面上对齐引入的边缘切除位移（彻底移除了旧版代码中人为硬编的 5 像素扩展边距误差以实现 1:1 纯粹对齐）。该函数的复用彻底取代了 pipeline 分离计算的冗余，确保图像选区(`mask/roi`)能与原始点云进行精确重合匹配。同时，该函数也已被接管进 `fuseThicknessWithImage` 及 `fuseThicknessWithXray` 的核心逻辑中，以替代手动从图像边界直接裁剪的粗略逻辑。
- **配置化检测模式**: 在 `config.yaml` 的解析对象 `AppConfig` 里引入了 `detection_mode` 设置项以取代单一的 LiDAR 聚类，目前支持 `lidar`, `mask`, 和 `roi` 三种模式串联流。
- **保存中间过程调试图像**: 创建了专用的 `results/` 文件夹来收容所有的可视化输出结果。现在在执行 `fuse_mode` 融合作业时，系统会自动在合成最终图之前，将等比缩放且应用了对比度动态增强 Jet Colormap 的纯厚度视场图 (`rescaled_thickness_for_xx_fusion.jpg`) 以及依据选区参数裁切完毕的干净相机源图 (`cropped_xx_for_fusion.jpg`) 分别存入 `results/` 目录供对齐验证参考。
- **Pipeline 分支处理**: 更新了 `app_pipeline.cpp` 中的 `executeDetectionAndFusion` 流程，现根据检测模式差异，系统分别自适应路由调用传统的 `detectByLidar` 或新加入的 `extractOresFromImage` 获取图像选区后交由 `detectByMask` / `detectByROI` 层处理获取点云子集，为跨传感器分割提供了统一网关。所有的基础地图输出也都已经被重定向到了 `results/` 目录。

## 2026-02-28### 核心输出指标升级 (Core Output Metrics Upgrade)
- **命令行补充点云分析**: 根据使用实际反馈与需求：
  1. 取消了在最终生成的各类厚度效果图上“强行叠加标出内部点云数量”的杂乱绘图行为，以保证视觉图谱本身只聚焦于矿石分布与厚度热力展现。
  2. 现在所有包含的点云统计信息（如：`Ore 1 [1250 pts]: Avg Thickness...`）被静默转入命令行标准输出 `std::cout`，确保系统终端留有详尽的数据记录供追溯。
  3. **皮带线分辨率测算 (`Belt Cloud Resolution`)**: 在应用地面对齐后，系统新增测算截断皮带方向(`Y`轴宽度)上的物理扫描分辨率，现在终端能够直观打出如 `Belt Cloud Resolution: ... mm/point (Width: 937mm)` 的点间距物理分辨率指标。
  4. **输出加上聚类策略序号**: 根据反馈，自动生成的基础黑白厚度图 `thickness_map` 以及它的所有色彩融合图版本（RGB 融合、X-ray 融合）的输出文件名后缀皆已默认绑定并附加上你当前配置文件中执行的 `cluster_strategy` （聚类策略）序号。例如：`thickness_map_strategy_2.png`。避免不同策略方案多次诊断测试时产生的输出效果被盲目覆盖掉。

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
