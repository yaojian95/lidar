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
