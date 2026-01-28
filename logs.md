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
