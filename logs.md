## 2026-01-27

### 配置更新 (Configuration Updates)
- **更新 `unit_scale`**: 在 `config.yaml` 中将校准值设置为 `0.003375`，以启用正确的物理距离测量（米）。
- **新增欧几里得聚类参数**: `cluster_tolerance`、`min_cluster_size` 和 `max_cluster_size` 现在可在 `config.yaml` 中配置，用于调整矿石检测的灵敏度。
- **改进配置保存**: 修复了 `utils.cpp`，使其能正确保存 `unit_scale`，并在更新配置文件时保留现有注释。

### 代码重构 (Code Refactoring)
- **重构地面过滤逻辑**: 将地面点移除逻辑从 `detectByLidar` 移动到 `OreAnalyzer` 类中专用的 `filterGroundPoints` 方法。这简化了检测逻辑，并避免了 `main.cpp` 中可视化过程中的重复过滤。

### 构建系统 (Build System)
- **优化编译**: 在 `CMakePresets.json` 中添加了 `windows-vcpkg-build` 预设，设置 `jobs: 2` 以限制并行编译任务，防止“堆空间不足 (out of heap space)”错误。
