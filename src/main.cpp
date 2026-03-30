#include "app_pipeline.h"
#include "ore_analysis.h"
#include "utils.h"
#include <filesystem>
#include <iostream>


int main() {
  // 1. Load config
  YAML::Node raw_yaml = Config::load("E:/multi_source_info/lidar/config.yaml");
  if (!raw_yaml) {
    std::cerr << "Failed to load config file!" << std::endl;
    return -1;
  }
  AppConfig config = Config::parseAppConfig(raw_yaml);

  // Auto-format results directory name based on resolution and mode
  std::string base_dir = config.results_dir;
  if (!base_dir.empty()) {
    // Format resolution: 0.002 -> 0p002
    char res_buf[32];
    snprintf(res_buf, sizeof(res_buf), "%.3f", config.thickness_map_resolution);
    std::string res_str(res_buf);
    size_t dot_pos = res_str.find('.');
    if (dot_pos != std::string::npos) {
      res_str.replace(dot_pos, 1, "p");
    }
    // Remove trailing zeros (but keep at least one digit after 'p')
    while (res_str.size() > dot_pos + 2 && res_str.back() == '0') {
      res_str.pop_back();
    }

    // New format: prefix_0p002_by_mask
    std::string formatted_dir =
        base_dir + "_" + res_str + "_by_" + config.detection_mode;
    config.results_dir = formatted_dir;
  }

  // Auto-create results directories
  try {
    std::filesystem::create_directories(config.results_dir);
    std::filesystem::create_directories(config.results_dir + "/ore_patches");
    std::cout << "Ensured results directory exists: " << config.results_dir
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Warning: Failed to create results directories: " << e.what()
              << std::endl;
  }

  // 2. Load Point Cloud
  auto cloud = AppPipeline::loadPointCloud(config.pcd_path);
  if (!cloud)
    return -1;
  std::cout << "Unit Scale: " << config.unit_scale << std::endl;

  // 3. Initialize Analyzer
  OreAnalyzer analyzer;
  analyzer.setPointCloud(cloud);
  analyzer.setConfig(config.unit_scale, config.ground_threshold);
  analyzer.setGroundThresholdParams(config.ground_filter_sigma,
                                    config.ground_filter_margin);

  // 4. Align Ground and Filter Belts
  bool aligned =
      AppPipeline::processCalibrationAndGround(analyzer, config, raw_yaml);

  if (aligned) {
    // 5. Detect Ore and Image Fusion
    AppPipeline::executeDetectionAndFusion(analyzer, config);
    cloud = analyzer.getAlignedCloud();
  }

  // 6. Visualization
  if (config.enable_visualization) {
    AppPipeline::visualizeResults(cloud, config.visual_plane, aligned);
  } else {
    std::cout << "Visualization disabled in config." << std::endl;
  }

  return 0;
}
