#include "app_pipeline.h"
#include "ore_analysis.h"
#include "utils.h"
#include <iostream>


int main() {
  // 1. Load config
  YAML::Node raw_yaml = Config::load("E:/multi_source_info/lidar/config.yaml");
  if (!raw_yaml) {
    std::cerr << "Failed to load config file!" << std::endl;
    return -1;
  }
  AppConfig config = Config::parseAppConfig(raw_yaml);

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
