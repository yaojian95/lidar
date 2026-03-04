#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

namespace Utils {
// In-memory X-ray Geometry Correction
// Scales only the cross-belt direction (X-axis/width) by 1/M.
// M = sdd / sod.
bool correctXrayGeometry(const cv::Mat &input_image, cv::Mat &output_image,
                         float sod = 20.0f, float sdd = 60.0f);
} // namespace Utils

// Data structure holding all central configuration parameters
struct AppConfig {
  // General & Point Cloud
  std::string pcd_path = "";
  float unit_scale = 1.0f;
  bool save_plane = false;
  bool visual_plane = true;
  bool enable_visualization = true;

  // Detection Mode
  std::string detection_mode = "lidar"; // lidar, roi, mask

  // Belt & Ground Properties
  float ground_threshold = 0.02f;
  float ground_filter_sigma = 6.0f;
  float ground_filter_margin = 1.0f;
  bool fit_belt_edges = false;
  float belt_min_y = -1e9f;
  float belt_max_y = 1e9f;

  // Ore Detection (Clustering)
  float cluster_tolerance = 0.05f;
  int min_cluster_size = 50;
  int max_cluster_size = 50000;

  // Hybrid Clustering Strategies
  int cluster_strategy = 0;
  float aspect_ratio_threshold = 2.5f;
  float density_threshold = 0.3f;
  float rg_smoothness = 15.0f;
  float rg_curvature = 1.0f;

  // Fusion Control
  std::string fuse_mode = "false";
  int fusion_channel = 2; // Default Red

  // RGB Fusion Crops
  int rgb_crop_up = 0;
  int rgb_crop_down = 0;
  int rgb_crop_left = 0;
  int rgb_crop_right = 0;

  // LiDAR Fusion Crops
  int lidar_crop_up = 0;
  int lidar_crop_down = 0;
  int lidar_crop_left = 0;
  int lidar_crop_right = 0;

  // X-Ray Fusion
  std::string xray_path = "";
  bool enable_xray_geometry_correction = true;
  float xray_sod = 20.0f;
  float xray_sdd = 60.0f;
  int xray_cut_left = 0;
  int xray_cut_right = 0;

  // X-Ray Fusion Crops
  int xray_crop_up = 0;
  int xray_crop_down = 0;
  int xray_crop_left = 0;
  int xray_crop_right = 0;
};

// Config helper class
class Config {
public:
  static AppConfig parseAppConfig(const YAML::Node &config);
  static YAML::Node load(const std::string &config_file);
  static void save(const std::string &config_file, const YAML::Node &config);

  // Convenience methods
  template <typename T>
  static T get(const YAML::Node &config, const std::string &key,
               const T &default_val) {
    if (config[key]) {
      return config[key].as<T>();
    }
    return default_val;
  }
};

// Plane coefficients structure
struct PlaneCoefficients {
  float a, b, c, d;
  bool valid = false;

  static PlaneCoefficients fromConfig(const YAML::Node &config);
  void saveToConfig(YAML::Node &config) const;
};

// Helper function to save config while preserving comments and formatting
void saveConfigWithComments(const std::string &config_file,
                            const YAML::Node &config);
