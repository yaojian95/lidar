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

// Config helper class
class Config {
public:
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
