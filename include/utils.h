#pragma once
#include <string>
#include <yaml-cpp/yaml.h>

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
