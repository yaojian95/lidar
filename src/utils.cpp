#include "utils.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

YAML::Node Config::load(const std::string &config_file) {
  try {
    return YAML::LoadFile(config_file);
  } catch (const YAML::Exception &e) {
    std::cerr << "Error loading config file " << config_file << ": " << e.what()
              << std::endl;
    return YAML::Node();
  }
}

void Config::save(const std::string &config_file, const YAML::Node &config) {
  try {
    std::ofstream fout(config_file);
    fout << config;
    fout.close();
    std::cout << "Config saved to " << config_file << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error saving config file " << config_file << ": " << e.what()
              << std::endl;
  }
}

AppConfig Config::parseAppConfig(const YAML::Node &config) {
  AppConfig app;
  if (!config)
    return app;

  app.pcd_path = get<std::string>(config, "pcd_path", "");
  app.results_dir = get<std::string>(config, "results_dir",
                                     "E:/multi_source_info/lidar/results");
  app.unit_scale = get<float>(config, "unit_scale", 1.0f);
  app.dx_mm = get<float>(config, "dx_mm", 0.0f);
  app.dy_mm = get<float>(config, "dy_mm", 0.0f);
  app.save_plane = get<bool>(config, "save_plane_equation", false);
  app.visual_plane = get<bool>(config, "visual_plane", true);
  app.enable_visualization = get<bool>(config, "enable_visualization", true);

  app.ground_threshold = get<float>(config, "ground_threshold", 0.02f);
  app.ground_filter_sigma = get<float>(config, "ground_filter_sigma", 6.0f);
  app.ground_filter_margin = get<float>(config, "ground_filter_margin", 1.0f);
  app.fit_belt_edges = get<bool>(config, "fit_belt_edges", false);
  app.belt_min_y = get<float>(config, "belt_min_y", -1e9f);
  app.belt_max_y = get<float>(config, "belt_max_y", 1e9f);

  app.thickness_map_resolution =
      get<float>(config, "thickness_map_resolution", 0.01f);

  app.detection_mode = get<std::string>(config, "detection_mode", "lidar");

  app.cluster_tolerance = get<float>(config, "cluster_tolerance", 0.05f);
  app.min_cluster_size = get<int>(config, "min_cluster_size", 50);
  app.max_cluster_size = get<int>(config, "max_cluster_size", 50000);

  app.cluster_strategy = get<int>(config, "cluster_strategy", 0);
  app.aspect_ratio_threshold =
      get<float>(config, "aspect_ratio_threshold", 2.5f);
  app.density_threshold = get<float>(config, "density_threshold", 0.3f);
  app.rg_smoothness = get<float>(config, "rg_smoothness", 15.0f);
  app.rg_curvature = get<float>(config, "rg_curvature", 1.0f);

  app.fuse_mode = get<std::string>(config, "fuse", "false");
  app.fusion_channel = get<int>(config, "fusion_channel", 2);

  app.rgb_crop_up = get<int>(config, "rgb_crop_up", 0);
  app.rgb_crop_down = get<int>(config, "rgb_crop_down", 0);
  app.rgb_crop_left = get<int>(config, "rgb_crop_left", 0);
  app.rgb_crop_right = get<int>(config, "rgb_crop_right", 0);

  app.lidar_crop_up = get<int>(config, "lidar_crop_up", 0);
  app.lidar_crop_down = get<int>(config, "lidar_crop_down", 0);
  app.lidar_crop_left = get<int>(config, "lidar_crop_left", 0);
  app.lidar_crop_right = get<int>(config, "lidar_crop_right", 0);

  app.xray_path = get<std::string>(config, "xray_path", "");
  app.enable_xray_geometry_correction =
      get<bool>(config, "enable_xray_geometry_correction", true);
  app.xray_sod = get<float>(config, "xray_sod", 20.0f);
  app.xray_sdd = get<float>(config, "xray_sdd", 60.0f);
  app.xray_cut_left = get<int>(config, "xray_cut_left", 0);
  app.xray_cut_right = get<int>(config, "xray_cut_right", 0);

  app.xray_crop_up = get<int>(config, "xray_crop_up", 0);
  app.xray_crop_down = get<int>(config, "xray_crop_down", 0);
  app.xray_crop_left = get<int>(config, "xray_crop_left", 0);
  app.xray_crop_right = get<int>(config, "xray_crop_right", 0);

  return app;
}

PlaneCoefficients PlaneCoefficients::fromConfig(const YAML::Node &config) {
  PlaneCoefficients plane;

  if (config["plane_a"] && config["plane_b"] && config["plane_c"] &&
      config["plane_d"]) {
    plane.a = config["plane_a"].as<float>();
    plane.b = config["plane_b"].as<float>();
    plane.c = config["plane_c"].as<float>();
    plane.d = config["plane_d"].as<float>();

    // Check if plane is valid (at least one coefficient is non-zero)
    if (plane.a != 0.0f || plane.b != 0.0f || plane.c != 0.0f) {
      plane.valid = true;
    }
  }

  return plane;
}

void PlaneCoefficients::saveToConfig(YAML::Node &config) const {
  config["plane_a"] = a;
  config["plane_b"] = b;
  config["plane_c"] = c;
  config["plane_d"] = d;

  std::cout << "Saved plane equation: " << a << "x + " << b << "y + " << c
            << "z + " << d << " = 0" << std::endl;
}

// Helper function to save config while preserving comments
void saveConfigWithComments(const std::string &config_file,
                            const YAML::Node &config) {
  // Read original file line by line
  std::ifstream infile(config_file);
  std::vector<std::string> lines;
  std::string line;
  bool in_plane_section = false;

  while (std::getline(infile, line)) {
    // Check if this line contains a plane coefficient or other updatable keys
    if (line.find("plane_a:") != std::string::npos ||
        line.find("plane_b:") != std::string::npos ||
        line.find("plane_c:") != std::string::npos ||
        line.find("plane_d:") != std::string::npos ||
        line.find("ground_threshold:") != std::string::npos ||
        line.find("belt_min_y:") != std::string::npos ||
        line.find("belt_max_y:") != std::string::npos ||
        line.find("ground_filter_sigma:") != std::string::npos ||
        line.find("ground_filter_margin:") != std::string::npos ||
        line.find("unit_scale:") != std::string::npos ||
        line.find("belt_speed:") != std::string::npos ||
        line.find("scan_rate:") != std::string::npos) {

      // Extract the key name
      std::string key;
      if (line.find("plane_a:") != std::string::npos)
        key = "plane_a";
      else if (line.find("plane_b:") != std::string::npos)
        key = "plane_b";
      else if (line.find("plane_c:") != std::string::npos)
        key = "plane_c";
      else if (line.find("plane_d:") != std::string::npos)
        key = "plane_d";
      else if (line.find("ground_threshold:") != std::string::npos)
        key = "ground_threshold";
      else if (line.find("belt_min_y:") != std::string::npos)
        key = "belt_min_y";
      else if (line.find("belt_max_y:") != std::string::npos)
        key = "belt_max_y";
      else if (line.find("ground_filter_sigma:") != std::string::npos)
        key = "ground_filter_sigma";
      else if (line.find("ground_filter_margin:") != std::string::npos)
        key = "ground_filter_margin";
      else if (line.find("unit_scale:") != std::string::npos)
        key = "unit_scale";
      else if (line.find("belt_speed:") != std::string::npos)
        key = "belt_speed";
      else if (line.find("scan_rate:") != std::string::npos)
        key = "scan_rate";

      // Get leading whitespace
      size_t first_non_space = line.find_first_not_of(" \t");
      std::string indent = (first_non_space != std::string::npos)
                               ? line.substr(0, first_non_space)
                               : "";

      // Replace with new value from config
      if (config[key]) {
        // Special handling for comments if present?
        // Simple approach: Replace the value part, keep the comment if we can
        // parse it. But current logic completely replaces the line: line =
        // indent + key + ": " + value This DESTROYS comments on the same line.
        // The user specifically asked for comments to be preserved.
        // My previous edit to config.yaml added a comment to unit_scale line.
        // The current implementation of saveConfigWithComments overwrites the
        // whole line.

        // Let's try to preserve the comment if it exists on the line.
        std::string original_comment = "";
        size_t comment_pos = line.find("#");
        if (comment_pos != std::string::npos) {
          original_comment = line.substr(comment_pos);
        }

        line = indent + key + ": " + std::to_string(config[key].as<float>());
        if (!original_comment.empty()) {
          line += " " + original_comment;
        }
      }
    }
    lines.push_back(line);
  }
  infile.close();

  // If plane coefficients don't exist in file, append them
  bool has_plane_a = false, has_plane_b = false, has_plane_c = false,
       has_plane_d = false;
  for (const auto &l : lines) {
    if (l.find("plane_a:") != std::string::npos)
      has_plane_a = true;
    if (l.find("plane_b:") != std::string::npos)
      has_plane_b = true;
    if (l.find("plane_c:") != std::string::npos)
      has_plane_c = true;
    if (l.find("plane_d:") != std::string::npos)
      has_plane_d = true;
  }

  bool has_threshold = false;
  bool has_belt_min = false;
  bool has_belt_max = false;
  for (const auto &l : lines) {
    if (l.find("ground_threshold:") != std::string::npos)
      has_threshold = true;
    if (l.find("belt_min_y:") != std::string::npos)
      has_belt_min = true;
    if (l.find("belt_max_y:") != std::string::npos)
      has_belt_max = true;
  }

  if (!has_plane_a || !has_plane_b || !has_plane_c || !has_plane_d) {
    lines.push_back("");
    lines.push_back("# Plane equation (auto-generated)");
    if (!has_plane_a && config["plane_a"])
      lines.push_back("plane_a: " +
                      std::to_string(config["plane_a"].as<float>()));
    if (!has_plane_b && config["plane_b"])
      lines.push_back("plane_b: " +
                      std::to_string(config["plane_b"].as<float>()));
    if (!has_plane_c && config["plane_c"])
      lines.push_back("plane_c: " +
                      std::to_string(config["plane_c"].as<float>()));
    if (!has_plane_d && config["plane_d"])
      lines.push_back("plane_d: " +
                      std::to_string(config["plane_d"].as<float>()));
    if (!has_plane_d && config["plane_d"])
      lines.push_back("plane_d: " +
                      std::to_string(config["plane_d"].as<float>()));
  }

  if (!has_threshold && config["ground_threshold"]) {
    lines.push_back("ground_threshold: " +
                    std::to_string(config["ground_threshold"].as<float>()));
  }
  if (!has_belt_min && config["belt_min_y"]) {
    lines.push_back("belt_min_y: " +
                    std::to_string(config["belt_min_y"].as<float>()));
  }
  if (!has_belt_max && config["belt_max_y"]) {
    lines.push_back("belt_max_y: " +
                    std::to_string(config["belt_max_y"].as<float>()));
  }

  // Add new keys if missing
  bool has_sigma = false;
  bool has_margin = false;
  for (const auto &l : lines) {
    if (l.find("ground_filter_sigma:") != std::string::npos)
      has_sigma = true;
    if (l.find("ground_filter_margin:") != std::string::npos)
      has_margin = true;
  }

  if (!has_sigma && config["ground_filter_sigma"])
    lines.push_back("ground_filter_sigma: " +
                    std::to_string(config["ground_filter_sigma"].as<float>()));
  if (!has_margin && config["ground_filter_margin"])
    lines.push_back("ground_filter_margin: " +
                    std::to_string(config["ground_filter_margin"].as<float>()));

  // Write back to file
  std::ofstream outfile(config_file);
  for (const auto &l : lines) {
    outfile << l << "\n";
  }
  outfile.close();
}

namespace Utils {
bool correctXrayGeometry(const cv::Mat &input_image, cv::Mat &output_image,
                         float sod, float sdd) {
  if (sod <= 0.0f || sdd <= 0.0f) {
    std::cerr << "Error: SOD and SDD must be positive values." << std::endl;
    return false;
  }
  if (input_image.empty()) {
    std::cerr << "Error: Input image is empty." << std::endl;
    return false;
  }

  float magnification = sdd / sod;
  if (magnification <= 0.001f) {
    std::cerr << "Error: Invalid magnification calculated (" << magnification
              << ")." << std::endl;
    return false;
  }

  // The X-ray sequence only has distortion in the cross-belt direction (X-axis,
  // width).
  double scale_factor_x = 1.0 / static_cast<double>(magnification);
  double scale_factor_y = 1.0;
  cv::resize(input_image, output_image, cv::Size(), scale_factor_x,
             scale_factor_y, cv::INTER_LINEAR);

  return true;
}
} // namespace Utils
