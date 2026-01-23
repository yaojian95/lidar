#include "utils.h"
#include <fstream>
#include <iostream>

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
    // Check if this line contains a plane coefficient
    if (line.find("plane_a:") != std::string::npos ||
        line.find("plane_b:") != std::string::npos ||
        line.find("plane_c:") != std::string::npos ||
        line.find("plane_d:") != std::string::npos) {

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

      // Get leading whitespace
      size_t first_non_space = line.find_first_not_of(" \t");
      std::string indent = (first_non_space != std::string::npos)
                               ? line.substr(0, first_non_space)
                               : "";

      // Replace with new value from config
      if (config[key]) {
        line = indent + key + ": " + std::to_string(config[key].as<float>());
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
  }

  // Write back to file
  std::ofstream outfile(config_file);
  for (const auto &l : lines) {
    outfile << l << "\n";
  }
  outfile.close();
}
