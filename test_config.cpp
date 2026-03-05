#include <iostream>
#include <yaml-cpp/yaml.h>

int main() {
  YAML::Node config = YAML::LoadFile("E:/multi_source_info/lidar/config.yaml");
  int left =
      config["lidar_crop_left"] ? config["lidar_crop_left"].as<int>() : 0;
  int right =
      config["lidar_crop_right"] ? config["lidar_crop_right"].as<int>() : 0;

  std::cout << "Read left: " << left << std::endl;
  std::cout << "Read right: " << right << std::endl;
  return 0;
}
