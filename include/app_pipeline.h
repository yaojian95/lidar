#pragma once
#include "utils.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <yaml-cpp/yaml.h>


// Forward declaration
class OreAnalyzer;

namespace AppPipeline {

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const std::string &pcd_path);

bool processCalibrationAndGround(OreAnalyzer &analyzer, AppConfig &appConfig,
                                 YAML::Node &raw_yaml);

void executeDetectionAndFusion(OreAnalyzer &analyzer,
                               const AppConfig &appConfig);

void visualizeResults(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                      bool visual_plane, bool aligned);

} // namespace AppPipeline
