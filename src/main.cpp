#include "ore_analysis.h"
#include "utils.h"
#include <iostream>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

int main() {
  // Load configuration
  YAML::Node config = Config::load("E:/multi_source_info/lidar/config.yaml");
  if (!config) {
    std::cerr << "Failed to load config file!" << std::endl;
    return -1;
  }

  std::string pcd_path = Config::get<std::string>(config, "pcd_path", "");
  float unit_scale = Config::get<float>(config, "unit_scale", 1.0f);
  bool save_plane = Config::get<bool>(config, "save_plane_equation", false);
  bool visual_plane = Config::get<bool>(config, "visual_plane", true);

  if (pcd_path.empty()) {
    PCL_ERROR("Could not find pcd_path in config.yaml\n");
    return -1;
  }

  std::cout << "Loading file: " << pcd_path << std::endl;
  std::cout << "Unit Scale: " << unit_scale << std::endl;

  // Load point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string ext = pcd_path.substr(pcd_path.find_last_of(".") + 1);

  if (ext == "pcd") {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
      PCL_ERROR("Couldn't read PCD file\n");
      return -1;
    }
  } else if (ext == "ply") {
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
      PCL_ERROR("Couldn't read PLY file\n");
      return -1;
    }
  } else {
    PCL_ERROR("Unsupported file extension: %s\n", ext.c_str());
    return -1;
  }

  std::cout << "Points: " << cloud->size() << std::endl;

  // Ore Analysis
  OreAnalyzer analyzer;
  analyzer.setPointCloud(cloud);
  analyzer.setConfig(unit_scale);

  std::cout << "Aligning to ground..." << std::endl;

  // Try to use cached plane first
  PlaneCoefficients cached_plane = PlaneCoefficients::fromConfig(config);
  bool aligned = false;

  if (cached_plane.valid && !save_plane) {
    std::cout << "Using cached plane equation from config." << std::endl;
    aligned = analyzer.alignToGroundWithPlane(cached_plane.a, cached_plane.b,
                                              cached_plane.c, cached_plane.d);
  } else {
    std::cout << "Detecting ground plane with RANSAC..." << std::endl;
    aligned = analyzer.alignToGround();

    if (aligned && save_plane) {
      float a, b, c, d;
      analyzer.getPlaneCoefficients(a, b, c, d);

      PlaneCoefficients plane{a, b, c, d, true};
      plane.saveToConfig(config);
      saveConfigWithComments("E:/multi_source_info/lidar/config.yaml", config);
    }
  }

  if (aligned) {
    std::cout << "Ground aligned successfully." << std::endl;

    // Auto-detection (LiDAR Clustering)
    std::cout << "Detecting ores..." << std::endl;
    auto ores = analyzer.detectByLidar();
    std::cout << "Found " << ores.size() << " potential ore chunks."
              << std::endl;

    for (auto &ore : ores) {
      analyzer.computeStats(ore, false);
      std::cout << "Ore " << ore.id << ": Avg Thickness = " << ore.avg_thickness
                << " (Max: " << ore.max_thickness << ")" << std::endl;
    }

    cloud = analyzer.getAlignedCloud();
  } else {
    std::cout << "Failed to align ground, showing original cloud." << std::endl;
  }

  // Visualization
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("PCD Viewer"));

  viewer->setBackgroundColor(0, 0, 0);

  // Color points by height (Z)
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> z_color(
      cloud, "z");
  viewer->addPointCloud<pcl::PointXYZ>(cloud, z_color, "cloud");

  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

  // Visualize ground plane if requested
  if (visual_plane && aligned) {
    pcl::ModelCoefficients plane_coeff;
    plane_coeff.values.resize(4);
    plane_coeff.values[0] = 0.0;
    plane_coeff.values[1] = 0.0;
    plane_coeff.values[2] = 1.0;
    plane_coeff.values[3] = 0.0;

    viewer->addPlane(plane_coeff, "ground_plane");
    std::cout << "Ground plane visualized (green transparent plane at Z=0)."
              << std::endl;
  }

  viewer->resetCamera();
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
