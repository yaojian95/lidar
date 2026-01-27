#include "ore_analysis.h"
#include "utils.h"
#include <iostream>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
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
  float ground_threshold =
      Config::get<float>(config, "ground_threshold", 0.02f);

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
  float ground_filter_sigma =
      Config::get<float>(config, "ground_filter_sigma", 6.0f);
  float ground_filter_margin =
      Config::get<float>(config, "ground_filter_margin", 1.0f);

  OreAnalyzer analyzer;
  analyzer.setPointCloud(cloud);
  analyzer.setConfig(unit_scale, ground_threshold);
  analyzer.setGroundThresholdParams(ground_filter_sigma, ground_filter_margin);

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

      // Also save the calculated ground threshold
      config["ground_threshold"] = analyzer.getGroundThreshold();

      saveConfigWithComments("E:/multi_source_info/lidar/config.yaml", config);
    }
  }

  if (aligned) {
    std::cout << "Ground aligned successfully." << std::endl;
    // Refresh threshold (critical if updated by analyzer)
    ground_threshold = analyzer.getGroundThreshold();
    std::cout << "Using ground_threshold for filtering: " << ground_threshold
              << std::endl;

    // Side Panel Calibration and Filtering
    // Known height = 17 cm = 0.17 m
    float known_panel_height = 0.17f;
    float new_scale = 1.0f;

    // Check config for behavior
    bool fit_belt_edges = Config::get<bool>(config, "fit_belt_edges", false);
    float belt_min_y = Config::get<float>(config, "belt_min_y", -1e9f);
    float belt_max_y = Config::get<float>(config, "belt_max_y", 1e9f);

    if (fit_belt_edges) {
      std::cout << "Detecting side panels for calibration (Known Height: "
                << known_panel_height << "m)..." << std::endl;
      if (analyzer.calibrateAndFilterSidePanels(known_panel_height,
                                                new_scale)) {
        std::cout << "Calibration Successful. New Unit Scale: " << new_scale
                  << std::endl;

        // Update Config
        if (std::abs(new_scale - unit_scale) > 0.0001f) {
          std::cout << "Updating config with new unit_scale." << std::endl;
          config["unit_scale"] = new_scale;
          // Also need to set it in analyzer for subsequent conversions
          analyzer.setConfig(new_scale, ground_threshold);
          unit_scale = new_scale;
        }

        // Save Belt Boundaries
        config["belt_min_y"] = analyzer.getBeltMinY();
        config["belt_max_y"] = analyzer.getBeltMaxY();
        std::cout << "Saving Belt Boundaries to config: "
                  << analyzer.getBeltMinY() << " to " << analyzer.getBeltMaxY()
                  << std::endl;

        // Always save if calibration was successful (to persist scale and belt)
        saveConfigWithComments("E:/multi_source_info/lidar/config.yaml",
                               config);
      } else {
        std::cout
            << "Side panel calibration failed or panels not found. Keeping "
               "original scale."
            << std::endl;
      }
    } else {
      std::cout << "Skipping calibration. Filtering using configured belt "
                   "boundaries: "
                << belt_min_y << " to " << belt_max_y << std::endl;
      analyzer.setBeltBoundaries(belt_min_y, belt_max_y);
      analyzer.filterSidePanels();
    }

    // Filter Ground Points (New Step)
    analyzer.filterGroundPoints();

    // Auto-detection (LiDAR Clustering)
    float cluster_tolerance =
        Config::get<float>(config, "cluster_tolerance", 0.05f);
    int min_cluster_size = Config::get<int>(config, "min_cluster_size", 50);
    int max_cluster_size = Config::get<int>(config, "max_cluster_size", 50000);

    std::cout << "Detecting ores with tolerance=" << cluster_tolerance
              << ", min_size=" << min_cluster_size
              << ", max_size=" << max_cluster_size << "..." << std::endl;

    auto ores =
        analyzer.detectByLidar(cluster_tolerance, min_cluster_size, max_cluster_size);
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
  // Visualization logic follows directly, no need to filter again

  // Center cloud for visualization
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  std::cout << "Point Cloud Quantized Centroid: " << centroid[0] << ", "
            << centroid[1] << ", " << centroid[2] << std::endl;

  // Shift to center (X, Y) but keep Z (ground plane at 0)
  Eigen::Matrix4f transform_centering = Eigen::Matrix4f::Identity();
  transform_centering(0, 3) = -centroid[0];
  transform_centering(1, 3) = -centroid[1];
  // Z remains 0

  pcl::transformPointCloud(*cloud, *cloud, transform_centering);

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
    // Calcluate bounds for ground plane and grid
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    // Add some margin
    double margin = 5.0;
    double min_x = min_pt.x - margin;
    double max_x = max_pt.x + margin;
    double min_y = min_pt.y - margin;
    double max_y = max_pt.y + margin;

    // Draw Ground as a thin cube (visualized as plane)
    viewer->addCube(min_x, max_x, min_y, max_y, -0.1, 0.0, 1.0, 1.0, 1.0,
                    "ground_plane");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "ground_plane");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0,
        "ground_plane"); // White

    // // Draw Grid (Scales)
    // double step = 10.0; // Grid every 10 units
    // int grid_id = 0;

    // // X-lines (varying Y)
    // double start_x = std::floor(min_x / step) * step;
    // for (double x = start_x; x <= max_x; x += step) {
    //   pcl::PointXYZ p1(x, min_y, 0.0);
    //   pcl::PointXYZ p2(x, max_y, 0.0);
    //   std::string id = "grid_x_" + std::to_string(grid_id++);
    //   viewer->addLine(p1, p2, 0.5, 0.5, 0.5, id);
    // }

    // // Y-lines (varying X)
    // double start_y = std::floor(min_y / step) * step;
    // for (double y = start_y; y <= max_y; y += step) {
    //   pcl::PointXYZ p1(min_x, y, 0.0);
    //   pcl::PointXYZ p2(max_x, y, 0.0);
    //   std::string id = "grid_y_" + std::to_string(grid_id++);
    //   viewer->addLine(p1, p2, 0.5, 0.5, 0.5, id);
    // }

    // std::cout << "Ground plane visualized (white transparent) with grid scale
    // ("
    //           << step << " units)." << std::endl;
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
