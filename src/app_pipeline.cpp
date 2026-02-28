#include "app_pipeline.h"
#include "ore_analysis.h"
#include <iostream>
#include <opencv2/core/utils/logger.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

namespace AppPipeline {

pcl::PointCloud<pcl::PointXYZ>::Ptr
loadPointCloud(const std::string &pcd_path) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcd_path.empty()) {
    PCL_ERROR("Could not find pcd_path in config.yaml\n");
    return nullptr;
  }

  std::cout << "Loading file: " << pcd_path << std::endl;
  std::string ext = pcd_path.substr(pcd_path.find_last_of(".") + 1);

  if (ext == "pcd") {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
      PCL_ERROR("Couldn't read PCD file\n");
      return nullptr;
    }
  } else if (ext == "ply") {
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
      PCL_ERROR("Couldn't read PLY file\n");
      return nullptr;
    }
  } else {
    PCL_ERROR("Unsupported file extension: %s\n", ext.c_str());
    return nullptr;
  }

  std::cout << "Points: " << cloud->size() << std::endl;
  return cloud;
}

bool processCalibrationAndGround(OreAnalyzer &analyzer, AppConfig &appConfig,
                                 YAML::Node &raw_yaml) {
  std::cout << "Aligning to ground..." << std::endl;
  PlaneCoefficients cached_plane = PlaneCoefficients::fromConfig(raw_yaml);
  bool aligned = false;

  if (cached_plane.valid && !appConfig.save_plane) {
    std::cout << "Using cached plane equation from config." << std::endl;
    aligned = analyzer.alignToGroundWithPlane(cached_plane.a, cached_plane.b,
                                              cached_plane.c, cached_plane.d);
  } else {
    std::cout << "Detecting ground plane with RANSAC..." << std::endl;
    aligned = analyzer.alignToGround();
    if (aligned && appConfig.save_plane) {
      float a, b, c, d;
      analyzer.getPlaneCoefficients(a, b, c, d);
      PlaneCoefficients plane{a, b, c, d, true};
      plane.saveToConfig(raw_yaml);
      raw_yaml["ground_threshold"] = analyzer.getGroundThreshold();
      saveConfigWithComments("E:/multi_source_info/lidar/config.yaml",
                             raw_yaml);
    }
  }

  if (aligned) {
    std::cout << "Ground aligned successfully." << std::endl;
    appConfig.ground_threshold = analyzer.getGroundThreshold();
    std::cout << "Using ground_threshold for filtering: "
              << appConfig.ground_threshold << std::endl;

    float known_panel_height = 170.0f;
    float new_scale = 1.0f;

    if (appConfig.fit_belt_edges) {
      std::cout << "Detecting side panels for calibration (Known Height: "
                << known_panel_height << "m)..." << std::endl;
      if (analyzer.calibrateAndFilterSidePanels(known_panel_height,
                                                new_scale)) {
        std::cout << "Calibration Successful. New Unit Scale: " << new_scale
                  << std::endl;
        if (std::abs(new_scale - appConfig.unit_scale) > 0.0001f) {
          std::cout << "Updating config with new unit_scale." << std::endl;
          raw_yaml["unit_scale"] = new_scale;
          appConfig.unit_scale = new_scale;
          analyzer.setConfig(new_scale, appConfig.ground_threshold);
        }

        appConfig.belt_min_y = analyzer.getBeltMinY();
        appConfig.belt_max_y = analyzer.getBeltMaxY();
        raw_yaml["belt_min_y"] = appConfig.belt_min_y;
        raw_yaml["belt_max_y"] = appConfig.belt_max_y;

        std::cout << "Saving Belt Boundaries to config: "
                  << appConfig.belt_min_y << " to " << appConfig.belt_max_y
                  << std::endl;
        saveConfigWithComments("E:/multi_source_info/lidar/config.yaml",
                               raw_yaml);
      } else {
        std::cout << "Side panel calibration failed. Keeping original scale."
                  << std::endl;
      }
    } else {
      std::cout << "Skipping calibration. Filtering using configured belt "
                   "boundaries: "
                << appConfig.belt_min_y << " to " << appConfig.belt_max_y
                << std::endl;

      // Calculate and output belt points resolution
      float physical_width =
          std::abs(appConfig.belt_max_y - appConfig.belt_min_y);
      if (physical_width > 0.0f) {
        // Estimate the number of points in a single scanline (cross-belt
        // direction) Since typical lidar sweeps form a profile line, we look
        // for the first Y "wraparound"
        size_t points_per_line = analyzer.getAlignedCloud()->size(); // fallback
        if (points_per_line > 1) {
          for (size_t i = 1; i < analyzer.getAlignedCloud()->size(); ++i) {
            float dy = std::abs(analyzer.getAlignedCloud()->points[i].y -
                                analyzer.getAlignedCloud()->points[i - 1].y);
            // A massive jump in Y typically implies completing one line and
            // returning to the start for the next
            if (dy > physical_width * 0.5f) {
              points_per_line = i;
              break;
            }
          }
        }

        if (points_per_line > 0) {
          // Resolution is Physical Distance / Point Count -> mm/point
          float resolution =
              physical_width / static_cast<float>(points_per_line);
          std::cout << "Belt Cloud Resolution: " << resolution
                    << " mm/point (Width: " << physical_width
                    << "mm, Points per line: " << points_per_line << ")"
                    << std::endl;
        }
      }

      analyzer.setBeltBoundaries(appConfig.belt_min_y, appConfig.belt_max_y);
      analyzer.filterSidePanels();
    }

    analyzer.filterGroundPoints();
  } else {
    std::cout << "Failed to align ground, showing original cloud." << std::endl;
  }
  return aligned;
}

void executeDetectionAndFusion(OreAnalyzer &analyzer,
                               const AppConfig &appConfig) {
  std::cout << "Detecting ores with tolerance=" << appConfig.cluster_tolerance
            << ", min_size=" << appConfig.min_cluster_size
            << ", max_size=" << appConfig.max_cluster_size << "..."
            << std::endl;

  analyzer.setHybridClusteringConfig(
      appConfig.cluster_strategy, appConfig.aspect_ratio_threshold,
      appConfig.density_threshold, appConfig.rg_smoothness,
      appConfig.rg_curvature);

  auto ores = analyzer.detectByLidar(appConfig.cluster_tolerance,
                                     appConfig.min_cluster_size,
                                     appConfig.max_cluster_size);
  std::cout << "Found " << ores.size() << " potential ore chunks." << std::endl;

  for (auto &ore : ores) {
    analyzer.computeStats(ore, false);
    size_t point_count =
        ore.point_indices ? ore.point_indices->indices.size() : 0;
    std::cout << "Ore " << ore.id << " [" << point_count
              << " pts]: Avg Thickness = " << ore.avg_thickness
              << " (Max: " << ore.max_thickness << ")" << std::endl;
  }

  std::cout << "Generating Global Thickness Map..." << std::endl;
  auto thickness_map =
      analyzer.generateGlobalThicknessMap(ores, appConfig.unit_scale);
  std::string map_file =
      "E:/multi_source_info/lidar/pcd_data/thickness_map_strategy_" +
      std::to_string(appConfig.cluster_strategy) + ".png";

  std::string fuse_mode = appConfig.fuse_mode;
  std::transform(fuse_mode.begin(), fuse_mode.end(), fuse_mode.begin(),
                 ::tolower);

  std::vector<Ore> *ores_ptr = (fuse_mode == "false") ? &ores : nullptr;

  if (analyzer.saveThicknessMapToImage(thickness_map, map_file, -1.0f,
                                       ores_ptr)) {
    std::cout << "Global Thickness Map saved to: " << map_file << std::endl;
  } else {
    std::cerr << "Failed to save thickness map!" << std::endl;
  }

  if (fuse_mode == "rgb") {
    OreAnalyzer::FusionCrops rgb_crops{
        appConfig.rgb_crop_up, appConfig.rgb_crop_down, appConfig.rgb_crop_left,
        appConfig.rgb_crop_right};
    OreAnalyzer::FusionCrops lidar_crops{
        appConfig.lidar_crop_up, appConfig.lidar_crop_down,
        appConfig.lidar_crop_left, appConfig.lidar_crop_right};
    std::string rgb_path =
        "E:/multi_source_info/lidar/pcd_data/stitched_ore.jpg";
    std::string fused_path =
        "E:/multi_source_info/lidar/pcd_data/fused_thickness_strategy_" +
        std::to_string(appConfig.cluster_strategy) + ".jpg";

    std::cout << "Fusing thickness map with RGB image..." << std::endl;
    if (analyzer.fuseThicknessWithImage(thickness_map, rgb_path, fused_path,
                                        appConfig.fusion_channel, rgb_crops,
                                        lidar_crops, &ores)) {
      std::cout << "Fused image saved to: " << fused_path << std::endl;
    } else {
      std::cerr << "Failed to fuse thickness map with RGB image." << std::endl;
    }
  } else if (fuse_mode == "xray") {
    if (!appConfig.xray_path.empty()) {
      OreAnalyzer::FusionCrops xray_crops{
          appConfig.xray_crop_up, appConfig.xray_crop_down,
          appConfig.xray_crop_left, appConfig.xray_crop_right};
      OreAnalyzer::FusionCrops lidar_crops{
          appConfig.lidar_crop_up, appConfig.lidar_crop_down,
          appConfig.lidar_crop_left, appConfig.lidar_crop_right};
      std::string fused_xray_path =
          "E:/multi_source_info/lidar/pcd_data/fused_thickness_xray_strategy_" +
          std::to_string(appConfig.cluster_strategy) + ".jpg";

      std::cout << "Fusing thickness map with X-ray image ("
                << appConfig.xray_path << ")..." << std::endl;
      if (analyzer.fuseThicknessWithXray(
              thickness_map, appConfig.xray_path, fused_xray_path,
              appConfig.xray_cut_left, appConfig.xray_cut_right,
              appConfig.enable_xray_geometry_correction, appConfig.xray_sod,
              appConfig.xray_sdd, xray_crops, lidar_crops, &ores)) {
        std::cout << "Fused X-ray image saved to: " << fused_xray_path
                  << std::endl;
      } else {
        std::cerr << "Failed to fuse thickness map with X-ray image."
                  << std::endl;
      }
    } else {
      std::cerr << "Error: X-ray path is empty but fuse mode is 'xray'."
                << std::endl;
    }
  } else {
    std::cout << "Fusion disabled (mode: " << fuse_mode << ")." << std::endl;
  }
}

void visualizeResults(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                      bool visual_plane, bool aligned) {
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  std::cout << "Point Cloud Quantized Centroid: " << centroid[0] << ", "
            << centroid[1] << ", " << centroid[2] << std::endl;

  Eigen::Matrix4f transform_centering = Eigen::Matrix4f::Identity();
  transform_centering(0, 3) = -centroid[0];
  transform_centering(1, 3) = -centroid[1];

  pcl::transformPointCloud(*cloud, *cloud, transform_centering);

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("PCD Viewer"));
  viewer->setBackgroundColor(0, 0, 0);

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> z_color(
      cloud, "z");
  viewer->addPointCloud<pcl::PointXYZ>(cloud, z_color, "cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

  if (visual_plane && aligned) {
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    double margin = 5.0;
    viewer->addCube(min_pt.x - margin, max_pt.x + margin, min_pt.y - margin,
                    max_pt.y + margin, -0.1, 0.0, 1.0, 1.0, 1.0,
                    "ground_plane");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "ground_plane");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0,
        "ground_plane");
  }

  viewer->resetCamera();
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

} // namespace AppPipeline
