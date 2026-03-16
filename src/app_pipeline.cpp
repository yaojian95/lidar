#include "app_pipeline.h"
#include "ore_analysis.h"
#include <iostream>
#include <numeric>
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

#include <direct.h>

// Helper: Save per-ore patches (thickness TIFF, mask, low/high XRT PNGs)
static void saveOrePatches(const OreAnalyzer::ThicknessMap &ores_map,
                           const std::vector<cv::Mat> &ore_masks,
                           const std::vector<Ore> &ores, const cv::Mat &xrt_low,
                           const cv::Mat &xrt_high,
                           const std::string &output_dir) {

  if (ores_map.data.empty() || ores_map.width <= 0 || ores_map.height <= 0)
    return;

  // Ensure output directory exists (Windows)
  _mkdir(output_dir.c_str());

  // Convert ThicknessMap to cv::Mat (float32)
  cv::Mat thickness_mat(ores_map.height, ores_map.width, CV_32FC1,
                        const_cast<float *>(ores_map.data.data()));

  for (size_t i = 0; i < ores.size() && i < ore_masks.size(); ++i) {
    const cv::Mat &mask = ore_masks[i];
    if (mask.empty())
      continue;

    // Find bounding rect of the mask
    cv::Rect bbox = cv::boundingRect(mask);
    if (bbox.width <= 0 || bbox.height <= 0)
      continue;

    // Clamp bbox to image dimensions
    bbox &= cv::Rect(0, 0, mask.cols, mask.rows);
    if (bbox.width <= 0 || bbox.height <= 0)
      continue;

    std::string prefix = output_dir + "/" + ores[i].id;

    // 1. Mask patch (8-bit)
    cv::Mat mask_patch = mask(bbox).clone();
    cv::imwrite(prefix + "_mask.png", mask_patch);

    // 2. Thickness patch (float32 TIFF)
    //    Apply mask within bbox: non-ore pixels = 0
    if (bbox.x + bbox.width <= thickness_mat.cols &&
        bbox.y + bbox.height <= thickness_mat.rows) {
      cv::Mat thick_patch = thickness_mat(bbox).clone();
      cv::Mat mask_roi = mask(bbox);
      // Zero out pixels outside the mask
      for (int r = 0; r < thick_patch.rows; ++r) {
        for (int c = 0; c < thick_patch.cols; ++c) {
          if (mask_roi.at<uchar>(r, c) == 0) {
            thick_patch.at<float>(r, c) = 0.0f;
          }
        }
      }
      cv::imwrite(prefix + "_thickness.tif", thick_patch);
    }

    // 3. Low-energy XRT patch
    if (!xrt_low.empty() && bbox.x + bbox.width <= xrt_low.cols &&
        bbox.y + bbox.height <= xrt_low.rows) {
      cv::Mat low_patch = xrt_low(bbox).clone();
      cv::imwrite(prefix + "_xrt_low.png", low_patch);
    }

    // 4. High-energy XRT patch
    if (!xrt_high.empty() && bbox.x + bbox.width <= xrt_high.cols &&
        bbox.y + bbox.height <= xrt_high.rows) {
      cv::Mat high_patch = xrt_high(bbox).clone();
      cv::imwrite(prefix + "_xrt_high.png", high_patch);
    }

    std::cout << "Saved patches for " << ores[i].id << " (" << bbox.width << "x"
              << bbox.height << ")" << std::endl;
  }
}

void executeDetectionAndFusion(OreAnalyzer &analyzer,
                               const AppConfig &appConfig) {
  std::vector<Ore> ores;
  std::vector<cv::Mat> ore_masks;
  std::string fuse_mode = appConfig.fuse_mode;
  std::transform(fuse_mode.begin(), fuse_mode.end(), fuse_mode.begin(),
                 ::tolower);

  cv::Mat fusion_target_image;
  cv::Mat fusion_target_image_high;

  if (fuse_mode == "rgb") {
    std::string rgb_path =
        "E:/multi_source_info/lidar/pcd_data/stitched_ore.jpg";
    cv::Mat raw_img = cv::imread(rgb_path);
    if (!raw_img.empty()) {
      cv::Rect roi(
          appConfig.rgb_crop_left, appConfig.rgb_crop_up,
          raw_img.cols - appConfig.rgb_crop_left - appConfig.rgb_crop_right,
          raw_img.rows - appConfig.rgb_crop_up - appConfig.rgb_crop_down);
      if (roi.width > 0 && roi.height > 0 && roi.x >= 0 && roi.y >= 0 &&
          roi.x + roi.width <= raw_img.cols &&
          roi.y + roi.height <= raw_img.rows) {
        fusion_target_image = raw_img(roi).clone();
        std::cout << "Prepared RGB Image (" << fusion_target_image.cols << "x"
                  << fusion_target_image.rows << ") from crops." << std::endl;
      } else {
        std::cerr << "Error: Invalid RGB crops resulted in empty image."
                  << std::endl;
      }
    } else {
      std::cerr << "Error: Could not read RGB image from " << rgb_path
                << std::endl;
    }
  } else if (fuse_mode == "xray" && !appConfig.xray_path.empty()) {
    cv::Mat raw_img = cv::imread(appConfig.xray_path, cv::IMREAD_GRAYSCALE);
    if (!raw_img.empty()) {
      int mid = raw_img.cols / 2;
      if (mid > 0) {
        cv::Mat low_energy = raw_img(cv::Rect(0, 0, mid, raw_img.rows));
        cv::Mat high_energy =
            raw_img(cv::Rect(mid, 0, raw_img.cols - mid, raw_img.rows));
        cv::flip(low_energy, low_energy, 1);
        cv::flip(high_energy, high_energy, 1);
        int new_width =
            mid - appConfig.xray_cut_left - appConfig.xray_cut_right;
        if (new_width > 0 && appConfig.xray_cut_left >= 0 &&
            appConfig.xray_cut_left + new_width <= low_energy.cols) {
          cv::Mat sliced = low_energy(
              cv::Rect(appConfig.xray_cut_left, 0, new_width, low_energy.rows));
          cv::Mat sliced_high = high_energy(cv::Rect(
              appConfig.xray_cut_left, 0, new_width, high_energy.rows));
          cv::Rect roi(appConfig.xray_crop_left, appConfig.xray_crop_up,
                       sliced.cols - appConfig.xray_crop_left -
                           appConfig.xray_crop_right,
                       sliced.rows - appConfig.xray_crop_up -
                           appConfig.xray_crop_down);
          if (roi.width > 0 && roi.height > 0 && roi.x >= 0 && roi.y >= 0 &&
              roi.x + roi.width <= sliced.cols &&
              roi.y + roi.height <= sliced.rows) {
            cv::Mat cropped = sliced(roi);
            cv::Mat cropped_high = sliced_high(roi);
            if (appConfig.enable_xray_geometry_correction) {
              bool low_corr = Utils::correctXrayGeometry(
                  cropped, fusion_target_image, appConfig.xray_sod,
                  appConfig.xray_sdd);
              bool high_corr = Utils::correctXrayGeometry(
                  cropped_high, fusion_target_image_high, appConfig.xray_sod,
                  appConfig.xray_sdd);
              if (low_corr && high_corr) {
                std::cout << "Applied X-ray Geometry Correction to both images."
                          << std::endl;
              } else {
                fusion_target_image = cropped.clone();
                fusion_target_image_high = cropped_high.clone();
              }
            } else {
              fusion_target_image = cropped.clone();
              fusion_target_image_high = cropped_high.clone();
            }
            std::cout << "Prepared X-ray Image (" << fusion_target_image.cols
                      << "x" << fusion_target_image.rows << ") from crops."
                      << std::endl;

            // Output Low and High-Energy images for fusion reference
            if (cv::imwrite(appConfig.results_dir +
                                "/"
                                "01_cropped_xray_for_fusion_low.jpg",
                            fusion_target_image) &&
                cv::imwrite(appConfig.results_dir +
                                "/"
                                "02_cropped_xray_for_fusion_high.jpg",
                            fusion_target_image_high)) {
              std::cout << "Saved pre-cropped X-ray images." << std::endl;
            }
          }
        } else {
          std::cerr << "Error: Invalid X-ray cuts." << std::endl;
        }

        // Move the contour drawing and saving logic here, directly after we
        // have fusion_target_images and ore_masks
        if (!ore_masks.empty()) {
          cv::Mat overlay_low = fusion_target_image.clone();
          if (overlay_low.channels() == 1)
            cv::cvtColor(overlay_low, overlay_low, cv::COLOR_GRAY2BGR);

          cv::Mat overlay_high;
          if (!fusion_target_image_high.empty()) {
            overlay_high = fusion_target_image_high.clone();
            if (overlay_high.channels() == 1)
              cv::cvtColor(overlay_high, overlay_high, cv::COLOR_GRAY2BGR);
          }

          for (size_t i = 0; i < ore_masks.size(); ++i) {
            // Find contours from mask for drawing
            std::vector<std::vector<cv::Point>> contour_pts;
            cv::findContours(ore_masks[i].clone(), contour_pts,
                             cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            cv::drawContours(overlay_low, contour_pts, -1,
                             cv::Scalar(0, 255, 0), 2);
            if (!overlay_high.empty())
              cv::drawContours(overlay_high, contour_pts, -1,
                               cv::Scalar(0, 0, 255), 2);

            // Label with ore ID at centroid
            cv::Rect bbox = cv::boundingRect(ore_masks[i]);
            cv::Point label_pos(bbox.x + bbox.width / 2 - 10,
                                bbox.y + bbox.height / 2 + 5);
            std::string label = std::to_string(i);
            cv::putText(overlay_low, label, label_pos, cv::FONT_HERSHEY_SIMPLEX,
                        0.6, cv::Scalar(0, 255, 255), 2);
            if (!overlay_high.empty())
              cv::putText(overlay_high, label, label_pos,
                          cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cv::Scalar(0, 255, 255), 2);
          }
        }
      }
    } else {
      std::cerr << "Error: Could not read X-ray image from "
                << appConfig.xray_path << std::endl;
    }
  }

  // 1. Generate Global Thickness Map FIRST (so we have a 1:1 map to work with
  // for image masks) We pass an empty ores vector and `use_crops=true` to
  // instruct generateGlobalThicknessMap to build it using strictly the point
  // cloud area that matches the camera fusion ROI.

  OreAnalyzer::FusionCrops lidar_crops{
      appConfig.lidar_crop_up, appConfig.lidar_crop_down,
      appConfig.lidar_crop_left, appConfig.lidar_crop_right};

  std::cout << "Generating Cropped Global Thickness Map (Resolution: "
            << appConfig.thickness_map_resolution << "m)..." << std::endl;
  auto thickness_map = analyzer.generateGlobalThicknessMap(
      std::vector<Ore>{}, appConfig.thickness_map_resolution, true,
      lidar_crops);

  std::string strategy_suffix =
      (appConfig.detection_mode == "lidar")
          ? "_strategy_" + std::to_string(appConfig.cluster_strategy)
          : "_byMask";

  if (appConfig.detection_mode == "lidar") {
    std::cout << "Detecting ores with LiDAR clustering (tolerance="
              << appConfig.cluster_tolerance
              << ", min_size=" << appConfig.min_cluster_size
              << ", max_size=" << appConfig.max_cluster_size << ")..."
              << std::endl;

    analyzer.setHybridClusteringConfig(
        appConfig.cluster_strategy, appConfig.aspect_ratio_threshold,
        appConfig.density_threshold, appConfig.rg_smoothness,
        appConfig.rg_curvature);

    ores = analyzer.detectByLidar(appConfig.cluster_tolerance,
                                  appConfig.min_cluster_size,
                                  appConfig.max_cluster_size);
  } else if (appConfig.detection_mode == "mask" ||
             appConfig.detection_mode == "roi") {
    if (fuse_mode == "false") {
      std::cerr << "Error: detection_mode " << appConfig.detection_mode
                << " requires fuse_mode to be enabled ('rgb' or 'xray')."
                << std::endl;
      return;
    }

    if (fusion_target_image.empty()) {
      std::cerr << "Error: Could not prepare image for detection. Checking "
                   "config paths and crops."
                << std::endl;
      return;
    }

    std::cout << "Extracting ores from prepared image for "
              << appConfig.detection_mode << " detection..." << std::endl;

    std::cout
        << "Extracting contours and mapping them directly to Thickness Map..."
        << std::endl;

    auto contours = analyzer.extractOresFromImage(
        fusion_target_image, thickness_map, lidar_crops, appConfig.unit_scale,
        fusion_target_image_high);

    std::cout << "Extracted " << contours.size() << " contours from image."
              << std::endl;

    int ore_id = 0;
    for (const auto &contour : contours) {
      Ore ore;
      if (appConfig.detection_mode == "mask") {
        ore = analyzer.detectByMask(contour.mask, thickness_map);
      } else { // roi
        std::cerr << "roi detection mode is temporarily deprecated in Method 2 "
                     "refactor."
                  << std::endl;
      }

      if (ore.point_indices && !ore.point_indices->indices.empty()) {
        ores.push_back(ore);
        ore_masks.push_back(contour.mask.clone());
      }
    }
  } else {
    std::cerr << "Unknown detection_mode: " << appConfig.detection_mode
              << std::endl;
  }

  // =====================================================================
  // Stage 1: RAW - save original grid BEFORE any resize/alignment
  // =====================================================================
  if (thickness_map.width > 0 && thickness_map.height > 0) {
    cv::Mat raw_map =
        cv::Mat(thickness_map.height, thickness_map.width, CV_32FC1,
                const_cast<float *>(thickness_map.data.data()))
            .clone();
    cv::Mat raw_map_t;
    cv::transpose(raw_map, raw_map_t);
    cv::imwrite(appConfig.results_dir + "/05_raw_thickness_map.tif", raw_map_t);
    std::cout << "Stage 1 (Raw): saved (" << raw_map_t.cols << "x"
              << raw_map_t.rows << ")" << std::endl;
  }

  // Resize Map to perfectly match visual image dimensions
  if (!fusion_target_image.empty()) {
    std::cout << "Aligning Thickness Map to Image Resolution ("
              << fusion_target_image.cols << "x" << fusion_target_image.rows
              << ")" << std::endl;
    analyzer.alignThicknessMapToImage(thickness_map,
                                      fusion_target_image.size());

    // Postponed LiDAR mask generation: now that the map is aligned/transposed/resized,
    // we can project 3D points directly into the final image coordinate space.
    if (appConfig.detection_mode == "lidar" && !ores.empty()) {
      std::cout << "Dynamically generating 2D ore masks from 3D LiDAR points (Aligned Space)..." << std::endl;
      for (const auto& ore : ores) {
        cv::Mat mask = cv::Mat::zeros(thickness_map.height, thickness_map.width, CV_8UC1);
        if (ore.point_indices && !ore.point_indices->indices.empty()) {
          for (int idx : ore.point_indices->indices) {
            const auto& pt = analyzer.getAlignedCloud()->points[idx];
            // Transposed mapping calculation:
            // pt.x -> image rows (Y in visual)
            // pt.y -> image cols (X in visual)
            float rel_x = (pt.x - thickness_map.min_x) / (thickness_map.max_x - thickness_map.min_x);
            float rel_y = (thickness_map.max_y - pt.y) / (thickness_map.max_y - thickness_map.min_y);
            
            int mask_r = static_cast<int>(rel_x * thickness_map.height);
            int mask_c = static_cast<int>(rel_y * thickness_map.width);
            
            if (mask_r >= 0 && mask_r < mask.rows && mask_c >= 0 && mask_c < mask.cols) {
              mask.at<uint8_t>(mask_r, mask_c) = 255;
            }
          }
          cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
          cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
          std::vector<std::vector<cv::Point>> contours;
          cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
          mask = cv::Mat::zeros(mask.size(), CV_8UC1);
          cv::drawContours(mask, contours, -1, cv::Scalar(255), cv::FILLED);
        }
        ore_masks.push_back(mask);
      }
    }
  }

  // Unified Spatial Sorting, ID Assignment and Overlay Generation
  // This must happen AFTER masks are available for all modes.
  if (!ore_masks.empty()) {
    if (ores.size() > 1) {
      std::cout << "Sorting " << ores.size() << " ores spatially..." << std::endl;
      OreAnalyzer::sortOresSpatially(ores, ore_masks);
    }

    // Assign Sequential IDs after sorting
    for (size_t i = 0; i < ores.size(); ++i) {
      ores[i].id = "ore_" + std::to_string(i);
    }

    // Generate and Save Contour Overlay Images
    cv::Mat overlay_low;
    if (!fusion_target_image.empty()) {
      overlay_low = fusion_target_image.clone();
      if (overlay_low.channels() == 1)
        cv::cvtColor(overlay_low, overlay_low, cv::COLOR_GRAY2BGR);
    } else {
      overlay_low = cv::Mat::zeros(thickness_map.height, thickness_map.width, CV_8UC3);
    }

    cv::Mat overlay_high;
    if (!fusion_target_image_high.empty()) {
      overlay_high = fusion_target_image_high.clone();
      if (overlay_high.channels() == 1)
        cv::cvtColor(overlay_high, overlay_high, cv::COLOR_GRAY2BGR);
    }

    for (size_t i = 0; i < ore_masks.size(); ++i) {
      std::vector<std::vector<cv::Point>> contour_pts;
      cv::findContours(ore_masks[i].clone(), contour_pts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      cv::drawContours(overlay_low, contour_pts, -1, cv::Scalar(0, 0, 255), 2);
      if (!overlay_high.empty())
        cv::drawContours(overlay_high, contour_pts, -1, cv::Scalar(0, 0, 255), 2);

      cv::Rect bbox = cv::boundingRect(ore_masks[i]);
      cv::Point label_pos(bbox.x + bbox.width / 2 - 10, bbox.y + bbox.height / 2 + 5);
      cv::putText(overlay_low, std::to_string(i), label_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
      if (!overlay_high.empty())
        cv::putText(overlay_high, std::to_string(i), label_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    }

    cv::imwrite(appConfig.results_dir + "/03_extracted_contours_overlay_low.jpg", overlay_low);
    if (!overlay_high.empty())
      cv::imwrite(appConfig.results_dir + "/04_extracted_contours_overlay_high.jpg", overlay_high);
    std::cout << "Saved unified contour overlays to 03/04_extracted_contours_overlay_low/high.jpg" << std::endl;
  }

  std::cout << "Found " << ores.size() << " potential ore chunks." << std::endl;

  for (auto &ore : ores) {
    analyzer.computeStats(ore, false);
    size_t point_count = ore.point_indices ? ore.point_indices->indices.size() : 0;
    std::cout << "Ore " << ore.id << " [" << point_count
              << " pts]: Avg Thickness = " << ore.avg_thickness
              << " mm (Min: " << ore.min_thickness
              << " mm, Max: " << ore.max_thickness
              << " mm, Std: " << ore.std_thickness << " mm)" << std::endl;
  }

  // =====================================================================
  // Stage 2: RESCALED - thickness map at XRT image dimensions
  //   (already aligned by alignThicknessMapToImage above)
  // =====================================================================
  cv::Mat rescaled_mat;
  if (thickness_map.is_image_aligned && thickness_map.width > 0) {
    rescaled_mat = cv::Mat(thickness_map.height, thickness_map.width, CV_32FC1,
                           const_cast<float *>(thickness_map.data.data()))
                       .clone();
    cv::imwrite(appConfig.results_dir + "/06_rescaled_thickness_map.tif",
                rescaled_mat);
    std::cout << "Stage 2 (Rescaled): saved (" << rescaled_mat.cols << "x"
              << rescaled_mat.rows << ")" << std::endl;
  }
  // =====================================================================
  // Stage 3: ORES - rescaled map masked by XRT ore contours
  // =====================================================================
  OreAnalyzer::ThicknessMap ores_thickness_map = thickness_map;

  if (!ore_masks.empty() && !rescaled_mat.empty()) {
    cv::Mat ores_mat = rescaled_mat.clone();

    // Build combined ore mask
    cv::Mat combined_mask = cv::Mat::zeros(ores_mat.size(), CV_8UC1);
    for (const auto &m : ore_masks) {
      if (m.size() == combined_mask.size()) {
        combined_mask |= m;
      }
    }

    // Zero out everything outside ore regions
    ores_mat.setTo(0.0f, combined_mask == 0);

    // Update the ores_thickness_map structure with the zeroed-out data
    if (ores_mat.isContinuous() &&
        ores_mat.total() == ores_thickness_map.data.size()) {
      std::memcpy(ores_thickness_map.data.data(), ores_mat.ptr<float>(),
                  ores_thickness_map.data.size() * sizeof(float));
    }

    cv::imwrite(appConfig.results_dir + "/07_ores_thickness_map.tif", ores_mat);
    std::cout << "Stage 3 (Ores): saved (" << ores_mat.cols << "x"
              << ores_mat.rows << ")" << std::endl;
  }

  // Save per-ore patches (thickness, mask, low/high XRT)
  if (!ore_masks.empty() && !ores.empty()) {
    std::string patches_dir = appConfig.results_dir + "/ore_patches";
    std::cout << "Saving per-ore patches to " << patches_dir << "..."
              << std::endl;
    saveOrePatches(ores_thickness_map, ore_masks, ores, fusion_target_image,
                   fusion_target_image_high, patches_dir);
  }

  if (fuse_mode == "rgb") {
    std::string fused_path = appConfig.results_dir + "/08_fused_thickness" +
                             strategy_suffix + ".jpg";

    std::cout << "Fusing ores thickness map with RGB image..." << std::endl;
    if (fusion_target_image.empty()) {
      std::cerr
          << "Failed to fuse: RGB image was not properly loaded or cropped."
          << std::endl;
    } else if (analyzer.fuseThicknessWithImage(
                   ores_thickness_map, fusion_target_image, fused_path,
                   appConfig.fusion_channel, lidar_crops, &ores)) {
      std::cout << "Fused image saved to: " << fused_path << std::endl;
    } else {
      std::cerr << "Failed to fuse thickness map with RGB image." << std::endl;
    }
  } else if (fuse_mode == "xray") {
    if (!appConfig.xray_path.empty()) {
      std::string fused_xray_path = appConfig.results_dir +
                                    "/08_fused_thickness_xray" +
                                    strategy_suffix + ".jpg";

      std::cout << "Fusing ores thickness map with X-ray image ("
                << appConfig.xray_path << ")..." << std::endl;

      if (fusion_target_image.empty()) {
        std::cerr << "Failed to fuse: X-ray image was not properly loaded, "
                     "cropped, or corrected."
                  << std::endl;
      } else if (analyzer.fuseThicknessWithXray(
                     ores_thickness_map, fusion_target_image, fused_xray_path,
                     lidar_crops, &ores)) {
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
