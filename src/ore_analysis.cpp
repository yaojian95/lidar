#include "ore_analysis.h"
#include "utils.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

OreAnalyzer::OreAnalyzer()
    : original_cloud_(new PointCloud), aligned_cloud_(new PointCloud) {}

void OreAnalyzer::setPointCloud(PointCloudPtr cloud) {
  original_cloud_ = cloud;
  // Reset aligned cloud to a copy initially
  pcl::copyPointCloud(*original_cloud_, *aligned_cloud_);
}

// The inline setters are defined in the header file.

void OreAnalyzer::setConfig(float scale, float ground_thresh) {
  unit_scale_ = scale;
  ground_threshold_ = ground_thresh;
}

bool OreAnalyzer::alignToGround() {
  if (aligned_cloud_->empty())
    return false;

  // RANSAC Plane Segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(ground_threshold_);
  seg.setInputCloud(aligned_cloud_);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    std::cerr << "Could not estimate a planar model for the given dataset."
              << std::endl;
    return false;
  }

  std::cout << "Successfully estimated planar model via RANSAC." << std::endl;

  // Delegate the actual matrix transformation to the unified method
  return alignToGroundWithPlane(
      coefficients->values[0], coefficients->values[1], coefficients->values[2],
      coefficients->values[3]);
}

bool OreAnalyzer::alignToGroundWithPlane(float a, float b, float c, float d) {
  if (aligned_cloud_->empty())
    return false;

  // Store coefficients
  plane_a_ = a;
  plane_b_ = b;
  plane_c_ = c;
  plane_d_ = d;

  // Compute rotation to align plane normal to Z axis
  Eigen::Vector3f normal(a, b, c);
  Eigen::Vector3f target_normal(0, 0, 1);

  // Calculate angle between normal and Z-axis
  // dot = |n|*|z|*cos(theta). n and z are vectors.
  float dot = normal.dot(target_normal);
  float norm_n = normal.norm();
  float norm_z = target_normal.norm();
  float cos_theta = dot / (norm_n * norm_z);
  // Clamp to [-1, 1] to avoid NaN
  if (cos_theta > 1.0f)
    cos_theta = 1.0f;
  if (cos_theta < -1.0f)
    cos_theta = -1.0f;
  float angle_rad = std::acos(cos_theta);
  float angle_deg = angle_rad * 180.0f / 3.14159265f;

  std::cout << "Ground Plane Alignment:" << std::endl;
  std::cout << "  Normal: (" << a << ", " << b << ", " << c << ")" << std::endl;
  std::cout << "  Tilt Angle (vs Z-axis): " << angle_deg << " degrees"
            << std::endl;
  if (angle_deg > 10.0f) {
    std::cout << "  [WARNING] Large tilt detected! Check sensor mounting."
              << std::endl;
  }

  Eigen::Quaternionf rotation;
  rotation.setFromTwoVectors(normal, target_normal);

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();

  // Compute translation based on plane equation
  // After rotation, the plane should be at some Z = constant
  // We can use the d coefficient to determine the offset
  float average_z = -d / sqrt(a * a + b * b + c * c); // Distance from origin
  transform(2, 3) = -average_z;

  // Apply full transform
  pcl::transformPointCloud(*aligned_cloud_, *aligned_cloud_, transform);

  std::cout << "Applied plane alignment. Translation Z: " << -average_z
            << std::endl;

  // Check orientation (same heuristic)
  double sum_z_aligned = 0;
  for (const auto &p : *aligned_cloud_)
    sum_z_aligned += p.z;

  std::cout << "Average Z of aligned cloud: "
            << sum_z_aligned / aligned_cloud_->size() << std::endl;
  if (sum_z_aligned < 0) {
    std::cout
        << "Detected inverted Z-axis (objects below ground). Flipping Z..."
        << std::endl;
    Eigen::Matrix4f flip = Eigen::Matrix4f::Identity();
    flip(2, 2) = -1.0f; // Flip Z
    pcl::transformPointCloud(*aligned_cloud_, *aligned_cloud_, flip);
  }
  // Also recalculate threshold for cached plane
  recalculateGroundThreshold();

  return true;
}

void OreAnalyzer::getPlaneCoefficients(float &a, float &b, float &c,
                                       float &d) const {
  a = plane_a_;
  b = plane_b_;
  c = plane_c_;
  d = plane_d_;
}

// =========================================================================
// Hybrid Clustering Diagnostic Methods
// =========================================================================

bool OreAnalyzer::isClusterSuspiciousAspectRatio(
    const pcl::PointCloud<PointT>::Ptr &cluster, float threshold) const {
  if (cluster->empty())
    return false;

  PointT min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);

  float length_x = std::abs(max_pt.x - min_pt.x);
  float length_y = std::abs(max_pt.y - min_pt.y);

  if (length_y < 0.001f || length_x < 0.001f)
    return false;

  float aspect = std::max(length_x / length_y, length_y / length_x);
  return aspect > threshold;
}

bool OreAnalyzer::isClusterSuspiciousConcavity(
    const pcl::PointCloud<PointT>::Ptr &cluster, float threshold) const {
  if (cluster->empty())
    return false;

  PointT min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);

  float length_x = std::abs(max_pt.x - min_pt.x);
  float length_y = std::abs(max_pt.y - min_pt.y);

  // Approximate the 2D bounding box area
  float bbox_area = length_x * length_y;
  if (bbox_area < 0.001f)
    return false;

  // To evaluate concavity, we compare the actual number of points against the
  // expected points if the bounding box was dense. This is a rough heuristic.
  // We assume uniform sampling. A better 2D heuristic: calculate the physical
  // area covered by the points vs bbox area. For simplicity based on points:
  float density_ratio =
      static_cast<float>(cluster->size()) /
      (bbox_area * 10000.0f); // Arbitrary scaling for heuristic
  // Since point count vs area is scale-dependent, a more robust 2D grid
  // approach is better:

  // 2D Grid Occupancy Approach:
  float resolution = 0.01f; // 1cm grid
  int cols = static_cast<int>(length_x / resolution) + 1;
  int rows = static_cast<int>(length_y / resolution) + 1;
  int total_cells = cols * rows;
  if (total_cells <= 0)
    return false;

  std::vector<bool> grid(total_cells, false);
  int occupied_cells = 0;

  for (const auto &p : *cluster) {
    int c = static_cast<int>((p.x - min_pt.x) / resolution);
    int r = static_cast<int>((p.y - min_pt.y) / resolution);
    if (c >= 0 && c < cols && r >= 0 && r < rows) {
      int idx = r * cols + c;
      if (!grid[idx]) {
        grid[idx] = true;
        occupied_cells++;
      }
    }
  }

  float occupancy_ratio =
      static_cast<float>(occupied_cells) / static_cast<float>(total_cells);
  return occupancy_ratio <
         threshold; // Lower occupancy = High concavity (irregular shape)
}

bool OreAnalyzer::isClusterSuspiciousMultiPeak(
    const pcl::PointCloud<PointT>::Ptr &cluster) const {
  if (cluster->points.size() < 100)
    return false;

  PointT min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);

  float length_x = std::abs(max_pt.x - min_pt.x);
  float length_y = std::abs(max_pt.y - min_pt.y);

  // Create a fast 2D height map (Z-map) to find peaks
  float resolution = 0.02f; // 2cm resolution
  int cols = static_cast<int>(length_x / resolution) + 1;
  int rows = static_cast<int>(length_y / resolution) + 1;

  if (cols <= 2 || rows <= 2 || cols > 500 || rows > 500)
    return false; // Too small or too large to reliable peak find

  std::vector<float> height_map(cols * rows, -1e9f);

  // Populate height map
  for (const auto &p : *cluster) {
    int c = static_cast<int>((p.x - min_pt.x) / resolution);
    int r = static_cast<int>((p.y - min_pt.y) / resolution);
    if (c >= 0 && c < cols && r >= 0 && r < rows) {
      int idx = r * cols + c;
      if (p.z > height_map[idx]) {
        height_map[idx] = p.z;
      }
    }
  }

  // Simple non-maximum suppression / local maxima finding
  int peak_count = 0;
  int window = 5; // 10cm window

  for (int r = window; r < rows - window; ++r) {
    for (int c = window; c < cols - window; ++c) {
      int center_idx = r * cols + c;
      float center_val = height_map[center_idx];

      if (center_val <= 0.01f)
        continue; // Ignore low peaks near ground

      bool is_local_max = true;
      for (int wr = -window; wr <= window; ++wr) {
        for (int wc = -window; wc <= window; ++wc) {
          if (wr == 0 && wc == 0)
            continue;
          int neighbor_idx = (r + wr) * cols + (c + wc);
          if (height_map[neighbor_idx] >= center_val) {
            is_local_max = false;
            break;
          }
        }
        if (!is_local_max)
          break;
      }

      if (is_local_max) {
        peak_count++;
        if (peak_count > 1)
          return true; // More than 1 distinct large peak found
        // Skip nearby cells to avoid double counting the same peak
        c += window;
      }
    }
  }
  return false;
}

#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

std::vector<pcl::PointIndices>
OreAnalyzer::applyRegionGrowing(const pcl::PointCloud<PointT>::Ptr &cluster,
                                float smoothness_deg, float curvature_thr,
                                int min_size, int max_size) const {

  std::vector<pcl::PointIndices> sub_clusters;
  if (cluster->empty())
    return sub_clusters;

  // 1. Estimate Normals
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cluster);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cluster);
  normal_estimator.setKSearch(30); // Use 30 neighbors for good normal stability
  normal_estimator.compute(*normals);

  // 2. Region Growing
  pcl::RegionGrowing<PointT, pcl::Normal> reg;
  reg.setMinClusterSize(min_size);
  reg.setMaxClusterSize(max_size);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(30);
  reg.setInputCloud(cluster);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(smoothness_deg / 180.0 *
                             M_PI); // Degrees to Radians
  reg.setCurvatureThreshold(curvature_thr);

  reg.extract(sub_clusters);

  return sub_clusters;
}

// =========================================================================

std::vector<Ore> OreAnalyzer::detectByLidar(float cluster_tolerance,
                                            int min_size, int max_size) {
  std::vector<Ore> ores;

  if (aligned_cloud_->empty())
    return ores;

  // 2. Euclidean Clustering directly on aligned_cloud_
  // Assumes filterGroundPoints() and filterSidePanels() have been called
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(aligned_cloud_);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(aligned_cloud_);
  ec.extract(cluster_indices);

  std::vector<pcl::PointIndices> final_cluster_indices;

  // Evaluate each Euclidean cluster against Hybrid Strategies
  for (const auto &indices : cluster_indices) {
    if (indices.indices.empty())
      continue;

    bool suspicious = false;

    // Fast pass if hybrid strategy is enabled
    if (hybrid_strategy_ > 0) {
      pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
      for (int idx : indices.indices) {
        cluster->points.push_back(aligned_cloud_->points[idx]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;

      if (hybrid_strategy_ == 1 &&
          isClusterSuspiciousAspectRatio(cluster, aspect_ratio_threshold_)) {
        suspicious = true;
      } else if (hybrid_strategy_ == 2 &&
                 isClusterSuspiciousConcavity(cluster, density_threshold_)) {
        suspicious = true;
      } else if (hybrid_strategy_ == 3 &&
                 isClusterSuspiciousMultiPeak(cluster)) {
        suspicious = true;
      }

      if (suspicious) {
        std::cout << "  [Hybrid Clustering] Cluster of size " << cluster->size()
                  << " flagged as suspicious. Applying Region Growing..."
                  << std::endl;

        auto sub_clusters = applyRegionGrowing(
            cluster, rg_smoothness_, rg_curvature_, min_size, max_size);

        if (!sub_clusters.empty()) {
          // Remap sub_cluster indices back to the original aligned_cloud_
          // indices
          for (const auto &sub_c : sub_clusters) {
            pcl::PointIndices mapped_indices;
            for (int sub_idx : sub_c.indices) {
              mapped_indices.indices.push_back(indices.indices[sub_idx]);
            }
            final_cluster_indices.push_back(mapped_indices);
          }
        } else {
          // Fallback if region growing fails to find anything
          final_cluster_indices.push_back(indices);
        }
      } else {
        final_cluster_indices.push_back(indices);
      }
    } else {
      // Strategy 0: Pure Euclidean
      final_cluster_indices.push_back(indices);
    }
  }

  // Helper struct for sorting
  struct ClusterInfo {
    pcl::PointIndices indices;
    float center_x;
  };
  std::vector<ClusterInfo> sorted_clusters;

  for (const auto &indices : final_cluster_indices) {
    if (indices.indices.empty())
      continue;

    // Calculate Centroid X
    float sum_x = 0.0f;
    for (int idx : indices.indices) {
      sum_x += aligned_cloud_->points[idx].x;
    }
    float avg_x = sum_x / indices.indices.size();
    sorted_clusters.push_back({indices, avg_x});
  }

  // Sort by Center X (Ascending)
  std::sort(sorted_clusters.begin(), sorted_clusters.end(),
            [](const ClusterInfo &a, const ClusterInfo &b) {
              return a.center_x < b.center_x;
            });

  int ore_id = 0;
  for (const auto &info : sorted_clusters) {
    Ore ore;
    ore.id = "ore_" + std::to_string(ore_id++);
    ore.point_indices.reset(new pcl::PointIndices);
    ore.point_indices->indices = info.indices.indices;
    ores.push_back(ore);
  }
  return ores;
}

Ore OreAnalyzer::detectByROI(float min_x, float max_x, float min_y,
                             float max_y) {
  Ore ore;
  ore.id = "roi_ore";
  ore.point_indices.reset(new pcl::PointIndices);

  for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
    const auto &pt = aligned_cloud_->points[i];
    if (pt.x >= min_x && pt.x <= max_x && pt.y >= min_y && pt.y <= max_y) {
      if (pt.z > ground_threshold_) { // Still ignore ground
        ore.point_indices->indices.push_back(i);
      }
    }
  }
  return ore;
}

Ore OreAnalyzer::detectByMask(const cv::Mat &mask, const ThicknessMap &map) {
  Ore ore;
  ore.id = "mask_ore";
  ore.point_indices.reset(new pcl::PointIndices);

  if (mask.empty() || map.data.empty())
    return ore;

  // We have the physical bounds of the map (which is perfectly aligned with the
  // crop boundaries). The user requested to map the 3D points directly to the
  // `mask` dimensions (which is the visual image size), rather than resizing
  // the mask down to the map resolution. This means the mask size represents
  // the full range [map.min_x, map.max_x] and [map.max_y, map.min_y].

  // Calculate physical width and height directly from the map's bounds.
  // We can NOT use `map.width * map.resolution` anymore because if this map
  // has passed through `alignThicknessMapToImage`, its `width` and `height`
  // now represent the high-res image pixel dimensions, not the grid cells.
  float map_phys_width = map.max_x - map.min_x;
  float map_phys_height = map.max_y - map.min_y;

  if (map_phys_width <= 0 || map_phys_height <= 0)
    return ore;

  for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
    const auto &pt = aligned_cloud_->points[i];
    if (pt.z <= ground_threshold_)
      continue;

    // Relative position in [0, 1] across the cropped physical region
    float rel_x = (pt.x - map.min_x) / map_phys_width;
    float rel_y = (map.max_y - pt.y) / map_phys_height;

    // Scale to the visual image (mask) pixel coordinates
    // Transposed axes: Image Col (X) = Point Cloud Y
    // Image Row (Y) = Point Cloud X
    int mask_col = static_cast<int>(rel_y * mask.cols);
    int mask_row = static_cast<int>(rel_x * mask.rows);

    if (mask_row >= 0 && mask_row < mask.rows && mask_col >= 0 &&
        mask_col < mask.cols) {
      if (mask.at<uchar>(mask_row, mask_col) > 0) { // Mask hit
        ore.point_indices->indices.push_back(i);
      }
    }
  }
  return ore;
}

std::vector<OreAnalyzer::ImageContour>
OreAnalyzer::extractOresFromImage(const cv::Mat &image, const ThicknessMap &map,
                                  FusionCrops lidar_crops, float unit_scale,
                                  const cv::Mat &high_image) {
  std::vector<ImageContour> results;
  if (image.empty() || map.data.empty())
    return results;

  // The global ThicknessMap is now generated with `use_crops=true`,
  // which means `map` is already cropped and dimensioned identically
  // to the physical boundaries of the fusion region (`lidar_crops`).
  // We just use the raw image contours as the mask directly!
  // `detectByMask` will project the 3D points proportional to the `mask`'s
  // sizes to detect them.

  // 2. Prepare the Image
  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image.clone();
  }

  cv::Mat binary;
  cv::threshold(gray, binary, 185, 255, cv::THRESH_BINARY_INV);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point>> valid_contours;

  // 3. For each valid contour drawn on the image, we extract its mask
  for (const auto &contour : contours) {
    if (cv::contourArea(contour) < 100)
      continue; // Filter noise

    valid_contours.push_back(contour);

    // Create a blank mask the same size as the image, and draw the single
    // contour filled
    cv::Mat single_contour_mask = cv::Mat::zeros(image.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> single_contour_vec = {contour};
    cv::drawContours(single_contour_mask, single_contour_vec, -1,
                     cv::Scalar(255), cv::FILLED);

    // We don't resize the mask anymore. We keep it at image dimensions.
    ImageContour ic;
    ic.mask = single_contour_mask;
    results.push_back(ic);
  }

  // Note: Contour overlay drawing and saving has been moved to app_pipeline.cpp
  // (as 03/04_extracted_contours_overlay) so they can reflect sorted Ore IDs.

  return results;
}

void OreAnalyzer::computeStats(Ore &ore, bool generate_map, float map_res) {
  if (ore.point_indices->indices.empty())
    return;

  // 1. Basic Stats
  float sum_z = 0.0f;
  float max_z = -1e9f;
  float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f;

  // Welford's online algorithm for computing variance in a single pass
  size_t count = 0;
  float mean = 0.0f;
  float M2 = 0.0f;
  float min_z = 1e9f;

  for (int idx : ore.point_indices->indices) {
    float z = aligned_cloud_->points[idx].z;
    const auto &pt = aligned_cloud_->points[idx];

    // Convert to physical units (millimeters)
    float physical_z = z * unit_scale_ * 1000.0f;

    sum_z += physical_z;
    if (physical_z > max_z)
      max_z = physical_z;
    if (physical_z < min_z)
      min_z = physical_z;

    if (pt.x < min_x)
      min_x = pt.x;
    if (pt.x > max_x)
      max_x = pt.x;
    if (pt.y < min_y)
      min_y = pt.y;
    if (pt.y > max_y)
      max_y = pt.y;

    // Welford's update step
    count++;
    float delta = physical_z - mean;
    mean += delta / count;
    float delta2 = physical_z - mean;
    M2 += delta * delta2;
  }

  ore.avg_thickness = count > 0 ? (sum_z / count) : 0.0f;
  ore.max_thickness = max_z;
  ore.min_thickness = (min_z == 1e9f) ? 0.0f : min_z;
  ore.std_thickness = count > 1 ? std::sqrt(M2 / count) : 0.0f;

  ore.min_x = min_x;
  ore.max_x = max_x;
  ore.min_y = min_y;
  ore.max_y = max_y;

  if (!generate_map)
    return;

  // 2. Thickness Map with Interpolation
  ore.map_resolution = map_res;
  ore.map_min_x = min_x;
  ore.map_min_y = min_y;

  int w = std::ceil((max_x - min_x) / map_res);
  int h = std::ceil((max_y - min_y) / map_res);

  // Initialize map with -1 (meaning empty)
  ore.thickness_map.assign(h, std::vector<float>(w, -1.0f));

  // Binning: Average points in each cell first
  std::vector<std::vector<float>> cell_sums(h, std::vector<float>(w, 0.0f));
  std::vector<std::vector<int>> cell_counts(h, std::vector<int>(w, 0));

  for (int idx : ore.point_indices->indices) {
    const auto &pt = aligned_cloud_->points[idx];
    int col = (pt.x - min_x) / map_res;
    int row = (pt.y - min_y) / map_res;
    if (col >= 0 && col < w && row >= 0 && row < h) {
      cell_sums[row][col] += (pt.z * unit_scale_ * 1000.0f);
      cell_counts[row][col]++;
    }
  }

  // Assign initial values
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      if (cell_counts[r][c] > 0) {
        ore.thickness_map[r][c] = cell_sums[r][c] / cell_counts[r][c];
      }
    }
  }

  // Interpolate holes (IDW or simple nearest neighbor filling)
  // Simple iterative filling for strictly internal holes could be complex.
  // Let's implement a simple 3x3 kernel fill for empty pixels that have valid
  // neighbors. Iterating a few times to close small gaps.
  for (int iter = 0; iter < 3; ++iter) {
    auto new_map = ore.thickness_map;
    for (int r = 0; r < h; ++r) {
      for (int c = 0; c < w; ++c) {
        if (ore.thickness_map[r][c] < 0) { // Empty
          float sum = 0;
          int count = 0;
          // Check neighbors
          for (int nr = r - 1; nr <= r + 1; ++nr) {
            for (int nc = c - 1; nc <= c + 1; ++nc) {
              if (nr >= 0 && nr < h && nc >= 0 && nc < w) {
                if (ore.thickness_map[nr][nc] >= 0) {
                  sum += ore.thickness_map[nr][nc];
                  count++;
                }
              }
            }
          }
          if (count > 0) {
            new_map[r][c] = sum / count;
          }
        }
      }
    }
    ore.thickness_map = new_map;
  }
}

void OreAnalyzer::recalculateGroundThreshold() {
  if (aligned_cloud_->empty())
    return;

  // Use statistical analysis for points near Z=0
  // 1. First pass: Collect points within a reasonable initial range (e.g. +/-
  // 0.5m)
  //    Why 0.5? Ground is at 0. Noise shouldn't exceed 50cm usually.
  std::vector<float> z_values;
  z_values.reserve(aligned_cloud_->size() / 10); // reserve some space

  for (const auto &pt : *aligned_cloud_) {
    if (std::abs(pt.z) < 0.5f) {
      z_values.push_back(pt.z);
    }
  }

  if (z_values.empty()) {
    std::cout << "Warning: No points near Z=0. Ground alignment might be wrong."
              << std::endl;
    return;
  }

  // 2. Compute Mean and StdDev
  double sum = std::accumulate(z_values.begin(), z_values.end(), 0.0);
  double mean = sum / z_values.size();

  double sq_sum = 0.0;
  for (float z : z_values) {
    sq_sum += (z - mean) * (z - mean);
  }
  double std_dev = std::sqrt(sq_sum / z_values.size());

  // 3. Set threshold
  // We want to cut off the ground noise.
  // 3-sigma rule covers 99.7% of normal distribution.
  // Updated: User reports belt points still visible. Increasing to configurable
  // sigma + margin.
  float new_threshold =
      static_cast<float>(mean + ground_sigma_ * std_dev + ground_margin_);

  // Sanity check: don't let it be too small (noise always exists)
  if (new_threshold < 0.02f)
    new_threshold = 0.02f;

  std::cout << "Recalculated Ground Threshold Statistics (N=" << z_values.size()
            << "):" << std::endl;
  std::cout << "  Mean Z: " << mean << std::endl;
  std::cout << "  StdDev: " << std_dev << std::endl;
  std::cout << "  Updating ground_threshold_ to: " << new_threshold << " (from "
            << ground_threshold_ << ")" << std::endl;

  ground_threshold_ = new_threshold;
}

bool OreAnalyzer::calibrateAndFilterSidePanels(float known_height_m,
                                               float &calculated_scale) {
  if (aligned_cloud_->empty())
    return false;

  // 1. Identify Left and Right Regions
  // Assuming Y-axis is the cross-belt direction
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*aligned_cloud_, min_pt, max_pt);

  float mid_y = (min_pt.y + max_pt.y) / 2.0f;
  float y_range = max_pt.y - min_pt.y;

  // Definition of Side Regions: Outer 20% on each side?
  float left_boundary = max_pt.y - (y_range * 0.25f);
  float right_boundary = min_pt.y + (y_range * 0.25f);

  // Collect candidate points (must be somewhat above ground)
  pcl::PointIndices::Ptr candidates_left(new pcl::PointIndices);
  pcl::PointIndices::Ptr candidates_right(new pcl::PointIndices);

  // Also keep candidates for filtering later
  std::vector<int> all_side_indices;

  for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
    const auto &pt = aligned_cloud_->points[i];
    if (pt.z > ground_threshold_) { // Above ground
      if (pt.y > left_boundary) {
        candidates_left->indices.push_back(i);
      } else if (pt.y < right_boundary) {
        candidates_right->indices.push_back(i);
      }
    }
  }

  if (candidates_left->indices.size() < 100 &&
      candidates_right->indices.size() < 100) {
    std::cout << "Not enough points on sides to detect panels." << std::endl;
    return false;
  }

  // 2. Detect Planes
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.05); // Tolerance for panel flatness
  seg.setInputCloud(aligned_cloud_);

  float left_height = 0.0f;
  float right_height = 0.0f;
  bool found_left = false;
  bool found_right = false;

  // Detect Left
  if (candidates_left->indices.size() > 100) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setIndices(candidates_left);
    seg.segment(*inliers, *coefficients);
    if (!inliers->indices.empty()) {
      // Compute height (Z span)
      float min_z = 1e9, max_z = -1e9;
      for (int idx : inliers->indices) {
        float z = aligned_cloud_->points[idx].z;
        if (z < min_z)
          min_z = z;
        if (z > max_z)
          max_z = z;
        all_side_indices.push_back(idx);
      }
      // Panels usually start at ground. So height is roughly max_z (relative to
      // ground 0) But let's check if min_z is close to ground.
      left_height = max_z; // Assuming Z=0 is bottom
      found_left = true;
      std::cout << "Found Left Panel. Height (Z): " << left_height
                << " (Z range: " << min_z << " to " << max_z << ")"
                << std::endl;
    }
  }

  // Detect Right
  if (candidates_right->indices.size() > 100) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setIndices(candidates_right);
    seg.segment(*inliers, *coefficients);
    if (!inliers->indices.empty()) {
      // Compute height
      float min_z = 1e9, max_z = -1e9;
      for (int idx : inliers->indices) {
        float z = aligned_cloud_->points[idx].z;
        if (z < min_z)
          min_z = z;
        if (z > max_z)
          max_z = z;
        all_side_indices.push_back(idx);
      }
      right_height = max_z;
      found_right = true;
      std::cout << "Found Right Panel. Height (Z): " << right_height
                << " (Z range: " << min_z << " to " << max_z << ")"
                << std::endl;
    }
  }

  if (!found_left && !found_right)
    return false;

  // 3. Calculate Scale
  float avg_detected_height = 0.0f;
  if (found_left && found_right)
    avg_detected_height = (left_height + right_height) / 2.0f;
  else if (found_left)
    avg_detected_height = left_height;
  else
    avg_detected_height = right_height;

  // Avoid division by zero
  if (avg_detected_height < 0.001f)
    return false;

  calculated_scale = known_height_m / avg_detected_height;

  std::cout << "Average Panel Height (Point Cloud Units): "
            << avg_detected_height << std::endl;
  std::cout << "Known Height: " << known_height_m << " m" << std::endl;
  std::cout << "Calculated Unit Scale: " << calculated_scale << std::endl;

  // 4. Filter them out (Update: Use geometric boundaries projected to ground
  // for robustness) Why projection? Because if the wall is inclined and we only
  // detecting the top part (Z >> 0), the simple min/max Y of inliers will be
  // "too outer" and won't filter the bottom noise at Z=0. We must find the Y at
  // Z=ground_threshold (inner edge).

  Eigen::Vector4f centroid_L, centroid_R;

  if (found_left) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr c(new pcl::ModelCoefficients);
    seg.setIndices(candidates_left);
    seg.segment(*inliers, *c);

    // Solve for Y at Z=ground_threshold_.
    // by = -(ax + cz + d) => y = -(ax + cz + d)/b
    // We need an X. This is tricky if plane is skewed.
    // But for side walls parallel to belt (X-axis), Y is roughly constant.
    // Let's use the average X of the inliers.
    pcl::compute3DCentroid(*aligned_cloud_, *inliers, centroid_L);
    float avg_x = centroid_L[0];

    // Projected Y
    float proj_y = -(c->values[0] * avg_x + c->values[2] * ground_threshold_ +
                     c->values[3]) /
                   c->values[1];

    // Left Panel (High Y side). Inner Edge is Min Y.
    // Panel leans out ( \ ). Z=ground is bottom. Y at bottom is smaller
    // (inner).
    belt_max_y_ = proj_y;
    std::cout << "Left Panel proj Y at ground: " << proj_y
              << " (Avg X: " << avg_x << ")" << std::endl;
  }

  if (found_right) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr c(new pcl::ModelCoefficients);
    seg.setIndices(candidates_right);
    seg.segment(*inliers, *c);

    pcl::compute3DCentroid(*aligned_cloud_, *inliers, centroid_R);
    float avg_x = centroid_R[0];

    // Projected Y
    float proj_y = -(c->values[0] * avg_x + c->values[2] * ground_threshold_ +
                     c->values[3]) /
                   c->values[1];

    // Right Panel (Low Y). Inner Edge is Max Y.
    // Panel leans out ( / ). Z=ground is bottom. Y at bottom is larger (inner).
    belt_min_y_ = proj_y;
    std::cout << "Right Panel proj Y at ground: " << proj_y
              << " (Avg X: " << avg_x << ")" << std::endl;
  }

  // Fallback if projection failed or unreasonable?
  pcl::PointXYZ min_pt_c, max_pt_c;
  pcl::getMinMax3D(*aligned_cloud_, min_pt_c, max_pt_c);
  if (belt_max_y_ > max_pt_c.y)
    belt_max_y_ = max_pt_c.y;
  if (belt_min_y_ < min_pt_c.y)
    belt_min_y_ = min_pt_c.y;

  std::cout << "Determined Belt Boundaries (Y): " << belt_min_y_ << " to "
            << belt_max_y_ << std::endl;

  // Remove points outside boundaries immediately?
  // "Filter using lateral coordinates" - handled in detectByLidar or here
  // Let's do it here physically to clean the cloud for visualization
  pcl::ExtractIndices<PointT> extract_box;
  extract_box.setInputCloud(aligned_cloud_);
  pcl::PointIndices::Ptr indices_outside(new pcl::PointIndices);

  for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
    float y = aligned_cloud_->points[i].y;
    if (y < belt_min_y_ || y > belt_max_y_) { // Outside belt
      indices_outside->indices.push_back(i);
    }
  }

  extract_box.setIndices(indices_outside);
  extract_box.setNegative(true);
  extract_box.filter(*aligned_cloud_);

  std::cout << "Removed " << indices_outside->indices.size()
            << " points outside belt boundaries." << std::endl;

  return true;
}

void OreAnalyzer::filterSidePanels() {
  if (aligned_cloud_->empty())
    return;

  std::cout << "Filtering side panels using Boundaries: " << belt_min_y_
            << " to " << belt_max_y_ << std::endl;

  pcl::ExtractIndices<PointT> extract_box;
  extract_box.setInputCloud(aligned_cloud_);
  pcl::PointIndices::Ptr indices_outside(new pcl::PointIndices);

  for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
    float y = aligned_cloud_->points[i].y;
    if (y < belt_min_y_ || y > belt_max_y_) { // Outside belt
      indices_outside->indices.push_back(i);
    }
  }

  extract_box.setIndices(indices_outside);
  extract_box.setNegative(true);
  extract_box.filter(*aligned_cloud_);

  std::cout << "Removed " << indices_outside->indices.size()
            << " points outside belt boundaries (Manual Filter)." << std::endl;
}

void OreAnalyzer::filterGroundPoints() {
  if (aligned_cloud_->empty())
    return;

  // Capture full X-axis extent BEFORE ground filtering.
  // After filtering, only ore points remain, which narrows the X range.
  // We need the pre-filter range so that lidar_crop_up/down can trim from
  // the full scan length rather than just the ore-occupied region.
  if (belt_min_x_ < -1e8f || belt_max_x_ > 1e8f) {
    pcl::PointXYZ pre_min, pre_max;
    pcl::getMinMax3D(*aligned_cloud_, pre_min, pre_max);
    if (belt_min_x_ < -1e8f)
      belt_min_x_ = pre_min.x;
    if (belt_max_x_ > 1e8f)
      belt_max_x_ = pre_max.x;
    std::cout << "Auto-captured Belt X Boundaries (pre-ground-filter): "
              << belt_min_x_ << " to " << belt_max_x_ << std::endl;
  }

  std::cout << "Filtering ground points (Z <= " << ground_threshold_ << ")..."
            << std::endl;

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(aligned_cloud_);
  pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);

  for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
    if (aligned_cloud_->points[i].z <= ground_threshold_) {
      ground_indices->indices.push_back(i);
    }
  }

  extract.setIndices(ground_indices);
  extract.setNegative(true); // Remove ground
  extract.filter(*aligned_cloud_);

  std::cout << "Removed " << ground_indices->indices.size()
            << " ground points. Remaining: " << aligned_cloud_->size()
            << std::endl;
}

OreAnalyzer::ThicknessMap
OreAnalyzer::generateGlobalThicknessMap(const std::vector<Ore> &ores,
                                        float resolution, bool use_crops,
                                        FusionCrops lidar_crops) {
  ThicknessMap map;
  map.resolution =
      resolution; // physical resolution of the generated map (meters)
  map.width = 0;
  map.height = 0;

  if (unit_scale_ <= 0.000001f)
    return map;

  // Convert physical resolution (meters) to point cloud units
  // X units * unit_scale_ = resolution (meters)
  // X point cloud units = resolution / unit_scale_
  float resolution_raw = resolution / unit_scale_;

  // 1. Determine Bounds
  float min_x = 1e9, max_x = -1e9;
  float min_y = 1e9, max_y = -1e9;
  bool points_found = false;

  if (use_crops) {
    // If crops are provided, explicitly compute the strict boundary box
    computeCropBounds(lidar_crops, min_x, max_x, min_y, max_y);
    points_found = true;
  } else if (!ores.empty()) {
    // Determine Bounds based on DETECTED ORES only
    for (const auto &ore : ores) {
      if (!ore.point_indices)
        continue;
      for (int idx : ore.point_indices->indices) {
        const auto &pt = aligned_cloud_->points[idx];
        if (pt.x < min_x)
          min_x = pt.x;
        if (pt.x > max_x)
          max_x = pt.x;
        if (pt.y < min_y)
          min_y = pt.y;
        if (pt.y > max_y)
          max_y = pt.y;
        points_found = true;
      }
    }
  } else {
    // Use the entire aligned cloud (ground points already filtered)
    for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
      const auto &pt = aligned_cloud_->points[i];
      if (pt.x < min_x)
        min_x = pt.x;
      if (pt.x > max_x)
        max_x = pt.x;
      if (pt.y < min_y)
        min_y = pt.y;
      if (pt.y > max_y)
        max_y = pt.y;
      points_found = true;
    }
  }

  if (!points_found || min_x >= max_x || min_y >= max_y)
    return map;

  // Use Belt Boundaries for Y if available (only if not strictly overriden by
  // crops)
  if (!use_crops && belt_min_y_ > -1e8 && belt_max_y_ < 1e8) {
    min_y = belt_min_y_;
    max_y = belt_max_y_;
  }

  // Removed 5-pixel margin to enforce strict alignment via crops.
  // min_x and max_x strictly follow point cloud or user bounds.

  // Validate Dimensions；
  // number of points (of lidar point cloud) in the thickness map
  int w = std::ceil((max_x - min_x) / resolution_raw);
  int h = std::ceil((max_y - min_y) / resolution_raw);

  // Sanity check
  if (w > 20000 || h > 20000) {
    std::cerr << "Error: Global Thickness Map too large! Dimensions: " << w
              << "x" << h << ". Check unit_scale or resolution." << std::endl;
    return map;
  }

  if (w <= 0 || h <= 0)
    return map;

  map.width = w;
  map.height = h;
  map.min_x = min_x;
  map.max_y = max_y;
  map.max_x = max_x;
  map.min_y = min_y;
  map.is_image_aligned = false;

  // Initialize with 0
  try {
    map.data.assign(w * h, 0.0f);
  } catch (const std::bad_alloc &e) {
    std::cerr << "Error: Failed to allocate memory for Thickness Map: "
              << e.what() << std::endl;
    map.width = 0;
    map.height = 0;
    return map;
  }

  std::cout << "Generating Global Thickness Map (" << map.width << "x"
            << map.height << "), Res: " << resolution << "m (" << resolution_raw
            << " raw units)"
            << ". Bounds Raw: X[" << min_x << ", " << max_x << "] Y[" << min_y
            << ", " << max_y << "]" << std::endl;

  // 2. Fill the map
  if (!ores.empty()) {
    for (const auto &ore : ores) {
      if (!ore.point_indices)
        continue;
      for (int idx : ore.point_indices->indices) {
        const auto &pt = aligned_cloud_->points[idx];

        int col = static_cast<int>((pt.x - min_x) / resolution_raw);
        int row = static_cast<int>((max_y - pt.y) / resolution_raw);

        if (col >= 0 && col < map.width && row >= 0 && row < map.height) {
          float thick = pt.z * unit_scale_ * 1000.0f;
          int index = row * map.width + col;
          // Keep max thickness in cell
          if (thick > map.data[index]) {
            map.data[index] = thick;
          }
        }
      }
    }
  } else {
    // Fill from entire cloud, constrain to pre-calculated bounds
    // (ground points already removed by filterGroundPoints)
    for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
      const auto &pt = aligned_cloud_->points[i];
      if (pt.x >= min_x && pt.x <= max_x && pt.y >= min_y && pt.y <= max_y) {
        int col = static_cast<int>((pt.x - min_x) / resolution_raw);
        int row = static_cast<int>((max_y - pt.y) / resolution_raw);

        if (col >= 0 && col < map.width && row >= 0 && row < map.height) {
          float thick = pt.z * unit_scale_ * 1000.0f;
          int index = row * map.width + col;
          if (thick > map.data[index]) {
            map.data[index] = thick;
          }
        }
      }
    }
  }

  std::cout << "Generated Global Thickness Map successfully." << std::endl;
  return map;
}

void OreAnalyzer::computeCropBounds(const FusionCrops &lidar_crops,
                                    float &roi_min_x, float &roi_max_x,
                                    float &roi_min_y, float &roi_max_y) {
  if (!aligned_cloud_ || aligned_cloud_->empty()) {
    std::cerr
        << "Warning: Cannot compute crop bounds without an aligned point cloud."
        << std::endl;
    roi_min_x = roi_max_x = roi_min_y = roi_max_y = 0.0f;
    return;
  }

  pcl::PointXYZ min_p, max_p;
  pcl::getMinMax3D(*aligned_cloud_, min_p, max_p);

  // 1. Determine base LiDAR bounds exactly as in generateGlobalThicknessMap
  float base_min_x = belt_min_x_ > -1e8f ? belt_min_x_ : min_p.x;
  float base_max_x = belt_max_x_ < 1e8f ? belt_max_x_ : max_p.x;
  float base_min_y = belt_min_y_ > -1e8f ? belt_min_y_ : min_p.y;
  float base_max_y = belt_max_y_ < 1e8f ? belt_max_y_ : max_p.y;

  // Removed 5-pixel margin to enforce strict alignment via crops.
  // Base bounds strictly follow point cloud or user bounds.

  // 2. Apply LiDAR crops directly in point cloud native units (no unit_scale
  // conversion). The crop values and the base bounds (min_p, max_p, belt_*_y)
  // are all in the same native coordinate system (e.g. mm).
  // Map Rows (col in un-transposed map) = Y in image = X in physical
  // Map Cols (row in un-transposed map) = X in image = Y in physical

  // Apply crops to base bounds (all in native point cloud units)
  // Point Cloud X corresponds to Image Up/Down (Y).
  //  - 'down' cuts from min_x (bottom of image = smaller X)
  //  - 'up' cuts from max_x (top of image = larger X)
  roi_min_x = base_min_x + lidar_crops.down;
  roi_max_x = base_max_x - lidar_crops.up;

  // Point Cloud Y corresponds to Image Left/Right (X).
  //  - 'right' cuts from min_y (right of image = smaller Y due to transpose
  //  visual mapping)
  //  - 'left' cuts from max_y (left of image = larger Y due to transpose visual
  //  mapping)
  roi_min_y = base_min_y + lidar_crops.right;
  roi_max_y = base_max_y - lidar_crops.left;
}

void OreAnalyzer::alignThicknessMapToImage(ThicknessMap &map,
                                           cv::Size target_size) {
  if (map.data.empty() || map.width <= 0 || map.height <= 0)
    return;
  if (target_size.width <= 0 || target_size.height <= 0)
    return;

  cv::Mat map_mat(map.height, map.width, CV_32FC1, map.data.data());

  // Transpose to match Image/Camera Axes (where Image X maps to Map Y/Rows,
  // Image Y maps to Map X/Cols)
  cv::Mat transposed;
  cv::transpose(map_mat, transposed);

  // Resize to completely match the image resolution
  cv::Mat resized;
  cv::resize(transposed, resized, target_size, 0, 0, cv::INTER_LINEAR);

  map.width = resized.cols;
  map.height = resized.rows;

  map.data.clear();
  map.data.reserve(resized.total());
  if (resized.isContinuous()) {
    map.data.assign((float *)resized.datastart, (float *)resized.dataend);
  } else {
    for (int r = 0; r < resized.rows; ++r) {
      map.data.insert(map.data.end(), resized.ptr<float>(r),
                      resized.ptr<float>(r) + resized.cols);
    }
  }

  map.is_image_aligned = true;
}

bool OreAnalyzer::saveThicknessMapToImage(const ThicknessMap &map,
                                          const std::string &filename,
                                          float max_val,
                                          const std::vector<Ore> *ores) {
  if (map.data.empty() || map.width <= 0 || map.height <= 0)
    return false;

  int w = map.width;
  int h = map.height;

  // Determine scaling factor
  float max_v = max_val;
  if (max_v <= 0.0f) {
    max_v = 0.0f;
    for (float v : map.data) {
      if (v > max_v)
        max_v = v;
    }
  }

  if (max_v <= 0.000001f)
    max_v = 1.0f; // Avoid div/0

  std::cout << "Saving Map to " << filename << ". Max Val: " << max_v
            << " mapped to 255." << std::endl;

  // Create OpenCV Mat (Height, Width, Type)
  cv::Mat image(h, w, CV_8UC1);

  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      float val = map.data[r * w + c];
      int pixel_val = static_cast<int>((val / max_v) * 255.0f);
      if (pixel_val > 255)
        pixel_val = 255;
      if (pixel_val < 0)
        pixel_val = 0;

      image.at<uint8_t>(r, c) = static_cast<uint8_t>(pixel_val);
    }
  }

  // Transpose the image as requested, ONLY if it's not already image-aligned
  cv::Mat transposed_image;
  if (map.is_image_aligned) {
    transposed_image = image;
  } else {
    cv::transpose(image, transposed_image);
  }

  // Convert grayscale to BGR if we need to draw colored text, or just draw
  // white text. Actually, to make text visible against white/black background,
  // let's convert to BGR so we can draw colored (e.g., Green or Red) labels.
  cv::Mat final_image;
  if (ores != nullptr && !ores->empty()) {
    cv::cvtColor(transposed_image, final_image, cv::COLOR_GRAY2BGR);

    for (const auto &ore : *ores) {
      if (ore.point_indices->indices.empty())
        continue;

      // Calculate center in physical coordinates
      float center_x = (ore.min_x + ore.max_x) / 2.0f;
      float center_y = (ore.min_y + ore.max_y) / 2.0f;

      int img_r, img_c;
      if (map.is_image_aligned) {
        float map_phys_width = map.max_x - map.min_x;
        float map_phys_height = map.max_y - map.min_y;
        float rel_x = (center_x - map.min_x) / map_phys_width;
        float rel_y = (map.max_y - center_y) / map_phys_height;
        // Transposed axes interpretation:
        // map X -> image rows (Y)
        // map Y -> image cols (X)
        img_r = static_cast<int>(rel_x * final_image.rows);
        img_c = static_cast<int>(rel_y * final_image.cols);
      } else {
        int map_c = static_cast<int>((center_x - map.min_x) / map.resolution);
        int map_r = static_cast<int>((map.max_y - center_y) / map.resolution);
        img_r = map_c;
        img_c = map_r;
      }

      if (img_r >= 0 && img_r < final_image.rows && img_c >= 0 &&
          img_c < final_image.cols) {
        std::string label = ore.id;
        if (label.find("ore_") == 0)
          label = label.substr(4);

        cv::Point pos(img_c, img_r);
        cv::putText(final_image, label, pos, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 0), 2); // Green text
      }
    }
  } else {
    final_image = transposed_image;
  }

  // Save using OpenCV
  return cv::imwrite(filename, final_image);
}

// Fuse Thickness Map with RGB Image
bool OreAnalyzer::fuseThicknessWithImage(const ThicknessMap &map,
                                         const cv::Mat &rgb_image_cropped,
                                         const std::string &output_filename,
                                         int channel, FusionCrops lidar_crops,
                                         const std::vector<Ore> *ores) {
  if (map.data.empty() || map.width <= 0 || map.height <= 0) {
    std::cerr << "Error: Thickness map is empty." << std::endl;
    return false;
  }

  if (rgb_image_cropped.empty()) {
    std::cerr << "Error: Pre-cropped RGB image is empty." << std::endl;
    return false;
  }

  std::cout << "Using pre-cropped RGB Image: " << rgb_image_cropped.cols << "x"
            << rgb_image_cropped.rows << std::endl;

  // 2. Convert Thickness Map to CV Matrix
  // Original Map: Width x Height
  // User says: "stitched_ore.jpg matches the saved thickness map"
  // But saved thickness map was TRANSPOSED in saveThicknessMapToImage.
  // So the "aligned" orientation is the TRANSPOSED one.

  // 2. Prepare Resized Map
  float max_v = 0.0f;
  for (int r = 0; r < map.height; ++r) {
    for (int c = 0; c < map.width; ++c) {
      if (map.data[r * map.width + c] > max_v)
        max_v = map.data[r * map.width + c];
    }
  }
  if (max_v <= 0.000001f)
    max_v = 1.0f;

  cv::Mat map_resized;
  float scale_x = 1.0f, scale_y = 1.0f;
  int src_offset_x = 0, src_offset_y = 0;

  cv::Mat final_image = rgb_image_cropped.clone();

  if (map.is_image_aligned && map.width == final_image.cols &&
      map.height == final_image.rows) {
    map_resized = cv::Mat(map.height, map.width, CV_32FC1,
                          const_cast<float *>(map.data.data()))
                      .clone();
    std::cout << "Fusion ROI Alignment: Direct 1:1 Pixel Mapping Used."
              << std::endl;
  } else {
    cv::Mat map_mat(map.height, map.width, CV_32FC1,
                    const_cast<float *>(map.data.data()));
    cv::Mat map_transposed;
    cv::transpose(map_mat, map_transposed);

    float roi_min_x, roi_max_x, roi_min_y, roi_max_y;
    computeCropBounds(lidar_crops, roi_min_x, roi_max_x, roi_min_y, roi_max_y);

    float resolution_raw = map.resolution / unit_scale_;
    if (unit_scale_ <= 1e-6f)
      resolution_raw = 1.0f;

    int start_row = static_cast<int>((roi_min_x - map.min_x) / resolution_raw);
    int end_row = static_cast<int>((roi_max_x - map.min_x) / resolution_raw);
    int start_col = static_cast<int>((map.max_y - roi_max_y) / resolution_raw);
    int end_col = static_cast<int>((map.max_y - roi_min_y) / resolution_raw);

    cv::Rect src_roi(start_col, start_row, end_col - start_col,
                     end_row - start_row);
    if (src_roi.width <= 0 || src_roi.height <= 0 || src_roi.x < 0 ||
        src_roi.y < 0 || src_roi.x + src_roi.width > map_transposed.cols ||
        src_roi.y + src_roi.height > map_transposed.rows) {
      std::cerr << "Error: Invalid LiDAR crops. Resulting ROI is invalid."
                << std::endl;
      return false;
    }

    src_offset_x = src_roi.x;
    src_offset_y = src_roi.y;

    cv::Mat map_cropped = map_transposed(src_roi);
    cv::resize(map_cropped, map_resized, final_image.size(), 0, 0,
               cv::INTER_LINEAR);

    scale_x = static_cast<float>(final_image.cols) / map_cropped.cols;
    scale_y = static_cast<float>(final_image.rows) / map_cropped.rows;

    std::cout << "Fusion ROI Alignment calculated dynamically." << std::endl;
  }

  // Extract base directory from output_filename
  std::string base_dir = "E:/multi_source_info/lidar/results";
  size_t last_slash = output_filename.find_last_of("/\\");
  if (last_slash != std::string::npos) {
    base_dir = output_filename.substr(0, last_slash);
  }

  // Save intermediate images
  cv::imwrite(base_dir + "/01_cropped_rgb_for_fusion.jpg", final_image);

  // Visualize rescaled thickness map with Jet colormap
  cv::Mat norm_map_resized;
  map_resized.convertTo(norm_map_resized, CV_8UC1, 255.0 / max_v);
  cv::Mat color_map_resized;
  cv::applyColorMap(norm_map_resized, color_map_resized, cv::COLORMAP_JET);
  cv::imwrite(base_dir + "/04_rescaled_thickness_for_rgb_fusion.jpg",
              color_map_resized);

  // 4. Overlay
  if (channel < 0 || channel > 2)
    channel = 2; // Default Red

  for (int r = 0; r < map_resized.rows; ++r) {
    for (int c = 0; c < map_resized.cols; ++c) {
      float val = map_resized.at<float>(r, c);

      // Handle "Empty" values
      if (val <= 0.001f)
        continue;

      // Position in Final Image (Already Cropped)
      int rgb_r = r;
      int rgb_c = c;

      if (rgb_r >= 0 && rgb_r < final_image.rows && rgb_c >= 0 &&
          rgb_c < final_image.cols) {

        // Normalize and clamp
        float norm_val = (val / max_v) * 255.0f;
        if (norm_val > 255.0f)
          norm_val = 255.0f;

        // Inject into channel
        if (final_image.type() == CV_8UC3) {
          final_image.at<cv::Vec3b>(rgb_r, rgb_c)[channel] =
              static_cast<uint8_t>(norm_val);
        }
      }
    }
  }

  // 5. Draw Labels if provided
  if (ores) {
    for (const auto &ore : *ores) {
      if (!ore.point_indices || ore.point_indices->indices.empty()) {
        continue;
      }
      // Calculate Centroid (Physical Units)
      float cx = (ore.min_x + ore.max_x) / 2.0f;
      float cy = (ore.min_y + ore.max_y) / 2.0f;

      int rgb_r, rgb_c;
      if (map.is_image_aligned && map.width == final_image.cols &&
          map.height == final_image.rows) {
        float map_phys_width = map.max_x - map.min_x;
        float map_phys_height = map.max_y - map.min_y;
        float rel_x = (cx - map.min_x) / map_phys_width;
        float rel_y = (map.max_y - cy) / map_phys_height;
        // Transposed axes interpretation:
        // map X -> image rows (Y)
        // map Y -> image cols (X)
        rgb_r = static_cast<int>(rel_x * final_image.rows);
        rgb_c = static_cast<int>(rel_y * final_image.cols);
      } else {
        float resolution_raw = map.resolution / unit_scale_;
        if (unit_scale_ <= 1e-6f)
          resolution_raw = 1.0f;

        int col_map = static_cast<int>((cx - map.min_x) / resolution_raw);
        int row_map = static_cast<int>((map.max_y - cy) / resolution_raw);

        int r_trans = col_map;
        int c_trans = row_map;

        int r_cropped = r_trans - src_offset_y;
        int c_cropped = c_trans - src_offset_x;

        rgb_r = static_cast<int>(r_cropped * scale_y);
        rgb_c = static_cast<int>(c_cropped * scale_x);
      }

      // Draw Text
      if (rgb_r >= 0 && rgb_r < final_image.rows && rgb_c >= 0 &&
          rgb_c < final_image.cols) {
        std::string label = ore.id;
        // Remove "ore_" prefix if present
        if (label.find("ore_") == 0) {
          label = label.substr(4);
        }

        // Text Color: White (255, 255, 255)
        // Position: Point(x, y) = (col, row)
        cv::Point pos(rgb_c, rgb_r);
        cv::putText(final_image, label, pos, cv::FONT_HERSHEY_SIMPLEX, 4.0,
                    cv::Scalar(255, 255, 255), 5);
      }
    }
  }

  // 6. Save
  std::cout << "Saving Fused Image to " << output_filename << std::endl;
  if (cv::imwrite(output_filename, final_image)) {
    return true;
  } else {
    std::cerr << "Error: Could not save fused image." << std::endl;
    return false;
  }
}

bool OreAnalyzer::fuseThicknessWithXray(const ThicknessMap &map,
                                        const cv::Mat &xray_image_cropped,
                                        const std::string &output_filename,
                                        FusionCrops lidar_crops,
                                        const std::vector<Ore> *ores) {
  if (map.data.empty() || map.width <= 0 || map.height <= 0) {
    std::cerr << "Error: Thickness map is empty." << std::endl;
    return false;
  }

  if (xray_image_cropped.empty()) {
    std::cerr << "Error: Pre-cropped X-ray image is empty." << std::endl;
    return false;
  }

  std::cout << "Using pre-cropped X-ray Image: " << xray_image_cropped.cols
            << "x" << xray_image_cropped.rows << std::endl;

  // Clone to ensure contiguous memory and for conversion
  cv::Mat final_image_gray = xray_image_cropped.clone();
  cv::Mat final_image;
  cv::cvtColor(final_image_gray, final_image, cv::COLOR_GRAY2BGR);

  // 4. Prepare Resized Map
  float max_v = 0.0f;
  for (int r = 0; r < map.height; ++r) {
    for (int c = 0; c < map.width; ++c) {
      if (map.data[r * map.width + c] > max_v)
        max_v = map.data[r * map.width + c];
    }
  }
  if (max_v <= 0.000001f)
    max_v = 1.0f;

  cv::Mat map_resized;
  float scale_x = 1.0f, scale_y = 1.0f;
  int src_offset_x = 0, src_offset_y = 0;

  if (map.is_image_aligned && map.width == final_image.cols &&
      map.height == final_image.rows) {
    map_resized = cv::Mat(map.height, map.width, CV_32FC1,
                          const_cast<float *>(map.data.data()))
                      .clone();
    std::cout << "X-ray Fusion ROI Alignment: Direct 1:1 Pixel Mapping Used."
              << std::endl;
  } else {
    cv::Mat map_mat(map.height, map.width, CV_32FC1,
                    const_cast<float *>(map.data.data()));
    cv::Mat map_transposed;
    cv::transpose(map_mat, map_transposed);

    float roi_min_x, roi_max_x, roi_min_y, roi_max_y;
    computeCropBounds(lidar_crops, roi_min_x, roi_max_x, roi_min_y, roi_max_y);

    float resolution_raw = map.resolution / unit_scale_;
    if (unit_scale_ <= 1e-6f)
      resolution_raw = 1.0f;

    int start_row = static_cast<int>((roi_min_x - map.min_x) / resolution_raw);
    int end_row = static_cast<int>((roi_max_x - map.min_x) / resolution_raw);
    int start_col = static_cast<int>((map.max_y - roi_max_y) / resolution_raw);
    int end_col = static_cast<int>((map.max_y - roi_min_y) / resolution_raw);

    cv::Rect src_roi(start_col, start_row, end_col - start_col,
                     end_row - start_row);
    if (src_roi.width <= 0 || src_roi.height <= 0 || src_roi.x < 0 ||
        src_roi.y < 0 || src_roi.x + src_roi.width > map_transposed.cols ||
        src_roi.y + src_roi.height > map_transposed.rows) {
      std::cerr << "Error: Invalid LiDAR crops for X-ray fusion." << std::endl;
      return false;
    }

    src_offset_x = src_roi.x;
    src_offset_y = src_roi.y;
    cv::Mat map_cropped = map_transposed(src_roi);
    cv::resize(map_cropped, map_resized, final_image.size(), 0, 0,
               cv::INTER_LINEAR);

    scale_x = static_cast<float>(final_image.cols) / map_cropped.cols;
    scale_y = static_cast<float>(final_image.rows) / map_cropped.rows;
  }

  // 7. Apply Colormap and Overlay
  // Create a mask to only blend where thickness > 0
  cv::Mat mask;
  cv::threshold(map_resized, mask, 0.001f, 255, cv::THRESH_BINARY);
  mask.convertTo(mask, CV_8UC1);

  // Find actual min and max thickness in the visible area to stretch the
  // contrast, but use Mean and StdDev to ignore noisy single-pixel spikes
  cv::Mat mean, stddev;
  cv::meanStdDev(map_resized, mean, stddev, mask);
  double m = mean.at<double>(0, 0);
  double s = stddev.at<double>(0, 0);

  // Use a strictly focused range to force high contrast
  // A Mean of ~11.0 with a StdDev of ~8.8 means the data is very skewed.
  // We'll clip aggressively at Mean + 0.8 * StdDev to throw all the heavier
  // chunks into Red territory and force the 0-15mm range to occupy the full
  // spectrum.
  double robust_min = std::max(0.0, m - 0.5 * s);
  double robust_max = m + 0.8 * s;

  std::cout << "[Jet Colormap] Contrast bounds aggressively set to min: "
            << robust_min << ", max: " << robust_max << std::endl;

  // Safe bounds check
  if (robust_max <= 0.0001)
    robust_max = 1.0;
  if (robust_min >= robust_max)
    robust_min = 0.0;

  // Normalize mapped thickness to 0-255 for the colormap, stretching to max
  // contrast
  cv::Mat norm_map = cv::Mat::zeros(map_resized.size(), CV_8UC1);
  for (int r = 0; r < map_resized.rows; ++r) {
    for (int c = 0; c < map_resized.cols; ++c) {
      if (mask.at<uchar>(r, c) > 0) {
        float val = map_resized.at<float>(r, c);
        // Exaggerate differences by stretching the robust bounds to 0 and 255
        float norm_val =
            (val - robust_min) / (robust_max - robust_min) * 255.0f;
        if (norm_val < 0.0f)
          norm_val = 0.0f;
        if (norm_val > 255.0f)
          norm_val = 255.0f;
        norm_map.at<uchar>(r, c) = static_cast<uchar>(norm_val);
      }
    }
  }

  // Apply Jet Colormap
  cv::Mat color_map;
  cv::applyColorMap(norm_map, color_map, cv::COLORMAP_JET);

  // Merge the colormap output with the X-ray RGB image using a mask and
  // blending
  float alpha = 0.8f; // Weight for the colormap
  float beta = 1.0f - alpha;

  for (int r = 0; r < final_image.rows; ++r) {
    for (int c = 0; c < final_image.cols; ++c) {
      if (mask.at<uchar>(r, c) > 0) {
        cv::Vec3b bg = final_image.at<cv::Vec3b>(r, c);
        cv::Vec3b fg = color_map.at<cv::Vec3b>(r, c);

        final_image.at<cv::Vec3b>(r, c)[0] =
            cv::saturate_cast<uchar>(fg[0] * alpha + bg[0] * beta);
        final_image.at<cv::Vec3b>(r, c)[1] =
            cv::saturate_cast<uchar>(fg[1] * alpha + bg[1] * beta);
        final_image.at<cv::Vec3b>(r, c)[2] =
            cv::saturate_cast<uchar>(fg[2] * alpha + bg[2] * beta);
      }
    }
  }

  // 8. Draw Labels
  if (ores) {
    for (const auto &ore : *ores) {
      if (!ore.point_indices || ore.point_indices->indices.empty())
        continue;

      float cx = (ore.min_x + ore.max_x) / 2.0f;
      float cy = (ore.min_y + ore.max_y) / 2.0f;

      int rgb_r, rgb_c;
      if (map.is_image_aligned && map.width == final_image.cols &&
          map.height == final_image.rows) {
        float map_phys_width = map.max_x - map.min_x;
        float map_phys_height = map.max_y - map.min_y;
        float rel_x = (cx - map.min_x) / map_phys_width;
        float rel_y = (map.max_y - cy) / map_phys_height;
        // Transposed axes interpretation:
        // map X -> image rows (Y)
        // map Y -> image cols (X)
        rgb_r = static_cast<int>(rel_x * final_image.rows);
        rgb_c = static_cast<int>(rel_y * final_image.cols);
      } else {
        float resolution_raw = map.resolution / unit_scale_;
        if (unit_scale_ <= 1e-6f)
          resolution_raw = 1.0f;

        int col_map = static_cast<int>((cx - map.min_x) / resolution_raw);
        int row_map = static_cast<int>((map.max_y - cy) / resolution_raw);

        int r_trans = col_map;
        int c_trans = row_map;

        int r_cropped = r_trans - src_offset_y;
        int c_cropped = c_trans - src_offset_x;

        rgb_r = static_cast<int>(r_cropped * scale_y);
        rgb_c = static_cast<int>(c_cropped * scale_x);
      }

      // Offset label slightly to the right so it doesn't obscure the center
      rgb_c += 40;

      if (rgb_r >= 0 && rgb_r < final_image.rows && rgb_c >= 0 &&
          rgb_c < final_image.cols) {
        std::string label = ore.id;
        if (label.find("ore_") == 0)
          label = label.substr(4);

        // Use Green for text to contrast with Red thickness
        cv::putText(final_image, label, cv::Point(rgb_c, rgb_r),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
      }
    }
  }

  std::cout << "Saving X-ray Fusion to " << output_filename << std::endl;
  return cv::imwrite(output_filename, final_image);
}

void OreAnalyzer::sortOresSpatially(std::vector<Ore> &ores,
                                    std::vector<cv::Mat> &masks) {
  if (ores.size() <= 1 || masks.size() != ores.size())
    return;

  std::vector<size_t> sort_idx(ores.size());
  std::iota(sort_idx.begin(), sort_idx.end(), 0);

  std::vector<cv::Point> centroids(ores.size());
  std::vector<int> heights(ores.size());
  for (size_t i = 0; i < masks.size(); ++i) {
    cv::Rect bbox = cv::boundingRect(masks[i]);
    centroids[i] = cv::Point(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    heights[i] = bbox.height;
  }

  // Use median ore height as the row-banding tolerance
  std::vector<int> sorted_heights = heights;
  std::sort(sorted_heights.begin(), sorted_heights.end());
  int band_tolerance = sorted_heights[sorted_heights.size() / 2];

  std::sort(sort_idx.begin(), sort_idx.end(),
            [&centroids, band_tolerance](size_t a, size_t b) {
              // If Y centroids are within band_tolerance, treat as same row
              int dy = centroids[a].y - centroids[b].y;
              if (std::abs(dy) > band_tolerance) {
                // Different rows: sort top-to-bottom
                return centroids[a].y < centroids[b].y;
              }
              // Same row: sort left-to-right
              return centroids[a].x < centroids[b].x;
            });

  // Apply sort order
  std::vector<Ore> sorted_ores(ores.size());
  std::vector<cv::Mat> sorted_masks(masks.size());
  for (size_t i = 0; i < sort_idx.size(); ++i) {
    sorted_ores[i] = ores[sort_idx[i]];
    sorted_masks[i] = masks[sort_idx[i]];
  }
  ores = std::move(sorted_ores);
  masks = std::move(sorted_masks);
}
