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

Ore OreAnalyzer::detectByMask(const std::vector<uint8_t> &mask, int width,
                              int height, float mask_min_x, float mask_max_x,
                              float mask_min_y, float mask_max_y) {
  Ore ore;
  ore.id = "mask_ore";
  ore.point_indices.reset(new pcl::PointIndices);

  float pixel_w = (mask_max_x - mask_min_x) / width;
  float pixel_h = (mask_max_y - mask_min_y) / height;

  for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
    const auto &pt = aligned_cloud_->points[i];

    // Map point to mask coordinates
    int px = static_cast<int>((pt.x - mask_min_x) / pixel_w);
    int py = static_cast<int>(
        (pt.y - mask_min_y) /
        pixel_h); // Assuming Y grows normally? Careful with image coords

    // Image coords usually Y down, physical Y usually up or down.
    // Let's assume consistent orientation for simplicity, or user pre-flipped
    // mask.

    if (px >= 0 && px < width && py >= 0 && py < height) {
      if (mask[py * width + px] > 0) { // Mask hit
        if (pt.z > ground_threshold_) {
          ore.point_indices->indices.push_back(i);
        }
      }
    }
  }
  return ore;
}

void OreAnalyzer::computeStats(Ore &ore, bool generate_map, float map_res) {
  if (ore.point_indices->indices.empty())
    return;

  // 1. Basic Stats
  float sum_z = 0.0f;
  float max_z = -1e9f;
  float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f;

  for (int idx : ore.point_indices->indices) {
    float z = aligned_cloud_->points[idx].z;
    const auto &pt = aligned_cloud_->points[idx];

    // Convert to physical units
    float physical_z = z * unit_scale_;

    sum_z += physical_z;
    if (physical_z > max_z)
      max_z = physical_z;

    if (pt.x < min_x)
      min_x = pt.x;
    if (pt.x > max_x)
      max_x = pt.x;
    if (pt.y < min_y)
      min_y = pt.y;
    if (pt.y > max_y)
      max_y = pt.y;
  }

  ore.avg_thickness = sum_z / ore.point_indices->indices.size();
  ore.max_thickness = max_z;
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
      cell_sums[row][col] += (pt.z * unit_scale_);
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
                                        float resolution) {
  ThicknessMap map;
  map.resolution = resolution;
  map.width = 0;
  map.height = 0;

  if (ores.empty() || unit_scale_ <= 0.000001f)
    return map;

  // Convert physical resolution (meters) to point cloud units
  // X units * unit_scale_ = resolution (meters)
  // X point cloud units = resolution / unit_scale_
  float resolution_raw = resolution / unit_scale_;

  // 1. Determine Bounds based on DETECTED ORES only
  // This avoids outliers in the rest of the cloud expanding the map to infinity
  float min_x = 1e9, max_x = -1e9;
  float min_y = 1e9, max_y = -1e9;

  bool points_found = false;

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

  if (!points_found)
    return map;

  // Use Belt Boundaries for Y if available
  if (belt_min_y_ > -1e8 && belt_max_y_ < 1e8) {
    min_y = belt_min_y_;
    max_y = belt_max_y_;
  }

  // Add a small buffer (margin) logic
  // Margin should be in raw units to keep image size reasonable
  float margin_raw = resolution_raw * 5.0f; // 5 pixels margin

  min_x -= margin_raw;
  max_x += margin_raw;

  if (belt_min_y_ <= -1e8 || belt_max_y_ >= 1e8) {
    min_y -= margin_raw;
    max_y += margin_raw;
  }

  // Validate Dimensions
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
  for (const auto &ore : ores) {
    if (!ore.point_indices)
      continue;
    for (int idx : ore.point_indices->indices) {
      const auto &pt = aligned_cloud_->points[idx];

      int col = static_cast<int>((pt.x - min_x) / resolution_raw);
      int row = static_cast<int>((max_y - pt.y) / resolution_raw);

      if (col >= 0 && col < map.width && row >= 0 && row < map.height) {
        float thick = pt.z * unit_scale_;
        int index = row * map.width + col;
        // Keep max thickness in cell
        if (thick > map.data[index]) {
          map.data[index] = thick;
        }
      }
    }
  }
  return map;
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

  // Transpose the image as requested
  cv::Mat transposed_image;
  cv::transpose(image, transposed_image);

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

      // Convert physical coordinates to pixel coordinates on the un-transposed
      // map Note: map generation uses: col = (x - min_x) / res row = (max_y -
      // y) / res
      int map_c = static_cast<int>((center_x - map.min_x) / map.resolution);
      int map_r = static_cast<int>((map.max_y - center_y) / map.resolution);

      // After transpose, image metrics swap:
      // transposed_row = map_c;
      // transposed_col = map_r;
      int img_r = map_c;
      int img_c = map_r;

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
                                         const std::string &rgb_filename,
                                         const std::string &output_filename,
                                         int channel, FusionCrops rgb_crops,
                                         FusionCrops lidar_crops,
                                         const std::vector<Ore> *ores) {
  if (map.data.empty() || map.width <= 0 || map.height <= 0) {
    std::cerr << "Error: Thickness map is empty." << std::endl;
    return false;
  }

  // 1. Load RGB Image
  cv::Mat rgb_image = cv::imread(rgb_filename);
  if (rgb_image.empty()) {
    std::cerr << "Error: Could not read RGB image from " << rgb_filename
              << std::endl;
    return false;
  }

  std::cout << "Loaded RGB Image: " << rgb_image.cols << "x" << rgb_image.rows
            << std::endl;

  // 2. Convert Thickness Map to CV Matrix
  // Original Map: Width x Height
  // User says: "stitched_ore.jpg matches the saved thickness map"
  // But saved thickness map was TRANSPOSED in saveThicknessMapToImage.
  // So the "aligned" orientation is the TRANSPOSED one.

  // Let's reconstruct the map as we saved it (Transposed)
  cv::Mat map_mat(map.height, map.width, CV_32FC1);
  // Find max value for normalization
  float max_v = 0.0f;
  for (int r = 0; r < map.height; ++r) {
    for (int c = 0; c < map.width; ++c) {
      float val = map.data[r * map.width + c];
      if (val > max_v)
        max_v = val;
      map_mat.at<float>(r, c) = val;
    }
  }

  if (max_v <= 0.000001f)
    max_v = 1.0f;
  // Transpose to match the saved image (and thus the RGB image)
  cv::Mat map_transposed;
  cv::transpose(map_mat, map_transposed);

  // 3. Define ROIs
  // Source ROI (LiDAR Thickness Map)
  cv::Rect src_roi(lidar_crops.left, lidar_crops.up,
                   map_transposed.cols - lidar_crops.left - lidar_crops.right,
                   map_transposed.rows - lidar_crops.up - lidar_crops.down);

  // Target ROI (RGB Image)
  cv::Rect tgt_roi(rgb_crops.left, rgb_crops.up,
                   rgb_image.cols - rgb_crops.left - rgb_crops.right,
                   rgb_image.rows - rgb_crops.up - rgb_crops.down);

  // Validate ROIs
  if (src_roi.width <= 0 || src_roi.height <= 0 || src_roi.x < 0 ||
      src_roi.y < 0 || src_roi.x + src_roi.width > map_transposed.cols ||
      src_roi.y + src_roi.height > map_transposed.rows) {
    std::cerr
        << "Error: Invalid LiDAR crops. Resulting ROI is invalid or empty."
        << std::endl;
    return false;
  }
  if (tgt_roi.width <= 0 || tgt_roi.height <= 0 || tgt_roi.x < 0 ||
      tgt_roi.y < 0 || tgt_roi.x + tgt_roi.width > rgb_image.cols ||
      tgt_roi.y + tgt_roi.height > rgb_image.rows) {
    std::cerr << "Error: Invalid RGB crops. Resulting ROI is invalid or empty."
              << std::endl;
    return false;
  }

  // Crop Source
  cv::Mat map_cropped = map_transposed(src_roi);

  // Crop Target (RGB)
  // We clone it so that the output image is physically smaller (just the ROI)
  cv::Mat final_image = rgb_image(tgt_roi).clone();

  // Resize Cropped Source to fit Target ROI
  cv::Mat map_resized;
  cv::resize(map_cropped, map_resized, final_image.size(), 0, 0,
             cv::INTER_LINEAR);

  // Calculate Scale Factors for label mapping
  float scale_x = static_cast<float>(final_image.cols) / map_cropped.cols;
  float scale_y = static_cast<float>(final_image.rows) / map_cropped.rows;

  std::cout << "Fusion ROI Alignment:" << std::endl;
  std::cout << "  LiDAR ROI: " << src_roi.width << "x" << src_roi.height
            << " (from " << map_transposed.cols << "x" << map_transposed.rows
            << ")" << std::endl;
  std::cout << "  RGB ROI:   " << tgt_roi.width << "x" << tgt_roi.height
            << " (Target)" << std::endl;
  std::cout << "  Scale:     x=" << scale_x << ", y=" << scale_y << std::endl;

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

      // Map to Map Grid (Original Map)
      // Note: map.resolution is in METERS (e.g. 0.01), but the grid is built in
      // RAW UNITS. We need to recover resolution_raw used during generation.
      // resolution_raw = resolution / unit_scale_
      float resolution_raw = map.resolution / unit_scale_;
      if (unit_scale_ <= 1e-6f)
        resolution_raw = 1.0f; // Safety

      // col = (cx - min_x) / res_raw
      // row = (max_y - cy) / res_raw
      int col_map = static_cast<int>((cx - map.min_x) / resolution_raw);
      int row_map = static_cast<int>((map.max_y - cy) / resolution_raw);

      // Logic for map_transposed:
      // map_mat(row_map, col_map) maps to map_transposed(col_map, row_map)
      // So in transposed image: row index is col_map, col index is row_map.
      int r_trans = col_map;
      int c_trans = row_map;

      // Apply Source Crop Offset
      // The point must be inside the source ROI to be visible
      int r_cropped = r_trans - src_roi.y;
      int c_cropped = c_trans - src_roi.x;

      // Scale to Target ROI
      int r_scaled = static_cast<int>(r_cropped * scale_y);
      int c_scaled = static_cast<int>(c_cropped * scale_x);

      // Apply Target ROI Offset (Position in RGB)
      // Since final_image IS the ROI, the offset is 0 relative to it.
      int rgb_r = r_scaled;
      int rgb_c = c_scaled;

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

bool OreAnalyzer::fuseThicknessWithXray(
    const ThicknessMap &map, const std::string &xray_filename,
    const std::string &output_filename, int cut_left, int cut_right,
    bool enable_geometry_correction, float sod, float sdd,
    FusionCrops xray_crops, FusionCrops lidar_crops,
    const std::vector<Ore> *ores) {
  if (map.data.empty() || map.width <= 0 || map.height <= 0) {
    std::cerr << "Error: Thickness map is empty." << std::endl;
    return false;
  }

  // 1. Load X-ray Image (Grayscale)
  cv::Mat xray_image = cv::imread(xray_filename, cv::IMREAD_GRAYSCALE);
  if (xray_image.empty()) {
    std::cerr << "Error: Could not read X-ray image from " << xray_filename
              << std::endl;
    return false;
  }

  std::cout << "Loaded X-ray Image: " << xray_image.cols << "x"
            << xray_image.rows << std::endl;

  // 2. Split (Left = Low Energy)
  int mid = xray_image.cols / 2;
  // Ensure mid is valid
  if (mid <= 0)
    return false;
  cv::Mat low_energy = xray_image(cv::Rect(0, 0, mid, xray_image.rows));

  // Flip horizontally as requested (original is mirrored)
  cv::flip(low_energy, low_energy, 1);

  // 3. Crop Low Energy
  int new_width = mid - cut_left - cut_right;
  if (new_width <= 0) {
    std::cerr << "Error: X-ray cuts define invalid width (" << new_width << ")."
              << std::endl;
    return false;
  }
  // Validate cuts
  if (cut_left < 0 || cut_right < 0 || cut_left + cut_right >= mid) {
    std::cerr << "Error: Invalid X-ray cuts." << std::endl;
    return false;
  }

  cv::Mat sliced_low =
      low_energy(cv::Rect(cut_left, 0, new_width, low_energy.rows));

  // 3b. Apply X-ray ROI Crops
  // xray_crops are applied relative to the sliced_low image
  int crop_width = sliced_low.cols - xray_crops.left - xray_crops.right;
  int crop_height = sliced_low.rows - xray_crops.up - xray_crops.down;

  if (crop_width <= 0 || crop_height <= 0) {
    std::cerr << "Error: X-ray ROI crops result in empty image." << std::endl;
    return false;
  }

  // Validate bounds
  if (xray_crops.left < 0 || xray_crops.up < 0 ||
      xray_crops.left + crop_width > sliced_low.cols ||
      xray_crops.up + crop_height > sliced_low.rows) {
    std::cerr << "Error: Invalid X-ray ROI crops." << std::endl;
    return false;
  }

  cv::Mat cropped_xray = sliced_low(
      cv::Rect(xray_crops.left, xray_crops.up, crop_width, crop_height));

  // --- 3c. Apply Geometry Correction AFTER cropping ---
  cv::Mat corrected_xray;
  if (enable_geometry_correction && sod > 0.0f && sdd > 0.0f) {
    if (Utils::correctXrayGeometry(cropped_xray, corrected_xray, sod, sdd)) {
      std::cout << "Applied X-ray Geometry Correction (after crop)."
                << std::endl;
      std::cout << "  SOD: " << sod << " mm, SDD: " << sdd
                << " mm, M: " << (sdd / sod) << std::endl;
      std::cout << "  Corrected ROI size: " << corrected_xray.cols << "x"
                << corrected_xray.rows << std::endl;
    } else {
      std::cout << "Skipping X-ray Geometry Correction: Execution failed."
                << std::endl;
      corrected_xray = cropped_xray;
    }
  } else {
    std::cout << "Skipping X-ray Geometry Correction (disabled in config or "
                 "invalid parameters)."
              << std::endl;
    corrected_xray = cropped_xray;
  }

  // Clone to ensure contiguous memory and for conversion
  cv::Mat final_image_gray = corrected_xray.clone();
  cv::Mat final_image;
  cv::cvtColor(final_image_gray, final_image, cv::COLOR_GRAY2BGR);

  // 4. Prepare Thickness Map
  cv::Mat map_mat(map.height, map.width, CV_32FC1);
  float max_v = 0.0f;
  for (int r = 0; r < map.height; ++r) {
    for (int c = 0; c < map.width; ++c) {
      float val = map.data[r * map.width + c];
      if (val > max_v)
        max_v = val;
      map_mat.at<float>(r, c) = val;
    }
  }

  if (max_v <= 0.000001f)
    max_v = 1.0f;

  // Transpose to match the saved image orientation
  cv::Mat map_transposed;
  cv::transpose(map_mat, map_transposed);

  // 5. Crop LiDAR Map (Source ROI)
  cv::Rect src_roi(lidar_crops.left, lidar_crops.up,
                   map_transposed.cols - lidar_crops.left - lidar_crops.right,
                   map_transposed.rows - lidar_crops.up - lidar_crops.down);

  // Validate ROI
  if (src_roi.width <= 0 || src_roi.height <= 0 || src_roi.x < 0 ||
      src_roi.y < 0 || src_roi.x + src_roi.width > map_transposed.cols ||
      src_roi.y + src_roi.height > map_transposed.rows) {
    std::cerr << "Error: Invalid LiDAR crops for X-ray fusion." << std::endl;
    return false;
  }

  cv::Mat map_cropped = map_transposed(src_roi);

  // 6. Resize Map to Fit Sliced X-ray
  // We assume the mapped area corresponds to the visible X-ray area after cuts.
  cv::Mat map_resized;
  cv::resize(map_cropped, map_resized, final_image.size(), 0, 0,
             cv::INTER_LINEAR);

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
    float scale_x = static_cast<float>(final_image.cols) / map_cropped.cols;
    float scale_y = static_cast<float>(final_image.rows) / map_cropped.rows;
    float resolution_raw = map.resolution / unit_scale_;
    if (unit_scale_ <= 1e-6f)
      resolution_raw = 1.0f;

    for (const auto &ore : *ores) {
      if (!ore.point_indices || ore.point_indices->indices.empty())
        continue;

      float cx = (ore.min_x + ore.max_x) / 2.0f;
      float cy = (ore.min_y + ore.max_y) / 2.0f;

      int col_map = static_cast<int>((cx - map.min_x) / resolution_raw);
      int row_map = static_cast<int>((map.max_y - cy) / resolution_raw);

      // Transposed
      int r_trans = col_map;
      int c_trans = row_map;

      // Crop
      int r_cropped = r_trans - src_roi.y;
      int c_cropped = c_trans - src_roi.x;

      // Scale
      int r_scaled = static_cast<int>(r_cropped * scale_y);
      int c_scaled = static_cast<int>(c_cropped * scale_x);

      int rgb_r = r_scaled;
      // Offset label slightly to the right so it doesn't obscure the center
      int rgb_c = c_scaled + 40;

      if (rgb_r >= 0 && rgb_r < final_image.rows && rgb_c >= 0 &&
          rgb_c < final_image.cols) {
        std::string label = ore.id;
        if (label.find("ore_") == 0)
          label = label.substr(4);

        // Use Green for text to contrast with Red thickness
        cv::putText(final_image, label, cv::Point(rgb_c, rgb_r),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
      }
    }
  }

  std::cout << "Saving X-ray Fusion to " << output_filename << std::endl;
  return cv::imwrite(output_filename, final_image);
}
