#include "ore_analysis.h"
#include <algorithm>
#include <cmath>
#include <numeric>
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

void OreAnalyzer::setConfig(float unit_scale, float ground_threshold) {
  unit_scale_ = unit_scale;
  ground_threshold_ = ground_threshold;
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

  // Store the plane coefficients
  plane_a_ = coefficients->values[0];
  plane_b_ = coefficients->values[1];
  plane_c_ = coefficients->values[2];
  plane_d_ = coefficients->values[3];

  // Compute rotation to align plane normal to Z axis (0,0,1)
  Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1],
                         coefficients->values[2]);
  Eigen::Vector3f target_normal(0, 0, 1);

  // Simple way: create a rotation from normal to target_normal
  Eigen::Quaternionf rotation;
  rotation.setFromTwoVectors(normal, target_normal);

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();

  // Apply temporary transform to find d' (distance to origin after rotation)
  // The current plane equation is ax+by+cz+d=0.
  // We want the new plane to be z = 0.
  // Let's just transform the cloud first based on rotation.
  PointCloudPtr rotated_cloud(new PointCloud);
  pcl::transformPointCloud(*aligned_cloud_, *rotated_cloud, transform);

  // Now find the Z of the inliers in the rotated cloud to determine translation
  float z_sum = 0.0f;
  for (int idx : inliers->indices) {
    z_sum += rotated_cloud->points[idx].z;
  }
  float average_z = z_sum / inliers->indices.size();

  // Final translation to bring the plane to Z=0
  transform(2, 3) = -average_z;

  // Apply full transform
  pcl::transformPointCloud(*aligned_cloud_, *aligned_cloud_, transform);

  std::cout << "Aligned ground to Z=0. Translation Z: " << -average_z
            << std::endl;

  // Check orientation: Are most points above or below Z=0?
  // We expect ores to be "above" ground (Z > 0).
  // If the centroid of the cloud (relative to ground) is negative, we might
  // need to flip. Actually, let's just use a simple heuristic: if average Z of
  // all points is negative, flip. Since ground is at 0, if there are objects,
  // they pull the average up or down. Exception: if there is noise below
  // ground. Better: Check 90th percentile vs 10th percentile distance from 0?
  // Let's rely on the user's observation: "Z axis direction is opposite".
  // Let's check the average Z of the aligned cloud.
  double sum_z_aligned = 0;
  for (const auto &p : *aligned_cloud_)
    sum_z_aligned += p.z;
  if (sum_z_aligned < 0) {
    std::cout
        << "Detected inverted Z-axis (objects below ground). Flipping Z..."
        << std::endl;
    Eigen::Matrix4f flip = Eigen::Matrix4f::Identity();
    flip(2, 2) = -1.0f; // Flip Z
    pcl::transformPointCloud(*aligned_cloud_, *aligned_cloud_, flip);
  }

  // Calculate actual ground thickness (Robustly)
  recalculateGroundThreshold();

  return true;

  return true;
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

  Eigen::Quaternionf rotation;
  rotation.setFromTwoVectors(normal, target_normal);

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();

  // Apply rotation
  PointCloudPtr rotated_cloud(new PointCloud);
  pcl::transformPointCloud(*aligned_cloud_, *rotated_cloud, transform);

  // Compute translation based on plane equation
  // After rotation, the plane should be at some Z = constant
  // We can use the d coefficient to determine the offset
  float average_z = -d / sqrt(a * a + b * b + c * c); // Distance from origin
  transform(2, 3) = -average_z;

  // Apply full transform
  pcl::transformPointCloud(*aligned_cloud_, *aligned_cloud_, transform);

  std::cout << "Aligned ground using cached plane. Translation Z: "
            << -average_z << std::endl;

  // Check orientation (same heuristic)
  double sum_z_aligned = 0;
  for (const auto &p : *aligned_cloud_)
    sum_z_aligned += p.z;
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

  int ore_id = 0;
  for (const auto &it : cluster_indices) {
    Ore ore;
    ore.id = "ore_" + std::to_string(ore_id++);
    ore.point_indices.reset(new pcl::PointIndices);
    // Indices now map 1:1 since we are clustering the aligned_cloud_ directly
    ore.point_indices->indices = it.indices;
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
