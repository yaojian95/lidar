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

  // 1. Filter out ground points (Z <= approx 0 + buffer)
  pcl::PointIndices::Ptr non_ground_indices(new pcl::PointIndices);
  for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
    if (aligned_cloud_->points[i].z >
        ground_threshold_ * 2.0f) { // Only take points clearly above ground
      non_ground_indices->indices.push_back(i);
    }
  }

  if (non_ground_indices->indices.empty())
    return ores;

  // 2. Euclidean Clustering
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(
      aligned_cloud_); // Use the full cloud, but we will extract only
                       // non-ground for clustering?
  // Actually, extract_clusters wants indices or a filtered cloud. Let's create
  // a sub-cloud for simplicity. Ideally we use setIndices, but let's stick to
  // extraction to avoid index mapping confusion initially.
  PointCloudPtr object_cloud(new PointCloud);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(aligned_cloud_);
  extract.setIndices(non_ground_indices);
  extract.setNegative(false);
  extract.filter(*object_cloud);

  // We need to map back indices from object_cloud to aligned_cloud_
  // This is tricky if we just copy. Let's keep a map.
  std::vector<int> object_to_original_map = non_ground_indices->indices;

  tree->setInputCloud(object_cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(object_cloud);
  ec.extract(cluster_indices);

  int ore_id = 0;
  for (const auto &it : cluster_indices) {
    Ore ore;
    ore.id = "ore_" + std::to_string(ore_id++);
    ore.point_indices.reset(new pcl::PointIndices);

    for (const auto &idx : it.indices) {
      // idx is index in object_cloud
      // map back to original aligned_cloud_
      ore.point_indices->indices.push_back(object_to_original_map[idx]);
    }
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
