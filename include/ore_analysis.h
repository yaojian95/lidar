#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

struct Ore {
  std::string id;
  pcl::PointIndicesPtr point_indices;

  // Stats
  float avg_thickness = 0.0f;
  float max_thickness = 0.0f;

  // Physical bounding box (x, y)
  float min_x, max_x, min_y, max_y;

  // Thickness Map (2D Grid)
  // Resolution is defined by the calculator
  std::vector<std::vector<float>> thickness_map;
  float map_resolution = 0.01f;
  float map_min_x = 0.0f;
  float map_min_y = 0.0f;
};

class OreAnalyzer {
public:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::Ptr;

  struct ThicknessMap {
    int width;
    int height;
    float resolution;        // meters per pixel
    std::vector<float> data; // 1D grid (row-major)
    float min_x, max_y;      // physical coordinates of the map's origin
    float max_x, min_y;      // physical boundaries of the opposite corner
    bool is_image_aligned = false;
  };

  void alignThicknessMapToImage(ThicknessMap &map, cv::Size target_size);

  struct FusionCrops {
    int up = 0;
    int down = 0;
    int left = 0;
    int right = 0;
  };

  // Used to export masks/contours directly extracted from images
  // For cross-sensor matching
  struct ImageContour {
    cv::Mat mask; // Size equals the ThicknessMap bounding box
  };

  OreAnalyzer();

  void setPointCloud(PointCloudPtr cloud);
  void setConfig(float unit_scale, float ground_threshold = 0.02f);

  // Step 1: Align Z=0 to the ground plane
  // Returns true if a plane was found
  bool alignToGround();

  // Use pre-computed plane coefficients (skip RANSAC)
  bool alignToGroundWithPlane(float a, float b, float c, float d);

  // Get the plane coefficients (after alignToGround)
  void getPlaneCoefficients(float &a, float &b, float &c, float &d) const;

  // Get the aligned cloud (for visualization)
  // Get the aligned cloud (for visualization)
  PointCloudPtr getAlignedCloud() const { return aligned_cloud_; }

  // Step 2: Detection Methods
  std::vector<Ore> detectByLidar(float cluster_tolerance = 0.05f,
                                 int min_size = 50, int max_size = 50000);

private:
  // Hybrid Clustering Diagnostic Methods
  bool
  isClusterSuspiciousAspectRatio(const pcl::PointCloud<PointT>::Ptr &cluster,
                                 float threshold) const;
  bool isClusterSuspiciousConcavity(const pcl::PointCloud<PointT>::Ptr &cluster,
                                    float threshold) const;
  bool isClusterSuspiciousMultiPeak(
      const pcl::PointCloud<PointT>::Ptr &cluster) const;

  // Secondary Clustering for Suspicious Ores
  std::vector<pcl::PointIndices>
  applyRegionGrowing(const pcl::PointCloud<PointT>::Ptr &cluster,
                     float smoothness_deg, float curvature_thr, int min_size,
                     int max_size) const;

public:
  Ore detectByROI(float min_x, float max_x, float min_y, float max_y);
  // Method 2: Mask corresponds exactly to the 2D grid of the fully generated
  // ThicknessMap
  Ore detectByMask(const cv::Mat &mask, const ThicknessMap &map);

  std::vector<ImageContour>
  extractOresFromImage(const cv::Mat &image, const ThicknessMap &map,
                       FusionCrops lidar_crops, float unit_scale,
                       const cv::Mat &high_image = cv::Mat());

  // Step 3: Calculation
  // If generate_map is true, it builds the 2D thickness grid using
  // interpolation
  void computeStats(Ore &ore, bool generate_map = false, float map_res = 0.01f);

  // Global Thickness Map
  // Generate a global 2D matrix of thickness values covering all detected ores
  ThicknessMap generateGlobalThicknessMap(const std::vector<Ore> &ores,
                                          float resolution, bool use_crops,
                                          FusionCrops lidar_crops);

  // Save the thickness map as a PGM image (grayscale)
  // filename: output path
  // max_val: value mapped to 255 (if <=0, uses max in map)
  // ores: Optional pointer to ores list for labeling IDs
  bool saveThicknessMapToImage(const ThicknessMap &map,
                               const std::string &filename,
                               float max_val = -1.0f,
                               const std::vector<Ore> *ores = nullptr);

  // Fuse Thickness Map with RGB Image
  // rgb_image_cropped: already cropped RGB image (cv::Mat)
  // output_filename: path to save the fused image
  // channel: 0=Blue, 1=Green, 2=Red (default)
  // lidar_crops: Crops for the LiDAR Thickness Map
  // ores: Optional pointer to ores list for labeling IDs on the RGB image
  bool fuseThicknessWithImage(const ThicknessMap &map,
                              const cv::Mat &rgb_image_cropped,
                              const std::string &output_filename, int channel,
                              FusionCrops lidar_crops,
                              const std::vector<Ore> *ores = nullptr);

  // Fuse Thickness Map with Sliced Low-Energy X-ray Image
  // xray_image_cropped: already cropped and corrected X-ray image (cv::Mat)
  // output_filename: path to save the fused image
  // lidar_crops: Crops for the LiDAR Thickness Map
  // ores: Optional pointer to ores list for labeling IDs
  bool fuseThicknessWithXray(const ThicknessMap &map,
                             const cv::Mat &xray_image_cropped,
                             const std::string &output_filename,
                             FusionCrops lidar_crops,
                             const std::vector<Ore> *ores = nullptr);

private:
  PointCloudPtr original_cloud_;
  PointCloudPtr aligned_cloud_;
  float unit_scale_ = 1.0f;
  float ground_threshold_ = 0.02f;

  // Hybrid Clustering Configuration
  int hybrid_strategy_ = 0;
  float aspect_ratio_threshold_ = 2.5f;
  float density_threshold_ = 0.3f;
  float rg_smoothness_ = 15.0f;
  float rg_curvature_ = 1.0f;

public:
  float getGroundThreshold() const { return ground_threshold_; }

  // Cached plane coefficients
  float plane_a_ = 0.0f;
  float plane_b_ = 0.0f;
  float plane_c_ = 1.0f;
  float plane_d_ = 0.0f;

  // Helper to robustly determine threshold from aligned cloud
  void recalculateGroundThreshold();

  // Detect side panels (belt guards), calibrate unit scale, and filter them out
  // Returns true if side panels were successfully detected
  bool calibrateAndFilterSidePanels(float known_height_m,
                                    float &calculated_scale);

  float getBeltMinY() const { return belt_min_y_; }
  float getBeltMaxY() const { return belt_max_y_; }
  void setBeltBoundaries(float min_y, float max_y) {
    belt_min_y_ = min_y;
    belt_max_y_ = max_y;
  }
  void setGroundThresholdParams(float sigma, float margin) {
    ground_sigma_ = sigma;
    ground_margin_ = margin;
  }
  void setHybridClusteringConfig(int strategy, float aspect_ratio_thr,
                                 float density_thr, float rg_smoothness,
                                 float rg_curvature) {
    hybrid_strategy_ = strategy;
    aspect_ratio_threshold_ = aspect_ratio_thr;
    density_threshold_ = density_thr;
    rg_smoothness_ = rg_smoothness;
    rg_curvature_ = rg_curvature;
  }

  // Explicitly Apply Filtering using current belt boundaries
  void filterSidePanels();

  // Filter out ground points
  void filterGroundPoints();

private:
  // Belt Boundaries (Y-axis)
  float belt_min_y_ = -1e9f;
  float belt_max_y_ = 1e9f;

  // Ground Threshold Params
  float ground_sigma_ = 3.0f;
  float ground_margin_ = 0.005f;

  // Computes the physical boundaries of the cropped region for both image
  // extraction and fusion. It takes into account the original LiDAR point cloud
  // extent, belt bounds, global thickness map resolution, 5-pixel safe margin,
  // and the user-specified LiDAR crops.
  void computeCropBounds(const FusionCrops &lidar_crops, float unit_scale,
                         float &roi_min_x, float &roi_max_x, float &roi_min_y,
                         float &roi_max_y);
};
