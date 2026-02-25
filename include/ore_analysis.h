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
  PointCloudPtr getAlignedCloud() const { return aligned_cloud_; }

  // Step 2: Detection Methods
  std::vector<Ore> detectByLidar(float cluster_tolerance = 0.05f,
                                 int min_size = 50, int max_size = 50000);
  Ore detectByROI(float min_x, float max_x, float min_y, float max_y);
  // Mask is a flat array (row-major), 1=ore, 0=background
  // This function assumes the mask covers a specific physical area [mask_min_x,
  // mask_max_x] x [mask_min_y, mask_max_y]
  Ore detectByMask(const std::vector<uint8_t> &mask, int width, int height,
                   float mask_min_x, float mask_max_x, float mask_min_y,
                   float mask_max_y);

  // Step 3: Calculation
  // If generate_map is true, it builds the 2D thickness grid using
  // interpolation
  void computeStats(Ore &ore, bool generate_map = false, float map_res = 0.01f);

private:
  PointCloudPtr original_cloud_;
  PointCloudPtr aligned_cloud_;
  float unit_scale_ = 1.0f;
  float ground_threshold_ = 0.02f;

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

  // Global Thickness Map
public:
  struct ThicknessMap {
    int width;
    int height;
    float resolution;        // meters per pixel
    std::vector<float> data; // 1D grid (row-major)
    float min_x;
    float max_y; // Top-left origin convention usually
  };

  // Generate a global 2D matrix of thickness values covering all detected ores
  ThicknessMap generateGlobalThicknessMap(const std::vector<Ore> &ores,
                                          float resolution = 0.01f);

  // Save the thickness map as a PGM image (grayscale)
  // filename: output path
  // max_val: value mapped to 255 (if <=0, uses max in map)
  // Save the thickness map as a PGM image (grayscale)
  // filename: output path
  // max_val: value mapped to 255 (if <=0, uses max in map)
  // ores: Optional pointer to ores list for labeling IDs
  bool saveThicknessMapToImage(const ThicknessMap &map,
                               const std::string &filename,
                               float max_val = -1.0f,
                               const std::vector<Ore> *ores = nullptr);

  struct FusionCrops {
    int up = 0;
    int down = 0;
    int left = 0;
    int right = 0;
  };

  // Fuse Thickness Map with RGB Image
  // rgb_filename: path to the RGB image (e.g., stiched_ore.jpg)
  // output_filename: path to save the fused image
  // channel: 0=Blue, 1=Green, 2=Red (default)
  // rgb_crops: Crops for the RGB image
  // lidar_crops: Crops for the LiDAR Thickness Map
  // ores: Optional pointer to ores list for labeling IDs on the RGB image
  bool fuseThicknessWithImage(const ThicknessMap &map,
                              const std::string &rgb_filename,
                              const std::string &output_filename, int channel,
                              FusionCrops rgb_crops, FusionCrops lidar_crops,
                              const std::vector<Ore> *ores = nullptr);

  // Fuse Thickness Map with Sliced Low-Energy X-ray Image
  // Input: Single channel Grayscale (Left=Low, Right=High)
  // Cuts apply to the Left (Low Energy) half.
  // Performs geometry correction (sdd/sod scaling on X-axis) after cutting.
  bool fuseThicknessWithXray(const ThicknessMap &map,
                             const std::string &xray_filename,
                             const std::string &output_filename, int cut_left,
                             int cut_right, bool enable_geometry_correction,
                             float sod, float sdd, FusionCrops xray_crops,
                             FusionCrops lidar_crops,
                             const std::vector<Ore> *ores = nullptr);
};
