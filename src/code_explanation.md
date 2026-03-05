# 矿石厚度计算系统 - 完整代码解析

## 目录
1. [系统架构](#系统架构)
2. [配置文件 (config.yaml)](#配置文件)
3. [工具函数 (utils)](#工具函数)
4. [核心分析类 (OreAnalyzer)](#核心分析类)
5. [主程序 (main.cpp)](#主程序)
6. [全局厚度图 (Global Thickness Map)](#全局厚度图与-opencv-集成)
7. [融合与可视化 (Fusion & Visualization)](#融合与可视化-fusion--visualization)

---

## 系统架构

```mermaid
graph TD
    A[config.yaml] --> B[main.cpp]
    B --> C[utils.h/cpp]
    B --> D[OreAnalyzer]
    D --> E[Ground Alignment]
    D --> F[Ore Detection]
    D --> G[Thickness Calculation]
    D --> M[Global Thickness Map]
    F --> H[LiDAR Clustering]
    F --> I[ROI Filtering]
    F --> J[Mask Filtering]
    G --> K[Average Thickness]
    G --> L[Local Thickness Map]
    M --> N[Generate 2D Map]
    M --> O[Save Image (OpenCV)]
```

---

## 配置文件

### `config.yaml`
```yaml
pcd_path: "E:/multi_source_info/lidar/pcd_data/combined_cloud_0121.ply"
unit_scale: 0.003375 # 单位转换系数 (0.003375 for millimeters to meters if raw data is large int)

# 平面检测选项
save_plane_equation: false  # 如果为 true，保存检测到的平面到配置文件
visual_plane: false         # 如果为 true，在可视化窗口中显示地面平面

# 平面方程 (当 save_plane_equation=true 时自动填充，或手动设置)
plane_a: 0.000003
plane_b: -0.008955
plane_c: 0.999960
plane_d: -2.044467
ground_threshold: 0.020702  # RANSAC 距离阈值

# 皮带边界 (Y轴)
belt_min_y: -458.361511
belt_max_y: 459.648468
fit_belt_edges: false       # 是否自动拟合皮带边缘

# 矿石检测参数 (欧几里得聚类)
cluster_tolerance: 20.0     # 聚类距离阈值
min_cluster_size: 50        # 最小点数
max_cluster_size: 50000     # 最大点数
```

**参数说明**:
- `pcd_path`: 点云文件路径。
- `unit_scale`: **核心物理缩放底数**。此参数仅负责将点云文件中记录的纯数值坐标`(x, y, z)`转换对齐至标准的**“米 (m)”**。例如，如果激光雷达记录点云的单位是毫米 (mm)，则必须严格配置 `0.001`（即 $1 mm \times 0.001 = 0.001 m$）。
- `save_plane_equation`: **平面缓存开关**。`true` 表示保存检测结果，避免重复计算。
- `plane_a/b/c/d`: 缓存的平面方程系数。
- `belt_min_y`, `belt_max_y`: **皮带物理边界**。限定厚度图高度的绝对基准。注意其数值域是在**原生点云坐标系**（如 mm）内定义的。
- `lidar_crop_up/down/left/right` (未展示在上方简略图中): **物理边缘裁切冗余量**。这些数值直接定义在点云的**原生物理坐标系单元**上（举例：如果设备原始数据是毫米，填 `150` 代表向内硬裁去 $150 mm$ 的物理空间）。裁切值直接以原生单位与点云包围盒边界做加减运算（不经过 `unit_scale` 换算），用于滤除处于皮带最外缘的检测噪点并限制厚度图画幅。
- `cluster_tolerance`: **聚类容差**。判定两个点是否属于同一块矿石的距离阈值。
- `min/max_cluster_size`: 矿石点数的有效范围，过滤噪声和过大误检。

---

## 工具函数

### `utils.h` / `utils.cpp`

**重要变化**: 现在使用 `yaml-cpp` 库代替手写解析，代码量减少 50%+。

#### 类: `Config`

**静态方法 1: `load`**
```cpp
static YAML::Node load(const std::string& config_file);
```

**功能**: 加载 YAML 配置文件

**实现**:
```cpp
YAML::Node Config::load(const std::string& config_file) {
    try {
        return YAML::LoadFile(config_file);
    } catch (const YAML::Exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return YAML::Node();
    }
}
```

**使用示例**:
```cpp
YAML::Node config = Config::load("config.yaml");
```

---

**静态方法 2: `save`**
```cpp
static void save(const std::string& config_file, const YAML::Node& config);
```

**功能**: 保存配置到文件

**实现**:
```cpp
void Config::save(const std::string& config_file, const YAML::Node& config) {
    std::ofstream fout(config_file);
    fout << config;
}
```

---

**模板方法 3: `get<T>`**
```cpp
template<typename T>
static T get(const YAML::Node& config, const std::string& key, const T& default_val);
```

**功能**: 从配置中读取指定类型的值，支持默认值

**实现**:
```cpp
template<typename T>
static T get(const YAML::Node& config, const std::string& key, const T& default_val) {
    if (config[key]) {
        return config[key].as<T>();
    }
    return default_val;
}
```

**新增结构: `AppConfig` 及解析方法**
```cpp
struct AppConfig { ... };
static AppConfig parseAppConfig(const YAML::Node &config);
```

**功能**: 将原先散落在 `main.cpp` 各处的 `Config::get<T>` 统一汇聚到一处，返回一个包含所有参数强类型成员的结构体 `AppConfig`，供主程序直接调用。

---

#### 结构体: `PlaneCoefficients`

```cpp
struct PlaneCoefficients {
    float a, b, c, d;
    bool valid = false;
    
    static PlaneCoefficients fromConfig(const YAML::Node& config);
    void saveToConfig(YAML::Node& config) const;
};
```

**方法 1: `fromConfig`**
```cpp
PlaneCoefficients PlaneCoefficients::fromConfig(const YAML::Node& config) {
    PlaneCoefficients plane;
    if (config["plane_a"] && config["plane_b"] && 
        config["plane_c"] && config["plane_d"]) {
        plane.a = config["plane_a"].as<float>();
        plane.b = config["plane_b"].as<float>();
        plane.c = config["plane_c"].as<float>();
        plane.d = config["plane_d"].as<float>();
        plane.valid = true;
    }
    return plane;
}
```

**方法 2: `saveToConfig`**
```cpp
void PlaneCoefficients::saveToConfig(YAML::Node& config) const {
    config["plane_a"] = a;
    config["plane_b"] = b;
    config["plane_c"] = c;
    config["plane_d"] = d;
}
```

**使用示例**:
```cpp
// 读取
YAML::Node config = Config::load("config.yaml");
PlaneCoefficients plane = PlaneCoefficients::fromConfig(config);
if (plane.valid) {
    // 使用缓存的平面
}

// 保存
PlaneCoefficients new_plane{0.1f, 0.2f, 0.9f, -0.5f, true};
new_plane.saveToConfig(config);
Config::save("config.yaml", config);
```
```cpp
std::string readConfigStr(const std::string& config_file, const std::string& key);
```

**功能**: 从 YAML 配置文件读取字符串值

**实现逻辑**:
1. 打开配置文件
2. 逐行读取
3. 移除 `#` 注释
4. 查找匹配的 `key:`
5. 提取冒号后的值，去除引号和空格

**示例**:
```cpp
std::string path = readConfigStr("config.yaml", "pcd_path");
// 返回: "E:/multi_source_info/lidar/pcd_data/vzcoal-detect-data-16.pcd"
```

#### 函数 2: `readConfigFloat`
```cpp
float readConfigFloat(const std::string& config_file, const std::string& key, float default_val);
```

**功能**: 从配置文件读取浮点数值，如果读取失败则返回默认值

**实现逻辑**:
1. 调用 `readConfigStr` 获取字符串
2. 使用 `std::stof` 转换为浮点数
3. 如果转换失败（异常），返回 `default_val`

#### 函数 3: `readConfigBool`
```cpp
bool readConfigBool(const std::string& config_file, const std::string& key, bool default_val);
```

**功能**: 从配置文件读取布尔值

**实现逻辑**:
1. 调用 `readConfigStr` 获取字符串
2. 转换为小写后判断是否为 `"true"`, `"1"`, `"yes"`
3. 如果是则返回 `true`，否则返回 `false`

#### 函数 4: `readPlaneFromConfig`
```cpp
struct PlaneCoefficients {
    float a, b, c, d;
    bool valid = false;
};
PlaneCoefficients readPlaneFromConfig(const std::string& config_file);
```

**功能**: 从配置文件读取平面方程系数

**实现逻辑**:
1. 读取 `plane_a`, `plane_b`, `plane_c`, `plane_d` 四个值
2. 如果至少有一个非零，则 `valid = true`
3. 返回 `PlaneCoefficients` 结构体

**使用场景**: 检查是否存在缓存的平面方程，避免重复 RANSAC 计算

#### 函数 5: `writePlaneToConfig`
```cpp
void writePlaneToConfig(const std::string& config_file, float a, float b, float c, float d);
```

**功能**: 将平面方程系数写入配置文件

**实现逻辑**:
1. 读取配置文件所有行
2. 删除旧的 `plane_a/b/c/d` 行（如果存在）
3. 在文件末尾追加新的平面系数
4. 输出保存成功的消息

**使用场景**: 当 `save_plane_equation=true` 时，保存 RANSAC 检测到的平面

---

## 核心分析类

### `ore_analysis.h` / `ore_analysis.cpp`

### 数据结构: `Ore`

```cpp
struct Ore {
    std::string id;                          // 矿石编号
    pcl::PointIndicesPtr point_indices;      // 点云索引
    
    // 统计信息
    float avg_thickness;                     // 平均厚度 (物理单位)
    float max_thickness;                     // 最大厚度 (物理单位)
    
    // 物理边界框
    float min_x, max_x, min_y, max_y;
    
    // 厚度图 (2D 网格)
    std::vector<std::vector<float>> thickness_map;
    float map_resolution;                    // 网格分辨率
    float map_min_x, map_min_y;              // 网格原点
};

// 全局厚度图结构体
struct ThicknessMap {
    int width;
    int height;
    float resolution;        // meters per pixel
    std::vector<float> data; // 1D grid (row-major)
    float min_x;
    float max_y;             // Top-left origin convention
};
```

---

### 类: `OreAnalyzer`

#### 成员变量
```cpp
private:
    PointCloudPtr original_cloud_;    // 原始点云
    PointCloudPtr aligned_cloud_;     // 对齐后的点云 (地面在 Z=0)
    float unit_scale_;                // 单位转换系数
    float ground_threshold_;          // 地面平面拟合阈值 (默认 0.02m)
    
    // 缓存的平面系数
    float plane_a_ = 0.0f;
    float plane_b_ = 0.0f;
    float plane_c_ = 1.0f;
    float plane_d_ = 0.0f;

public:
    // ... 现有方法 ...

    // 生成全局厚度图
    void generateGlobalThicknessMap(float resolution, const std::string& output_filename, float min_val = 0.0f);

    // 保存厚度图为图片 (使用 OpenCV)
    bool saveThicknessMapToImage(const ThicknessMap& map, const std::string& filename);
```

---

### 核心方法详解

#### 1. `alignToGround()` - 地面对齐

**目的**: 将传送带平面旋转平移到 Z=0，使得 Z 坐标直接等于物理厚度

**算法流程**:

```cpp
bool OreAnalyzer::alignToGround() {
    // Step 1: RANSAC 平面拟合
    pcl::SACSegmentation<PointT> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ground_threshold_);
    seg.segment(*inliers, *coefficients);
```

**RANSAC 原理**:
1. 随机选 3 个点 → 计算平面方程 $ax + by + cz + d = 0$
2. 统计距离 < `threshold` 的点（内点）
3. 重复多次，选内点最多的平面
4. 输出: `coefficients = [a, b, c, d]`（法向量 + 距离）

```cpp
    // Step 2: 调用统一的对齐流
    return alignToGroundWithPlane(
        coefficients->values[0], coefficients->values[1], 
        coefficients->values[2], coefficients->values[3]);
}
```

**对齐逻辑**:
- RANSAC 成功提取出这 4 个平面系数值之后，直接将它们全权交由缓存驱动的核心函数 `alignToGroundWithPlane` 执行具体的点云旋转、代数平移与轴向翻转等全部复合矩阵操作，统一代码流向。
- 如果提取失败，程序将中止对齐。

**注意**: 此方法实际上也是通过向核心流水线输送 `plane_a/b/c/d` 参数，间接存储了当前实时检测到的地面系数状态。

---

#### 1.5 `alignToGroundWithPlane()` - 使用缓存平面对齐

**目的**: 跳过 RANSAC 计算，直接使用已知的平面方程进行对齐

**使用场景**: 
- 配置文件中已有 `plane_a/b/c/d` 且 `save_plane_equation=false`
- 避免每次运行都重新计算 RANSAC（节省时间）

**算法流程**:
```cpp
bool OreAnalyzer::alignToGroundWithPlane(float a, float b, float c, float d) {
    // 存储系数
    plane_a_ = a; plane_b_ = b; plane_c_ = c; plane_d_ = d;
    
    // 计算旋转矩阵（与 alignToGround 相同）
    Eigen::Vector3f normal(a, b, c);
    Eigen::Quaternionf rotation;
    rotation.setFromTwoVectors(normal, Eigen::Vector3f(0, 0, 1));
    
    // 代数计算平移量 (距离公式倒推)
    // 根据空间解析几何「点到平面的距离公式」，原点 (0,0,0) 到平面 ax+by+cz+d=0 的有向距离即为 d / sqrt(a*a+b*b+c*c)。
    // 由于我们刚刚计算把该平面的法向量旋转到了 Z 轴正方向 (0,0,1)，此时这个平面与 XY 平面平行。
    // 因此，它的 Z 轴截距 (即整个平面上所有点的统一 Z 坐标) 必然就等于该距离的反向。
    float average_z = -d / sqrt(a*a + b*b + c*c);
    transform(2, 3) = -average_z; // 在 4x4 齐次矩阵中注入 Z 方向平移分量，以将其归零
    
    // 应用复合变换 (此时的 transform 矩阵已同时整合了旋转和平移，一步完成最终的对齐，省去了冗余的中间态计算)
    pcl::transformPointCloud(*aligned_cloud_, *aligned_cloud_, transform);
}
```

**优点**: 
- 速度极快（既跳过了 RANSAC 迭代，又通过纯代数几何公式直接解算出平移距离，无需经历任何中间态点云的旋转与遍历提取）
- 结果一致（每次运行都严格使用相同的缓存平面参数）

---

#### 1.6 `getPlaneCoefficients()` - 获取平面系数

**目的**: 获取当前存储的平面方程系数

```cpp
void OreAnalyzer::getPlaneCoefficients(float& a, float& b, float& c, float& d) const {
    a = plane_a_;
    b = plane_b_;
    c = plane_c_;
    d = plane_d_;
}
```

**使用场景**: 
- 在 `main.cpp` 中保存平面到配置文件
- 在可视化时显示平面

---

#### 2. `detectByLidar()` - 基于点云的自动检测

**目的**: 使用聚类算法自动找到独立的矿石块

**算法流程**:

```cpp
std::vector<Ore> OreAnalyzer::detectByLidar(
    float cluster_tolerance,  // 聚类距离阈值 (默认 0.05m)
    int min_size,             // 最小点数 (默认 50)
    int max_size              // 最大点数 (默认 50000)
) 
```

**Step 1: 过滤地面点**
```cpp
for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
    if (aligned_cloud_->points[i].z > ground_threshold_) {
        non_ground_indices->indices.push_back(i);
    }
}
```
- 只保留 Z > 阈值的点（矿石）
- 地面点被排除

**Step 2: 欧几里得聚类**
```cpp
pcl::EuclideanClusterExtraction<PointT> ec;
ec.setClusterTolerance(cluster_tolerance);  // 5cm
ec.setMinClusterSize(min_size);
ec.setMaxClusterSize(max_size);
ec.extract(cluster_indices);
```

**聚类原理**:
1. 选一个未访问的点作为种子
2. 找到所有距离 < `cluster_tolerance` 的邻居
3. 递归扩展邻居的邻居
4. 形成一个簇（cluster）
5. 重复直到所有点被访问

**Step 3: 混合架构聚类后处理 (Hybrid Clustering)**
因为纯距离驱动的 `EuclideanClusterExtraction` 容易将物理上挨得很近的多个矿石误识别为一块大的。现在引入了智能联合诊断：

```cpp
if (hybrid_strategy_ > 0) {
    bool suspicious = false;
    // 策略 1: 宽高比检测 (长细比排查)
    if (hybrid_strategy_ == 1 && isClusterSuspiciousAspectRatio(cluster, aspect_ratio_threshold_)) suspicious = true;
    
    // 策略 2: 下凹密实度检测 (异形排查)
    else if (hybrid_strategy_ == 2 && isClusterSuspiciousConcavity(cluster, density_threshold_)) suspicious = true;
    
    // 策略 3: 多焦点Z图检测 (扫描高峰排查)
    else if (hybrid_strategy_ == 3 && isClusterSuspiciousMultiPeak(cluster)) suspicious = true;

    if (suspicious) {
        // 如果触发了上述警报，挂载带法线估计的超声波切刀 RegionGrowing 区域生长法
        auto sub_clusters = applyRegionGrowing(cluster, rg_smoothness_, rg_curvature_, min_size, max_size);
        // ...用 sub_clusters 替换原有的粗聚类
    }
}
```

**诊断手段与二次切割原理**:
1. 宽长比失调: `max(X / Y, Y / X) > threshold` 表明极有可能是两块石头连一块儿了。
2. 形状下凹/不密实: 将实际点云反投影至其自己的包围边界框 `[MinX, MaxX] × [MinY, MaxY]` 计算 2D 网格填充率，占比极少说明形状像葫芦或哑铃。
3. `MultiPeak`: 在内部生成 Z轴 微缩地形图，如果有多个互不隶属的高耸山峰，必是多块分离的石头。
4. **`RegionGrowing` (区域生长切断)**: 使用法线 `pcl::NormalEstimation` 获取点云表面切角。石头堆叠处通常会产生锐利的高折变角度凹槽（两球相交处），`reg.setSmoothnessThreshold()` 一旦扫描到折角大于给定的平滑容忍度（默认 15度），生长即刻中断，从而完美沿石头接缝处切成互相独立的新个体！

**Step 4: 创建 Ore 对象**
```cpp
for (const auto& it : final_cluster_indices) {
    Ore ore;
    ore.id = "ore_" + std::to_string(ore_id++);
    ore.point_indices = 映射回原始点云的索引;
    ores.push_back(ore);
}
```

---

#### 3. `detectByROI()` - 基于 X 射线 ROI 的检测

**目的**: 根据外部提供的矩形区域提取矿石点

```cpp
Ore OreAnalyzer::detectByROI(
    float min_x, float max_x,
    float min_y, float max_y
) {
    Ore ore;
    for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
        const auto& pt = aligned_cloud_->points[i];
        if (pt.x >= min_x && pt.x <= max_x && 
            pt.y >= min_y && pt.y <= max_y &&
            pt.z > ground_threshold_) {
            ore.point_indices->indices.push_back(i);
        }
    }
    return ore;
}
```

**使用场景**: X 射线图像检测到矿石位置 `[100, 150] x [200, 250]` 像素，转换为物理坐标后调用此函数

---

#### 4. `detectByMask()` - 基于 X 射线 Mask 的检测

**目的**: 根据任意形状的二值掩码提取矿石点

```cpp
Ore OreAnalyzer::detectByMask(
    const std::vector<uint8_t>& mask,  // 1D 数组，1=矿石，0=背景
    int width, int height,             // Mask 尺寸
    float mask_min_x, float mask_max_x,
    float mask_min_y, float mask_max_y // Mask 覆盖的物理区域
) {
```

**坐标映射**:
```cpp
float pixel_w = (mask_max_x - mask_min_x) / width;
float pixel_h = (mask_max_y - mask_min_y) / height;

for (点云中的每个点 pt) {
    int px = (pt.x - mask_min_x) / pixel_w;
    int py = (pt.y - mask_min_y) / pixel_h;
    
    if (mask[py * width + px] > 0) {  // 在掩码内
        ore.point_indices->indices.push_back(i);
    }
}
```

**使用场景**: X 射线图像经过语义分割得到精确的矿石轮廓

---

#### 4.5- **`extractOresFromImage(...)`**: 新增的两个重载提取方法。它们接收彩色 (`rgb`) 或灰度 (`xrt`) 图像和对应的物理坐标包围盒，将其转换为二值化遮罩矩阵，并通过 `cv::findContours` 提取轮廓区域内每个矿石碎片，最后将各碎片转化为附带基于图像像素尺寸计算得出的精确物理坐标框的新类型 `ImageContour` 的集合。
- **`computeCropBounds(lidar_crops, roi_min_x, roi_max_x, roi_min_y, roi_max_y)`**: 统一物理边界求解函数。基于原始点云包围盒（`getMinMax3D`）和皮带边界（`belt_min/max_y`）计算基础范围，然后**直接以原生点云单位**（如 mm）施加 `lidar_crop_*` 裁切值（不经过 `unit_scale` 转换）。确保 `app_pipeline.cpp` 和 `ore_analysis.cpp` 中所有需要裁切边界的流程共用同一套精确的物理 XY 范围。
- **`ImageContour`**: 封装了从图像中提取的单体矿石轮廓特征的结构体，除了包含裁剪后的 8-bit 单通道二值化 `cv::Mat mask` 对象外，还保存有在全局物理地图中所处区域精确定位用的坐标变量 `phys_min_x/max_x/min_y/max_y` 和对应的像素级宽高。
```cpp
struct ImageContour {
  std::vector<uint8_t> mask;
  int width = 0;
  int height = 0;
  float phys_min_x = 0.0f, phys_max_x = 0.0f;
  float phys_min_y = 0.0f, phys_max_y = 0.0f;
};

std::vector<ImageContour> OreAnalyzer::extractOresFromImage(
    const cv::Mat& image, 
    float phys_image_min_x, float phys_image_max_x, 
    float phys_image_min_y, float phys_image_max_y
) {
```

**实现逻辑**:
1. **灰度与二值化**: `cv::cvtColor` 转灰度后，使用 `cv::threshold` (结合大津法 `cv::THRESH_OTSU`) 自动分离矿石与背景。
2. **形态学去噪**: 使用椭圆结构核通过 `cv::morphologyEx` 执行开闭运算，消除孤立噪点并平滑边缘。
3. **寻找轮廓**: `cv::findContours` 提取外层独立轮廓。
4. **物理映射**: 根据传入的全局图像物理边界 (`phys_image_min_x` 等) 与图像总像素宽高比，线性插值出当前轮廓 `cv::boundingRect` 对应的专属局部物理坐标。
5. **截取掩码**: 将轮廓外接矩形范围内的二值块抠出，展平存入 `std::vector<uint8_t>` 作为 `mask` 返回。

**使用场景**: 融合模式下系统启用由图像主导的矿石分割，替代单纯的点云距离聚类。

---

#### 5. `computeStats()` - 厚度计算

```cpp
void OreAnalyzer::computeStats(
    Ore& ore,
    bool generate_map,      // 是否生成厚度图
    float map_res           // 网格分辨率 (默认 0.01m = 1cm)
) {
```

**Part A: 基础统计（平均厚度）**
```cpp
for (int idx : ore.point_indices->indices) {
    float z = aligned_cloud_->points[idx].z;
    float physical_z = z * unit_scale_;  // 转换为物理单位
    
    sum_z += physical_z;
    if (physical_z > max_z) max_z = physical_z;
}

ore.avg_thickness = sum_z / 点数;
ore.max_thickness = max_z;
```

**Part B: 厚度图生成（插值）**

如果 `generate_map = true`:

**Step 1: 创建 2D 网格**
```cpp
int w = ceil((max_x - min_x) / map_res);
int h = ceil((max_y - min_y) / map_res);
ore.thickness_map.assign(h, std::vector<float>(w, -1.0f));  // -1 表示空
```

**Step 2: 点云投影到网格**
```cpp
for (点云中的每个点) {
    int col = (pt.x - min_x) / map_res;
    int row = (pt.y - min_y) / map_res;
    
    cell_sums[row][col] += pt.z;
    cell_counts[row][col]++;
}

// 计算每个格子的平均值
thickness_map[r][c] = cell_sums[r][c] / cell_counts[r][c];
```

**Step 3: 插值填充空洞**
```cpp
for (迭代 3 次) {
    for (每个空格子 [r, c]) {
        // 3x3 邻域平均
        sum = 0, count = 0;
        for (nr = r-1 to r+1) {
            for (nc = c-1 to c+1) {
                if (thickness_map[nr][nc] >= 0) {
                    sum += thickness_map[nr][nc];
                    count++;
                }
            }
        }
        if (count > 0) {
            new_map[r][c] = sum / count;
        }
    }
}
```

**插值原理**: 
- 使用邻居的平均值填充空格
- 迭代 3 次可以填充较大的空洞
- 更高级的方法: IDW (反距离加权)、Kriging

---

#### 6. `saveThicknessMapToImage()` - 保存与底图着色

```cpp
bool OreAnalyzer::saveThicknessMapToImage(
    const ThicknessMap &map, 
    const std::string &filename,
    float max_val,
    const std::vector<Ore> *ores) 
```

**功能扩展**: 除了传统的将网格压印成灰度深度图并执行转置外，如果主流程中关闭了视觉融合 (`fuse: false`)，则将原始的 255 灰度通道扩展为 BGR 三通道，并且在此函数内部提前根据物理坐标 `[min_x, max_x/y]` 将它们逆换算成转置后图片的像素系，**直接在素颜深度图 `thickness_map.png` 上绘制绿色的矿石 ID 框图**。

---

## 主程序

### `main.cpp` 流程

```cpp
int main() {
    // 1. 加载配置（使用 yaml-cpp）
    YAML::Node config = Config::load("config.yaml");
    if (!config) {
        std::cerr << "Failed to load config!" << std::endl;
        return -1;
    }
    
    // 2. 读取配置项
    std::string pcd_path = Config::get<std::string>(config, "pcd_path", "");
    float unit_scale = Config::get<float>(config, "unit_scale", 1.0f);
    bool save_plane = Config::get<bool>(config, "save_plane_equation", false);
    bool visual_plane = Config::get<bool>(config, "visual_plane", true);
    
    // 3. 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (ext == "pcd") {
        pcl::io::loadPCDFile(pcd_path, *cloud);
    } else if (ext == "ply") {
        pcl::io::loadPLYFile(pcd_path, *cloud);
    }
    
    // 4. 创建分析器
    OreAnalyzer analyzer;
    analyzer.setPointCloud(cloud);
    analyzer.setConfig(unit_scale);
    
    // 5. 地面对齐（智能选择：缓存 vs RANSAC）
    PlaneCoefficients cached_plane = PlaneCoefficients::fromConfig(config);
    bool aligned = false;
    
    if (cached_plane.valid && !save_plane) {
        // 使用缓存的平面
        aligned = analyzer.alignToGroundWithPlane(
            cached_plane.a, cached_plane.b, cached_plane.c, cached_plane.d);
    } else {
        // 重新检测平面
        aligned = analyzer.alignToGround();
        
        // 如果需要，保存平面到配置文件
        if (aligned && save_plane) {
            float a, b, c, d;
            analyzer.getPlaneCoefficients(a, b, c, d);
            
            PlaneCoefficients plane{a, b, c, d, true};
            plane.saveToConfig(config);
            Config::save("config.yaml", config);  // 保存到文件
        }
    }
    
    // 6. 检测矿石
    if (aligned) {
        auto ores = analyzer.detectByLidar();
        
        // 7. 计算厚度
        for (auto& ore : ores) {
            analyzer.computeStats(ore, false);
            std::cout << ore.id << ": " << ore.avg_thickness << std::endl;
        }
    }
    
    // 8. 可视化
    cloud = analyzer.getAlignedCloud();
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> z_color(cloud, "z");
    viewer->addPointCloud(cloud, z_color, "cloud");
    
    // 9. 可视化地面平面（可选）
    if (visual_plane && aligned) {
        pcl::ModelCoefficients plane_coeff;
        plane_coeff.values = {0.0, 0.0, 1.0, 0.0};
        viewer->addPlane(plane_coeff, "ground_plane");
    }
}
```
    
    // 2. 加载点云
    if (ext == "pcd") {
        pcl::io::loadPCDFile(pcd_path, *cloud);
    } else if (ext == "ply") {
        pcl::io::loadPLYFile(pcd_path, *cloud);
    }
    
    // 3. 创建分析器
    OreAnalyzer analyzer;
    analyzer.setPointCloud(cloud);
    analyzer.setConfig(unit_scale);
    
    // 4. 地面对齐（智能选择：缓存 vs RANSAC）
    PlaneCoefficients cached_plane = readPlaneFromConfig("config.yaml");
    bool aligned = false;
    
    if (cached_plane.valid && !save_plane) {
        // 使用缓存的平面
        std::cout << "Using cached plane equation." << std::endl;
        aligned = analyzer.alignToGroundWithPlane(cached_plane.a, cached_plane.b,
                                                   cached_plane.c, cached_plane.d);
    } else {
        // 重新检测平面
        std::cout << "Detecting ground plane with RANSAC..." << std::endl;
        aligned = analyzer.alignToGround();
        
        // 如果需要，保存平面到配置文件
        if (aligned && save_plane) {
            float a, b, c, d;
            analyzer.getPlaneCoefficients(a, b, c, d);
            writePlaneToConfig("config.yaml", a, b, c, d);
        }
    }
    
    // 5. 检测矿石
    if (aligned) {
        auto ores = analyzer.detectByLidar();
        
        // 6. 计算厚度
        for (auto& ore : ores) {
            analyzer.computeStats(ore, false);
            std::cout << ore.id << ": " << ore.avg_thickness << std::endl;
        }
    }
    
    // 7. 可视化
    cloud = analyzer.getAlignedCloud();
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> z_color(cloud, "z");
    viewer->addPointCloud(cloud, z_color, "cloud");
    
    // 8. 可视化地面平面（可选）
    if (visual_plane && aligned) {
        pcl::ModelCoefficients plane_coeff;
        plane_coeff.values = {0.0, 0.0, 1.0, 0.0};  // Z=0 平面
        viewer->addPlane(plane_coeff, "ground_plane");
    }
}
```

**可视化说明**:
- `PointCloudColorHandlerGenericField<PointT> z_color(cloud, "z")`: 根据 Z 值着色
- 地面（Z≈0）显示为蓝色
- 矿石（Z>0）显示为红色/黄色

---

## 构建配置

### `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.20)
project(pcl_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
find_package(PCL CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)  # 新增

add_executable(pcl_demo 
    src/main.cpp 
    src/utils.cpp 
    src/ore_analysis.cpp
)

target_link_libraries(pcl_demo PRIVATE 
    ${PCL_LIBRARIES} 
    yaml-cpp::yaml-cpp  # 新增
)
target_include_directories(pcl_demo PRIVATE ${PCL_INCLUDE_DIRS} include)
target_compile_definitions(pcl_demo PRIVATE ${PCL_DEFINITIONS})
```

**关键点**:
- `include` 目录被添加到搜索路径
- `src/ore_analysis.cpp` 必须被添加
- **新增**: `yaml-cpp` 库用于配置文件解析

**安装 yaml-cpp**:
```bash
# 使用 vcpkg
vcpkg install yaml-cpp

# 或使用系统包管理器
# Ubuntu/Debian
sudo apt-get install libyaml-cpp-dev

# macOS
brew install yaml-cpp
```

---

## 使用示例

### 场景 1: 自动检测所有矿石
```cpp
auto ores = analyzer.detectByLidar(0.05f, 50, 50000);
for (auto& ore : ores) {
    analyzer.computeStats(ore, false);
    std::cout << ore.avg_thickness << " meters" << std::endl;
}
```

### 场景 2: X 射线提供 ROI
```cpp
// X 射线检测到矿石在 [0.5, 0.8] x [1.0, 1.3] 米
Ore ore = analyzer.detectByROI(0.5, 0.8, 1.0, 1.3);
analyzer.computeStats(ore, true, 0.005);  // 生成 5mm 分辨率的厚度图

// 输出厚度图
for (int r = 0; r < ore.thickness_map.size(); ++r) {
    for (int c = 0; c < ore.thickness_map[r].size(); ++c) {
        std::cout << ore.thickness_map[r][c] << " ";
    }
    std::cout << std::endl;
}
```

### 场景 3: X 射线提供 Mask
```cpp
std::vector<uint8_t> mask = load_from_xray_image();  // 640x480
Ore ore = analyzer.detectByMask(mask, 640, 480, 0.0, 2.0, 0.0, 1.5);
analyzer.computeStats(ore, false);
```

---

## 参数调优指南

| 参数 | 默认值 | 调整建议 |
|------|--------|----------|
| `ground_threshold` | 0.02m | 传送带不平 → 增大到 0.03~0.05 |
| `cluster_tolerance` | 0.05m | 矿石间距小 → 减小到 0.02~0.03 |
| `min_cluster_size` | 50 | 小矿石被忽略 → 减小到 20~30 |
| `map_resolution` | 0.01m | 需要更细致 → 减小到 0.005 (5mm) |
| `unit_scale` | 1.0 | 根据实际单位调整 |

---

## 常见问题

### Q1: 为什么检测不到矿石？
**A**: 检查以下几点：
1. `alignToGround()` 是否成功？（输出 "Ground aligned successfully"）
2. 地面阈值是否合适？（矿石的 Z > `ground_threshold * 2`）
3. 聚类参数是否合理？（`min_cluster_size` 太大会忽略小矿石）

### Q2: 厚度值不准确？
**A**: 
1. 检查 `unit_scale` 是否正确
2. 查看原始点云的 Z 值范围（`pcl_viewer` 工具）
3. 验证地面对齐是否成功（地面点的 Z 应该接近 0）

### Q3: 厚度图有很多空洞？
**A**: 
1. 增加插值迭代次数（当前是 3 次）
2. 使用更小的 `map_resolution`
3. 考虑使用更高级的插值算法（如 IDW）

---

**最后更新**: 2026-01-21 16:35

**更新内容**:
- **重构为 yaml-cpp**: 使用标准库代替手写解析，代码量减少 50%+
- 添加了平面可视化功能 (`visual_plane`)
- 添加了平面方程缓存功能 (`save_plane_equation`)
- 新增 `alignToGroundWithPlane()` 和 `getPlaneCoefficients()` 方法

---

## 6. 全局厚度图与 OpenCV 集成

### 6.1 功能目标
将所有检测到的矿石厚度信息投影到一个二维平面的灰度图中，用于后续的拼接和分析。

### 6.2 实现原理 (`generateGlobalThicknessMap`)

**坐标映射**:
为了确保图片上的 1 个像素对应实际物理世界的固定长度（如 1cm），我们需要进行单位换算。
```cpp
// 物理分辨率 (米) -> 点云原始单位
float resolution_raw = resolution / unit_scale_;
```

**边界确定**:
- **X 轴 (长度)**: 由检测到的矿石实际分布范围决定，动态调整。
- **Y 轴 (宽度)**: 强制使用皮带的物理边界 (`belt_min_y` 到 `belt_max_y`)，确保每一帧生成的图片高度一致，方便拼接。

**投影逻辑**:
使用扁平的一维向量 `std::vector<float> data` 存储矩阵，减少内存碎片。
```cpp
int col = (pt.x - min_x) / resolution_raw;
int row = (max_y - pt.y) / resolution_raw; // 注意 Y 轴方向通常翻转
int index = row * width + col;
// 取最大厚度
if (thickness > data[index]) data[index] = thickness;
```

### 6.3 OpenCV 保存 (`saveThicknessMapToImage`)

**依赖**:
`CMakeLists.txt` 中集成了 OpenCV：
```cmake
find_package(OpenCV REQUIRED)
target_link_libraries(pcl_demo PRIVATE ${OpenCV_LIBS})
```

**实现**:
使用 `cv::Mat` 存储数据，并利用 `cv::imwrite` 保存为无损 PNG 格式。
在保存前，我们执行了 **转置操作 (`cv::transpose`)**，这是为了匹配下游 Python 拼接脚本 (`stack_png.py`) 的输入要求。

```cpp
cv::Mat image(h, w, CV_8UC1);
// ... 填充数据并归一化到 0-255 ...

cv::Mat transposed_image;
cv::transpose(image, transposed_image); // 核心：转置
cv::imwrite(filename, transposed_image);
```



---

### 7. 融合与可视化 (Fusion & Visualization)

**全局配置**:
- **参数**: `fuse` (在 `config.yaml` 中设置)
- **可选值**: `"rgb"`, `"xray"`, `"false"` (不区分大小写)

### RGB 融合
- **模式**: `fuse: "rgb"`
- **输入**: RGB 图像 (`stitched_ore.jpg`)
- **流程**:
    1. 读取 RGB 图像。
    2. 根据 `rgb_crop_*` 和 `lidar_crop_*` 参数对齐两个来源的 ROI。
    3. 将 LiDAR 厚度图投影到 RGB 图像上。
    4. 将厚度值归一化并写入红色通道 (Channel 2)。
    5. 在矿石位置标注 ID。
    6. 保存结果为 `fused_thickness.jpg`。

### X-ray 融合
- **模式**: `fuse: "xray"`

### X-ray 几何校正 (Geometry Correction)
- **输入**: 原始 X 射线探测器图像 (单通道灰度)
- **参数控制**: 受 `config.yaml` 中的布尔值开关 `enable_xray_geometry_correction` 控制。
- **功能**: 由于点光源的成像原理，X 射线线阵探测器上的图像放大率为 `M = sdd / sod` (sdd = 光源到探测器距离, sod = 光源到物体距离)。同时，由于皮带在运动，沿皮带方向（高度/Y轴）并不会产生因为点光源带来的额外畸变扩散，因此我们只在跨皮带方向（宽度/X轴）按比例缩小图像尺寸 (`1/M`)，以将探测器上的图像还原为物体所在平面的真实物理尺寸。
- **公共接口**: 该功能已被分离提炼至通用的 `Utils::correctXrayGeometry(cv::Mat& input, cv::Mat& output, sod, sdd)` 接口，方便任何拥有 OpenCV 矩阵的模块直接在内存中执行校正。
- **集成**: 在主程序中读取 `config.yaml` 里的参数，随后直接在融合模块对截取后的内存图像区块应用上述 `Utils` 接口。
- **使用示例**:
```cpp
// sod = 40mm, sdd = 60mm
Utils::correctXrayGeometry(input_mat, output_mat, 40.0f, 60.0f);
```
- **输入**: X 射线图像 (单通道灰度)
- **流程**:
    1. 前置步骤：检查并应用 `correctXrayGeometry` 对原始图像进行几何校正（依赖 `xray_sod` 和 `xray_sdd`）。
    2. 读取校正后的 X 射线图像。
    3. 分割为左右两半（左=低能，右=高能）。
    3. 根据 `xray_cut_*` 参数裁剪低能图像边缘。
    4. 根据 `xray_crop_*` 参数进一步选择 ROI。
    5. 将 LiDAR 厚度图调整大小并叠加到 X 射线图像的红色通道。
    6. 保存结果为 `fused_thickness_xray.jpg`。

### 7.1 双源 ROI 对齐 (Dual-Source ROI Alignment)

**背景**:
RGB 相机和 LiDAR 传感器通常具有不同的视场 (FOV) 和分辨率。直接叠加会导致像素错位。为了解决这个问题，我们引入了 **双源 ROI 裁剪机制**。

**原理**:
1. **定义源 ROI (LiDAR)**: 在原始厚度图上定义一个有效矩形区域 (`lidar_crop_*`)。
2. **定义目标 ROI (RGB)**: 在原始 RGB 图像上定义一个有效矩形区域 (`rgb_crop_*`)。
3. **映射与缩放**: 系统将 LiDAR ROI 内的数据缩放，使其精确覆盖 RGB ROI 的区域。

**代码实现**:
```cpp
// 1. 裁剪源 (LiDAR)
cv::Mat map_cropped = map_transposed(src_roi);

// 2. 裁剪目标 (RGB) - 输出图像尺寸等于此 ROI 尺寸
cv::Mat final_image = rgb_image(tgt_roi).clone();

// 3. 缩放源以匹配目标
cv::Mat map_resized;
cv::resize(map_cropped, map_resized, final_image.size());

// 4. 融合与绘制
// 所有的标签坐标也会经过同样的 Transform (Shift -> Scale -> Shift)
```

### 7.2 配置参数 (Configuration)

新的融合参数位于 `config.yaml` 中：

```yaml
# RGB ROI Crops (定义 RGB 图片上的有效区域)
rgb_crop_up: 700   # 顶部裁剪掉 700 像素 (例如去除无效背景)
rgb_crop_down: 0
rgb_crop_left: 0
rgb_crop_right: 0

# LiDAR ROI Crops (定义厚度图上的有效区域)
# 调整这些值直到 LiDAR 图像的内容与 RGB 裁剪区域的内容完全对齐
lidar_crop_up: 0
lidar_crop_down: 0
lidar_crop_left: 15 # 左侧裁剪掉 15 像素
lidar_crop_right: 0
```

```

### 7.3 深度图全对齐与纯净图层 (Method 2 升级)

在系统的演进中，为了实现完美无损且坐标一致的雷达-视觉融合，取消了传统的“在融合阶段强行推算并裁剪低清雷达图”的操作。取而代之的是采用 **全局前置全采样** 以及 **自适应扩充对齐 (alignThicknessMapToImage)** 策略。

#### 执行流程与纯净图层 (`ores_thickness_map`)

在 `app_pipeline.cpp` 的核心控制流中，关于深度图的生成与使用呈现出双保险和双用途的特色设计：

1. **背景参考图 (`thickness_map`)**: 
   - 率先生成覆盖整个传送带/ROI区域的物理底图。
   - 这张图用来在随后的基于图像（Mask 检测）模式下，通过 `detectByMask` 直接被像素级读取，完成 2D 到 3D 的坐标逆映射寻找点云块。
2. **纯净矿石图 (`ores_thickness_map`)**:
   - 当检测算法（不论是纯 LiDAR 聚类，还是 Mask 反抠）执行完毕，并确认了当前帧所有的有效矿石实例 `std::vector<Ore> ores` 后，二次调用 `generateGlobalThicknessMap` 并仅仅传入被确认的 `ores` 对象列表。
   - 这会形成一张等比大小，**但是背景和地面噪声全部被掏空**的纯净深度底图。
3. **强制升维重塑 (`alignThicknessMapToImage`)**:
   - 无论是第一张还是第二张底图，只要系统中启用了图像模态融合 (`fuse_mode=rgb|xray`)，这层点云深度图数据结构都会被强行转置并经过 `cv::resize`，**在像素数量和长宽方向上被拉伸得与高分辨率的高清相机照片一模一样**。
4. **图像 1:1 融合**:
   - 在进入 `fuseThicknessWithImage` 等融合渲染器时，它接收到的永远是被清洗过且被放大的 `ores_thickness_map`，而且具有与底层彩色图片一致的宽高和方向。这样整个 OpenCV 融合只剩下纯粹的矩阵贴图操作，从而彻底消除了过去复杂的转置与映射漂移 Bug，画面极为干净清晰。
