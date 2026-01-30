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
- `unit_scale`: 物理单位转换系数。例如 `0.003375` 可能用于将特定传感器单位转换为米。
- `save_plane_equation`: **平面缓存开关**。`true` 表示保存检测结果，避免重复计算。
- `plane_a/b/c/d`: 缓存的平面方程系数。
- `belt_min_y`, `belt_max_y`: **皮带物理边界**。用于限制生成的厚度图的高度，确保图像与实地皮带宽度对应。
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

**使用示例**:
```cpp
YAML::Node config = Config::load("config.yaml");
std::string path = Config::get<std::string>(config, "pcd_path", "");
float scale = Config::get<float>(config, "unit_scale", 1.0f);
bool flag = Config::get<bool>(config, "visual_plane", true);
```

**优点**:
- 类型安全（编译时检查）
- 自动类型转换
- 简洁的 API

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
    // Step 2: 计算旋转矩阵
    Eigen::Vector3f normal(a, b, c);
    Eigen::Vector3f target_normal(0, 0, 1);  // Z 轴
    Eigen::Quaternionf rotation;
    rotation.setFromTwoVectors(normal, target_normal);
```

**旋转逻辑**:
- 将平面法向量 `(a, b, c)` 旋转到 `(0, 0, 1)`
- 使用四元数计算旋转矩阵

```cpp
    // Step 3: 计算平移量
    pcl::transformPointCloud(*aligned_cloud_, *rotated_cloud, transform);
    float average_z = (所有内点的 Z 坐标之和) / 内点数量;
    transform(2, 3) = -average_z;  // Z 方向平移
```

**平移逻辑**:
- 旋转后，地面可能在 Z=5 或 Z=-3
- 计算地面点的平均 Z 值
- 平移使其归零

```cpp
    // Step 4: 应用完整变换
    pcl::transformPointCloud(*aligned_cloud_, *aligned_cloud_, transform);
}
```

**结果**: 
- 地面点的 Z ≈ 0
- 矿石点的 Z = 实际厚度

**注意**: 此方法会自动将检测到的平面系数存储在 `plane_a_`, `plane_b_`, `plane_c_`, `plane_d_` 中。

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
    
    // 计算平移量
    float average_z = -d / sqrt(a*a + b*b + c*c);
    transform(2, 3) = -average_z;
    
    // 应用变换
    pcl::transformPointCloud(*aligned_cloud_, *aligned_cloud_, transform);
}
```

**优点**: 
- 速度快（无需 RANSAC 迭代）
- 结果一致（每次运行使用相同的平面）

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
) {
```

**Step 1: 过滤地面点**
```cpp
for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
    if (aligned_cloud_->points[i].z > ground_threshold_ * 2.0f) {
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

**Step 3: 创建 Ore 对象**
```cpp
for (const auto& it : cluster_indices) {
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

## 7. 融合与可视化 (Fusion & Visualization)

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

