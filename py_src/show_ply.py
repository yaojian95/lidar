import open3d as o3d
import numpy as np
from importlib import reload
import cv2 as cv
import matplotlib.pyplot as plt
# import simple
# reload(simple)
import os
# print(os.path.exists("E:/multi_source_info/lidar/pcd_data/SaveData_contours_test_0227.ply"))
# pcd = o3d.io.read_point_cloud("E:/multi_source_info/lidar/pcd_data/pcd_data/combined_cloud_0121.ply")
# pcd = o3d.io.read_point_cloud("E:/multi_source_info/lidar//pcd_data/SaveData_0205_cropped.ply")
pcd = o3d.io.read_point_cloud("E:/multi_source_info/lidar/pcd_data/20260304_100_banyan_ores_xrt_pointcloud_data/SaveData_0304_banyan_1_50_cropped.ply")
# pcd = o3d.io.read_point_cloud("E:/multi_source_info/lidar/pcd_data/SaveData_contours_test_0227.ply")
print(f"点数: {len(pcd.points)}")
print(f"颜色: {'有' if pcd.has_colors() else '无'}")
print(f"法向量: {'有' if pcd.has_normals() else '无'}")

# 获取深度（Z值）
points = np.asarray(pcd.points)
print(points.shape)
depths = points[:, 2]  # 所有点的深度值

# 统计深度信息
print(f"最小深度: {depths.min():.3f}m")
print(f"最大深度: {depths.max():.3f}m")
print(f"平均深度: {depths.mean():.3f}m")

# 获取边界框
min_bound = pcd.get_min_bound()  # [min_x, min_y, min_z]
max_bound = pcd.get_max_bound()  # [max_x, max_y, max_z]

# 提取z方向的最小值和最大值
z_min = min_bound[2]
z_max = max_bound[2]

print(f"z方向最小值: {z_min}")
print(f"z方向最大值: {z_max}")

# 提取z方向的最小值和最大值
y_min = min_bound[1]
y_max = max_bound[1]

print(f"y方向最小值: {y_min}")
print(f"y方向最大值: {y_max}")

pcd.colors = o3d.utility.Vector3dVector()
o3d.visualization.draw_geometries([pcd])