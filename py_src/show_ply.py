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
pcd = o3d.io.read_point_cloud("E:/multi_source_info/lidar//pcd_data/SaveData_0205_cropped.ply")
# pcd = o3d.io.read_point_cloud("E:/multi_source_info/lidar/pcd_data/SaveData_contours_test_0227.ply")
print(f"点数: {len(pcd.points)}")
print(f"颜色: {'有' if pcd.has_colors() else '无'}")
print(f"法向量: {'有' if pcd.has_normals() else '无'}")

pcd.colors = o3d.utility.Vector3dVector()
o3d.visualization.draw_geometries([pcd])