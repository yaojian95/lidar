import open3d as o3d
import numpy as np
from importlib import reload
import cv2 as cv
import matplotlib.pyplot as plt
import os

# 加载点云文件
# file_path = "E:/multi_source_info/lidar/pcd_data/position_1_cropped.ply"
# file_path = "E:/multi_source_info/lidar/pcd_data/20260309_banyan_detong_63/savedata_1_63_cropped.ply"
file_path = "E:/multi_source_info/data_dir/20260325_yinshan/1_98_position_1_160kV_cropped.ply"
if not os.path.exists(file_path):
    print(f"Error: {file_path} not found.")
else:
    pcd = o3d.io.read_point_cloud(file_path)
    print(f"点数: {len(pcd.points)}")
    # print(f"颜色: {'有' if pcd.has_colors() else '无'}")
    # print(f"法向量: {'有' if pcd.has_normals() else '无'}")

    # 获取坐标点
    points = np.asarray(pcd.points)
    print(points.shape)

    # 统计深度信息 (Z轴)
    depths = points[:, 2]
    # print(f"最小深度: {depths.min():.3f}m")
    # print(f"最大深度: {depths.max():.3f}m")
    print(f"平均深度: {depths.mean():.3f}m")

    # 获取边界框 (Bounding Box)
    min_bound = pcd.get_min_bound()
    max_bound = pcd.get_max_bound()
    print(f"X范围: {min_bound[0]:.2f} 到 {max_bound[0]:.2f}")
    print(f"Y范围: {min_bound[1]:.2f} 到 {max_bound[1]:.2f}")
    print(f"Z范围: {min_bound[2]:.2f} 到 {max_bound[2]:.2f}")

    # --- 扫描分辨率分析 (Scanning Resolution Analysis) ---
    print("\n--- 扫描分辨率分析 (Analysis) ---")
    # 1. 计算每一帧的点数 (Points Per Frame) - 使用自相关分析 Y 轴周期性
    y_values = points[:, 1]
    # 取前2万点作为样本进行周期性探测
    n_sample = min(20000, len(y_values))
    y_small = y_values[:n_sample] - np.mean(y_values[:n_sample])
    corr = np.correlate(y_small, y_small, mode='full')
    corr = corr[len(corr)//2:]

    # 寻找第一个显著峰值进行周期估算
    min_search_dist = 100
    if len(corr) > min_search_dist:
        # np.argmax 返回最大值的索引
        points_per_frame = np.argmax(corr[min_search_dist:]) + min_search_dist
        print(f"每一帧的点数 (Points per frame): {points_per_frame}")
        
        # 2. 计算分辨率 (Resolution)
        # dy: 横向间距 (垂直于运动方向)
        y_width = max_bound[1] - min_bound[1]
        dy = y_width / (points_per_frame - 1) if points_per_frame > 1 else 0
        # print(f"横向分辨率 (dy): {dy:.4f} units/point")
        
        # dx: 纵向间距 (沿着运动方向)
        # 估算总扫描帧数 = 总点数 / 每帧点数
        total_frames = len(points) / points_per_frame
        x_length = max_bound[0] - min_bound[0]
        dx = x_length / total_frames if total_frames > 0 else 0
        # print(f"纵向分辨率 (dx): {dx:.4f} units/frame")
        
        # # 物理估算 (基于 config.yaml 的 unit_scale: 0.001, 即 1 unit = 1 mm)
        print(f"\n物理分辨率预估 (1 unit = 1 mm):")
        print(f"  dx ≈ {dx:.3f} mm, dy ≈ {dy:.3f} mm")
        print(f"  点密度 ≈ {1.0/(dx*dy):.2f} pts/mm2")
    else:
        print("数据量不足，无法自动测算周期。")

    # 可视化 (可选)
    # pcd.colors = o3d.utility.Vector3dVector()
    # o3d.visualization.draw_geometries([pcd])