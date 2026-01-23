import open3d as o3d
import numpy as np
import glob
import os

def simple_stitch_ore_pcd(pcd_folder, output_path="stitched_ore.ply"):
    """简化版矿石点云拼接"""
    
    # 1. 加载所有PCD文件
    pcd_files = sorted(glob.glob(os.path.join(pcd_folder, "*.pcd")))
    print(f"找到 {len(pcd_files)} 个PCD文件")
    
    # 2. 初始化总点云
    combined_pcd = o3d.geometry.PointCloud()
    
    # 3. 简单拼接（假设皮带匀速运动）
    for i, file_path in enumerate(pcd_files):
        pcd = o3d.io.read_point_cloud(file_path)
        
        if len(pcd.points) == 0:
            continue
        
        # 应用平移（每帧移动固定距离）
        translation = np.array([i * 0.1, 0, 0])  # 假设每帧移动10cm
        pcd.translate(translation)
        
        # 合并
        combined_pcd += pcd
        
        # 定期简化
        if len(combined_pcd.points) > 500000:
            combined_pcd = combined_pcd.voxel_down_sample(0.01)
        
        # print(f"已处理 {i+1}/{len(pcd_files)}: {os.path.basename(file_path)}")
    
    # # 4. 保存结果
    # o3d.io.write_point_cloud(output_path, combined_pcd)
    # print(f"拼接完成！保存至: {output_path}")
    
    # # 5. 可视化
    # o3d.visualization.draw_geometries(
    #     [combined_pcd],
    #     window_name="矿石点云拼接结果",
    #     width=1200,
    #     height=800,
    #     point_show_normal=False
    # )
    
    return combined_pcd

# 使用示例
if __name__ == "__main__":
    simple_stitch_ore_pcd("data/ore_belt", "simple_stitched.ply")