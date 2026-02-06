import open3d as o3d
import numpy as np
import glob
import os
import cv2

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

def stack_tif_files(image_folder, axis=0):
    """
    将指定文件夹内的所有tif文件堆叠/拼接成一张大图
    :param image_folder: tif文件所在的文件夹路径
    :param axis: 拼接方向，0为垂直拼接(vstack)，1为水平拼接(hstack)
    """
    if not os.path.exists(image_folder):
        print(f"Error: Folder not found: {image_folder}")
        return

    # 1. 获取所有tif文件
    search_pattern = os.path.join(image_folder, "*.tif")
    tif_files = sorted(glob.glob(search_pattern))
    
    if not tif_files:
        print(f"Warning: No .tif files found in {image_folder}")
        return

    print(f"Found {len(tif_files)} tif files in {image_folder}")

    # 2. 读取所有图片
    images = []
    for f in tif_files:
        # cv2.IMREAD_UNCHANGED ensures we keep 16-bit or other depths if present
        img = cv2.imread(f, cv2.IMREAD_UNCHANGED)
        if img is not None:
            images.append(img)
        else:
            print(f"Warning: Failed to read {f}")

    if not images:
        print("No valid images to stack.")
        return

    # 3. 拼接图片
    try:
        # axis=0: vertical stack (height increases)
        # axis=1: horizontal stack (width increases)
        stacked_image = np.concatenate(images, axis=axis)
        
        # 4. 生成输出路径
        # 所在文件夹的上一层
        folder_abs = os.path.abspath(image_folder)
        parent_dir = os.path.dirname(folder_abs)
        folder_name = os.path.basename(folder_abs)
        
        output_filename = f"{folder_name}.png"
        output_path = os.path.join(parent_dir, output_filename)
        
        # 5. 保存
        cv2.imwrite(output_path, stacked_image)
        print(f"Stacked image saved to: {output_path} (Shape: {stacked_image.shape})")
        
    except ValueError as e:
        print(f"Error during stacking (check image dimensions match): {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

# 使用示例
if __name__ == "__main__":
    # simple_stitch_ore_pcd("data/ore_belt", "simple_stitched.ply")
    stack_tif_files(r"E:\multi_source_info\lidar\pcd_data\20260205\xray_0p5ms-1", axis=0)
    # pass