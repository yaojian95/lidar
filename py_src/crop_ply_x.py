import open3d as o3d
import numpy as np
import argparse
import os

def crop_ply_x_middle(input_path, output_path=None):
    """
    Reads a PLY file and keeps only the points in the middle 1/3 of the X range.
    """
    print(f"Loading point cloud from: {input_path}")
    pcd = o3d.io.read_point_cloud(input_path)
    
    if pcd.is_empty():
        print("Error: Point cloud is empty or file not found.")
        return

    # Get points as numpy array
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None
    normals = np.asarray(pcd.normals) if pcd.has_normals() else None
    
    # Calculate X range
    min_x = np.min(points[:, 0])
    max_x = np.max(points[:, 0])
    x_range = max_x - min_x
    
    print(f"X Range: {min_x:.3f} to {max_x:.3f} (Width: {x_range:.3f})")
    
    # Define crop boundaries (Middle 1/3)
    # Remove first 1/3: start > min_x + r/3
    # Remove last 1/3: end < max_x - r/3
    crop_min_x = min_x + (x_range / 3.0)
    crop_max_x = max_x - (x_range / 3.0)
    
    print(f"Cropping to keep X between: {crop_min_x:.3f} and {crop_max_x:.3f}")
    
    # Filter points
    mask = (points[:, 0] >= crop_min_x) & (points[:, 0] <= crop_max_x)
    
    cropped_points = points[mask]
    print(f"Original points: {len(points)}, Remaining points: {len(cropped_points)}")
    
    if len(cropped_points) == 0:
        print("Warning: No points remaining after crop!")
        return

    # Create new point cloud
    cropped_pcd = o3d.geometry.PointCloud()
    cropped_pcd.points = o3d.utility.Vector3dVector(cropped_points)
    
    if colors is not None:
        cropped_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
    if normals is not None:
        cropped_pcd.normals = o3d.utility.Vector3dVector(normals[mask])
        
    # Determine output path
    if output_path is None:
        file_dir = os.path.dirname(input_path)
        file_name = os.path.basename(input_path)
        name_no_ext, ext = os.path.splitext(file_name)
        output_path = os.path.join(file_dir, f"{name_no_ext}_cropped{ext}")
        
    print(f"Saving cropped point cloud to: {output_path}")
    o3d.io.write_point_cloud(output_path, cropped_pcd)
    print("Done.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Crop PLY file to keep middle 1/3 of X-axis.")
    parser.add_argument("input", help="Path to input PLY file")
    parser.add_argument("--output", help="Path to output PLY file (optional)", default=None)
    
    args = parser.parse_args()
    
    crop_ply_x_middle(args.input, args.output)
