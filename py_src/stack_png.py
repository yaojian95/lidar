import cv2
import os
import numpy as np

def stack_images():
    # Define paths
    source_dir = r"E:\multi_source_info\camera\0121"
    # Using absolute path for destination as requested, or relative to this script
    # User asked for "pcd_data path", assuming E:\multi_source_info\lidar\pcd_data
    dest_dir = r"E:\multi_source_info\lidar\pcd_data"
    output_path = os.path.join(dest_dir, "stitched_ore.jpg")

    # Ensure destination directory exists
    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)
        print(f"Created destination directory: {dest_dir}")

    # Get list of image files
    extensions = ('.jpg', '.jpeg')
    image_files = [f for f in os.listdir(source_dir) if f.lower().endswith(extensions)]
    
    # Sort files to ensure correct order
    image_files.sort()
    
    if not image_files:
        print(f"No image files found in {source_dir}")
        return

    print(f"Found {len(image_files)} images. Stacking...")

    images = []
    for filename in image_files:
        file_path = os.path.join(source_dir, filename)
        img = cv2.imread(file_path)
        if img is not None:
            images.append(img)
        else:
            print(f"Warning: Could not read image {filename}")

    if not images:
        print("No valid images to stack.")
        return

    # Stack images vertically
    try:
        # Check if all images have the same width
        width = images[0].shape[1]
        for i, img in enumerate(images):
            if img.shape[1] != width:
                print(f"Resizing image {image_files[i]} to match width {width}")
                # Maintain aspect ratio? Or just resize width? 
                # Usually vstack needs same width.
                # Let's resize to match the first image's width
                height = int(img.shape[0] * (width / img.shape[1]))
                images[i] = cv2.resize(img, (width, height))

        stacked_image = np.vstack(images)
        
        # Save the result
        cv2.imwrite(output_path, stacked_image)
        print(f"Successfully saved stitched image to {output_path}")
        
    except Exception as e:
        print(f"Error while stacking images: {e}")

if __name__ == "__main__":
    stack_images()
