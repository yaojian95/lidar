import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

def auto_calibrate_xrt(image_path, cut_left=0, cut_right=0, manual_factor=None, save_path=None, sample_row=None):
    # 1. Load image
    raw_img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if raw_img is None:
        print(f"Error: Could not load image {image_path}")
        return None

    # 2. Split into Low and High Energy
    mid = raw_img.shape[1] // 2
    low_energy = raw_img[:, :mid]
    high_energy = raw_img[:, mid:]

    # 3. Flip horizontally (as per C++ logic)
    low_energy = cv2.flip(low_energy, 1)
    high_energy = cv2.flip(high_energy, 1)

    # 4. Apply initial waste edge cuts
    if cut_left > 0 or cut_right > 0:
        new_w = mid - cut_left - cut_right
        low_energy = low_energy[:, cut_left:cut_left+new_w]
        high_energy = high_energy[:, cut_left:cut_left+new_w]

    h, w = low_energy.shape
    center_x = w / 2.0

    if manual_factor is None:
        print(f"Calculating correction factor automatically for {os.path.basename(image_path)}...")
        # 5. Detect Features (ORB)
        orb = cv2.ORB_create(5000)
        kp1, des1 = orb.detectAndCompute(low_energy, None)
        kp2, des2 = orb.detectAndCompute(high_energy, None)

        if des1 is None or des2 is None:
            print("Error: Could not find features for matching.")
            return None

        # 6. Match Features
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)

        # 7. Extract matched points
        src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 2)

        # 8. Robust scale estimation (RANSAC-like)
        dx1 = src_pts[:, 0] - center_x
        dx2 = dst_pts[:, 0] - center_x
        dy = src_pts[:, 1] - dst_pts[:, 1]
        
        mask = (np.abs(dy) < 5) & (np.abs(dx1) > 20)
        dx1 = dx1[mask]
        dx2 = dx2[mask]

        if len(dx1) < 10:
            print("Error: Not enough good matches for scale estimation.")
            return None

        scale_factor = np.sum(dx1 * dx2) / np.sum(dx1**2)
        correction_factor = 1.0 / scale_factor
        print(f"Estimated Scaling: {scale_factor:.4f} -> Correction Factor: {correction_factor:.4f}")
    else:
        correction_factor = manual_factor
        print(f"Using provided correction factor: {correction_factor:.4f} for {os.path.basename(image_path)}")

    # 9. Perform correction
    resized_high = cv2.resize(high_energy, None, fx=correction_factor, fy=1.0, interpolation=cv2.INTER_LINEAR)
    
    target_w = low_energy.shape[1]
    curr_w = resized_high.shape[1]
    
    if curr_w < target_w:
        total_pad = target_w - curr_w
        pad_l = total_pad // 2
        pad_r = total_pad - pad_l
        corrected_high = cv2.copyMakeBorder(resized_high, 0, 0, pad_l, pad_r, cv2.BORDER_CONSTANT, value=0)
    else:
        total_crop = curr_w - target_w
        crop_l = total_crop // 2
        corrected_high = resized_high[:, crop_l:crop_l+target_w]

    # 10. Visualization (Compare Before and After)
    plt.figure(figsize=(18, 14))
    
    # 10.1 Before Correction Overlay
    plt.subplot(3, 2, 1)
    overlay_before = np.zeros((h, w, 3), dtype=np.uint8)
    overlay_before[..., 0] = low_energy
    overlay_before[..., 1] = high_energy
    overlay_before[..., 2] = (low_energy.astype(np.uint16) + high_energy.astype(np.uint16)) // 2
    plt.imshow(overlay_before)
    plt.title(f"BEFORE Correction: {os.path.basename(image_path)} (Red=Low, Green=High_Raw)")
    
    # 10.2 After Correction Overlay
    plt.subplot(3, 2, 2)
    overlay_after = np.zeros((h, target_w, 3), dtype=np.uint8)
    overlay_after[..., 0] = low_energy
    overlay_after[..., 1] = corrected_high
    overlay_after[..., 2] = (low_energy.astype(np.uint16) + corrected_high.astype(np.uint16)) // 2
    plt.imshow(overlay_after)
    plt.title(f"AFTER Correction (Factor: {correction_factor:.4f})")
    
    # 10.3 Sample Row Profile - BEFORE
    cur_row = sample_row if sample_row is not None else h // 2
    plt.subplot(3, 2, 3)
    plt.plot(low_energy[cur_row, :], 'r', label='Low Energy', alpha=0.7)
    plt.plot(high_energy[cur_row, :], 'g', label='High Energy (Raw)', alpha=0.7)
    plt.title(f"Intensity Profile BEFORE (Row {cur_row})")
    plt.xlabel("Column Index")
    plt.ylabel("Intensity")
    plt.legend()
    plt.grid(True)
    
    # 10.4 Sample Row Profile - AFTER
    plt.subplot(3, 2, 4)
    plt.plot(low_energy[cur_row, :], 'r', label='Low Energy', alpha=0.7)
    plt.plot(corrected_high[cur_row, :], 'g', label='High Energy (Corrected)', alpha=0.7)
    plt.title(f"Intensity Profile AFTER (Row {cur_row})")
    plt.xlabel("Column Index")
    plt.ylabel("Intensity")
    plt.legend()
    plt.grid(True)
    
    # 10.5 Difference BEFORE
    plt.subplot(3, 2, 5)
    diff_before = cv2.absdiff(low_energy, high_energy)
    plt.imshow(diff_before, cmap='hot')
    plt.title("Difference BEFORE")
    plt.axis('off')
    
    # 10.6 Difference AFTER
    plt.subplot(3, 2, 6)
    diff_after = cv2.absdiff(low_energy, corrected_high)
    plt.imshow(diff_after, cmap='hot')
    plt.title("Difference AFTER")
    plt.axis('off')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path)
        print(f"Results saved to {save_path}")
    else:
        plt.show()
    plt.close()

    return correction_factor

if __name__ == "__main__":
    # Define input images from different paths
    # IMG1 = "E:/multi_source_info/lidar/pcd_data/20260309_banyan_detong_63/banyan_Detong_1_63_347.png"
    # IMG2 = "E:/multi_source_info/lidar/pcd_data/20260325_yinshan/20260325_yinshan/big_ores_position_2_160kV.tif"
    IMG1 = "E:/multi_source_info/data_dir/20260401/1_20_150kV.tif"
    IMG2 = "E:/multi_source_info/data_dir/20260401/1_20_160kV.tif"
    
    # Set cuts (assuming same hardware setup)
    C_LEFT = 130
    C_RIGHT = 120
    
    results_dir = "E:/multi_source_info/lidar/py_src"
    os.makedirs(results_dir, exist_ok=True)

    # Set Specific rows for each image to inspect
    ROW1 = 347 # Middle of an interesting section for IMG1
    ROW2 = 1000 # Middle of an interesting section for IMG2

    # 1. Calibrate on the first image and save
    if os.path.exists(IMG1):
        factor = auto_calibrate_xrt(IMG1, cut_left=C_LEFT, cut_right=C_RIGHT, 
                                   save_path=os.path.join(results_dir, "yinshan_calibration_1_auto.png"),
                                   sample_row=ROW1)
        
        # 2. Apply same factor to the second image
        if factor and os.path.exists(IMG2):
            auto_calibrate_xrt(IMG2, cut_left=C_LEFT, cut_right=C_RIGHT, manual_factor=factor,
                              save_path=os.path.join(results_dir, "yinshan_calibration_2_applied.png"),
                              sample_row=ROW2)
        else:
            if not factor: print("Calibration on IMG1 failed.")
            if not os.path.exists(IMG2): print(f"IMG2 not found: {IMG2}")
    else:
        print(f"IMG1 not found: {IMG1}")
