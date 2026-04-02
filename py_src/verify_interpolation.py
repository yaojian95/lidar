import cv2
import numpy as np
import matplotlib.pyplot as plt

def verify_interpolation():
    # 1. Create a 1D-like 2D image with a "Spike" pattern
    # Pattern: [255, 0, 0, 0, 0, 0, 0, 0, 0, 0] repeated
    # This represents a very thin feature (1 pixel wide) every 10 pixels.
    width = 1000
    pattern_size = 10
    src = np.zeros((100, width), dtype=np.uint8)
    for i in range(0, width, pattern_size):
        src[:, i] = 255

    # 2. Shrink the image by 10x (fx = 0.1)
    # The sampling step is exactly 10 pixels.
    fx = 0.1
    
    # Method 1: INTER_NEAREST (Picks one pixel)
    # Target index 0 -> Src 0 (Value 255)
    # Target index 1 -> Src 10 (Value 255)
    dst_nearest = cv2.resize(src, None, fx=fx, fy=1.0, interpolation=cv2.INTER_NEAREST)

    # Method 2: INTER_LINEAR (Linear interpolation of 2 neighbors)
    # For fx=0.1, target index j maps to src_x = j*10 + 4.5 (approx center of the 10-pixel block)
    # Since src[4] and src[5] are both 0, the result will be 0. 
    # It COMPLETELY MISSES the spikes at 0, 10, 20...
    dst_linear = cv2.resize(src, None, fx=fx, fy=1.0, interpolation=cv2.INTER_LINEAR)

    # Method 3: INTER_AREA (Averages the entire 10-pixel area)
    # It sums [0:10], [10:20]... each contains one '255' and nine '0's.
    # Result: 255 / 10 = 25.5
    dst_area = cv2.resize(src, None, fx=fx, fy=1.0, interpolation=cv2.INTER_AREA)

    # 3. Visualization
    plt.figure(figsize=(15, 10))

    # Row 1: Original Pattern (Crop first 100 pixels for visibility)
    plt.subplot(4, 1, 1)
    plt.plot(src[0, :100], 'k-', label='Original Pattern')
    plt.title("Original Signal (Spikes at 0, 10, 20...)")
    plt.ylim(-10, 270)
    plt.legend()

    # Row 2: NEAREST Result
    plt.subplot(4, 1, 2)
    plt.plot(dst_nearest[0, :10], 'bo-', label='INTER_NEAREST')
    plt.title("INTER_NEAREST (Picks one pixel -> Results in 255, 255... Alias!)")
    plt.ylim(-10, 270)
    plt.legend()

    # Row 3: LINEAR Result
    plt.subplot(4, 1, 3)
    plt.plot(dst_linear[0, :10], 'rx-', label='INTER_LINEAR')
    plt.title("INTER_LINEAR (Samples between spikes -> Results in 0, 0... Missing Information!)")
    plt.ylim(-10, 270)
    plt.legend()

    # Row 4: AREA Result
    plt.subplot(4, 1, 4)
    plt.plot(dst_area[0, :10], 'g^-', label='INTER_AREA')
    plt.title("INTER_AREA (Averages the local 10-pixel area -> Results in 25.5, 25.5... Energy Preserved)")
    plt.ylim(-10, 270)
    plt.legend()

    plt.tight_layout()
    save_path = "E:/multi_source_info/lidar/results/interpolation_comparison.png"
    plt.savefig(save_path)
    print(f"Comparison plot saved to {save_path}")
    # plt.show()
    plt.close()

    # 4. Print first few values for precision check
    print("\nFirst 10 pixels of 1D results (fx=0.1, step=10, pattern=spike_at_0):")
    print(f"Original index 0: {src[0, 0]}")
    print(f"Original index 1-9: {src[0, 1:10]}")
    print("-" * 50)
    print(f"NEAREST: {dst_nearest[0, :5].astype(float)} (Hits source indices 0, 10, 20...)")
    print(f"LINEAR:  {dst_linear[0, :5].astype(float)}  (Samples between 4th and 5th pixels, both are 0)")
    print(f"AREA:    {dst_area[0, :5].astype(float)}  (Average of all 10 pixels: 255/10 = 25.5)")

if __name__ == "__main__":
    verify_interpolation()
