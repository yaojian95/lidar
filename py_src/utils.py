import cv2
import numpy as np

def correct_high_energy_distortion(image: np.ndarray, correction_factor: float) -> np.ndarray:
    """
    针对高能和低能闪烁体探测器高度不同导致的扇形投影畸变进行校正。
    该函数通过横向缩放及对称补齐/裁剪，使校正后的图像保持原始尺寸。

    参数:
        image (np.ndarray): 输入的高能图像 (grayscale or color)。
        correction_factor (float): 横向修正系数 (例如 0.9909)。

    返回:
        np.ndarray: 校正后的图像，尺寸与输入一致。
    """
    if correction_factor == 1.0 or image is None:
        return image
    
    h, w = image.shape[:2]
    
    # 1. 按照系数进行横向缩放 (Vertical scale stays at 1.0)
    # cv2.resize works with (width, height)
    resized = cv2.resize(image, (0, 0), fx=correction_factor, fy=1.0, interpolation=cv2.INTER_LINEAR)
    
    new_w = resized.shape[1]
    
    # 2. 对图像进行对称平移补齐或裁剪，使其保持原始宽度 w
    if new_w < w:
        # 补齐 (Padding)
        total_pad = w - new_w
        pad_left = total_pad // 2
        pad_right = total_pad - pad_left
        # cv2.copyMakeBorder(src, top, bottom, left, right, borderType, value)
        result = cv2.copyMakeBorder(resized, 0, 0, pad_left, pad_right, cv2.BORDER_CONSTANT, value=0)
    elif new_w > w:
        # 裁剪 (Cropping)
        total_crop = new_w - w
        crop_left = total_crop // 2
        result = resized[:, crop_left : crop_left + w].clone() if hasattr(resized, 'clone') else resized[:, crop_left : crop_left + w].copy()
    else:
        result = resized
        
    return result
