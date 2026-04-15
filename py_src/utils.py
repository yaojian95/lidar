import cv2
import numpy as np

def correct_high_energy_distortion(image: np.ndarray, fx: float, fy: float = 1.0) -> np.ndarray:
    """
    针对高能和低能闪烁体探测器高度不同导致的扇形投影畸变进行校正。
    该函数通过双向缩放及对称补齐/裁剪，使校正后的图像保持原始尺寸。
    
    Parameters:
        image (np.ndarray): 输入的高能图像 (grayscale or color)。
        fx (float): 横向修正系数。
        fy (float): 纵向修正系数 (默认 1.0)。
        fx和fy只是为了防止图像转置，实际只有一个方向畸变。0.9909 for both 银山设备和厂房设备。

    Returns:
        np.ndarray: 校正后的图像，尺寸与输入一致。
    """
    if (fx == 1.0 and fy == 1.0) or image is None:
        return image
    
    h, w = image.shape[:2]
    
    # 1. 按照系数进行双向缩放
    resized = cv2.resize(image, (0, 0), fx=fx, fy=fy, interpolation=cv2.INTER_LINEAR)
    
    new_h, new_w = resized.shape[:2]
    
    # 2. 对图像进行横向补齐或裁剪，使其保持原始宽度 w
    if new_w < w:
        total_pad = w - new_w
        pad_left = total_pad // 2
        pad_right = total_pad - pad_left
        resized = cv2.copyMakeBorder(resized, 0, 0, pad_left, pad_right, cv2.BORDER_CONSTANT, value=0)
    elif new_w > w:
        total_crop = new_w - w
        crop_left = total_crop // 2
        resized = resized[:, crop_left : crop_left + w].copy()

    # 3. 对图像进行纵向补齐或裁剪，使高度保持原始尺寸 h
    if new_h < h:
        total_pad = h - new_h
        pad_top = total_pad // 2
        pad_bot = total_pad - pad_top
        resized = cv2.copyMakeBorder(resized, pad_top, pad_bot, 0, 0, cv2.BORDER_CONSTANT, value=0)
    elif new_h > h:
        total_crop = new_h - h
        crop_top = total_crop // 2
        resized = resized[crop_top : crop_top + h, :].copy()
        
    return resized
