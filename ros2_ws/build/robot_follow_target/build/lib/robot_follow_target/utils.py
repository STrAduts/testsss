import cv2
import numpy as np

def gaussian_filter(image, kernel_size=(5,5), sigma_x=0):
    """高斯滤波：降低图像噪声"""
    return cv2.GaussianBlur(image, kernel_size, sigma_x)

def monocular_distance(real_size, focal_length, pixel_size):
    """单目测距核心公式：距离 = (实际尺寸 × 焦距) / 像素尺寸"""
    if pixel_size == 0:
        return 0.0
    return (real_size * focal_length) / pixel_size

def calculate_offset_angle(pixel_offset, focal_length, image_center):
    """计算偏移角（像素→角度）"""
    theta_rad = np.arctan(pixel_offset / focal_length)  # 弧度
    theta_deg = np.degrees(theta_rad)                  # 转换为角度
    return theta_deg