import numpy as np
from scipy.ndimage import gaussian_filter1d
from rclpy.node import Node

class MonoDistanceEstimator:
    def __init__(self, node: Node):
        # 从参数服务器读取配置
        self.node = node
        self.fx = node.get_parameter('camera_fx').value
        self.fy = node.get_parameter('camera_fy').value
        self.target_width = node.get_parameter('target_real_width').value
        self.window_size = node.get_parameter('gaussian_window_size').value
        
        # 存储历史测距值（用于高斯滤波）
        self.distance_history = []

    def calculate_distance(self, target_pixel_width: float) -> float:
        """单目测距核心公式：距离 = (实际宽度 * 焦距) / 像素宽度"""
        if target_pixel_width < 1e-6:  # 避免除零
            return 0.0
        distance = (self.target_width * self.fx) / target_pixel_width
        # 高斯滤波平滑
        self.distance_history.append(distance)
        if len(self.distance_history) > self.window_size:
            self.distance_history.pop(0)
        # 滤波后距离
        filtered_dist = gaussian_filter1d(np.array(self.distance_history), sigma=1.0)[-1]
        return filtered_dist

    def get_pixel_width_from_pose(self, robot_pose, target_pose) -> float:
        """仿真环境下：从位姿反推目标像素宽度（替代实际视觉检测）"""
        # 计算机器人与目标的实际距离
        dx = target_pose.position.x - robot_pose.position.x
        dy = target_pose.position.y - robot_pose.position.y
        real_dist = np.hypot(dx, dy)
        # 反推像素宽度：像素宽度 = (实际宽度 * 焦距) / 距离
        pixel_width = (self.target_width * self.fx) / real_dist
        return pixel_width