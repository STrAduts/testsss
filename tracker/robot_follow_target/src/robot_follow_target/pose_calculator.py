import numpy as np
from geometry_msgs.msg import Pose

class PoseCalculator:
    def __init__(self, node):
        self.node = node
        self.angle_tol = node.get_parameter('angle_tolerance').value  # 角度阈值z（度）

    def calculate_relative_pose(self, robot_pose: Pose, target_pose: Pose) -> tuple:
        """计算相对位姿：x(米), y(米), w(偏移角，度)"""
        # 1. 相对坐标（二维平面）
        rel_x = target_pose.position.x - robot_pose.position.x
        rel_y = target_pose.position.y - robot_pose.position.y
        
        # 2. 计算机器人朝向（四元数转欧拉角）
        robot_ori = robot_pose.orientation
        robot_yaw = self.quaternion_to_yaw(robot_ori.x, robot_ori.y, robot_ori.z, robot_ori.w)
        
        # 3. 目标相对于机器人的角度（世界坐标系）
        target_angle_world = np.arctan2(rel_y, rel_x)
        # 4. 偏移角（机器人坐标系下）
        offset_angle = target_angle_world - robot_yaw
        # 转角度并归一化到[-180, 180]
        offset_angle_deg = np.degrees(offset_angle)
        offset_angle_deg = (offset_angle_deg + 180) % 360 - 180
        
        return rel_x, rel_y, offset_angle_deg

    def quaternion_to_yaw(self, x, y, z, w) -> float:
        """四元数转偏航角（yaw），弧度"""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw