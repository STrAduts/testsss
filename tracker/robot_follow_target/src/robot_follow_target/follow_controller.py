import numpy as np
from geometry_msgs.msg import Twist

class FollowController:
    def __init__(self, node, pub):
        self.node = node
        self.cmd_vel_pub = pub  # cmd_vel发布器
        # 控制参数
        self.fixed_dist = node.get_parameter('fixed_distance').value
        self.dist_tol = node.get_parameter('distance_tolerance').value
        self.angle_tol = node.get_parameter('angle_tolerance').value / 2  # 半角阈值
        self.linear_kp = node.get_parameter('linear_kp').value
        self.angular_kp = node.get_parameter('angular_kp').value

    def control(self, current_dist: float, offset_angle_deg: float):
        """
        核心控制逻辑：
        - 距离超出[L-M, L+M]时，调整线速度
        - 角度超出±z/2度时，调整角速度
        """
        cmd = Twist()
        
        # 1. 线速度控制（趋近/远离固定距离）
        dist_error = current_dist - self.fixed_dist
        if abs(dist_error) > self.dist_tol:
            cmd.linear.x = -self.linear_kp * dist_error  # 负号：误差正→远离→前进
        else:
            cmd.linear.x = 0.0  # 距离达标，停止线运动
        
        # 2. 角速度控制（保持目标在视场中央）
        if abs(offset_angle_deg) > self.angle_tol:
            cmd.angular.z = self.angular_kp * np.radians(offset_angle_deg)
        else:
            cmd.angular.z = 0.0  # 角度达标，停止角运动
        
        # 限制最大速度（避免失控）
        cmd.linear.x = np.clip(cmd.linear.x, -0.5, 0.5)
        cmd.angular.z = np.clip(cmd.angular.z, -0.3, 0.3)
        
        self.cmd_vel_pub.publish(cmd)
        self.node.get_logger().info(
            f"控制指令：线速度={cmd.linear.x:.2f} m/s，角速度={cmd.angular.z:.2f} rad/s"
        )