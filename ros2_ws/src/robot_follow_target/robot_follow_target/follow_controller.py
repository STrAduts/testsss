import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist

class FollowController(Node):
    def __init__(self):
        super().__init__('follow_controller')
        
        # 声明并读取参数
        self.declare_parameter('target_distance_L', 2.0)
        self.declare_parameter('distance_tolerance_M', 0.2)
        self.declare_parameter('fov_tolerance_z', 5.0)
        self.declare_parameter('linear_speed_gain', 0.5)
        self.declare_parameter('angular_speed_gain', 0.3)
        
        # 解析参数
        self.L = self.get_parameter('target_distance_L').value
        self.M = self.get_parameter('distance_tolerance_M').value
        self.z = self.get_parameter('fov_tolerance_z').value
        self.linear_gain = self.get_parameter('linear_speed_gain').value
        self.angular_gain = self.get_parameter('angular_speed_gain').value
        
        # 订阅目标位姿话题
        self.pose_sub = self.create_subscription(
            Vector3, '/object_pose', self.pose_callback, 10)
        
        # 发布小车控制指令（/cmd_vel）
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 发布机器人-目标位置关系（满足要求的话题）
        self.rel_pose_pub = self.create_publisher(Vector3, '/robot_target_pose', 10)
        
        self.get_logger().info(f"跟踪控制节点启动 | L={self.L}m | M={self.M}m | z={self.z}°")

    def pose_callback(self, pose_msg):
        # 提取位姿信息
        distance = pose_msg.x       # 实际距离（米）
        offset_y = pose_msg.y       # 横向偏移（米）
        offset_angle = pose_msg.z   # 偏移角（度）
        
        # 发布位置关系话题（满足用户要求）
        self.rel_pose_pub.publish(pose_msg)
        
        # 初始化控制指令
        cmd_vel = Twist()
        
        # 1. 距离控制：保持L±M
        distance_error = distance - self.L
        if abs(distance_error) > self.M:
            # 比例控制：误差越大，线速度越快（反向）
            cmd_vel.linear.x = -self.linear_gain * distance_error
        else:
            cmd_vel.linear.x = 0.0
        
        # 2. 角度控制：保持视场中央±z/2
        angle_error = offset_angle
        if abs(angle_error) > self.z / 2:
            # 比例控制：误差越大，角速度越快（反向）
            cmd_vel.angular.z = -self.angular_gain * angle_error
        else:
            cmd_vel.angular.z = 0.0
        
        # 限制最大速度（防止失控）
        max_linear = 0.5
        max_angular = 0.3
        cmd_vel.linear.x = max(min(cmd_vel.linear.x, max_linear), -max_linear)
        cmd_vel.angular.z = max(min(cmd_vel.angular.z, max_angular), -max_angular)
        
        # 发布控制指令
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().debug(f"距离误差：{distance_error:.2f}m | 角度误差：{angle_error:.2f}° | 线速度：{cmd_vel.linear.x:.2f}m/s | 角速度：{cmd_vel.angular.z:.2f}rad/s")

def main(args=None):
    rclpy.init(args=args)
    node = FollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()