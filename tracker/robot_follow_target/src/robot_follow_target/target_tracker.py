import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Vector3
from gazebo_msgs.srv import GetEntityState
from robot_follow_target.mono_distance_estimator import MonoDistanceEstimator
from robot_follow_target.pose_calculator import PoseCalculator
from robot_follow_target.follow_controller import FollowController

class TargetTrackerNode(Node):
    def __init__(self):
        super().__init__('target_tracker_node')
        
        # 1. 声明并加载参数
        self.declare_parameters(
            namespace='follow_params',
            parameters=[
                ('fixed_distance', 2.0),
                ('distance_tolerance', 0.2),
                ('angle_tolerance', 10.0),
                ('gaussian_window_size', 5),
                ('camera_fx', 550.0),
                ('camera_fy', 550.0),
                ('camera_cx', 320.0),
                ('camera_cy', 240.0),
                ('target_real_width', 0.5),
                ('linear_kp', 0.5),
                ('angular_kp', 0.1)
            ]
        )
        
        # 2. 创建话题发布器：机器人-目标位姿(x,y,w)
        self.pose_pub = self.create_publisher(Vector3, '/robot_target_pose', 10)
        
        # 3. 创建cmd_vel发布器（控制小车）
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 4. 初始化工具类
        self.distance_estimator = MonoDistanceEstimator(self)
        self.pose_calculator = PoseCalculator(self)
        self.follow_controller = FollowController(self, self.cmd_vel_pub)
        
        # 5. 创建Gazebo状态查询客户端（仿真获取机器人/目标位姿）
        self.gazebo_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not self.gazebo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("等待/gazebo/get_entity_state服务...")
        
        # 6. 定时器（10Hz执行跟踪逻辑）
        self.timer = self.create_timer(0.1, self.tracker_callback)

    def get_entity_pose(self, entity_name: str) -> Pose:
        """查询Gazebo中实体的位姿"""
        req = GetEntityState.Request()
        req.name = entity_name
        req.reference_frame = 'world'
        future = self.gazebo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error(f"获取{entity_name}位姿失败")
            return Pose()
        return future.result().state.pose

    def tracker_callback(self):
        """跟踪逻辑主回调"""
        # 1. 获取机器人和目标的位姿（仿真环境）
        robot_pose = self.get_entity_pose('R2')  # 场地包中机器人实体名R2
        target_pose = self.get_entity_pose('target')  # 场地包中目标实体名target
        
        # 2. 单目测距（仿真：从位姿反推像素宽度）
        pixel_width = self.distance_estimator.get_pixel_width_from_pose(robot_pose, target_pose)
        current_dist = self.distance_estimator.calculate_distance(pixel_width)
        
        # 3. 计算相对位姿(x,y,w)
        rel_x, rel_y, offset_angle_deg = self.pose_calculator.calculate_relative_pose(robot_pose, target_pose)
        
        # 4. 发布位姿话题
        pose_msg = Vector3()
        pose_msg.x = rel_x
        pose_msg.y = rel_y
        pose_msg.z = offset_angle_deg  # w偏移角存在z字段（话题要求x,y,w）
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(
            f"发布位姿：x={rel_x:.2f}m, y={rel_y:.2f}m, w={offset_angle_deg:.2f}° | 当前距离={current_dist:.2f}m"
        )
        
        # 5. 运动控制
        self.follow_controller.control(current_dist, offset_angle_deg)

def main(args=None):
    rclpy.init(args=args)
    node = TargetTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()