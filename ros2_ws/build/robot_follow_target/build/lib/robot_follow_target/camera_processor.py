import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from robot_follow_target.utils import gaussian_filter, monocular_distance, calculate_offset_angle

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        
        # 声明并读取参数
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_fx', 600.0)
        self.declare_parameter('camera_fy', 600.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        self.declare_parameter('target_real_width', 0.2)
        self.declare_parameter('target_real_height', 0.2)
        self.declare_parameter('gaussian_kernel_size', [5,5])
        self.declare_parameter('gaussian_sigma_x', 0)
        self.declare_parameter('hsv_lower', [0, 120, 70])
        self.declare_parameter('hsv_upper', [10, 255, 255])
        self.declare_parameter('min_contour_area', 100)
        
        # 解析参数
        self.camera_topic = self.get_parameter('camera_topic').value
        self.fx = self.get_parameter('camera_fx').value
        self.fy = self.get_parameter('camera_fy').value
        self.cx = self.get_parameter('camera_cx').value
        self.cy = self.get_parameter('camera_cy').value
        self.target_real_width = self.get_parameter('target_real_width').value
        self.gaussian_kernel_size = tuple(self.get_parameter('gaussian_kernel_size').value)
        self.gaussian_sigma_x = self.get_parameter('gaussian_sigma_x').value
        self.hsv_lower = np.array(self.get_parameter('hsv_lower').value)
        self.hsv_upper = np.array(self.get_parameter('hsv_upper').value)
        self.min_contour_area = self.get_parameter('min_contour_area').value
        
        # 初始化CV桥接器（ROS图像→OpenCV图像）
        self.bridge = CvBridge()
        
        # 订阅摄像头图像话题
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10)
        
        # 发布目标位姿话题（x:距离, y:横向偏移, z:偏移角）
        self.pose_pub = self.create_publisher(Vector3, '/object_pose', 10)
        
        self.get_logger().info(f"摄像头处理节点启动，订阅话题：{self.camera_topic}")

    def image_callback(self, msg):
        try:
            # ROS图像消息 → OpenCV图像（BGR格式）
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败：{e}")
            return
        
        # 1. 高斯滤波去噪
        blurred = gaussian_filter(cv_image, self.gaussian_kernel_size, self.gaussian_sigma_x)
        
        # 2. 颜色检测（红色目标）
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        
        # 3. 轮廓检测（找最大轮廓作为目标）
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)
            if area > self.min_contour_area:
                # 计算目标包围框
                x, y, w, h = cv2.boundingRect(max_contour)
                # 目标中心像素坐标
                center_x = x + w // 2
                center_y = y + h // 2
                
                # 绘制检测结果（调试用）
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # 4. 单目测距（基于宽度）
                distance = monocular_distance(self.target_real_width, self.fx, w)
                
                # 5. 计算横向偏移（米）
                pixel_offset_x = center_x - self.cx
                offset_y = (pixel_offset_x * distance) / self.fx
                
                # 6. 计算偏移角（度）
                offset_angle = calculate_offset_angle(pixel_offset_x, self.fx, self.cx)
                
                # 发布位姿消息
                pose_msg = Vector3()
                pose_msg.x = distance  # 距离（米）
                pose_msg.y = offset_y  # 横向偏移（米）
                pose_msg.z = offset_angle  # 偏移角（度）
                self.pose_pub.publish(pose_msg)
                
                self.get_logger().debug(f"距离：{distance:.2f}m | 偏移：{offset_y:.2f}m | 角度：{offset_angle:.2f}°")
        
        # 显示摄像头画面（调试）
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()