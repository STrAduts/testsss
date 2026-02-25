from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 获取原有场地包和本包的路径
    rc2026_field_share = get_package_share_directory('rc2026_field')
    follow_target_share = get_package_share_directory('robot_follow_target')
    
    # 2. 包含原有场地的launch（自动加载Gazebo+目标方块+机器人URDF）
    rc2026_launch_path = os.path.join(rc2026_field_share, 'launch', 'rc2026_field_sim.launch.py')
    rc2026_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rc2026_launch_path)
    )
    
    # 3. 加载本工程配置文件
    config_path = PathJoinSubstitution([
        FindPackageShare('robot_follow_target'),
        'config',
        'follow_params.yaml'
    ])
    
    # 4. 启动摄像头处理节点
    camera_node = Node(
        package='robot_follow_target',
        executable='camera_processor',
        name='camera_processor',
        parameters=[config_path],
        output='screen'
    )
    
    # 5. 启动跟踪控制节点
    follow_node = Node(
        package='robot_follow_target',
        executable='follow_controller',
        name='follow_controller',
        parameters=[config_path],
        output='screen'
    )
    
    # 组装Launch描述
    ld = LaunchDescription()
    ld.add_action(rc2026_launch)    # 先启动场地
    ld.add_action(camera_node)      # 再启动摄像头处理
    ld.add_action(follow_node)      # 最后启动跟踪控制
    return ld