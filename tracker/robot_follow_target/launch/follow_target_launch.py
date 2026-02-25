from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    # 1. 找到场地包和当前包的路径
    rc2026_field_share = FindPackageShare('rc2026_field')
    follow_target_share = FindPackageShare('robot_follow_target')
    
    # 2. 包含场地包的仿真launch（加载gazebo+机器人+目标）
    field_launch_path = PathJoinSubstitution([
        rc2026_field_share, 'launch', 'rc2026_field_sim.launch.py'
    ])
    field_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(field_launch_path)
    )
    
    # 3. 设置GAZEBO_MODEL_PATH（确保场地模型能被找到）
    gazebo_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        PathJoinSubstitution([rc2026_field_share, 'resource'])
    )
    
    # 4. 启动跟踪节点（加载参数文件）
    follow_params_path = PathJoinSubstitution([
        follow_target_share, 'config', 'follow_params.yaml'
    ])
    tracker_node = Node(
        package='robot_follow_target',
        executable='target_tracker',
        name='target_tracker_node',
        output='screen',
        parameters=[follow_params_path]
    )
    
    # 5. 组装LaunchDescription
    ld = LaunchDescription()
    ld.add_action(gazebo_model_path)
    ld.add_action(field_launch)
    ld.add_action(tracker_node)
    return ld