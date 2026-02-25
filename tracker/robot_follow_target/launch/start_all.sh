#!/bin/bash
set -e

# 颜色输出函数
green_echo() { echo -e "\033[32m$1\033[0m"; }
red_echo() { echo -e "\033[31m$1\033[0m"; }

# 1. 检查ROS2环境
if [ -z "$ROS_DISTRO" ] || [ "$ROS_DISTRO" != "humble" ]; then
    red_echo "错误：未加载ROS2 Humble环境！"
    red_echo "请先执行：source /opt/ros/humble/setup.bash"
    exit 1
fi

# 2. 定义工作空间路径（需根据实际修改！）
WORKSPACE_DIR="$HOME/ros2_ws"  # 替换为你的ROS2工作空间路径
FIELD_PKG_DIR="$WORKSPACE_DIR/src/rc2026_field_hw"  # 场地包路径
FOLLOW_PKG_DIR="$WORKSPACE_DIR/src/robot_follow_target"  # 本包路径

# 3. 检查包路径
if [ ! -d "$FIELD_PKG_DIR" ]; then
    red_echo "错误：场地包路径不存在！请修改WORKSPACE_DIR和FIELD_PKG_DIR"
    exit 1
fi
if [ ! -d "$FOLLOW_PKG_DIR" ]; then
    red_echo "错误：跟踪包路径不存在！请修改WORKSPACE_DIR和FOLLOW_PKG_DIR"
    exit 1
fi

# 4. 编译工作空间
green_echo "===== 编译ROS2工作空间 ====="
cd "$WORKSPACE_DIR"
colcon build --packages-select rc2026_field robot_follow_target --symlink-install

# 5. 加载工作空间环境
green_echo "===== 加载环境变量 ====="
source "$WORKSPACE_DIR/install/setup.bash"

# 6. 启动仿真+跟踪节点
green_echo "===== 启动Gazebo仿真+跟踪节点 ====="
ros2 launch robot_follow_target follow_target_launch.py

# 异常处理
trap 'red_echo "程序异常退出！"; exit 1' SIGINT SIGTERM