#!/bin/bash

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

# 检查工作空间结构
if [ ! -d "src" ]; then
    echo -e "${RED}错误：请在ROS2工作空间根目录运行（需包含src文件夹）${NC}"
    exit 1
fi

# 编译包
echo -e "${GREEN}===== 编译ROS2包 =====${NC}"
colcon build --symlink-install --packages-select rc2026_field robot_follow_target
if [ $? -ne 0 ]; then
    echo -e "${RED}编译失败！请检查依赖和代码${NC}"
    exit 1
fi

# 加载环境变量
echo -e "${GREEN}===== 加载环境变量 =====${NC}"
source install/setup.bash

# 启动仿真和跟踪节点
echo -e "${GREEN}===== 启动Gazebo+跟踪节点 =====${NC}"
ros2 launch robot_follow_target gazebo_load_both.launch.py

# 结束提示
echo -e "${GREEN}===== 节点已停止 =====${NC}"