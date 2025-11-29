#!/bin/bash

# 自动导航演示脚本
# 此脚本展示了如何使用自动导航功能

echo "=========================================="
echo "自动导航演示"
echo "=========================================="
echo ""
echo "请确保已经执行以下步骤:"
echo "1. 终端1: 启动仿真环境"
echo "   ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
echo ""
echo "2. 终端2: 启动导航系统"  
echo "   ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=False"
echo ""
echo "现在将启动自动导航节点..."
echo ""
read -p "按 Enter 继续, 或 Ctrl+C 取消..."

# 切换到工作空间
cd ~/ros_ws

# Source 环境
source install/setup.zsh

# 启动自动导航
ros2 launch pb2025_nav_bringup auto_navigation_launch.py
