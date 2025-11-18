#!/usr/bin/env python3
"""
自动导航启动文件
在已有的导航系统基础上,启动自动导航节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='red_standard_robot1',
        description='机器人命名空间'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )

    # 自动导航节点
    auto_navigator_node = Node(
        package='pb2025_nav_bringup',
        executable='auto_navigation.py',
        name='auto_navigator',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
        }]
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        auto_navigator_node,
    ])
