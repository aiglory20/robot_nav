#!/usr/bin/env python3
# Copyright 2025 aiglory20
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
        "namespace", default_value="red_standard_robot1", description="机器人命名空间"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="是否使用仿真时间"
    )

    # 自动导航节点
    auto_navigator_node = Node(
        package="pb2025_nav_bringup",
        executable="auto_navigation.py",
        name="auto_navigator",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "namespace": LaunchConfiguration("namespace"),
            }
        ],
    )

    return LaunchDescription(
        [
            namespace_arg,
            use_sim_time_arg,
            auto_navigator_node,
        ]
    )
