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
坐标获取工具 - 订阅 RViz 发布的目标点,显示坐标信息
使用方法:
1. 运行此脚本
2. 在 RViz 中使用 "Nav2 Goal" 或 "2D Pose Estimate" 工具点击地图
3. 脚本会输出点击位置的坐标
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from math import atan2, pi


class CoordinateGetter(Node):
    def __init__(self):
        super().__init__("coordinate_getter")

        # 声明参数
        self.declare_parameter("namespace", "red_standard_robot1")
        self.namespace = self.get_parameter("namespace").value

        # 订阅 Nav2 Goal (从 2D Nav Goal 工具)
        self.goal_sub = self.create_subscription(
            PoseStamped, f"/{self.namespace}/goal_pose", self.goal_callback, 10
        )

        # 订阅初始位姿 (从 2D Pose Estimate 工具)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.pose_callback, 10
        )

        self.waypoint_count = 0

        self.get_logger().info("=" * 60)
        self.get_logger().info("坐标获取工具已启动!")
        self.get_logger().info("请在 RViz 中使用以下工具点击地图:")
        self.get_logger().info('  - "Nav2 Goal" (2D Nav Goal) - 设置目标点')
        self.get_logger().info('  - "2D Pose Estimate" - 获取任意位置坐标')
        self.get_logger().info("=" * 60)

    def quaternion_to_yaw(self, q):
        """将四元数转换为 yaw 角度(弧度)"""
        # yaw = atan2(2.0 * (q.w * q.z + q.x * q.y),
        #             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)
        return yaw

    def print_coordinate(self, x, y, yaw, source):
        """打印坐标信息"""
        self.waypoint_count += 1

        self.get_logger().info("")
        self.get_logger().info(f"--- 航点 {self.waypoint_count} ({source}) ---")
        self.get_logger().info(f"  X:   {x:.3f} m")
        self.get_logger().info(f"  Y:   {y:.3f} m")
        self.get_logger().info(f"  Yaw: {yaw:.3f} rad ({yaw * 180 / pi:.1f}°)")
        self.get_logger().info("")
        self.get_logger().info("复制到代码中:")
        self.get_logger().info(
            f"  {{'name': 'Point {chr(64 + self.waypoint_count)}', "
            f"'x': {x:.2f}, 'y': {y:.2f}, 'yaw': {yaw:.2f}}},"
        )
        self.get_logger().info("-" * 60)

    def goal_callback(self, msg):
        """处理 Nav2 Goal"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.orientation)
        self.print_coordinate(x, y, yaw, "Nav2 Goal")

    def pose_callback(self, msg):
        """处理 2D Pose Estimate"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.print_coordinate(x, y, yaw, "2D Pose Estimate")


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateGetter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n获取了 {} 个坐标点".format(node.waypoint_count))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
