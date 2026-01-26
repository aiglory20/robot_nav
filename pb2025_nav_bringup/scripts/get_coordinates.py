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
坐标获取工具 - 实时显示机器人位置并获取 RViz 发布的目标点坐标
使用方法:
1. 运行此脚本 - 会实时显示机器人当前在地图上的位置
2. 在 RViz 中使用 "Nav2 Goal" 或 "2D Pose Estimate" 工具点击地图
3. 脚本会输出点击位置的坐标
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from math import atan2, pi, degrees
import sys


class CoordinateGetter(Node):
    def __init__(self):
        super().__init__("coordinate_getter")

        # 声明参数
        self.declare_parameter("namespace", "red_standard_robot1")
        self.namespace = self.get_parameter("namespace").value

        # TF监听器 - 用于获取地图位置
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅里程计 - 用于显示实时位置
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry", self.odom_callback, 10
        )

        # 订阅 Nav2 Goal (从 2D Nav Goal 工具)
        self.goal_sub = self.create_subscription(
            PoseStamped, f"/{self.namespace}/goal_pose", self.goal_callback, 10
        )

        # 订阅初始位姿 (从 2D Pose Estimate 工具)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.pose_callback, 10
        )

        # 定时器 - 每2秒输出一次当前位置
        self.timer = self.create_timer(2.0, self.print_current_position)

        self.waypoint_count = 0
        self.last_odom = None
        self.show_position = True  # 控制是否显示实时位置

        self.get_logger().info("=" * 70)
        self.get_logger().info("🤖 坐标获取工具已启动!")
        self.get_logger().info("功能:")
        self.get_logger().info("  1. 实时显示机器人在地图上的位置 (每2秒)")
        self.get_logger().info('  2. 在 RViz 中使用 "Nav2 Goal" 获取目标点坐标')
        self.get_logger().info('  3. 在 RViz 中使用 "2D Pose Estimate" 获取任意位置坐标')
        self.get_logger().info("=" * 70)

    def quaternion_to_yaw(self, q):
        """将四元数转换为 yaw 角度(弧度)"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)
        return yaw

    def odom_callback(self, msg):
        """里程计回调 - 保存最新数据"""
        self.last_odom = msg

    def print_current_position(self):
        """定时输出机器人当前位置"""
        if not self.show_position:
            return

        print("\n" + "─" * 70)
        print("🤖 机器人实时位置")

        # 1. 显示里程计位置（odom坐标系）
        if self.last_odom:
            x = self.last_odom.pose.pose.position.x
            y = self.last_odom.pose.pose.position.y
            yaw = self.quaternion_to_yaw(self.last_odom.pose.pose.orientation)

            vx = self.last_odom.twist.twist.linear.x
            vy = self.last_odom.twist.twist.linear.y
            vz = self.last_odom.twist.twist.angular.z

            print(f"📍 里程计 (odom): X={x:7.3f}m  Y={y:7.3f}m  Yaw={degrees(yaw):6.1f}°")
            print(f"🏃 速度: Vx={vx:6.3f}m/s  Vy={vy:6.3f}m/s  Wz={vz:6.3f}rad/s")
        else:
            print("⚠️  等待里程计数据 (/odometry)...")

        # 2. 显示地图位置（map坐标系）
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
            yaw = self.quaternion_to_yaw(q)

            print(f"🗺️  地图位置 (map): X={x:7.3f}m  Y={y:7.3f}m  Yaw={degrees(yaw):6.1f}°")
            print(f"   ✅ 机器人已定位到地图上")

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print(f"❌ 无法获取地图位置 (map->base_footprint)")
            print(f"   提示: 检查定位节点是否运行或设置 use_gicp_localization:=False")

        print("─" * 70)
        sys.stdout.flush()

    def print_coordinate(self, x, y, yaw, source):
        """打印航点坐标信息"""
        self.waypoint_count += 1

        # 临时关闭实时位置显示
        self.show_position = False

        print("\n")
        print("=" * 70)
        print(f"🎯 航点 {self.waypoint_count} ({source})")
        print("=" * 70)
        print(f"  X:   {x:.3f} m")
        print(f"  Y:   {y:.3f} m")
        print(f"  Yaw: {yaw:.3f} rad ({yaw * 180 / pi:.1f}°)")
        print("")
        print("📋 复制到代码中:")
        print(f"  {{'name': 'Point_{chr(64 + self.waypoint_count)}', "
              f"'x': {x:.2f}, 'y': {y:.2f}, 'yaw': {yaw:.2f}}},")
        print("=" * 70)
        print("")
        sys.stdout.flush()

        # 3秒后恢复实时位置显示
        self.create_timer(3.0, self.resume_position_display)

    def resume_position_display(self):
        """恢复实时位置显示"""
        self.show_position = True

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
