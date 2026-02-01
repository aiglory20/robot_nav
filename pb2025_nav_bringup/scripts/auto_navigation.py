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
自动导航节点 - 依次导航到预定义的三个点 (A -> B -> C)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
import time
from tf2_ros import Buffer, TransformListener
import tf2_py as tf2


class AutoNavigator(Node):
    def __init__(self):
        super().__init__("auto_navigator")

        # 声明参数 (use_sim_time 会被 launch 文件自动设置,无需声明)
        self.declare_parameter("namespace", "")

        # 获取参数
        self.namespace = self.get_parameter("namespace").value

        # 创建 NavigateToPose action client
        # 如果namespace为空，直接使用 /navigate_to_pose
        if self.namespace:
            action_name = f"/{self.namespace}/navigate_to_pose"
        else:
            action_name = "/navigate_to_pose"
        
        self._action_client = ActionClient(self, NavigateToPose, action_name)

        # 发布目标点姿态，供控制器读取目标 yaw
        self.goal_pose_pub = self.create_publisher(PoseStamped, "goal_pose", 10)
        
        # 发布cmd_vel用于强制停止底盘
        from geometry_msgs.msg import Twist
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # TF2 buffer和listener，用于获取map->chassis变换
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅里程计，用于判断当前yaw是否对齐目标
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry", self.odom_callback, 10
        )

        self.get_logger().info(f"等待 action server: {action_name}")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server 已连接!")

        # 定义三个目标点 (x, y, yaw, wait_time) - rmuc_2025 地图上的坐标
        # wait_time: 到达后停留的时间（秒）
        self.waypoints = [
            {"name": "Point B", "x": -3.182, "y": -0.839, "yaw": 45.0, "wait_time": 10.0},
            {"name": "Point C", "x": -6.887, "y": 4.630, "yaw": 0.0, "wait_time": 1.0},
            {"name": "Point A", "x": 0.024, "y": -0.025, "yaw": 60.0, "wait_time": 1.0},
        ]
        
        # 原始坐标（问题可能在Point A太近）：
        # Point A: x=0.024, y=-0.025, yaw=60.0
        # X=  0.024m  Y= -0.025m  Yaw=   8.4° A点
        # X= -3.182m  Y= -0.839m  Yaw=  28.5° B点
        # X= -6.887m  Y=  4.630m  Yaw=  26.7° C点

        self.current_waypoint_index = 0
        self.is_navigating = False
        self.last_odom = None
        self.current_goal_yaw_rad = None
        self.current_goal_handle = None
        
        # Nav2 goal_checker tolerances (from nav2_params.yaml)
        self.xy_goal_tolerance = 0.12  # meters
        self.yaw_goal_tolerance = 0.35  # radians (20 degrees)
        
        # 自主到达判断
        self.goal_reached_count = 0
        self.goal_reached_threshold = 5  # 连续5次满足条件就强制前往下一个点

    def create_pose_stamped(self, x, y, yaw):
        """创建 PoseStamped 消息"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # 从 yaw 角度(度)转换为四元数
        from math import sin, cos, radians

        yaw = radians(yaw)

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(yaw / 2.0)
        pose.pose.orientation.w = cos(yaw / 2.0)

        return pose

    def quaternion_to_yaw(self, q):
        """将四元数转换为yaw角度(弧度)"""
        import math

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """将角度规范到 [-pi, pi]"""
        import math

        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, msg):
        """里程计回调"""
        self.last_odom = msg

    def send_goal(self, waypoint):
        """发送导航目标"""
        self.get_logger().info(
            f"导航到 {waypoint['name']}: x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint['yaw']}°"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(
            waypoint["x"], waypoint["y"], waypoint["yaw"]
        )

        # 保存目标yaw(弧度)
        from math import radians

        self.current_goal_yaw_rad = radians(waypoint["yaw"])

        # 同步发布目标点姿态
        self.goal_pose_pub.publish(goal_msg.pose)

        self.is_navigating = True
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("目标被拒绝!")
            self.is_navigating = False
            return

        self.get_logger().info("目标已接受,开始导航...")
        self.current_goal_handle = goal_handle  # 保存以便取消
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """反馈回调"""
        feedback = feedback_msg.feedback
        distance_remaining = getattr(feedback, "distance_remaining", None)
        if distance_remaining is not None:
            yaw_status = "未知"
            goal_checker_status = "未知"
            
            if self.current_goal_yaw_rad is not None:
                try:
                    # 使用TF获取map->chassis的yaw，与controller保持一致
                    transform = self.tf_buffer.lookup_transform(
                        "map", "chassis", rclpy.time.Time()
                    )
                    current_yaw = self.quaternion_to_yaw(transform.transform.rotation)
                    yaw_error = self.normalize_angle(self.current_goal_yaw_rad - current_yaw)
                    yaw_error_deg = abs(yaw_error * 180.0 / 3.141592653589793)
                    current_yaw_deg = current_yaw * 180.0 / 3.141592653589793
                    target_yaw_deg = self.current_goal_yaw_rad * 180.0 / 3.141592653589793
                    
                    # Check yaw alignment
                    yaw_status = "已对齐" if yaw_error_deg <= 5.0 else f"未对齐(当前{current_yaw_deg:.1f}° 目标{target_yaw_deg:.1f}° 误差{yaw_error_deg:.1f}°)"
                    
                    # Get current position from transform
                    current_x = transform.transform.translation.x
                    current_y = transform.transform.translation.y
                    
                    # Get goal position (assume it's from current waypoint)
                    waypoint = self.waypoints[self.current_waypoint_index]
                    goal_x = waypoint['x']
                    goal_y = waypoint['y']
                    
                    # Calculate actual distance to goal
                    import math
                    actual_distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
                    
                    # Check goal_checker criteria
                    xy_satisfied = actual_distance <= self.xy_goal_tolerance
                    yaw_satisfied = abs(yaw_error) <= self.yaw_goal_tolerance
                    
                    xy_status = f"✓ {actual_distance:.3f}m" if xy_satisfied else f"✗ {actual_distance:.3f}m"
                    yaw_checker_status = f"✓ {yaw_error_deg:.1f}°" if yaw_satisfied else f"✗ {yaw_error_deg:.1f}°"
                    
                    goal_checker_status = f"xy:{xy_status}(<{self.xy_goal_tolerance}m) yaw:{yaw_checker_status}(<{self.yaw_goal_tolerance*180/3.14159:.1f}°)"
                    
                    if xy_satisfied and yaw_satisfied:
                        goal_checker_status = "🎯 " + goal_checker_status
                        self.goal_reached_count += 1
                        
                        # 连续满足条件，主动前往下一个点
                        if self.goal_reached_count >= self.goal_reached_threshold:
                            self.get_logger().info(
                                f"✅ 自主判断已到达目标！连续{self.goal_reached_count}次满足条件"
                            )
                            # 取消当前goal并前往下一个
                            if self.current_goal_handle is not None:
                                self.current_goal_handle.cancel_goal_async()
                                self.force_next_waypoint()
                            return
                    else:
                        self.goal_reached_count = 0  # 重置计数器
                    
                except Exception as e:
                    yaw_status = f"TF错误: {str(e)}"
                    goal_checker_status = f"错误: {str(e)}"
                    self.goal_reached_count = 0

            self.get_logger().info(
                f"导航中... 剩余:{distance_remaining:.2f}m | {yaw_status} | Goal检查: {goal_checker_status}"
            )

    def force_next_waypoint(self):
        """强制前往下一个航点（绕过Nav2的goal_checker）"""
        waypoint = self.waypoints[self.current_waypoint_index]
        wait_time = waypoint.get('wait_time', 1.0)  # 默认1秒
        
        self.get_logger().info(f"✅ 强制完成 {waypoint['name']}!")
        self.current_goal_handle = None
        self.goal_reached_count = 0
        
        # 验证最终位置
        if self.current_goal_yaw_rad is not None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "chassis", rclpy.time.Time()
                )
                current_yaw = self.quaternion_to_yaw(transform.transform.rotation)
                yaw_error = self.normalize_angle(self.current_goal_yaw_rad - current_yaw)
                yaw_error_deg = abs(yaw_error * 180.0 / 3.141592653589793)
                current_yaw_deg = current_yaw * 180.0 / 3.141592653589793
                target_yaw_deg = self.current_goal_yaw_rad * 180.0 / 3.141592653589793
                self.get_logger().info(
                    f"最终位置: 当前yaw={current_yaw_deg:.1f}° 目标yaw={target_yaw_deg:.1f}° 误差={yaw_error_deg:.1f}°"
                )
            except Exception as e:
                pass
        
        # 发送零速度命令确保底盘完全停止
        from geometry_msgs.msg import Twist
        stop_cmd = Twist()
        for _ in range(5):  # 连续发送5次确保收到
            self.cmd_vel_pub.publish(stop_cmd)
            time.sleep(0.02)
        
        # 根据配置的停留时间等待（每隔0.5秒发送一次零速度保持停止）
        self.get_logger().info(f"⏸️  在 {waypoint['name']} 停留 {wait_time:.1f} 秒...")
        elapsed = 0.0
        interval = 0.5
        while elapsed < wait_time:
            self.cmd_vel_pub.publish(stop_cmd)
            sleep_time = min(interval, wait_time - elapsed)
            time.sleep(sleep_time)
            elapsed += sleep_time
        
        # 移动到下一个航点
        self.current_waypoint_index += 1
        if self.current_waypoint_index < len(self.waypoints):
            self.send_goal(self.waypoints[self.current_waypoint_index])
        else:
            self.get_logger().info("🎉 所有航点已完成! 自动导航结束.")

    def get_result_callback(self, future):
        """结果回调"""
        result_future = future.result()
        result = result_future.result
        status_code = result_future.status
        self.is_navigating = False

        waypoint = self.waypoints[self.current_waypoint_index]

        if status_code != GoalStatus.STATUS_SUCCEEDED:
            # 解析状态码
            status_names = {
                1: "ACCEPTED (目标已接受)",
                2: "EXECUTING (执行中)",
                3: "CANCELING (取消中)",
                4: "SUCCEEDED (成功)",
                5: "CANCELED (已取消)",
                6: "ABORTED (中止 - 规划器失败/目标不可达/controller错误)",
                7: "PREEMPTED (被抢占)",
            }
            status_name = status_names.get(status_code, f"未知({status_code})")
            
            self.get_logger().error(
                f"❌ 导航到 {waypoint['name']} 失败! 状态: {status_name}"
            )
            
            # 打印诊断信息
            if status_code == 6:  # ABORTED
                self.get_logger().error(
                    "可能原因:\n"
                    "  1. 目标点不可达（在障碍物内或地图外）\n"
                    "  2. 规划器无法生成路径\n"
                    "  3. Controller执行失败\n"
                    "  4. 目标点太近且yaw误差太大\n"
                    f"  5. 检查目标: x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint['yaw']}°"
                )
            
            # 如果goal_handle仍然存在，尝试取消
            if self.current_goal_handle is not None:
                self.get_logger().info("取消当前导航任务...")
                cancel_future = self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            
            self.get_logger().error("自动导航终止。")
            return  # 失败时停止，不继续下一个点

        # 只有成功时才继续
        self.get_logger().info(f"✅ 成功到达 {waypoint['name']}!")
        self.current_goal_handle = None  # 清空已完成的goal_handle
        
        # 验证最终位置和朝向
        if self.current_goal_yaw_rad is not None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "chassis", rclpy.time.Time()
                )
                current_yaw = self.quaternion_to_yaw(transform.transform.rotation)
                yaw_error = self.normalize_angle(self.current_goal_yaw_rad - current_yaw)
                yaw_error_deg = abs(yaw_error * 180.0 / 3.141592653589793)
                current_yaw_deg = current_yaw * 180.0 / 3.141592653589793
                target_yaw_deg = self.current_goal_yaw_rad * 180.0 / 3.141592653589793
                self.get_logger().info(
                    f"最终位置: 当前yaw={current_yaw_deg:.1f}° 目标yaw={target_yaw_deg:.1f}° 误差={yaw_error_deg:.1f}°"
                )
            except Exception as e:
                self.get_logger().warn(f"无法验证最终姿态: {e}")

        # 等待5秒后继续下一个点
        time.sleep(5.0)

        # 移动到下一个航点
        self.current_waypoint_index += 1
        if self.current_waypoint_index < len(self.waypoints):
            self.send_goal(self.waypoints[self.current_waypoint_index])
        else:
            self.get_logger().info("🎉 所有航点已完成! 自动导航结束.")

    def start_navigation(self):
        """开始自动导航"""
        if len(self.waypoints) > 0:
            self.get_logger().info(f"开始自动导航,共 {len(self.waypoints)} 个航点")
            self.send_goal(self.waypoints[0])
        else:
            self.get_logger().error("没有定义航点!")


def main(args=None):
    rclpy.init(args=args)

    navigator = AutoNavigator()

    # 等待5秒确保导航系统完全启动（包括costmap、localization等）
    navigator.get_logger().info("等待5秒,确保导航系统完全启动...")
    time.sleep(5.0)

    # 开始自动导航
    navigator.start_navigation()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("收到中断信号,停止导航")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
