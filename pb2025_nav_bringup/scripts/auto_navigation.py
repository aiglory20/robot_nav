#!/usr/bin/env python3
"""
自动导航节点 - 依次导航到预定义的三个点 (A -> B -> C)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import time


class AutoNavigator(Node):
    def __init__(self):
        super().__init__('auto_navigator')

        # 声明参数 (use_sim_time 会被 launch 文件自动设置,无需声明)
        self.declare_parameter('namespace', 'red_standard_robot1')

        # 获取参数
        self.namespace = self.get_parameter('namespace').value

        # 创建 NavigateToPose action client
        action_name = f'/{self.namespace}/navigate_to_pose'
        self._action_client = ActionClient(self, NavigateToPose, action_name)

        self.get_logger().info(f'等待 action server: {action_name}')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server 已连接!')

        # 定义三个目标点 (x, y, yaw) - rmuc_2025 地图上的坐标
        # 这些坐标是示例,您可以根据实际地图调整
        self.waypoints = [
            {'name': 'Point A', 'x': 5.36035, 'y': 4.56871, 'yaw': 0.0},
            {'name': 'Point B', 'x': 10.731984, 'y': 0.920454, 'yaw': 0.0},
            {'name': 'Point C', 'x': 1.74057, 'y': -2.87552, 'yaw': 0.0},
        ]

        self.current_waypoint_index = 0
        self.is_navigating = False

    def create_pose_stamped(self, x, y, yaw):
        """创建 PoseStamped 消息"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # 从 yaw 角度转换为四元数
        from math import sin, cos
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(yaw / 2.0)
        pose.pose.orientation.w = cos(yaw / 2.0)

        return pose

    def send_goal(self, waypoint):
        """发送导航目标"""
        self.get_logger().info(
            f'导航到 {waypoint["name"]}: x={waypoint["x"]}, y={waypoint["y"]}, yaw={waypoint["yaw"]}')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(
            waypoint['x'], waypoint['y'], waypoint['yaw'])

        self.is_navigating = True
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝!')
            self.is_navigating = False
            return

        self.get_logger().info('目标已接受,开始导航...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """反馈回调"""
        feedback = feedback_msg.feedback
        # 可以在这里处理导航过程中的反馈信息
        # self.get_logger().info(f'导航中... 剩余距离: {feedback.distance_remaining:.2f}m')

    def get_result_callback(self, future):
        """结果回调"""
        result = future.result().result
        self.is_navigating = False

        waypoint = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f'到达 {waypoint["name"]}!')

        # 等待1秒后继续下一个点
        time.sleep(1.0)

        # 移动到下一个航点
        self.current_waypoint_index += 1
        if self.current_waypoint_index < len(self.waypoints):
            self.send_goal(self.waypoints[self.current_waypoint_index])
        else:
            self.get_logger().info('所有航点已完成! 自动导航结束.')

    def start_navigation(self):
        """开始自动导航"""
        if len(self.waypoints) > 0:
            self.get_logger().info(f'开始自动导航,共 {len(self.waypoints)} 个航点')
            self.send_goal(self.waypoints[0])
        else:
            self.get_logger().error('没有定义航点!')


def main(args=None):
    rclpy.init(args=args)

    navigator = AutoNavigator()

    # 等待2秒确保导航系统完全启动
    navigator.get_logger().info('等待2秒,确保导航系统启动...')
    time.sleep(2.0)

    # 开始自动导航
    navigator.start_navigation()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('收到中断信号,停止导航')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
