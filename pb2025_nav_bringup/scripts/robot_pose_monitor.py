#!/usr/bin/env python3
"""
机器人位姿监控节点
使用 tf2_ros 的 lookupTransform() 函数查询机器人在 map 坐标系下的位姿
特别关注 yaw 值（车辆朝向角度）
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException
import math
from geometry_msgs.msg import TransformStamped

# Usage:有命名空间时
'''python3 robot_pose_monitor.py \
  --ros-args \
  -p namespace:=red_standard_robot1 \
  -p source_frame:=chassis \
  -p publish_rate:=1.0 \
  -r /tf:=/red_standard_robot1/tf \
  -r /tf_static:=/red_standard_robot1/tf_static'''
# Usage:无命名空间时
'''python3 robot_pose_monitor.py '''

class RobotPoseMonitor(Node):
    """监控机器人位姿的节点"""

    def __init__(self):
        super().__init__('robot_pose_monitor')

        # 声明参数
        self.declare_parameter('source_frame', 'chassis')
        self.declare_parameter('namespace', '')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('show_map', True)  # 是否显示map->chassis
        self.declare_parameter('show_odom', True)  # 是否显示odom->chassis

        # 获取参数
        self.source_frame = self.get_parameter('source_frame').value
        namespace = self.get_parameter('namespace').value
        publish_rate = self.get_parameter('publish_rate').value
        self.show_map = self.get_parameter('show_map').value
        self.show_odom = self.get_parameter('show_odom').value

        # 注意：TF frames的名称不包含命名空间前缀，
        # 只有话题名称(/tf, /tf_static)需要命名空间

        self.get_logger().info('='*60)
        self.get_logger().info('机器人位姿监控节点已启动')
        self.get_logger().info(
            f'  TF话题命名空间: {namespace if namespace else "无"}')
        self.get_logger().info(f'  源坐标系: {self.source_frame}')
        self.get_logger().info(
            f'  显示 map -> {self.source_frame}: {self.show_map}')
        self.get_logger().info(
            f'  显示 odom -> {self.source_frame}: {self.show_odom}')
        self.get_logger().info(f'  更新频率: {publish_rate} Hz')
        self.get_logger().info('='*60)

        # 创建 TF Buffer 和 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 创建定时器
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

    def quaternion_to_euler(self, x, y, z, w):
        """
        将四元数转换为欧拉角 (roll, pitch, yaw)

        Args:
            x, y, z, w: 四元数分量

        Returns:
            tuple: (roll, pitch, yaw) 弧度值
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def timer_callback(self):
        """定时器回调函数，查询并打印机器人位姿"""
        self.get_logger().info('='*60)

        # 查询 map -> chassis
        if self.show_map:
            self.query_and_display_transform('map', self.source_frame)

        # 查询 odom -> chassis
        if self.show_odom:
            self.query_and_display_transform('odom', self.source_frame)

    def query_and_display_transform(self, target_frame, source_frame):
        """查询并显示指定的变换"""
        try:
            # 使用 lookupTransform() 查询变换
            # 参数: 目标坐标系, 源坐标系, 时间 (rclpy.time.Time() 表示最新)
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )

            # 提取位置信息
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # 提取四元数
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            # 转换为欧拉角
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)

            # 打印结果
            self.get_logger().info('-' * 60)
            self.get_logger().info(
                f'📍 机器人位姿 [{target_frame} -> {source_frame}]:')
            '''self.get_logger().info(f'  位置:')
            self.get_logger().info(f'    x = {x:.3f} m')
            self.get_logger().info(f'    y = {y:.3f} m')
            self.get_logger().info(f'    z = {z:.3f} m')
            self.get_logger().info(f'  姿态 (欧拉角):')
            self.get_logger().info(
                f'    roll  = {roll:.3f} rad ({math.degrees(roll):.2f}°)')
            self.get_logger().info(
                f'    pitch = {pitch:.3f} rad ({math.degrees(pitch):.2f}°)')'''
            self.get_logger().info(
                f'    yaw   = {yaw:.3f} rad ({math.degrees(yaw):.2f}°) ← 车辆朝向')
            '''self.get_logger().info(f'  四元数:')
            self.get_logger().info(
                f'    x = {qx:.4f}, y = {qy:.4f}, z = {qz:.4f}, w = {qw:.4f}')'''

        except TransformException as ex:
            self.get_logger().warn(
                f'⚠️  无法获取变换 {target_frame} -> {source_frame}: {ex}',
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = RobotPoseMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
