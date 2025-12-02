"""工具：手动发送自定义十六进制帧，或订阅 /cmd_vel 自动构造底盘 CAN 帧."""

from __future__ import annotations

import argparse
import importlib
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

# ! 运行方式，先运行nav2,再另一个终端python can.py --mode cmd_vel --port /dev/ttyACM0 --cmd-topic /cmd_vel --echo

# ---------------------------------------------------------------------------
# Serial helper (兼容不同 pyserial 安装方式)


def _resolve_serial_class():
    try:
        serial_mod = importlib.import_module('serial')
        serial_cls = getattr(serial_mod, 'Serial', None)
        if serial_cls is None:
            if sys.platform.startswith('win'):
                serial_cls = importlib.import_module('serial.serialwin32').Serial
            else:
                serial_cls = importlib.import_module('serial.serialposix').Serial
        if not hasattr(serial_mod, 'STOPBITS_ONE'):
            try:
                serialutil = importlib.import_module('serial.serialutil')
                _consts = [
                    'PARITY_NONE', 'PARITY_EVEN', 'PARITY_ODD', 'PARITY_MARK', 'PARITY_SPACE',
                    'STOPBITS_ONE', 'STOPBITS_ONE_POINT_FIVE', 'STOPBITS_TWO',
                    'FIVEBITS', 'SIXBITS', 'SEVENBITS', 'EIGHTBITS',
                    'CR', 'LF', 'XON', 'XOFF', 'PARITY_NAMES'
                ]
                for _name in _consts:
                    if hasattr(serialutil, _name) and not hasattr(serial_mod, _name):
                        setattr(serial_mod, _name, getattr(serialutil, _name))
            except Exception:
                pass
        return serial_cls
    except Exception as exc:  # pragma: no cover - import guard
        raise ImportError(
            "无法导入 pyserial，请先安装: python -m pip install pyserial"
        ) from exc


Serial = _resolve_serial_class()


# ---------------------------------------------------------------------------
# 通用帧编码/解码工具


START_BYTE_DEFAULT = 0x7B
END_BYTE_DEFAULT = 0x7D
FRAME_LENGTH = 11


def parse_hex_string(text: str) -> Optional[bytes]:
    cleaned = text.replace(' ', '').replace('\t', '')
    if not cleaned or len(cleaned) % 2 != 0:
        return None
    try:
        return bytes.fromhex(cleaned)
    except ValueError:
        return None


def xor_checksum(data: Iterable[int]) -> int:
    result = 0
    for b in data:
        result ^= (b & 0xFF)
    return result & 0xFF


def clamp_int16(value: int) -> int:
    return max(-0x8000, min(0x7FFF, value))


def float_to_int16(value: float, scale: float, deadband: float = 0.0) -> int:
    if abs(value) < deadband:
        value = 0.0
    scaled = int(round(value * scale))
    return clamp_int16(scaled)


def build_velocity_frame(
    vx: int,
    vy: int,
    wz: int,
    reserve1: int,
    reserve2: int,
    start_byte: int,
    end_byte: int,
) -> bytes:
    frame = bytearray(FRAME_LENGTH)
    frame[0] = start_byte & 0xFF
    frame[1] = reserve1 & 0xFF
    frame[2] = reserve2 & 0xFF
    frame[3:5] = vx.to_bytes(2, 'big', signed=True)
    frame[5:7] = vy.to_bytes(2, 'big', signed=True)
    frame[7:9] = wz.to_bytes(2, 'big', signed=True)
    frame[9] = xor_checksum(frame[:9])
    frame[10] = end_byte & 0xFF
    return bytes(frame)


# ---------------------------------------------------------------------------
# ROS2 bridge


@dataclass
class BridgeConfig:
    start_byte: int
    end_byte: int
    linear_multiplier: float
    angular_multiplier: float
    deadband_linear: float
    deadband_angular: float
    reserve1: int
    reserve2: int
    cmd_topic: str
    echo: bool
    keepalive: float


def _import_ros_interfaces():  # pragma: no cover - only runs in cmd_vel 模式
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    return rclpy, Node, Twist


def run_cmd_vel_bridge(serial_handle, cfg: BridgeConfig, ros_args: list[str]) -> None:
    rclpy, Node, Twist = _import_ros_interfaces()

    class CmdVelBridge(Node):
        def __init__(self):
            super().__init__('cmd_vel_to_can_bridge')
            self.serial = serial_handle
            self.cfg = cfg
            self.last_frame: Optional[bytes] = None
            self.subscription = self.create_subscription(
                Twist, cfg.cmd_topic, self.cmd_callback, 10)
            self.keepalive_timer = None
            if cfg.keepalive > 0.0:
                self.keepalive_timer = self.create_timer(cfg.keepalive, self.publish_keepalive)
            self.get_logger().info(
                f"已订阅 {cfg.cmd_topic}，线速度倍率={cfg.linear_multiplier}，角速度倍率={cfg.angular_multiplier}")

        def cmd_callback(self, msg: Twist):
            vx = float_to_int16(msg.linear.x, cfg.linear_multiplier, cfg.deadband_linear)
            vy = float_to_int16(msg.linear.y, cfg.linear_multiplier, cfg.deadband_linear)
            wz = float_to_int16(msg.angular.z, cfg.angular_multiplier, cfg.deadband_angular)
            frame = build_velocity_frame(
                vx, vy, wz, cfg.reserve1, cfg.reserve2, cfg.start_byte, cfg.end_byte)
            try:
                self.serial.write(frame)
            except Exception as exc:  # pragma: no cover - hardware errors
                self.get_logger().error(f"串口写入失败: {exc}")
                return
            self.last_frame = frame
            if cfg.echo:
                self.get_logger().info(
                    f"TX vx={vx} vy={vy} wz={wz} frame={' '.join(f'{b:02X}' for b in frame)}")

        def publish_keepalive(self):
            if not self.last_frame:
                return
            try:
                self.serial.write(self.last_frame)
            except Exception as exc:  # pragma: no cover
                self.get_logger().warning(f"Keepalive 发送失败: {exc}")

    rclpy.init(args=ros_args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，准备退出')
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# 手动交互模式


def interactive_console(serial_handle, default_msg: str) -> None:
    print("输入要发送的十六进制字符串（例如: 7B 00 00 01），或直接回车使用默认：")
    while True:
        try:
            user = input(f"hex (or 'quit') [default: {default_msg}]: ")
        except EOFError:
            user = ''

        if user is None:
            break
        user = user.strip()
        if user.lower() == 'quit':
            break
        if user == '':
            user = default_msg

        payload = parse_hex_string(user)
        if payload is None:
            print("无法解析为十六进制字节串，请确认输入（例如: '7B 00 01' 或 '7B0001'）。")
            continue

        try:
            serial_handle.write(payload)
        except Exception as exc:
            print(f"写入串口失败: {exc}")
            break

        print(f"已发送 (hex): {' '.join(f'{b:02X}' for b in payload)}")
        time.sleep(0.05)
        try:
            waiting = serial_handle.in_waiting
        except Exception:
            waiting = 0
        if waiting > 0:
            received = serial_handle.read(waiting)
            if received:
                hex_repr = ' '.join(f"{b:02X}" for b in received)
                print(f"已接收 (hex): {hex_repr}")
            else:
                print("已接收: <空>")
        else:
            print("未接收到数据")


# ---------------------------------------------------------------------------
# CLI


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="串口/底盘调试工具：手动发送或将 /cmd_vel 映射为 CAN 帧")
    parser.add_argument('--mode', choices=['cmd_vel', 'manual'], default='cmd_vel',
                        help='cmd_vel: ROS2 订阅发送；manual: 交互式十六进制发送')
    parser.add_argument('--port', default='/dev/ttyACM0', help='串口设备，例如 /dev/ttyUSB0')
    parser.add_argument('--baudrate', type=int, default=115200, help='串口波特率')
    parser.add_argument('--timeout', type=float, default=0.02, help='串口超时时间 (s)')
    parser.add_argument('--default-msg', default="7B 00 00 00 64 00 00 00 00 1F 7D",
                        help='手动模式下默认发送帧')
    parser.add_argument('--cmd-topic', default='cmd_vel', help='ROS2 cmd_vel 话题名')
    parser.add_argument('--linear-multiplier', type=float, default=1000.0,
                        help='线速度缩放系数 (m/s * multiplier -> mm/s -> int16)')
    parser.add_argument('--angular-multiplier', type=float, default=1000.0,
                        help='角速度缩放系数 (rad/s * multiplier -> int16)')
    parser.add_argument('--deadband-linear', type=float, default=0.0,
                        help='线速度死区，小于该值视为 0')
    parser.add_argument('--deadband-angular', type=float, default=0.0,
                        help='角速度死区，小于该值视为 0')
    parser.add_argument('--reserve1', type=int, default=0,
                        help='第2字节（预留1）')
    parser.add_argument('--reserve2', type=int, default=0,
                        help='第3字节（预留2）')
    parser.add_argument('--start-byte', type=lambda v: int(v, 0), default=START_BYTE_DEFAULT,
                        help='帧起始字节 (默认 0x7B)')
    parser.add_argument('--end-byte', type=lambda v: int(v, 0), default=END_BYTE_DEFAULT,
                        help='帧结束字节 (默认 0x7D)')
    parser.add_argument('--keepalive', type=float, default=0.2,
                        help='cmd_vel 模式下的保活周期 (s)，<=0 关闭保活')
    parser.add_argument('--echo', action='store_true', help='打印每一帧的编码结果')
    return parser


def main() -> None:
    parser = build_arg_parser()
    args, ros_args = parser.parse_known_args()

    serial_handle = Serial(port=args.port, baudrate=args.baudrate, timeout=args.timeout)
    print(f"已连接串口 {args.port} @ {args.baudrate}bps")
    try:
        if args.mode == 'manual':
            interactive_console(serial_handle, args.default_msg)
            return

        cfg = BridgeConfig(
            start_byte=args.start_byte & 0xFF,
            end_byte=args.end_byte & 0xFF,
            linear_multiplier=args.linear_multiplier,
            angular_multiplier=args.angular_multiplier,
            deadband_linear=args.deadband_linear,
            deadband_angular=args.deadband_angular,
            reserve1=args.reserve1 & 0xFF,
            reserve2=args.reserve2 & 0xFF,
            cmd_topic=args.cmd_topic,
            echo=args.echo,
            keepalive=max(0.0, args.keepalive),
        )
        run_cmd_vel_bridge(serial_handle, cfg, ros_args)
    finally:
        try:
            serial_handle.close()
        except Exception:
            pass
        print('串口已关闭')


if __name__ == '__main__':
    main()