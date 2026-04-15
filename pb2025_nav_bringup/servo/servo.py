import time
import threading
import sys
import serial
import serial.tools.list_ports

# 串口配置
DEFAULT_BAUDRATE = 115200

# 机械臂舵机配置 (PWM范围: 500-2500, 对应角度: 0°-270°)
SERVO_CONFIGS = {
    0: {"name": "肩部前后", "min_pwm": 500, "max_pwm": 2500, "min_desc": "后上", "max_desc": "前上"},
    1: {"name": "肩部上下", "min_pwm": 1155, "max_pwm": 1790, "min_desc": "最上", "max_desc": "最内"},
    2: {"name": "肘部前后", "min_pwm": 934, "max_pwm": 1988, "min_desc": "后外翻", "max_desc": "前内曲"},
    3: {"name": "机械爪",   "min_pwm": 924, "max_pwm": 1612, "min_desc": "最大开口", "max_desc": "夹住"},
}

class BusServoController:
    def __init__(self):
        self.ser = None
        self.running = True
        
        self.connect_serial()
        
        if self.ser:
            self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
            self.rx_thread.start()
            
            self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
            self.input_thread.start()
        else:
            self.running = False

    def connect_serial(self):
        """扫描并连接串口"""
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("❌ 未发现可用串口设备！请检查连接。")
            return

        print("\n" + "="*40)
        print("🔌 发现可用串口:")
        for i, p in enumerate(ports):
            print(f"  [{i}] {p.device} - {p.description}")
        print("="*40)
        
        try:
            idx_str = input("👉 请输入要连接的串口序号 (默认按回车选 [0]): ").strip()
            idx = int(idx_str) if idx_str.isdigit() else 0
            if 0 <= idx < len(ports):
                port = ports[idx].device
                self.ser = serial.Serial(port, DEFAULT_BAUDRATE, timeout=0.1)
                print(f"✅ 成功连接串口: {port} @ {DEFAULT_BAUDRATE} bps")
            else:
                print("❌ 无效的序号！")
        except Exception as e:
            print(f"❌ 连接串口失败: {e}")

    def rx_loop(self):
        """串口接收线程"""
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            # 清除当前输入行，打印接收到的消息，然后再重新打印输入提示符
                            sys.stdout.write('\r' + ' ' * 50 + '\r')
                            print(f"📥 [下位机回复] {line}")
                            sys.stdout.write(">> ")
                            sys.stdout.flush()
                except Exception as e:
                    print(f"\n❌ 串口读取错误: {e}")
                    self.running = False
            time.sleep(0.01)

    def send_cmd(self, cmd_str):
        """发送串口指令"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd_str.encode('utf-8'))
                print(f"📤 [发送指令] {cmd_str}")
            except Exception as e:
                print(f"❌ 发送失败: {e}")

    def get_angle_range(self, config):
        min_angle = (config["min_pwm"] - 500) / 2000.0 * 270.0
        max_angle = (config["max_pwm"] - 500) / 2000.0 * 270.0
        return min_angle, max_angle

    def validate_and_convert(self, srv_id, angle):
        """验证角度并转换为PWM"""
        if srv_id not in SERVO_CONFIGS:
            # 未知ID，使用默认范围
            pwm = int(500 + (angle / 270.0) * 2000)
            if not (500 <= pwm <= 2500):
                print(f"⚠️ 警告: 舵机 {srv_id} 角度 {angle}° 超出物理极限 (0°-270°)！指令被拦截。")
                return None
            return pwm

        config = SERVO_CONFIGS[srv_id]
        pwm = int(500 + (angle / 270.0) * 2000)
        
        if pwm < config["min_pwm"] or pwm > config["max_pwm"]:
            min_angle, max_angle = self.get_angle_range(config)
            print(f"❌ 错误: 舵机 [{srv_id}] ({config['name']}) 的目标角度 {angle}° 超出安全范围!")
            print(f"   👉 安全角度范围: {min_angle:.1f}° ~ {max_angle:.1f}°")
            print(f"   👉 对应 PWM 范围: {config['min_pwm']} ~ {config['max_pwm']}")
            return None
            
        return pwm

    def show_help(self):
        print("\n" + "🤖 机械臂总线舵机控制台 " + "="*30)
        print("【关节安全约束与状态映射】")
        for sid, cfg in SERVO_CONFIGS.items():
            min_a, max_a = self.get_angle_range(cfg)
            print(f"  ID {sid} ({cfg['name']:<6}):")
            print(f"      ├─ 最小: {min_a:>5.1f}° (PWM: {cfg['min_pwm']:<4}) -> 状态: [{cfg['min_desc']}]")
            print(f"      └─ 最大: {max_a:>5.1f}° (PWM: {cfg['max_pwm']:<4}) -> 状态: [{cfg['max_desc']}]")
        print("-" * 55)
        print("【指令输入格式】")
        print("  1. 运动指令: (ID, 角度, 时间) - 支持多舵机联动")
        print("     示例 1: (0, 90, 1000)               -> 0号舵机1秒内转到90°")
        print("     示例 2: (0, 90, 1000; 1, 120, 1000) -> 0号和1号舵机同时运动")
        print("  2. 原生指令: 支持 # 或 { 开头的底层指令 (如 #000P1500T1000!)")
        print("  3. 快捷命令:")
        print("     [h] 帮助信息    [c] 单片机连接测试")
        print("     [o] 回到启动位  [i] 广播读取ID")
        print("     [k] 释放扭力    [r] 恢复扭力")
        print("     [p] 读取位置    [v] 读取温压")
        print("     [q] 退出程序")
        print("=" * 43)

    def input_loop(self):
        """用户输入线程"""
        self.show_help()

        while self.running:
            try:
                line = input(">> ").strip()
                if not line:
                    continue
                
                # 统一转小写处理快捷命令
                cmd_lower = line.lower()
                
                if cmd_lower in ['q', '0', 'quit', 'exit']:
                    print("👋 退出程序...")
                    self.running = False
                    break
                elif cmd_lower == 'h':
                    self.show_help()
                elif cmd_lower == 'c':
                    self.send_cmd("$DRS!")
                    print("📡 正在测试单片机连接...")
                elif cmd_lower == 'o':
                    # 为了动作平滑，统一添加 1000ms 的执行时间，并组合成一条总线联动指令
                    self.send_cmd("{#000P1496T1000!#001P1689T1000!#002P1470T1000!#003P1012T1000!}")
                    print("🏠 机械臂正在回到启动位...")
                elif cmd_lower == 'i':
                    self.send_cmd("#255PID!")
                elif cmd_lower == 'k':
                    self.send_cmd("#255PULK!")
                    print("⚠️ 已释放所有舵机扭力，现在可以手动掰动机械臂。")
                elif cmd_lower == 'r':
                    self.send_cmd("#255PULR!")
                    print("🔒 已恢复所有舵机扭力。")
                elif cmd_lower == 'p':
                    self.send_cmd("#255PRAD!")
                elif cmd_lower == 'v':
                    self.send_cmd("#255PRTV!")
                elif line.startswith('(') or line.startswith('（'):
                    self.parse_and_send_coordinates(line)
                elif line.startswith('#') or line.startswith('{'):
                    self.send_cmd(line)
                else:
                    print("❓ 未知指令！输入 'h' 查看帮助。")
            except Exception as e:
                print(f"❌ 输入处理错误: {e}")

    def parse_and_send_coordinates(self, input_str):
        """解析并发送格式化坐标指令"""
        # 统一替换中文符号
        input_str = input_str.replace('（', '(').replace('）', ')').replace('；', ';').replace('，', ',')
        
        content = input_str.strip('();') 
        if not content:
            return

        groups = content.split(';')
        cmds = []
        seen_ids = set()
        
        for group in groups:
            group = group.strip()
            if not group:
                continue
            
            parts = group.split(',')
            if len(parts) != 3:
                print(f"❌ 格式错误: '{group}' -> 应为 'ID,角度,时间'")
                return # 有一个错误就整体取消，保证安全
            
            try:
                srv_id = int(parts[0].strip())
                angle = float(parts[1].strip())
                time_ms = int(parts[2].strip())
                
                if srv_id in seen_ids:
                    print(f"⚠️ 警告: 发现重复的舵机ID {srv_id}，指令取消。")
                    return
                seen_ids.add(srv_id)
                
                pwm = self.validate_and_convert(srv_id, angle)
                if pwm is None:
                    # 验证失败，取消整个动作组的发送
                    print("🚫 动作组包含不安全指令，已全部拦截！")
                    return
                
                # 生成单条指令格式
                cmd = f"#{srv_id:03d}P{pwm:04d}T{time_ms:04d}!"
                cmds.append(cmd)
                
            except ValueError:
                print(f"❌ 数值错误: '{group}'，请确保ID/角度/时间为数字。")
                return

        if not cmds:
            return

        # 发送指令组合
        if len(cmds) == 1:
            self.send_cmd(cmds[0])
        else:
            full_cmd = "{" + "".join(cmds) + "}"
            self.send_cmd(full_cmd)

def main():
    controller = BusServoController()
    try:
        while controller.running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n👋 强制退出程序...")
    finally:
        controller.running = False
        if controller.ser and controller.ser.is_open:
            controller.ser.close()

if __name__ == '__main__':
    main()