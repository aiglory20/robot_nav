import time
import threading

class ActionGroupManager:
    def __init__(self, controller):
        """
        初始化动作组管理器
        :param controller: 必须提供 send_cmd(cmd) 方法的控制器对象
        """
        self.controller = controller
        
        # 使用序列号机制代替单一布尔值，每次发生新动作或急停，序列号+1
        # 所有正在运行的线程发现自己的 my_seq != 全局 action_seq 时，自动安全退出
        self.action_seq = 0  
        self.action_lock = threading.Lock()

    def send(self, cmd):
        """发送串口指令"""
        if self.controller and hasattr(self.controller, 'send_cmd'):
            self.controller.send_cmd(cmd)

    def emergency_stop(self):
        """通用急停"""
        self.action_seq += 1  # 核心：任何新急停都会立即使旧任务失效
        
        self.send("#255PDST!")
        print("🛑 [动作组] 已发送急停指令")
        
        # 急停后固定等待 0.2s
        time.sleep(0.2)

    def action_reset(self):
        """动作0 - 复位 (回到启动位)"""
        self.action_seq += 1  # 打断之前所有的任务
        
        def task():
            with self.action_lock:
                self.emergency_stop()
                my_seq = self.action_seq  # 记录属于当前复位任务的专属序列号
                
                print("🔄 [动作组] 准备执行复位动作...")
                self.send("{#000P1496T2000!#001P1689T2000!#002P1470T2000!#003P1012T2000!}")
                print("✅ [动作组] 复位动作指令已发送")

        threading.Thread(target=task, daemon=True).start()

    def start_loop(self, loop_func, my_seq):
        """启动一个新的循环动作线程"""
        threading.Thread(target=loop_func, args=(my_seq,), daemon=True).start()

    # ==========================
    # 小循环动作定义
    # ==========================
    def loop_peace_sign(self, my_seq):
        """小动作组1 - peace sign"""
        while self.action_seq == my_seq:
            self.send("#003P0924T0500!")
            time.sleep(1.0)
            if self.action_seq != my_seq: break
            
            self.send("#003P1612T0500!")
            time.sleep(1.0)

    def loop_shake_hand(self, my_seq):
        """小动作组2 - shake hand"""
        while self.action_seq == my_seq:
            self.send("#002P2025T0500!")
            time.sleep(1.0)
            if self.action_seq != my_seq: break
            
            self.send("#002P1757T0500!")
            time.sleep(1.0)

    # ==========================
    # 主动作定义
    # ==========================
    def action_1_indicate(self):
        """动作1 - 指示"""
        self.action_seq += 1
        
        def task():
            with self.action_lock:
                self.emergency_stop()
                my_seq = self.action_seq  # 获取急停后更新的专属序列号
                
                print("👉 [动作组] 开始执行动作1 (指示)...")
                self.send("#000P2031T0600!")
                time.sleep(0.6)
                if self.action_seq != my_seq: return
                
                self.send("#001P1324T0600!")
                time.sleep(0.6)
                if self.action_seq != my_seq: return
                
                self.send("#002P2025T0500!")
                time.sleep(0.5)
                if self.action_seq != my_seq: return
                
                # 开始循环小动作
                self.start_loop(self.loop_peace_sign, my_seq)

        threading.Thread(target=task, daemon=True).start()

    def action_2_wave(self):
        """动作2 - 挥手"""
        self.action_seq += 1
        
        def task():
            with self.action_lock:
                self.emergency_stop()
                my_seq = self.action_seq
                
                print("👋 [动作组] 开始执行动作2 (挥手)...")
                self.send("#000P2031T0600!")
                time.sleep(0.6)
                if self.action_seq != my_seq: return
                
                self.send("#001P1324T0600!")
                time.sleep(0.6)
                if self.action_seq != my_seq: return
                
                self.send("#003P1012T0600!")
                time.sleep(0.6)
                if self.action_seq != my_seq: return
                
                # 开始循环小动作
                self.start_loop(self.loop_shake_hand, my_seq)

        threading.Thread(target=task, daemon=True).start()

    def action_3_yeah(self):
        """动作3 - 比个耶"""
        self.action_seq += 1
        
        def task():
            with self.action_lock:
                self.emergency_stop()
                my_seq = self.action_seq
                
                print("✌️ [动作组] 开始执行动作3 (比个耶)...")
                self.send("#000P1495T0500!")
                time.sleep(0.5)
                if self.action_seq != my_seq: return
                
                self.send("#001P1683T0500!")
                time.sleep(0.5)
                if self.action_seq != my_seq: return
                
                self.send("#002P2060T0500!")
                time.sleep(0.5)
                if self.action_seq != my_seq: return
                
                # 开始循环小动作
                self.start_loop(self.loop_peace_sign, my_seq)

        threading.Thread(target=task, daemon=True).start()