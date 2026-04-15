import hmac, hashlib, base64, json, time, threading
import urllib.parse, uuid, pyaudio
from websocket import create_connection
import datetime
import sys
import numpy as np

# 【核心配置】
APP_ID = "11a82bfd"
ACCESS_KEY_ID = "1cdc0fd08228e91ae8cd06a1fdbae0b3"
ACCESS_KEY_SECRET = "ZmU2ZjBmMmFiM2Q2YmM0N2ZjM2UzNDc0"

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1280  # 40ms帧长
INTERVAL = 0.04
VAD_THRESHOLD = 500  # Voice Activity Detection threshold (adjust based on mic)
SILENCE_TIMEOUT = 0.5 # Seconds of silence to trigger stop
MAX_RECORD_TIME = 60 # Max recording time in seconds

class MiniASR:
    def __init__(self):
        self.ws = None
        self.session_id = None
        self.stop = False

    # 1. 鉴权签名
    def sign(self):
        params = {
            "appId": APP_ID, "accessKeyId": ACCESS_KEY_ID,
            "uuid": uuid.uuid4().hex, "utc": datetime.datetime.now(
                datetime.timezone(datetime.timedelta(hours=8))
            ).strftime("%Y-%m-%dT%H:%M:%S%z"),
            "audio_encode": "pcm_s16le", "lang": "autodialect", "samplerate": "16000"
        }
        sorted_params = sorted(params.items())
        base_str = "&".join(f"{urllib.parse.quote(k)}={urllib.parse.quote(v)}" for k, v in sorted_params)
        sign = hmac.new(ACCESS_KEY_SECRET.encode(), base_str.encode(), hashlib.sha1).digest()
        params["signature"] = base64.b64encode(sign).decode()
        return params

    # 2. 连接WebSocket
    def connect(self):
        url = "wss://office-api-ast-dx.iflyaisol.com/ast/communicate/v1"
        try:
            params = self.sign()
            full_url = f"{url}?{urllib.parse.urlencode(params)}"
            print(f"Connecting to: {url}")
            self.ws = create_connection(full_url, enable_multithread=True)
            threading.Thread(target=self.recv, daemon=True).start()
            print("WebSocket Connected.")
        except Exception as e:
            print(f"Connection failed: {e}")
            self.ws = None

    # 3. 接收识别结果
    def recv(self):
        while self.ws and self.ws.connected:
            try:
                msg = self.ws.recv()
                if not msg:
                    break
                try:
                    data = json.loads(msg)
                    if data.get("res_type") == "asr" and "data" in data:
                        # 检查数据结构是否存在
                        cn_data = data["data"].get("cn", {})
                        if "st" in cn_data and "rt" in cn_data["st"]:
                            st_list = cn_data["st"]["rt"]
                            text_result = ""
                            for rt in st_list:
                                for ws in rt.get("ws", []):
                                    for cw in ws.get("cw", []):
                                        text_result += cw.get("w", "")
                            
                            if text_result:
                                print(f"识别结果: {text_result}")
                except json.JSONDecodeError:
                    pass
            except Exception as e:
                # 忽略连接关闭的错误
                if "1000" in str(e) or "1006" in str(e): 
                    pass
                else:
                    print(f"Receive error: {e}")
                break

    # 4. 麦克风实时发音频
    def run_mic(self):
        if not self.ws:
            print("WebSocket not connected. Exiting run_mic.")
            return

        p = pyaudio.PyAudio()
        try:
            stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
        except Exception as e:
            print(f"Failed to open microphone: {e}")
            self.close()
            return

        print("\n=== 开始录音 ===")
        print(f"等待语音输入... (当前静音阈值: {VAD_THRESHOLD})")
        print("或者按回车键强制停止...")

        def wait_stop():
            try:
                input()
            except EOFError:
                pass
            self.stop = True
        
        input_thread = threading.Thread(target=wait_stop, daemon=True)
        input_thread.start()

        # VAD variables
        silence_start_time = None
        has_spoken = False
        start_time = time.time()
        
        try:
            while not self.stop:
                try:
                    data = stream.read(CHUNK, exception_on_overflow=False)
                    self.ws.send_binary(data)
                    
                    # VAD Logic
                    audio_data = np.frombuffer(data, dtype=np.int16)
                    if len(audio_data) > 0:
                        # Use simple mean of absolute values for faster "volume" approximation or keep RMS
                        # RMS is better
                        rms = np.sqrt(np.mean(audio_data.astype(np.float64)**2))
                    else:
                        rms = 0
                        
                    current_time = time.time()
                    
                    # Debug print (optional, can be commented out)
                    # print(f"RMS: {rms:.2f}", end="\r")

                    if rms > VAD_THRESHOLD:
                        if not has_spoken:
                            print(f"\n[检测到语音输入] (RMS: {rms:.0f})")
                            has_spoken = True
                        silence_start_time = None 
                    else:
                        if has_spoken:
                            if silence_start_time is None:
                                silence_start_time = current_time
                            elif current_time - silence_start_time > SILENCE_TIMEOUT:
                                print(f"\n[检测到 {SILENCE_TIMEOUT}s 静音] 自动停止录音。")
                                self.stop = True
                        else:
                            # Optional: Timeout if waiting too long for initial speech
                            if current_time - start_time > 10 and not has_spoken:
                                print("\n[超时] 10秒内未检测到语音，自动退出。")
                                self.stop = True

                    if current_time - start_time > MAX_RECORD_TIME:
                        print(f"\n[超时] 达到最大录音时长 {MAX_RECORD_TIME}s。")
                        self.stop = True

                    time.sleep(0.01) # Reduce sleep to process audio faster
                except Exception as e:
                    print(f"Error sending audio: {e}")
                    break
            
            # 发送结束帧
            if self.ws and self.ws.connected:
                end_payload = {"end": True}
                if self.session_id:
                    end_payload["sessionId"] = self.session_id
                self.ws.send(json.dumps(end_payload))
                print("发送结束信号...")
                
        except KeyboardInterrupt:
            print("Interrupted by user.")
        finally:
            print("停止音频流...")
            stream.stop_stream()
            stream.close()
            p.terminate()

    def close(self):
        if self.ws:
            self.ws.close()
            print("WebSocket Closed.")

if __name__ == "__main__":
    asr = MiniASR()
    asr.connect()
    if asr.ws:
        asr.run_mic()
        time.sleep(1) # Wait a bit for final results
        asr.close()
    else:
        print("Failed to initialize ASR.")
