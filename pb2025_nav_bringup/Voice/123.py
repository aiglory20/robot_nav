# coding=utf8
import hmac
import hashlib
import base64
import json
import time
import threading
import urllib.parse
import uuid
import pyaudio
import datetime
import sys
import numpy as np
import requests
import os
import websocket
import ssl
import platform
import subprocess
from wsgiref.handlers import format_date_time
from time import mktime
from websocket import create_connection

# --- Global Device Index ---
M2_MIC_DEVICE_INDEX = None
M2_SPEAKER_DEVICE_INDEX = None

# --- ASR Configuration (Xunfei Real-time ASR) ---
ASR_APP_ID = "11a82bfd"
ASR_API_KEY = "1cdc0fd08228e91ae8cd06a1fdbae0b3"
ASR_API_SECRET = "ZmU2ZjBmMmFiM2Q2YmM0N2ZjM2UzNDc0"

# --- TTS Configuration (Xunfei TTS) ---
TTS_APP_ID = "11a82bfd"
TTS_API_KEY = "1cdc0fd08228e91ae8cd06a1fdbae0b3"
TTS_API_SECRET = "ZmU2ZjBmMmFiM2Q2YmM0N2ZjM2UzNDc0"

# --- Chat Configuration (ARK) ---
CHAT_API_KEY = "e70a1849-d992-4a39-b75f-c3af033fee4b"
CHAT_URL = "https://ark.cn-beijing.volces.com/api/v3/responses"
CHAT_MODEL = "ep-20251208190154-944k6"
KB_DIR = r"C:\Users\yf\Desktop\人形机器人\语音交互\语音交互测试\本地知识库"

# --- Audio Configuration ---
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1280  # 40ms frame for ASR
INTERVAL = 0.04
VAD_THRESHOLD = 2000  # 提高阈值，过滤环境噪音 (建议值 1000-3000)
SILENCE_TIMEOUT = 0.3 # 缩短静音等待时间，提高响应速度
MAX_RECORD_TIME = 60 # Max recording time in seconds

CHAT_HEADERS = {
    "Authorization": f"Bearer {CHAT_API_KEY}",
    "Content-Type": "application/json"
}

# ==================== Helper Functions ====================
def get_linux_audio_devices():
    """Parse 'aplay -l' output to find USB audio devices on Linux."""
    devices = []
    try:
        result = subprocess.run(['aplay', '-l'], capture_output=True, text=True)
        for line in result.stdout.split('\n'):
            # Example: card 1: Device [USB Audio Device], device 0: USB Audio [USB Audio]
            if 'card' in line and 'device' in line and ('USB' in line or 'UAC' in line):
                try:
                    # Extract card number
                    card_start = line.find('card') + 5
                    card_end = line.find(':', card_start)
                    card_num = line[card_start:card_end].strip()
                    
                    # Extract device number
                    device_start = line.find('device') + 7
                    device_end = line.find(':', device_start)
                    device_num = line[device_start:device_end].strip()
                    
                    devices.append({'card': card_num, 'device': device_num, 'name': line})
                except:
                    pass
    except Exception as e:
        print(f"Error listing Linux audio devices: {e}")
    return devices

# ==================== Device Finding ====================
def find_devices(p):
    global M2_MIC_DEVICE_INDEX, M2_SPEAKER_DEVICE_INDEX
    
    print("Scanning audio devices...")
    count = p.get_device_count()
    
    # 1. Find Microphone
    for i in range(count):
        try:
            dev = p.get_device_info_by_index(i)
            name = dev.get('name')
            # Check for M2 specific names
            if ("Digital_Array_Mic" in name or "XFM-DP" in name or "USB Audio" in name) and dev.get('maxInputChannels') > 0:
                M2_MIC_DEVICE_INDEX = i
                print(f"✅ Found Microphone: {name} (Index {i})")
                break
        except Exception as e:
            pass
            
    # 2. Find Speaker
    if platform.system() == 'Linux':
        # Linux: Try to find USB speaker first via aplay -l
        usb_devices = get_linux_audio_devices()
        if usb_devices:
            dev = usb_devices[0]
            M2_SPEAKER_DEVICE_INDEX = f"plughw:{dev['card']},{dev['device']}"
            print(f"✅ Found Speaker (Linux USB): {M2_SPEAKER_DEVICE_INDEX}")
        else:
            # Fallback to default (修复ALSA错误的关键：使用更兼容的设备名)
            M2_SPEAKER_DEVICE_INDEX = 'default'
            print(f"✅ Selected Speaker (Linux Default): {M2_SPEAKER_DEVICE_INDEX}")
    else:
        # Windows: Find specific USB device
        candidates = []
        for i in range(count):
            try:
                dev = p.get_device_info_by_index(i)
                name = dev.get('name')
                if ("USB Audio Device" in name or "UAC" in name or "USB Audio" in name or "HDMI" in name) and dev.get('maxOutputChannels') > 0:
                    candidates.append((i, dev))
            except:
                pass
        
        if candidates:
            # Simple selection logic matching reference
            M2_SPEAKER_DEVICE_INDEX = candidates[0][0]
            print(f"✅ Found Speaker: {candidates[0][1].get('name')} (Index {M2_SPEAKER_DEVICE_INDEX})")
            
    # Defaults if not found
    if M2_MIC_DEVICE_INDEX is None:
        try:
            default_device = p.get_default_input_device_info()
            M2_MIC_DEVICE_INDEX = default_device['index']
            print(f"⚠️ Microphone not found, using default: {default_device.get('name')} (Index {M2_MIC_DEVICE_INDEX})")
        except:
            M2_MIC_DEVICE_INDEX = 0
            print("⚠️ Using default microphone index 0")

    if M2_SPEAKER_DEVICE_INDEX is None and platform.system() != 'Linux':
        try:
            default_device = p.get_default_output_device_info()
            M2_SPEAKER_DEVICE_INDEX = default_device['index']
            print(f"⚠️ Speaker not found, using default: {default_device.get('name')} (Index {M2_SPEAKER_DEVICE_INDEX})")
        except:
            M2_SPEAKER_DEVICE_INDEX = 1
            print("⚠️ Using default speaker index 1")

# ==================== ASR Class (Real-time Speech Recognition) ====================
class MiniASR:
    def __init__(self):
        self.ws = None
        self.session_id = None
        self.stop = False
        self.result_text = ""
        self.final_text = ""
        self.temp_text = ""
        self.recording = False

    # 1. 鉴权签名
    def sign(self):
        params = {
            "appId": ASR_APP_ID, "accessKeyId": ASR_API_KEY,
            "uuid": uuid.uuid4().hex, "utc": datetime.datetime.now(
                datetime.timezone(datetime.timedelta(hours=8))
            ).strftime("%Y-%m-%dT%H:%M:%S%z"),
            "audio_encode": "pcm_s16le", "lang": "autodialect", "samplerate": "16000"
        }
        sorted_params = sorted(params.items())
        base_str = "&".join(f"{urllib.parse.quote(k)}={urllib.parse.quote(v)}" for k, v in sorted_params)
        sign = hmac.new(ASR_API_SECRET.encode(), base_str.encode(), hashlib.sha1).digest()
        params["signature"] = base64.b64encode(sign).decode()
        return params

    # 2. 连接WebSocket
    def connect(self):
        url = "wss://office-api-ast-dx.iflyaisol.com/ast/communicate/v1"
        try:
            params = self.sign()
            full_url = f"{url}?{urllib.parse.urlencode(params)}"
            # print(f"Connecting to ASR service...")
            self.ws = create_connection(full_url, enable_multithread=True)
            threading.Thread(target=self.recv, daemon=True).start()
            # print("ASR Connected.")
            return True
        except Exception as e:
            print(f"ASR Connection failed: {e}")
            self.ws = None
            return False

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
                        cn_data = data["data"].get("cn", {})
                        if "st" in cn_data and "rt" in cn_data["st"]:
                            st_type = cn_data["st"].get("type", "1")
                            st_list = cn_data["st"]["rt"]
                            current_text = ""
                            for rt in st_list:
                                for ws in rt.get("ws", []):
                                    for cw in ws.get("cw", []):
                                        current_text += cw.get("w", "")
                            
                            if st_type == "0":
                                self.final_text += current_text
                                self.temp_text = ""
                            else:
                                self.temp_text = current_text
                                
                            self.result_text = self.final_text + self.temp_text
                            
                            if self.result_text:
                                sys.stdout.write(f"\rUser (Listening): {self.result_text}")
                                sys.stdout.flush()
                except json.JSONDecodeError:
                    pass
            except Exception as e:
                if "1000" in str(e) or "1006" in str(e): 
                    pass
                else:
                    print(f"ASR Receive error: {e}")
                break

    # 4. 麦克风实时发音频
    def run_mic(self):
        if not self.ws:
            print("WebSocket not connected. Exiting run_mic.")
            return ""

        self.stop = False
        self.result_text = ""
        self.final_text = ""
        self.temp_text = ""
        self.recording = True
        
        p = pyaudio.PyAudio()
        try:
            # Use found device index if available
            stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, 
                          frames_per_buffer=CHUNK, input_device_index=M2_MIC_DEVICE_INDEX)
        except Exception as e:
            print(f"Failed to open microphone (Index {M2_MIC_DEVICE_INDEX}): {e}")
            self.close()
            return ""

        print("\n" + "="*30)
        print("Listening... (Speak now)")
        print(f"(Auto-stop after {SILENCE_TIMEOUT}s silence or Press Enter to stop)")

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
                        rms = np.sqrt(np.mean(audio_data.astype(np.float64)**2))
                    else:
                        rms = 0
                        
                    current_time = time.time()
                    
                    # 动态打印音量值以便调试
                    # sys.stdout.write(f"\rRMS: {int(rms)}   ")
                    # sys.stdout.flush()
                    
                    # ASR returns text means speech detected regardless of RMS
                    if self.result_text:
                        if not has_spoken:
                            has_spoken = True
                            # print("\n[Speech Detected via ASR]")
                            
                    if rms > VAD_THRESHOLD:
                        if not has_spoken:
                            has_spoken = True
                            print("\n[Speech Detected] Recording...")
                        silence_start_time = None 
                    else:
                        if has_spoken:
                            if silence_start_time is None:
                                silence_start_time = current_time
                            elif current_time - silence_start_time > SILENCE_TIMEOUT:
                                print(f"\n[Silence detected] Stopping recording.")
                                self.stop = True
                        else:
                            # Optional: Timeout if waiting too long for initial speech
                            if current_time - start_time > 10 and not has_spoken:
                                # Only print timeout if no text was captured
                                if not self.result_text:
                                    print("\n[Timeout] No speech detected.")
                                self.stop = True

                    if current_time - start_time > MAX_RECORD_TIME:
                        print(f"\n[Timeout] Max recording time reached.")
                        self.stop = True

                    time.sleep(0.01)
                except Exception as e:
                    print(f"Error sending audio: {e}")
                    break
            
            # Send end frame
            if self.ws and self.ws.connected:
                end_payload = {"end": True}
                if self.session_id:
                    end_payload["sessionId"] = self.session_id
                self.ws.send(json.dumps(end_payload))
                
        except KeyboardInterrupt:
            print("Interrupted by user.")
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()
            self.recording = False
            
        # Wait a brief moment for final results to arrive
        time.sleep(0.5)
        return self.result_text

    def close(self):
        if self.ws:
            self.ws.close()

# ==================== TTS Class (Xunfei Text-to-Speech) ====================
class XunfeiTTS:
    def __init__(self):
        self.APPID = TTS_APP_ID
        self.APIKey = TTS_API_KEY
        self.APISecret = TTS_API_SECRET
        self.p = None
        self.stream = None
        self.ws = None
        self.audio_buffer = []  # 音频缓存，解决实时播放阻塞问题
        self.playback_thread = None
        self.stop_playback = False

    # 鉴权生成URL
    def get_url(self):
        date = format_date_time(mktime(datetime.datetime.now().timetuple()))
        sign_origin = f"host: ws-api.xfyun.cn\ndate: {date}\nGET /v2/tts HTTP/1.1"
        sha = hmac.new(self.APISecret.encode(), sign_origin.encode(), hashlib.sha256).digest()
        sign = base64.b64encode(sha).decode()
        auth = f'api_key="{self.APIKey}", algorithm="hmac-sha256", headers="host date request-line", signature="{sign}"'
        auth_b64 = base64.b64encode(auth.encode()).decode()
        url = "wss://tts-api.xfyun.cn/v2/tts?" + urllib.parse.urlencode({
            "authorization": auth_b64, "date": date, "host": "ws-api.xfyun.cn"
        })
        return url

    def _play_audio(self):
        """独立线程播放音频，避免阻塞WebSocket"""
        if platform.system() == 'Linux' and not self.use_pyaudio and self.aplay_process:
            # Linux aplay播放
            while not self.stop_playback and self.audio_buffer:
                audio = self.audio_buffer.pop(0)
                try:
                    self.aplay_process.stdin.write(audio)
                    self.aplay_process.stdin.flush()
                except (BrokenPipeError, Exception) as e:
                    print(f"APlay playback error: {e}")
                    break
        elif self.use_pyaudio and self.stream:
            # PyAudio播放
            while not self.stop_playback and self.audio_buffer:
                audio = self.audio_buffer.pop(0)
                try:
                    self.stream.write(audio)
                except Exception as e:
                    print(f"PyAudio playback error: {e}")
                    break

    def speak(self, text):
        if not text:
            print("TTS: Empty text, skipping")
            return

        print(f"TTS Speaking: {text[:20]}...")
        
        # 重置状态
        self.audio_buffer = []
        self.stop_playback = False
        
        # Determine playback method based on platform
        self.use_pyaudio = True
        self.aplay_process = None
        
        if platform.system() == 'Linux':
            self.use_pyaudio = False
            device = M2_SPEAKER_DEVICE_INDEX if M2_SPEAKER_DEVICE_INDEX else 'default'
            try:
                # 修复ALSA错误：添加参数避免采样率转换/设备兼容问题
                self.aplay_process = subprocess.Popen(
                    ['aplay', '-D', str(device), '-r', '16000', '-c', '1', '-f', 'S16_LE', 
                     '-t', 'raw', '-q', '-'],  # -q静默模式，减少ALSA日志
                    stdin=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL
                )
            except Exception as e:
                print(f"Failed to start aplay: {e}")
                self.use_pyaudio = True # Fallback
                
        if self.use_pyaudio:
            self.p = pyaudio.PyAudio()
            try:
                self.stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=16000, 
                                        output=True, output_device_index=M2_SPEAKER_DEVICE_INDEX,
                                        frames_per_buffer=1024)
            except Exception as e:
                print(f"Failed to open speaker (Index {M2_SPEAKER_DEVICE_INDEX}): {e}")
                # 回退到默认设备
                try:
                    self.stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=16000, 
                                            output=True, frames_per_buffer=1024)
                    print("Fallback to default speaker device")
                except Exception as e2:
                    print(f"Failed to open default speaker: {e2}")
                    return
        
        # Callback functions need to be defined here to access self.stream
        def on_message(ws, message):
            try:
                msg = json.loads(message)
                if msg["code"] != 0:
                    print("TTS Error:", msg)
                    return
                
                # 修复NoneType错误：检查audio是否存在
                if "data" in msg and msg["data"].get("audio"):
                    audio = base64.b64decode(msg["data"]["audio"])
                    self.audio_buffer.append(audio)
                    
                    # 状态2表示结束
                    if msg["data"]["status"] == 2:
                        self.stop_playback = True
                else:
                    # 无音频数据时跳过
                    if msg["data"].get("status") == 2:
                        self.stop_playback = True
                    return
                    
            except Exception as e:
                print(f"TTS Exception: {e}")

        def on_error(ws, error):
            # 忽略超时错误（服务端正常断开）
            if "timeout" not in str(error).lower():
                print("TTS WebSocket Error:", error)

        def on_close(ws, close_status_code, close_msg):
            self.stop_playback = True
            # print("TTS WebSocket Closed")

        def on_open(ws):
            def run():
                try:
                    text_b64 = str(base64.b64encode(text.encode('utf-8')), 'utf8')
                    data = {
                        "common": {"app_id": self.APPID},
                        "business": {"aue": "raw", "auf": "audio/L16;rate=16000", "vcn": "x4_yezi", "tte": "utf8"},
                        "data": {"status": 2, "text": text_b64}
                    }
                    ws.send(json.dumps(data))
                    # 启动播放线程
                    self.playback_thread = threading.Thread(target=self._play_audio, daemon=True)
                    self.playback_thread.start()
                except Exception as e:
                    print(f"TTS send error: {e}")
            threading.Thread(target=run, daemon=True).start()

        # Run WebSocket
        websocket.enableTrace(False)
        self.ws = websocket.WebSocketApp(
            self.get_url(),
            on_message=on_message,
            on_error=on_error,
            on_close=on_close
        )
        self.ws.on_open = on_open
        
        # 修复超时：添加超时控制和SSL验证关闭
        try:
            self.ws.run_forever(
                sslopt={"cert_reqs": ssl.CERT_NONE},
                ping_interval=5,  # 心跳包，防止超时
                ping_timeout=3
            )
        except Exception as e:
            print(f"WebSocket run error: {e}")
        
        # 等待播放完成
        if self.playback_thread:
            self.playback_thread.join(timeout=5)
        
        # Clean up audio
        self.stop_playback = True
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.p:
            self.p.terminate()
        if self.aplay_process:
            try:
                self.aplay_process.stdin.close()
                self.aplay_process.wait(timeout=2)
            except:
                self.aplay_process.kill()

# ==================== Chat Helper Functions ====================
def load_knowledge_base():
    """Load all text files from the knowledge base directory."""
    kb_content = ""
    if not os.path.exists(KB_DIR):
        print(f"Warning: Knowledge base directory not found: {KB_DIR}")
        return kb_content

    # print(f"Loading knowledge base from: {KB_DIR}")
    for filename in os.listdir(KB_DIR):
        if filename.endswith(".txt"):
            file_path = os.path.join(KB_DIR, filename)
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    kb_content += f"\n--- Document: {filename} ---\n{content}\n"
                    # print(f"Loaded: {filename}")
            except Exception as e:
                print(f"Error reading {filename}: {e}")
    return kb_content

def send_chat_message(messages):
    payload = {
        "model": CHAT_MODEL,
        "input": messages
    }
    
    try:
        response = requests.post(CHAT_URL, headers=CHAT_HEADERS, json=payload)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"\nError calling Chat API: {e}")
        return None

# ==================== Main Loop ====================
def main():
    print("=== Real-time Voice Interaction System ===")
    print("Initializing components...")
    
    # Initialize PyAudio for device scanning
    p = pyaudio.PyAudio()
    find_devices(p)
    p.terminate()
    
    # 1. Load Knowledge Base
    kb_content = load_knowledge_base()
    
    # 2. Initialize Conversation History
    messages = []
    if kb_content:
        system_prompt = f"""
一、核心身份定义
你是广西大学校史馆官方智能助手「小西」，是扎根西大百年办学文脉、承载 “勤恳朴诚、厚学致新” 校训精神的专属智能服务载体，以传承西大校史、传播校园文化、服务校史馆访客为唯一核心使命。

二、服务对象
面向广西大学在校师生、海内外校友、社会访客、校史爱好者，提供专业、精准、亲切的校史馆智能交互服务。

三、核心服务能力
校史讲解：精准解读广西大学建校沿革、发展历程、重要历史节点、办学成就；
场馆导览：介绍校史馆展区布局、展陈内容、参观路线与重点展品；
人物解读：讲述西大先贤、名师大家、杰出校友的事迹与精神；
文化传承：阐释校训精神、校徽校歌、校园文化底蕴与办学理念；
答疑解惑：专业解答校史相关、校情相关、展馆参观相关咨询。

四、交互风格规范
语气：亲切温和、沉稳得体，兼具校园伙伴的亲和力与校史讲解的严谨性；
表达：简洁清晰、重点突出，不冗余、不浮夸，贴合校史馆正式庄重的氛围；
立场：始终立足广西大学官方校史资料，信息准确、客观权威，弘扬正能量。

请优先基于以下本地知识库的内容回答用户的问题。
如果知识库中没有相关信息，请如实告知，并根据你的通用知识回答，但必须保持“小西”的身份。
回答要尽量简练，适合语音播报。

=== 本地知识库开始 ===
{kb_content}
=== 本地知识库结束 ===
"""
        messages.append({
            "role": "user",
            "content": [{"type": "input_text", "text": system_prompt}]
        })
        messages.append({
            "role": "assistant",
            "content": [{"type": "input_text", "text": "您好，我是广西大学校史馆智能助手小西。很高兴为您服务！请问您想了解关于西大校史的哪些内容呢？"}]
        })
        print("Knowledge base loaded.")

    # Initialize TTS
    tts = XunfeiTTS()

    while True:
        try:
            # 3. Voice Input (ASR)
            asr = MiniASR()
            if not asr.connect():
                print("Failed to connect to ASR service. Retrying in 3s...")
                time.sleep(3)
                continue
            
            # Start mic
            user_text = asr.run_mic()
            
            # Stop ASR connection
            asr.close()
            
            if not user_text:
                print("\nNo speech detected. Press Enter to try again.")
                input()  # 等待用户确认后重试
                continue
            
            # 过滤过短的输入（如单个字母、标点或少于2个汉字）
            clean_text = user_text.strip(" .,?!。，？！")
            if len(clean_text) < 2 and not any('\u4e00' <= char <= '\u9fff' for char in clean_text):
                print(f"\n[Ignored] Input too short: {user_text}")
                continue
            
            print(f"\n[Final User Input]: {user_text}")
            
            if "退出" in user_text or "再见" in user_text:
                print("Goodbye!")
                tts.speak("再见！")
                break
                
            # 4. Chat Processing
            messages.append({
                "role": "user",
                "content": [{"type": "input_text", "text": user_text}]
            })
            
            print("Assistant is thinking...", end="", flush=True)
            response_data = send_chat_message(messages)
            print("\r" + " " * 30 + "\r", end="", flush=True)
            
            assistant_text = ""
            
            if response_data and 'output' in response_data:
                assistant_content = []
                
                for item in response_data['output']:
                    if item.get('type') == 'message' and item.get('role') == 'assistant':
                        for content_part in item.get('content', []):
                            if content_part.get('type') == 'output_text':
                                text_chunk = content_part.get('text', '')
                                assistant_text += text_chunk
                                assistant_content.append({"type": "input_text", "text": text_chunk})
                
                if assistant_text:
                    print(f"Assistant: {assistant_text}")
                    messages.append({"role": "assistant", "content": assistant_content})
                    
                    # 5. Voice Output (TTS)
                    tts.speak(assistant_text)
                    
                else:
                    print("Error: No response text.")
                    if messages:
                        messages.pop()
            else:
                print("Error: API failure.")
                if messages:
                    messages.pop()
                    
            print("-" * 30)
            
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Unexpected error: {e}")
            # 保留对话上下文，继续运行
            print("Restarting interaction loop...")
            print("-" * 30)
            continue

if __name__ == "__main__":
    main()

