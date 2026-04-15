import sys
import websocket
import json
import time
import uuid
import threading
import numpy as np
import sounddevice as sd
import requests
import serial
import queue
import platform
import glob
# 移除了大量不必要的科学计算库
from openai import OpenAI
import httpx
import subprocess
import os
import wave
import tempfile
import ssl
from collections import deque
import pyaudio
import datetime
import urllib.parse
import hmac
import hashlib
import base64
from enum import Enum, auto

# 文档处理相关库（延迟导入）
# import pdfplumber
# import xlrd
# import docx

# 导入树莓派表情指令发送函数
sys.path.append('/home/jetson')
from nano_client import send_expression

# ===================== 机器人状态定义 =====================
class RobotState(Enum):
    IDLE = auto()          # 空闲 / 等待人声 
    RECORDING = auto()     # 正在录音 
    ASR = auto()           # 语音识别中 
    THINKING = auto()      # AI处理中 
    SPEAKING = auto()      # TTS播放中 
    ERROR = auto()         # 异常

# ===================== 基础配置 =====================
APP_ID = '121532777'
API_KEY = 'gjhcH6Jjdf7LQ1l21gJR4JOi'
SECRET_KEY = 'rselmAwOQyt7EQ4OsfYJNCMYnrT3Iw2W'
DEV_PID = 15372

# 树莓派配置
RASPBERRY_IP = "192.168.85.155"
RASPBERRY_PORT = 8888

# 本地文档目录
LOCAL_DOCS_DIR = "C:\\Users\\yf\\Desktop\\人形机器人\\语音交互\\本地文档"
PDF_DOCS_DIR = "C:\\Users\\yf\\Desktop\\人形机器人\\语音交互"

# 全局文档内容缓存
local_docs_content = {}
local_pdfs_content = {}

# 情绪关键词映射
EMOTION_KEYWORDS = {
    "愤怒": ["愤怒", "生气", "发火", "恼火", "恼怒", "气愤", "暴躁", "发怒"],
    "兴奋": ["兴奋", "开心", "高兴", "快乐", "愉快", "喜悦", "雀跃", "欣喜", "欢喜"],
    "惊恐": ["惊恐", "害怕", "恐惧", "惊慌", "慌张", "惊恐", "畏惧", "胆怯"],
    "难过": ["难过", "伤心", "悲伤", "悲哀", "沮丧", "低落", "痛心", "忧伤"],
    "不屑": ["不屑", "轻视", "蔑视", "看不起", "小瞧", "鄙夷", "藐视"],
    "左看": ["左看", "看左边", "向左看", "望向左边", "转头向左"],
    "右看": ["右看", "看右边", "向右看", "望向右边", "转头向右"],
    "眨眼": ["眨眼", "眨眼睛", "眨一下眼", "眨眨眼"],
    "慢眨眼": ["慢眨眼", "慢慢眨眼", "温柔眨眼"]
}

# 创建一个不使用代理的httpx客户端
client = httpx.Client(trust_env=False, transport=httpx.HTTPTransport(local_address='0.0.0.0'))

AI_CLIENT = OpenAI(
    base_url='https://qianfan.baidubce.com/v2',
    api_key='bce-v3/ALTAK-mdpXzVbkWI2cigoGYtUzL/3e82971f57cd97f82b402788ea60d0981185fdf4',
    http_client=client
)
AI_MODEL = "ernie-4.5-turbo-128k"

# ===================== 音频参数 =====================
CHUNK = 1024  # 减小缓冲区，提高响应速度
CHANNELS = 1
RATE = 16000
SPEAKER_RATE = 24000
MAX_RECORD_SECONDS = 30
SILENCE_DURATION = 0.8  # 略微缩短静音等待，提高响应速度 (1.0 -> 0.8)
MIN_VOICE_DURATION = 0.2  # 降低最小语音时长，避免短指令被丢弃

# 动态查找设备索引
M2_MIC_DEVICE_INDEX = None
M2_SPEAKER_DEVICE_INDEX = None

# 全局状态
is_ai_speaking = False
chat_history = []
audio_queue = queue.Queue()

# ===================== 极简高效 VAD =====================
class SimpleVAD:
    """极简版人声检测，专注于速度和低延迟"""
    
    def __init__(self, sample_rate=16000):
        self.sample_rate = sample_rate
        self.energy_threshold = 0
        self.noise_level = 0
        self.adaptive_factor = 0.05
        
    def update_noise(self, audio_chunk):
        """快速更新噪音基线"""
        energy = np.sqrt(np.mean(np.square(audio_chunk.astype(np.float32))))
        if self.noise_level == 0:
            self.noise_level = energy
        else:
            self.noise_level = (1 - self.adaptive_factor) * self.noise_level + self.adaptive_factor * energy
            
    def is_voice(self, audio_chunk, sensitivity=1.5):
        """基于能量和过零率的快速检测"""
        if len(audio_chunk) < 256:
            return False
            
        # 1. 能量检测 (最快)
        energy = np.sqrt(np.mean(np.square(audio_chunk.astype(np.float32))))
        if energy < self.noise_level * sensitivity:
            self.update_noise(audio_chunk) # 是噪音，更新基线
            return False
            
        # 2. 过零率检测 (辅助排除高频噪音)
        # 转换为float计算更准，但为了速度，直接用符号位变化
        zcr = np.sum(np.abs(np.diff(np.sign(audio_chunk)))) / (2 * len(audio_chunk))
        
        # 人声通常 ZCR 在 0.02 到 0.4 之间
        if 0.02 < zcr < 0.45:
            return True
            
        return False

# 初始化 VAD
simple_vad = SimpleVAD(sample_rate=RATE)

# ===================== 文档处理函数 =====================
def load_pdf_document(file_path):
    """加载PDF文档并提取内容"""
    try:
        with pdfplumber.open(file_path) as pdf:
            content = []
            for page in pdf.pages:
                text = page.extract_text()
                if text:
                    content.append(text)
            return "\n".join(content)
    except Exception as e:
        return f"Error loading PDF: {str(e)}"

def load_excel_document(file_path):
    """加载Excel文档并提取内容"""
    try:
        workbook = xlrd.open_workbook(file_path)
        content = []
        for sheet_name in workbook.sheet_names():
            sheet = workbook.sheet_by_name(sheet_name)
            sheet_content = [f"Sheet: {sheet_name}"]
            for row in range(sheet.nrows):
                row_data = []
                for col in range(sheet.ncols):
                    cell_value = sheet.cell_value(row, col)
                    row_data.append(str(cell_value))
                sheet_content.append("\t".join(row_data))
            content.append("\n".join(sheet_content))
        return "\n\n".join(content)
    except Exception as e:
        return f"Error loading Excel: {str(e)}"

def load_word_document(file_path):
    """加载Word文档并提取内容"""
    try:
        doc = docx.Document(file_path)
        content = []
        for paragraph in doc.paragraphs:
            if paragraph.text.strip():
                content.append(paragraph.text)
        return "\n".join(content)
    except Exception as e:
        return f"Error loading Word: {str(e)}"

def load_local_documents():
    """加载所有本地文档"""
    global local_docs_content, local_pdfs_content
    
    # 加载本地文档文件夹中的文档
    if os.path.exists(LOCAL_DOCS_DIR):
        for filename in os.listdir(LOCAL_DOCS_DIR):
            file_path = os.path.join(LOCAL_DOCS_DIR, filename)
            if os.path.isfile(file_path):
                if filename.endswith('.pdf'):
                    local_docs_content[filename] = load_pdf_document(file_path)
                elif filename.endswith('.xls') or filename.endswith('.xlsx'):
                    local_docs_content[filename] = load_excel_document(file_path)
                elif filename.endswith('.doc') or filename.endswith('.docx'):
                    local_docs_content[filename] = load_word_document(file_path)
    
    # 加载根目录中的PDF文档
    if os.path.exists(PDF_DOCS_DIR):
        for filename in os.listdir(PDF_DOCS_DIR):
            if filename.endswith('.pdf'):
                file_path = os.path.join(PDF_DOCS_DIR, filename)
                if os.path.isfile(file_path):
                    local_pdfs_content[filename] = load_pdf_document(file_path)
    
    print(f"✅ 已加载 {len(local_docs_content)} 个本地文档")
    print(f"✅ 已加载 {len(local_pdfs_content)} 个PDF文档")

def search_documents(query):
    """搜索文档内容，返回相关结果（按需加载）"""
    global local_docs_content, local_pdfs_content
    
    # 延迟导入文档处理库
    import pdfplumber
    import xlrd
    import docx
    
    relevant_results = []
    query_lower = query.lower()
    
    # 提取查询中的关键词
    import re
    keywords = re.findall(r'[\u4e00-\u9fa5a-zA-Z0-9]+', query_lower)
    keywords = [kw for kw in keywords if len(kw) >= 2]  # 过滤短词
    
    if not keywords:
        return []
    
    # 按需加载本地文档
    if not local_docs_content and os.path.exists(LOCAL_DOCS_DIR):
        for filename in os.listdir(LOCAL_DOCS_DIR):
            file_path = os.path.join(LOCAL_DOCS_DIR, filename)
            if os.path.isfile(file_path):
                try:
                    if filename.endswith('.pdf'):
                        local_docs_content[filename] = load_pdf_document(file_path)
                    elif filename.endswith('.xls') or filename.endswith('.xlsx'):
                        local_docs_content[filename] = load_excel_document(file_path)
                    elif filename.endswith('.doc') or filename.endswith('.docx'):
                        local_docs_content[filename] = load_word_document(file_path)
                except Exception:
                    pass
    
    # 按需加载PDF文档
    if not local_pdfs_content and os.path.exists(PDF_DOCS_DIR):
        for filename in os.listdir(PDF_DOCS_DIR):
            if filename.endswith('.pdf'):
                file_path = os.path.join(PDF_DOCS_DIR, filename)
                if os.path.isfile(file_path):
                    try:
                        local_pdfs_content[filename] = load_pdf_document(file_path)
                    except Exception:
                        pass
    
    # 搜索本地文档
    for filename, content in local_docs_content.items():
        # 过滤错误的文档内容
        if 'Error loading' in content or 'Package not found' in content:
            continue
            
        content_lower = content.lower()
        
        # 检查是否包含任何关键词
        if any(keyword in content_lower for keyword in keywords):
            # 提取相关片段
            lines = content.split('\n')
            for i, line in enumerate(lines):
                line_lower = line.lower()
                # 检查该行是否包含任何关键词
                if any(keyword in line_lower for keyword in keywords):
                    start = max(0, i-2)
                    end = min(len(lines), i+3)
                    snippet = '\n'.join(lines[start:end])
                    
                    # 检查是否重复
                    is_duplicate = False
                    for result in relevant_results:
                        if result['content'] == snippet:
                            is_duplicate = True
                            break
                    
                    if not is_duplicate:
                        relevant_results.append({
                            'source': f"本地文档/{filename}",
                            'content': snippet
                        })
                        if len(relevant_results) >= 3:
                            break
        if len(relevant_results) >= 3:
            break
    
    # 搜索PDF文档
    if len(relevant_results) < 3:
        for filename, content in local_pdfs_content.items():
            content_lower = content.lower()
            
            # 检查是否包含任何关键词
            if any(keyword in content_lower for keyword in keywords):
                # 提取相关片段
                lines = content.split('\n')
                for i, line in enumerate(lines):
                    line_lower = line.lower()
                    # 检查该行是否包含任何关键词
                    if any(keyword in line_lower for keyword in keywords):
                        start = max(0, i-2)
                        end = min(len(lines), i+3)
                        snippet = '\n'.join(lines[start:end])
                        
                        # 检查是否重复
                        is_duplicate = False
                        for result in relevant_results:
                            if result['content'] == snippet:
                                is_duplicate = True
                                break
                        
                        if not is_duplicate:
                            relevant_results.append({
                                'source': f"PDF文档/{filename}",
                                'content': snippet
                            })
                            if len(relevant_results) >= 3:
                                break
            if len(relevant_results) >= 3:
                break
    
    return relevant_results

# ===================== 情绪解析函数 =====================
def analyze_emotion(text):
    """解析文本中的情绪关键词"""
    if not text or not isinstance(text, str):
        return "静态"
    
    text_lower = text.lower()
    for expression, keywords in EMOTION_KEYWORDS.items():
        for keyword in keywords:
            if keyword in text_lower:
                return expression
    return "静态"

def send_emotion_to_raspberry(text):
    """解析情绪并发送到树莓派"""
    def async_send():
        try:
            emotion = analyze_emotion(text)
            # 快速发送表情指令，不打印信息
            send_expression(emotion, RASPBERRY_IP, RASPBERRY_PORT)
        except:
            pass
    
    threading.Thread(target=async_send, daemon=True).start()

# ===================== M2 控制器 =====================
class MicArrayController:
    def __init__(self, port, baudrate):
        self.ser = None
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.5)
        except:
            pass

    def Msg_Packet(self, Content):
        Sync_Head = 0xA5
        User_ID = 0x01
        Msg_Type = 0x05
        MsgId_L = 0x01
        MsgId_H = 0x00
        MsgLen_Byte = len(Content).to_bytes(2, 'big')
        Msg_Len_L = MsgLen_Byte[1]
        Msg_Len_H = MsgLen_Byte[0]
        CheckCode = ((~sum([Sync_Head, User_ID, Msg_Type, Msg_Len_L, Msg_Len_H, MsgId_L, MsgId_H] + list(Content))) & 0xFF) + 1
        Msg = bytes([Sync_Head, User_ID, Msg_Type, Msg_Len_L, Msg_Len_H, MsgId_L, MsgId_H]) + Content + bytes([CheckCode])
        return Msg

    def mic_circle_set(self):
        if self.ser and self.ser.isOpen():
            cmd = '{"type":"switch_mic","content":{"mic":"mic6_circle"}}'
            self.ser.write(self.Msg_Packet(cmd.encode()))
            print("✅ 已发送环形六麦设置指令")
    
    def set_circle_mic_beam(self):
        """新增：环形六麦动态波束配置，适配360°远距离拾音"""
        if self.ser and self.ser.isOpen():
            cmd = '{"type":"manual_wakeup","content":{"beam":-1}}'  # beam=-1启用动态波束
            self.ser.write(self.Msg_Packet(cmd.encode()))
            print("✅ 环形六麦动态波束启用，适配360°远距离拾音")

# ===================== 设备查找 =====================
def find_devices():
    global M2_MIC_DEVICE_INDEX, M2_SPEAKER_DEVICE_INDEX
    try:
        devices = sd.query_devices()
        
        # 快速查找麦克风
        for i, dev in enumerate(devices):
            if ("Digital_Array_Mic" in dev['name'] or "XFM-DP" in dev['name'] or "USB Audio" in dev['name']) and dev['max_input_channels'] > 0:
                M2_MIC_DEVICE_INDEX = i
                break
        
        # 快速查找扬声器
        if platform.system() == 'Linux':
            M2_SPEAKER_DEVICE_INDEX = 'plughw:0,0'
        else:
            candidates = []
            for i, dev in enumerate(devices):
                if ("USB Audio Device" in dev['name'] or "UAC" in dev['name'] or "USB Audio" in dev['name'] or "HDMI" in dev['name']) and dev['max_output_channels'] > 0:
                    candidates.append((i, dev))
                    
            if candidates:
                if platform.system() == 'Windows':
                    try:
                        wasapi = [c for c in candidates if "WASAPI" in sd.query_hostapis(c[1]['hostapi'])['name']]
                        M2_SPEAKER_DEVICE_INDEX = wasapi[0][0] if wasapi else candidates[0][0]
                    except:
                        M2_SPEAKER_DEVICE_INDEX = candidates[0][0]
                else:
                    M2_SPEAKER_DEVICE_INDEX = candidates[0][0]
        
        # 设置默认设备
        if M2_MIC_DEVICE_INDEX is None:
            try:
                M2_MIC_DEVICE_INDEX = sd.default.device[0]
            except:
                M2_MIC_DEVICE_INDEX = 0

        if M2_SPEAKER_DEVICE_INDEX is None and platform.system() != 'Linux':
            try:
                M2_SPEAKER_DEVICE_INDEX = sd.default.device[1]
            except:
                M2_SPEAKER_DEVICE_INDEX = 1
    except:
        # 如果设备扫描失败，使用默认值
        M2_MIC_DEVICE_INDEX = 0
        M2_SPEAKER_DEVICE_INDEX = 1

# ===================== Linux专用音频播放函数 =====================
def play_audio_linux(audio_data, sample_rate=24000):
    """在Linux系统下使用ALSA命令播放音频"""
    try:
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            temp_filename = f.name
            
        with wave.open(temp_filename, 'wb') as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(audio_data)
        
        print(f"🔊 使用ALSA播放语音，采样率: {sample_rate}Hz")
        
        cmd = ['aplay', '-D', 'plughw:0,0', '-r', str(sample_rate), '-c', '1', '-f', 'S16_LE', temp_filename]
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode != 0:
            print(f"❌ ALSA播放失败: {result.stderr}")
            cmd = ['aplay', '-D', 'default', temp_filename]
            subprocess.run(cmd)
        
        os.unlink(temp_filename)
        
    except Exception as e:
        print(f"❌ Linux音频播放失败: {e}")

# ===================== 音频处理工具函数 =====================
def calculate_rms(audio):
    """计算音频的均方根能量"""
    if len(audio) == 0:
        return 0
    return np.sqrt(np.mean(np.square(audio.astype(np.float64))))

def calculate_zcr(audio):
    """计算过零率"""
    if len(audio) < 2:
        return 0
    zero_crossings = np.sum(np.abs(np.diff(np.sign(audio)))) / 2
    return zero_crossings / len(audio)

# ===================== 音频回调函数 =====================
def audio_callback(indata, frames, time, status):
    """音频回调函数"""
    if status:
        print(f"音频回调状态: {status}")
    
    if is_ai_speaking:
        return
    
    # 确保 indata 是 numpy 数组
    if hasattr(indata, 'copy'):
        audio_data = indata.copy()
        # 确保数据是二维数组 [frames, channels]
        if audio_data.ndim == 1:
            audio_data = audio_data.reshape(-1, 1)
        audio_queue.put(audio_data)

# ===================== 增强噪音学习 =====================
def learn_environment_noise_enhanced(duration=3.0):
    """增强版环境噪音特征学习（延长时间+低频过滤）"""
    print("🧠 正在深度学习环境噪音特征...")
    
    noise_frames = []
    
    def filter_low_freq_noise(frame):
        """简单的低通滤波，不需要复杂的 scipy 滤波"""
        # 简单的滑动平均代替 filtfilt，虽然效果差一点但速度快很多
        # 这里为了极致性能，暂时直接返回原数据，或仅做简单的直流去除
        return frame - np.mean(frame)
    
    def noise_callback(indata, frames, time, status):
        if not is_ai_speaking:
            if hasattr(indata, 'copy'):
                audio_data = indata.copy()
                if audio_data.ndim == 2:
                    audio_frame = audio_data.flatten()
                    audio_frame = filter_low_freq_noise(audio_frame)  # 新增低频过滤
                    noise_frames.append(audio_frame)
    
    try:
        with sd.InputStream(device=M2_MIC_DEVICE_INDEX, channels=CHANNELS, 
                          samplerate=RATE, dtype='int16', 
                          callback=noise_callback, blocksize=CHUNK):
            time.sleep(duration)
        
        if noise_frames:
            print(f"✅ 采集到 {len(noise_frames)} 个噪音帧")
            
            # 分析噪音特征
            for i, frame in enumerate(noise_frames[:20]):
                simple_vad.update_noise(frame)
            
            print("✅ 噪音模型学习完成")
            
            # 计算平均能量
            energies = [calculate_rms(frame) for frame in noise_frames]
            avg_energy = np.median(energies) if energies else 40
            
            # 移除了复杂的频谱分析日志
            
            return avg_energy
        else:
            print("⚠️ 未能采集到噪音样本")
            return 40
    except Exception as e:
        print(f"❌ 噪音学习失败: {e}")
        return 40

# ===================== 极简高效 VAD =====================
# 移除了 calculate_spectral_centroid 和 is_human_voice_enhanced 函数，功能已集成到 SimpleVAD 中

# ===================== 增强录音函数 =====================
def record_audio_enhanced(noise_baseline):
    """增强版录音函数（新增AGC动态增益+远距离适配）"""
    global is_ai_speaking

    # 不在回调里丢帧，让队列持续有数据
    # 清空队列
    while not audio_queue.empty():
        try:
            audio_queue.get_nowait()
        except queue.Empty:
            break

    # 新增AGC参数
    AGC_MAX_GAIN = 6.0  # 最大增益（避免放大噪音）
    AGC_TARGET = noise_baseline * 3.0  # 目标语音能量（高于噪音基线3倍）

    # 准备录音，减少打印输出
    
    frames = []
    has_voice = False
    silence_count = 0
    voice_start_time = 0
    voice_frame_count = 0
    consecutive_voice_frames = 0  # 连续人声帧计数
    
    silence_blocks = int(SILENCE_DURATION * RATE / CHUNK)
    max_blocks = int(RATE / CHUNK * MAX_RECORD_SECONDS)
    min_voice_blocks = int(MIN_VOICE_DURATION * RATE / CHUNK)
    min_consecutive_voice = 1  # 最小连续人声帧数，进一步降低以提高响应速度

    # 帧计数器，用于降低 VAD 调用频率
    frame_idx = 0

    try:
        with sd.InputStream(device=M2_MIC_DEVICE_INDEX, channels=CHANNELS, 
                          samplerate=RATE, dtype='int16', 
                          callback=audio_callback, blocksize=CHUNK):
            for block_count in range(max_blocks):
                try:
                    data = audio_queue.get(timeout=0.1)
                    
                    if not isinstance(data, np.ndarray):
                        continue
                    
                    # 将数据展平为一维数组
                    if data.ndim == 2:
                        audio_chunk = data.flatten()
                    else:
                        audio_chunk = data
                    
                    if len(audio_chunk) == 0:
                        continue
                    
                    # 使用极简 VAD 检测
                    frame_idx += 1
                    # 每3帧做一次完整检查，其他时候只做能量检查
                    if frame_idx % 3 != 0:
                        is_voice = calculate_rms(audio_chunk) > noise_baseline * 1.3
                    else:
                        is_voice = simple_vad.is_voice(audio_chunk)
                    
                    # 增强版人声检测
                    if is_voice:
                        consecutive_voice_frames += 1
                        
                        # 需要连续几帧人声才开始录音，防止误触发
                        if not has_voice and consecutive_voice_frames >= min_consecutive_voice:
                            has_voice = True
                            voice_start_time = time.time()
                            voice_frame_count = 0
                        
                        if has_voice:
                            # 新增AGC动态增益
                            frame_energy = calculate_rms(audio_chunk)
                            if frame_energy < AGC_TARGET and frame_energy > 0:
                                gain = min(AGC_TARGET / frame_energy, AGC_MAX_GAIN)
                                audio_chunk = (audio_chunk * gain).astype(np.int16)
                            
                            frames.append(audio_chunk)
                            voice_frame_count += 1
                            silence_count = 0
                    else:
                        consecutive_voice_frames = 0
                        
                        if has_voice:
                            silence_count += 1
                            # 在人声段中，即使检测不到人声也保存音频（可能是辅音或弱音）
                            frames.append(audio_chunk)
                            
                            if silence_count >= silence_blocks:
                                if voice_frame_count >= min_voice_blocks:
                                    break
                                else:
                                    frames = []
                                    has_voice = False
                                    silence_count = 0
                                    voice_frame_count = 0
                        else:
                            # 更新噪音模型
                            simple_vad.update_noise(audio_chunk)

                except queue.Empty:
                    if has_voice:
                        silence_count += 1
                        if silence_count >= silence_blocks:
                            if voice_frame_count >= min_voice_blocks:
                                break
                            else:
                                frames = []
                                has_voice = False
                                silence_count = 0
                                voice_frame_count = 0
                    elif block_count > int(RATE / CHUNK * 5): # 5秒无语音超时
                        # print("\n⏰ 等待超时") # 减少日志打印
                        break
                        
    except Exception as e:
        print(f"\n❌ 录音出错: {e}")
        return b""

    # 后处理
    if not frames:
        # 完全没有录到声音（可能是静音超时），不打印错误直接返回
        return b""
        
    if len(frames) < 3:
        # print("⚠️ 录音片段过短，忽略") # 减少干扰日志
        return b""
    
    # 合并所有帧
    full_audio = np.concatenate(frames)
    
    # 最终质量检查（降低要求）
    final_energy = calculate_rms(full_audio)
    min_energy = max(noise_baseline * 1.2, 20)  # 大幅降低能量要求
    
    if final_energy < min_energy:
        print(f"⚠️ 录音能量较低: {final_energy:.2f} < {min_energy:.2f}，但继续处理...")
    
    if voice_frame_count < min_voice_blocks:
        print(f"❌ 有效人声帧不足: {voice_frame_count} < {min_voice_blocks}")
        return b""
    
    return full_audio.tobytes()

# ===================== ASR识别函数 =====================
class AudioRecorder:
    """音频录制器"""
    
    def __init__(self, sample_rate=16000, chunk_size=1280):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.is_recording = False
        
    def start_recording(self):
        """开始录制"""
        try:
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )
            self.is_recording = True
            return True
        except Exception:
            return False
    
    def read_audio_chunk(self):
        """读取音频数据块"""
        if self.is_recording and self.stream:
            try:
                return self.stream.read(self.chunk_size, exception_on_overflow=False)
            except Exception:
                return None
        return None
    
    def stop_recording(self):
        """停止录制"""
        if self.is_recording:
            self.is_recording = False
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
            self.audio.terminate()


def extract_text_from_result(msg_json):
    """从识别结果中提取文字"""
    try:
        # 从消息中提取文字内容
        if msg_json.get('msg_type') == 'result' and msg_json.get('res_type') == 'asr':
            data = msg_json.get('data', {})
            cn_data = data.get('cn', {})
            st_data = cn_data.get('st', {})
            rt_list = st_data.get('rt', [])
            
            if rt_list:
                words = []
                # 遍历所有rt条目
                for rt_item in rt_list:
                    ws_list = rt_item.get('ws', [])
                    for ws_item in ws_list:
                        cw_list = ws_item.get('cw', [])
                        for cw_item in cw_list:
                            word = cw_item.get('w', '')
                            if word and word.strip():  # 只添加非空文字
                                words.append(word)
                
                # 合并所有文字
                if words:
                    return ''.join(words)
    except Exception:
        pass
    
    return ""


class RTASRClient:
    """讯飞RTASR客户端"""
    
    def __init__(self):
        self.app_id = "11a82bfd"
        self.access_key_id = "1cdc0fd08228e91ae8cd06a1fdbae0b3"
        self.access_key_secret = "ZmU2ZjBmMmFiM2Q2YmM0N2ZjM2UzNDc0"
        self.base_ws_url = "wss://office-api-ast-dx.iflyaisol.com/ast/communicate/v1"
        
        self.ws = None
        self.is_connected = False
        self.recv_thread = None
        self.session_id = None
        self.current_text = ""
        self.last_partial = ""
    
    def _get_utc_time(self):
        """生成UTC时间格式"""
        beijing_tz = datetime.timezone(datetime.timedelta(hours=8))
        return datetime.datetime.now(beijing_tz).strftime("%Y-%m-%dT%H:%M:%S%z")
    
    def _generate_auth_params(self):
        """生成鉴权参数"""
        auth_params = {
            "accessKeyId": self.access_key_id,
            "appId": self.app_id,
            "uuid": uuid.uuid4().hex,
            "utc": self._get_utc_time(),
            "audio_encode": "pcm_s16le",
            "lang": "autodialect",
            "samplerate": "16000"
        }
        
        sorted_params = dict(sorted([(k, v) for k, v in auth_params.items() if v and str(v).strip()]))
        base_str = "&".join([f"{urllib.parse.quote(k)}={urllib.parse.quote(v)}" for k, v in sorted_params.items()])
        signature = hmac.new(
            self.access_key_secret.encode(),
            base_str.encode(),
            hashlib.sha1
        ).digest()
        auth_params["signature"] = base64.b64encode(signature).decode()
        return auth_params
    
    def _recv_msg(self):
        """接收服务端消息"""
        while self.is_connected and self.ws:
            try:
                msg = self.ws.recv()
                if not msg:
                    break
                    
                if isinstance(msg, str):
                    msg_json = json.loads(msg)
                    
                    if msg_json.get('msg_type') == 'action' and 'sessionId' in msg_json.get('data', {}):
                        self.session_id = msg_json['data']['sessionId']
                    
                    if msg_json.get('msg_type') == 'result':
                        # 提取文字内容
                        extracted_text = extract_text_from_result(msg_json)
                        if extracted_text:
                            # 🔑 关键：只追加"新增部分"
                            if extracted_text.startswith(self.last_partial):
                                new_part = extracted_text[len(self.last_partial):]
                            else:
                                new_part = extracted_text
                            
                            self.current_text += new_part
                            self.last_partial = extracted_text
                            
                            print(f"\r识别中: {self.current_text}", end='', flush=True)
                            
            except (Exception, json.JSONDecodeError):
                self.close()
                break
    
    def connect(self):
        """建立WebSocket连接"""
        if self.is_connected and self.ws:
            return True

        try:
            auth_params = self._generate_auth_params()
            full_ws_url = f"{self.base_ws_url}?{urllib.parse.urlencode(auth_params)}"
            
            self.ws = websocket.create_connection(full_ws_url, timeout=15)
            self.is_connected = True
            
            self.recv_thread = threading.Thread(target=self._recv_msg, daemon=True)
            self.recv_thread.start()
            
            time.sleep(1) # 等待连接建立
            return True
            
        except Exception as e:
            print(f"ASR 连接失败: {e}")
            return False
    
    def send_audio(self, audio_bytes):
        """直接发送录好的音频，不重复录音"""
        if not self.is_connected:
            if not self.connect():
                return ""
        
        # 重置状态
        self.current_text = ""
        self.last_partial = ""
        
        try:
            # 这里的 chunk_size 建议与讯飞要求的 1280 字节保持一致
            chunk_size = 1280
            for i in range(0, len(audio_bytes), chunk_size):
                chunk = audio_bytes[i:i + chunk_size]
                if self.ws:
                    self.ws.send(chunk, websocket.ABNF.OPCODE_BINARY)
                    # 模拟实时发送速率，但可以比实时快很多 (0.002s sleep 约为 20倍速)
                    time.sleep(0.002)
            
            # 发送一个结束标识（根据讯飞文档，有时需要发送特定指令或直接关闭）
            # 这里我们等待一小会儿让结果返回
            wait_start = time.time()
            # 动态等待：如果收到结果就不用死等
            while time.time() - wait_start < 0.4:
                time.sleep(0.05)
                # 如果当前文本已经稳定且不为空，可以提前一点退出吗？
                # RTASR 通常会持续修正，多等一下比较稳妥，但0.4s足够了
            
            return self.current_text
        except Exception as e:
            print(f"ASR 发送失败: {e}")
            self.close()
            return ""
        
    def close(self):
        """关闭连接"""
        self.is_connected = False
        if self.ws:
            try:
                self.ws.close()
            except:
                pass
        self.ws = None


def recognize_audio_fast(client, audio_bytes):
    """使用讯飞实时语音识别"""
    if not audio_bytes or len(audio_bytes) < 1600:
        return ""
    
    # 直接使用已连接的客户端发送音频
    return client.send_audio(audio_bytes)

# ===================== 快速备用ASR方案 =====================
def recognize_audio_fallback_fast(audio_bytes):
    """快速备用语音识别方案"""
    if not audio_bytes or len(audio_bytes) < 1600:
        return ""
    
    try:
        # 快速获取access_token
        url = "https://aip.baidubce.com/oauth/2.0/token"
        params = {
            "grant_type": "client_credentials",
            "client_id": API_KEY,
            "client_secret": SECRET_KEY
        }
        response = requests.get(url, params=params, timeout=2)
        access_token = response.json()["access_token"]
        
        # 快速进行语音识别
        asr_url = "https://vop.baidu.com/server_api"
        headers = {'Content-Type': 'application/json'}
        data = {
            "format": "pcm",
            "rate": RATE,
            "channel": 1,
            "cuid": str(uuid.uuid4()),
            "token": access_token,
            "dev_pid": DEV_PID,
            "speech": audio_bytes.hex(),
            "len": len(audio_bytes)
        }
        
        response = requests.post(asr_url, json=data, headers=headers, timeout=3)
        result = response.json()
        
        if "result" in result:
            text = result["result"][0] if result["result"] else ""
            return text
        else:
            return ""
            
    except:
        return ""

# ===================== 智能ASR选择 =====================
def recognize_audio(client, audio_bytes):
    """智能选择ASR方案"""
    text = recognize_audio_fast(client, audio_bytes)
    
    if not text.strip():
        text = recognize_audio_fallback_fast(audio_bytes)
    
    return text

# ===================== 快速百度 TTS =====================
def get_access_token():
    url = "https://aip.baidubce.com/oauth/2.0/token"
    params = {
        "grant_type": "client_credentials",
        "client_id": API_KEY,
        "client_secret": SECRET_KEY
    }
    try:
        response = requests.get(url, params=params, timeout=10)
        response.raise_for_status()
        return response.json()["access_token"]
    except Exception as e:
        print(f"❌ 获取access_token失败: {e}")
        return None

def tts_and_play_fast(text):
    global is_ai_speaking
    # 立即设置标志位，防止状态机提前跳回 IDLE
    is_ai_speaking = True
    threading.Thread(
        target=_tts_and_play_impl,
        args=(text,),
        daemon=True
    ).start()

def _tts_and_play_impl(text):
    global is_ai_speaking
    # 延迟导入 resample，避免影响启动速度
    try:
        from scipy.signal import resample
    except ImportError:
        resample = None

    is_ai_speaking = True

    try:
        print("🔊 正在合成语音...")
        token = get_access_token()
        if not token:
            print("❌ 无法获取访问令牌")
            is_ai_speaking = False
            return
            
        tts_url = "https://tsn.baidu.com/text2audio"

        params = {
            "tex": text,
            "tok": token,
            "cuid": "test_voice",
            "ctp": 1,
            "lan": "zh",
            "spd": 6,
            "pit": 5,
            "vol": 9,
            "per": 4,
            "aue": 6
        }

        resp = requests.post(tts_url, data=params, timeout=15)
        
        if resp.status_code != 200 or b'error' in resp.content.lower():
            print("❌ TTS API调用失败")
            is_ai_speaking = False
            return
            
        audio_data = resp.content
        
        print(f"🔊 收到TTS音频数据: {len(audio_data)} 字节")
        
        if platform.system() == 'Linux':
            print("🔊 使用ALSA播放语音...")
            play_audio_linux(audio_data, SPEAKER_RATE)
        else:
            try:
                audio_np = np.frombuffer(audio_data, dtype=np.int16)
                
                num_samples = len(audio_np)
                target_samples = int(num_samples * (SPEAKER_RATE / 16000))
                
                if resample is not None:
                    resampled_audio = resample(audio_np, target_samples).astype(np.int16)
                    print(f"🔊 正在播放语音回复...")
                    sd.play(resampled_audio * 0.8, samplerate=SPEAKER_RATE, device=M2_SPEAKER_DEVICE_INDEX)
                else:
                    # 如果没有 resample，直接播放原音频（可能音调会变或需要用其他方式）
                    # 但为了安全，直接抛出异常让外层 catch 去用 fallback
                    raise ImportError("scipy.signal.resample not available")
                
                sd.wait()
            except Exception as e:
                # print(f"❌ 音频播放失败: {e}") # 减少日志
                try:
                    # 备用：直接以原始采样率播放（16000Hz）
                    # 百度TTS默认返回16k，如果SPEAKER_RATE是24k，直接播16k是可以的，sd.play支持指定samplerate
                    sd.play(audio_np, samplerate=16000, device=M2_SPEAKER_DEVICE_INDEX)
                    sd.wait()
                except Exception as e2:
                    print(f"❌ 备用播放也失败: {e2}")

        print("✅ 语音播放完成")

    except Exception as e:
        print(f"❌ 语音合成/播放失败: {e}")
    finally:
        # 关键修改：播放结束后强制等待一小段时间，让物理回声消散，防止麦克风录入自己的声音
        time.sleep(0.8)
        is_ai_speaking = False

# ===================== 快速AI处理 =====================
def handle_ai_fast(text):
    if not text.strip():
        return

    if len(chat_history) > 0 and chat_history[-1].get("role") == "user":
        last_user_text = chat_history[-1].get("content", "")
        if text == last_user_text:
            return

    chat_history.append({"role": "user", "content": text})

    def ai_process():
        try:
            # 搜索本地文档
            relevant_docs = search_documents(text)
            
            # 优化后的 System Prompt
            system_prompt = (
                "你叫小西，是广西大学的人形服务机器人助手。你的性格热情友好、语气亲切。"
                "【重要指令】：请直接输出给用户的最终回答，严禁输出任何思维链、思考过程或 <think> 标签。"
                "请始终以'小西'自称，并保持专业、热情的服务态度。回复中可以适当加入情绪词汇，让表情切换更丰富。注意：请尽量保持回复简短，控制在50字以内。"
            )
            
            # 如果找到相关文档，构建包含文档内容的提示
            if relevant_docs:
                doc_context = "根据以下本地文档内容回答问题：\n"
                for doc in relevant_docs:
                    doc_context += f"\n【来源：{doc['source']}】\n{doc['content']}\n"
                doc_context += "\n请基于以上文档内容回答用户问题，如果文档内容不足，请结合你的知识进行补充。"
                
                # 创建包含文档内容的用户消息
                user_message = f"{text}\n\n{doc_context}"
            else:
                # 没有找到相关文档，使用普通对话
                user_message = text
            
            # 使用火山引擎ARK API
            import httpx
            
            # 配置火山引擎ARK API
            ARK_API_KEY = "e70a1849-d992-4a39-b75f-c3af033fee4b"
            ARK_API_URL = "https://ark.cn-beijing.volces.com/api/v3/responses"
            ARK_MODEL = "ep-20251208190154-944k6"
            
            # 构建请求数据
            request_data = {
                "model": ARK_MODEL,
                "input": [
                    {
                        "role": "system",
                        "content": [{"type": "input_text", "text": system_prompt}]
                    },
                    {
                        "role": "user",
                        "content": [{"type": "input_text", "text": user_message}]
                    }
                ]
            }
            
            headers = {
                "Authorization": f"Bearer {ARK_API_KEY}",
                "Content-Type": "application/json"
            }
            
            response = httpx.post(ARK_API_URL, json=request_data, headers=headers, timeout=15)
            
            if response.status_code == 200:
                result = response.json()
                reply = ""

                # 优先解析标准 OpenAI 格式 (大多数 DeepSeek/豆包模型适用)
                if "choices" in result:
                    # 某些推理模型会将思考过程放在 reasoning_content，我们只取 content
                    reply = result["choices"][0]["message"].get("content", "")
                
                # 兼容你代码中原有的 output 结构
                elif "output" in result and len(result["output"]) > 0:
                    output_node = result["output"][0]
                    if "summary" in output_node and len(output_node["summary"]) > 0:
                        reply = output_node["summary"][0].get("text", "")
                    else:
                        reply = str(output_node)

                if reply:
                    # --- 强化过滤逻辑：彻底移除 <think> 及其内容 ---
                    import re
                    # 1. 移除成对的标签及其内部所有内容
                    reply = re.sub(r'<think>.*?</think>', '', reply, flags=re.DOTALL | re.IGNORECASE)
                    # 2. 移除可能因为截断导致的残留标签（如只有后半部分或只有前半部分）
                    reply = re.sub(r'^.*?</think>', '', reply, flags=re.DOTALL | re.IGNORECASE)
                    reply = re.sub(r'<think>.*$', '', reply, flags=re.DOTALL | re.IGNORECASE)
                    
                    reply = reply.strip()

                    # 如果过滤后内容为空（模型只输出了思考），给一个保底回复
                    if not reply:
                        reply = "小西正在为您查询，请稍等。"

                    # 清理回答格式 (保留原有的特殊清理逻辑)
                    if "用户现在" in reply:
                        parts = reply.split("有了，试试这个：")
                        if len(parts) > 1:
                            reply = parts[1]

                    # 添加到聊天历史并执行后续动作
                    chat_history.append({"role": "assistant", "content": reply})
                    send_emotion_to_raspberry(reply)
                    tts_and_play_fast(reply)
                else:
                    print(f"❌ 无法从响应中提取文字: {result}")
            else:
                print(f"❌ API调用失败: {response.status_code}")
                
        except Exception as e:
            import traceback
            print(f"❌ AI处理异常: {e}")
            traceback.print_exc()

    threading.Thread(target=ai_process, daemon=True).start()

# ===================== 主循环 =====================
def main():
    global is_ai_speaking

    # ========== 初始化 ========== 
    port = 'COM3'
    if platform.system() == 'Linux':
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        port = ports[0] if ports else '/dev/ttyUSB0'

    mic_ctrl = MicArrayController(port, 115200)
    if mic_ctrl.ser:
        mic_ctrl.mic_circle_set()
        mic_ctrl.set_circle_mic_beam()

    find_devices()

    # noise_baseline = 40.0  # 快速模式：不做噪音学习
    # 恢复快速噪音学习，以适应不同环境底噪，解决小声听不到的问题
    noise_baseline = learn_environment_noise_enhanced(duration=1.0)
    print(f"✅ 环境噪音基线: {noise_baseline:.2f}")

    # 初始化 ASR 客户端，提前建立连接
    print("🔌 正在连接 ASR 服务...")
    asr_client = RTASRClient()
    if asr_client.connect():
        print("✅ ASR 服务已连接")
    else:
        print("⚠️ ASR 服务连接失败，将在需要时重试")

    print("\n" + "=" * 60)
    print("🤖 语音助手启动完成（完整状态机版）")
    print("=" * 60)
    print("🎤 增强VAD + AGC 已启用")
    print("🧠 ASR → AI → TTS 全流程状态机")
    print(f"🤝 表情服务器: {RASPBERRY_IP}:{RASPBERRY_PORT}")
    print("=" * 60)

    # ========== 状态机变量 ========== 
    state = RobotState.IDLE
    last_text = ""
    last_active_time = time.time()

    # ========== 主循环 ========== 
    while True:
        try:
            # ================= IDLE ================= 
            if state == RobotState.IDLE:
                time.sleep(0.01)

                if is_ai_speaking:
                    continue

                state = RobotState.RECORDING
                continue

            # ================= RECORDING ================= 
            if state == RobotState.RECORDING:
                audio_bytes = record_audio_enhanced(noise_baseline)

                if not audio_bytes or len(audio_bytes) < 1600:
                    state = RobotState.IDLE
                    continue

                state = RobotState.ASR
                continue

            # ================= ASR ================= 
            if state == RobotState.ASR:
                text = recognize_audio(asr_client, audio_bytes)
                text = text.strip()

                if not text:
                    state = RobotState.IDLE
                    continue

                # 防止重复识别 
                if text == last_text:
                    state = RobotState.IDLE
                    continue

                print(f"\n🗣 识别结果: {text}")
                last_text = text
                state = RobotState.THINKING
                continue

            # ================= THINKING ================= 
            if state == RobotState.THINKING:
                handle_ai_fast(last_text)
                state = RobotState.SPEAKING
                continue

            # ================= SPEAKING ================= 
            if state == RobotState.SPEAKING:
                # 等待TTS播放完成 
                while is_ai_speaking:
                    time.sleep(0.02)
                
                # 双重保险：再次清空音频队列，确保不残留尾音
                while not audio_queue.empty():
                    try:
                        audio_queue.get_nowait()
                    except queue.Empty:
                        break

                last_active_time = time.time()
                state = RobotState.IDLE
                continue

            # ================= ERROR ================= 
            if state == RobotState.ERROR:
                print("⚠️ 状态异常，重置系统")
                is_ai_speaking = False
                state = RobotState.IDLE

        except KeyboardInterrupt:
            print("\n👋 用户终止程序")
            break
        except Exception as e:
            print(f"\n❌ 主循环异常: {e}")
            state = RobotState.ERROR
            time.sleep(0.5)

if __name__ == "__main__":
    main()