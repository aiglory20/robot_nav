# coding=utf8
import asyncio
import websockets
import json
import time
import requests
import os
import pyaudio

# 核心配置：度小鹿-甜美女声（固定发音人参数5118）
PER = 5118  # 度小鹿-甜美女声专属参数
BASE_URL = "wss://aip.baidubce.com/ws/2.0/speech/publiccloudspeech/v1/tts"

# Pyaudio配置
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024

# 用户提供的API Key和Secret Key
API_KEY = "fk2G86S2wmdFFJQJPbZTRwHq"
SECRET_KEY = "MP7vJfLKqOaFM4x4HVxMcYxsCXrXbQI2"

def get_access_token():
    """
    使用 AK，SK 生成鉴权签名（Access Token）
    :return: access_token，或是None(如果错误)
    """
    url = "https://aip.baidubce.com/oauth/2.0/token"
    params = {
        "grant_type": "client_credentials",
        "client_id": API_KEY,
        "client_secret": SECRET_KEY
    }
    try:
        response = requests.post(url, params=params)
        response.raise_for_status()
        return response.json().get("access_token")
    except Exception as e:
        print(f"Failed to get access token: {e}")
        return None

class BaiduTTSCore:
    def __init__(self, access_token):
        """初始化核心合成类（仅保留鉴权和基础URL）"""
        self.access_token = access_token
        # 拼接完整WS URL（固定度小鹿5118）
        self.ws_url = f"{BASE_URL}?access_token={access_token}&per={PER}"
        self.websocket = None

    async def synthesize_audio(self, text):
        """
        核心流式合成方法（一步到位）
        :param text: 要合成的文本（建议单段≤1000字）
        """
        # 1. 建立WebSocket连接
        try:
            # extra_headers might cause issues in some versions/environments if passed to loop.create_connection
            # Baidu API accepts access_token in query string which is already in self.ws_url
            self.websocket = await websockets.connect(self.ws_url)
        except Exception as e:
            raise Exception(f"连接失败: {e}")

        # Pyaudio setup
        p = pyaudio.PyAudio()
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        output=True)

        try:
            # 2. 发送开始合成请求（固定参数：语速5、音调5、音量5、pcm格式）
            start_msg = json.dumps({
                "type": "system.start",
                "payload": {"spd": 5, "pit": 5, "vol": 5, "audio_ctrl": "{\"sampling_rate\":16000}", "aue": 6} # aue=6 for PCM
            })
            await self.websocket.send(start_msg)
            # 校验开始响应
            start_resp = json.loads(await self.websocket.recv())
            if start_resp.get("code") != 0:
                raise Exception(f"开始合成失败: {start_resp}")

            # 3. 发送待合成文本（核心步骤）
            text_msg = json.dumps({"type": "text", "payload": {"text": text}})
            await self.websocket.send(text_msg)

            # 4. 实时接收音频流并播放
            print("正在播放合成音频...")
            while True:
                try:
                    resp = await asyncio.wait_for(self.websocket.recv(), timeout=10)
                    if isinstance(resp, bytes):
                        # Play audio data
                        stream.write(resp)
                    else:
                        resp_json = json.loads(resp)
                        # 遇到结束/错误则终止
                        if resp_json.get("type") in ["system.finish", "system.error"]:
                            if resp_json.get("code") != 0:
                                raise Exception(f"合成错误: {resp_json}")
                            break
                except asyncio.TimeoutError:
                    print("Receive timeout.")
                    break

            # 5. 发送结束请求
            try:
                finish_msg = json.dumps({"type": "system.finish"})
                await self.websocket.send(finish_msg)
            except Exception:
                pass
                
            print("播放完成！")

        finally:
            # 6. 关闭连接和流
            if self.websocket:
                await self.websocket.close()
            stream.stop_stream()
            stream.close()
            p.terminate()

# 直接调用示例
if __name__ == "__main__":
    # 获取Access Token
    print("获取Access Token...")
    ACCESS_TOKEN = get_access_token()
    
    if not ACCESS_TOKEN:
        print("无法获取Access Token，请检查API Key和Secret Key。")
        exit(1)
        
    print(f"Access Token获取成功: {ACCESS_TOKEN[:10]}...")

    # 要合成的文本（度小鹿支持中英文）
    SYNTH_TEXT = "你好呀！我是度小鹿，很高兴为你提供甜美女声的语音合成服务～Hello, I'm Du Xiaolu."

    # 核心调用逻辑
    async def main():
        tts_core = BaiduTTSCore(ACCESS_TOKEN)
        await tts_core.synthesize_audio(SYNTH_TEXT)
        while True:
            text = input("请输入要合成的文本（q退出）：")
            if text == 'q':
                break
            await tts_core.synthesize_audio(text)

    asyncio.run(main())
