# -*- coding:utf-8 -*-
import websocket
import hashlib
import base64
import hmac
import json
import ssl
import _thread as thread
import pyaudio
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime

# -------------------------- 请填你的讯飞信息 --------------------------
APPID = "11a82bfd"
APIKey = "1cdc0fd08228e91ae8cd06a1fdbae0b3"
APISecret = "ZmU2ZjBmMmFiM2Q2YmM0N2ZjM2UzNDc0"

# 播放配置（必须匹配小露输出格式：16k 16bit 单声道）
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, output=True)

# -------------------------- 核心合成+播放 --------------------------
def on_message(ws, message):
    try:
        msg = json.loads(message)
        if msg["code"] != 0:
            print("错误：", msg)
            return
        
        if "data" in msg:
            audio = base64.b64decode(msg["data"]["audio"])
            stream.write(audio)  # 实时播放
            
            if msg["data"]["status"] == 2:
                print("✅ 播放完成")
                stream.stop_stream()
                stream.close()
                p.terminate()
                ws.close()
    except Exception as e:
        print(f"Exception: {e}")

def on_error(ws, error):
    print("### error:", error)

def on_close(ws, close_status_code, close_msg):
    print("### closed ###")

def on_open(ws):
    def run():
        text = input("请说：")
        # Ensure text is not empty
        if not text:
            text = "你好，我是讯飞语音合成助手。"
        
        text_b64 = str(base64.b64encode(text.encode('utf-8')), 'utf8')
        data = {
            "common": {"app_id": APPID},
            "business": {"aue": "raw", "auf": "audio/L16;rate=16000", "vcn": "x4_yezi", "tte": "utf8"},
            "data": {"status": 2, "text": text_b64}
        }
        ws.send(json.dumps(data))
    thread.start_new_thread(run, ())

# 鉴权生成URL
def get_url():
    # 生成RFC1123格式的时间戳
    date = format_date_time(mktime(datetime.now().timetuple()))
    
    # 拼接字符串
    sign_origin = f"host: ws-api.xfyun.cn\ndate: {date}\nGET /v2/tts HTTP/1.1"
    
    # 进行hmac-sha256加密
    sha = hmac.new(APISecret.encode(), sign_origin.encode(), hashlib.sha256).digest()
    sign = base64.b64encode(sha).decode()
    
    # 拼接Authorization参数
    auth = f'api_key="{APIKey}", algorithm="hmac-sha256", headers="host date request-line", signature="{sign}"'
    auth_b64 = base64.b64encode(auth.encode()).decode()
    
    # 拼接URL
    url = "wss://tts-api.xfyun.cn/v2/tts?" + urlencode({
        "authorization": auth_b64, "date": date, "host": "ws-api.xfyun.cn"
    })
    return url

# 启动
if __name__ == "__main__":
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp(get_url(), on_message=on_message, on_error=on_error, on_close=on_close)
    ws.on_open = on_open
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
