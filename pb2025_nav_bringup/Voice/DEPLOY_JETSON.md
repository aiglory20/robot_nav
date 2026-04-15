# Jetson Nano/NX 部署指南

本指南将帮助你在 NVIDIA Jetson Nano 或 NX 上部署并运行 `voice_interaction.py` 脚本。

## 1. 系统环境准备

Jetson Nano 通常运行 Ubuntu 系统。请打开终端并执行以下命令安装必要的系统库：

```bash
# 更新源
sudo apt-get update

# 安装 PortAudio（PyAudio 的依赖）和 ALSA 工具
sudo apt-get install -y portaudio19-dev alsa-utils python3-pyaudio

# 安装 pip（如果尚未安装）
sudo apt-get install -y python3-pip
```

## 2. 安装 Python 依赖

在项目目录下运行以下命令安装 Python 库：

```bash
pip3 install -r requirements.txt
```

如果 `numpy` 安装缓慢，可以使用 apt 安装：
```bash
sudo apt-get install -y python3-numpy
```

## 3. 硬件连接与配置

1.  **麦克风**：连接 USB 麦克风（如 M2 麦克风阵列）。
2.  **扬声器**：连接 USB 扬声器或通过 HDMI/3.5mm 接口连接音频输出设备。

### 检查音频设备

运行以下命令查看录音设备（麦克风）：
```bash
arecord -l
```
记下你的麦克风卡号（card X）和设备号（device Y），例如 `card 1: ... device 0`。

运行以下命令查看播放设备（扬声器）：
```bash
aplay -l
```
记下你的扬声器卡号和设备号。

## 4. 脚本配置调整 (重要)

打开 `voice_interaction.py`，找到 `find_devices` 函数（约第 60 行）。

**Linux 下的扬声器配置**：
脚本默认使用 `plughw:0,0` 作为播放设备。如果你的扬声器不是默认设备（例如是 USB 音响，可能是 `plughw:1,0`），请修改代码：

```python
    # 2. Find Speaker
    if platform.system() == 'Linux':
        # 修改这里的 plughw:X,Y，其中 X 是卡号，Y 是设备号
        # 例如：如果 aplay -l 显示 USB Audio 是 card 1, device 0，则改为 'plughw:1,0'
        M2_SPEAKER_DEVICE_INDEX = 'plughw:1,0' 
        print(f"✅ Selected Speaker (Linux): {M2_SPEAKER_DEVICE_INDEX}")
```

**麦克风配置**：
脚本会自动查找包含 "Digital_Array_Mic", "XFM-DP" 或 "USB Audio" 名称的设备。如果你的麦克风名称不同，可能需要手动指定索引或修改匹配关键词。

## 5. 运行脚本

确保 `本地知识库` 文件夹在同一目录下。

```bash
python3 voice_interaction.py
```

## 6. 常见问题排查

- **报错 `OSError: [Errno -9996] Invalid input device index`**：
  这说明没有找到合适的麦克风。请运行 `python3` 进入交互模式，使用 `pyaudio` 列出所有设备查看名称：
  ```python
  import pyaudio
  p = pyaudio.PyAudio()
  for i in range(p.get_device_count()):
      print(i, p.get_device_info_by_index(i).get('name'))
  ```
  然后修改脚本中的匹配逻辑。

- **无声音输出**：
  尝试使用 `aplay` 直接测试播放：
  ```bash
  aplay -D plughw:0,0 /usr/share/sounds/alsa/Front_Center.wav
  ```
  如果 `plughw:0,0` 没声音，尝试 `plughw:1,0` 等，找到正确的设备后更新脚本。

- **权限问题**：
  如果提示权限不足，尝试将当前用户加入 audio 组：
  ```bash
  sudo usermod -aG audio $USER
  ```
  然后重启或注销重新登录。
