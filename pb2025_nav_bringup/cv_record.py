import pyrealsense2 as rs
import numpy as np
import cv2
from pyzbar import pyzbar
import datetime

# 配置深度和彩色流
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 启动流
profile = pipeline.start(config)

# 获取深度传感器的深度尺度
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print(f"Depth Scale: {depth_scale}")

# 创建对齐对象（将深度图对齐到彩色图）
align_to = rs.stream.color
align = rs.align(align_to)

# 视频录制相关变量
is_recording = False
video_writer = None
recording_start_time = None

try:
    while True:
        # 等待一组连贯的帧：深度和彩色
        frames = pipeline.wait_for_frames()

        # 将对齐帧对齐到彩色流
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # 将图像转换为numpy数组
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # 在彩色图像中识别二维码
        barcodes = pyzbar.decode(color_image)

        # 遍历检测到的所有二维码
        for barcode in barcodes:
            # 提取二维码边界框的位置
            (x, y, w, h) = barcode.rect

            # 在彩色图像上绘制边界框
            cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 准备显示文本
            barcode_data = barcode.data.decode("utf-8")
            barcode_type = barcode.type

            # 计算二维码区域的中心点
            center_x = x + w // 2
            center_y = y + h // 2

            # 在中心点附近取一个小区域求平均深度
            bbox_size = 5
            depth_sum = 0
            count = 0

            for i in range(-bbox_size, bbox_size):
                for j in range(-bbox_size, bbox_size):
                    current_x = center_x + i
                    current_y = center_y + j
                    if 0 <= current_x < depth_image.shape[1] and 0 <= current_y < depth_image.shape[0]:
                        dist = depth_image[current_y, current_x] * depth_scale
                        if 0.1 < dist < 10.0:
                            depth_sum += dist
                            count += 1

            if count > 0:
                average_distance = depth_sum / count
                distance_text = f"Dist: {average_distance:.2f}m"
            else:
                distance_text = "Dist: N/A"

            # 在图像上绘制二维码数据和距离
            text = f"{distance_text}"
            cv2.putText(color_image, text, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 在控制台打印信息
            print(
                f"Found QR code: {barcode_data}, Type: {barcode_type}, {distance_text}")

        # 显示录制状态
        if is_recording:
            # 计算录制时长
            recording_time = datetime.datetime.now() - recording_start_time
            minutes = recording_time.seconds // 60
            seconds = recording_time.seconds % 60
            status_text = f"Recording: {minutes:02d}:{seconds:02d}"
            cv2.putText(color_image, status_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(color_image, "Press 'p' to stop", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(color_image, "Press 'p' to start recording",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 录制视频
        if is_recording and video_writer is not None:
            video_writer.write(color_image)

        # 显示图像
        cv2.imshow('QR Code with Depth', color_image)

        # 按键处理
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            if not is_recording:
                # 开始录制
                is_recording = True
                recording_start_time = datetime.datetime.now()

                # 生成文件名（包含时间戳）
                timestamp = recording_start_time.strftime("%Y%m%d_%H%M%S")
                filename = f"recording_{timestamp}.avi"

                # 获取帧尺寸
                frame_height, frame_width = color_image.shape[:2]

                # 创建VideoWriter对象
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                video_writer = cv2.VideoWriter(
                    filename, fourcc, 30.0, (frame_width, frame_height))

                print(f"开始录制视频: {filename}")
            else:
                # 停止录制
                is_recording = False
                if video_writer is not None:
                    video_writer.release()
                    video_writer = None
                    print("录制已停止")

                # 计算总录制时间
                recording_end_time = datetime.datetime.now()
                total_time = recording_end_time - recording_start_time
                minutes = total_time.seconds // 60
                seconds = total_time.seconds % 60
                print(f"录制时长: {minutes:02d}:{seconds:02d}")

finally:
    # 停止流并关闭所有窗口
    pipeline.stop()
    if video_writer is not None:
        video_writer.release()
    cv2.destroyAllWindows()
