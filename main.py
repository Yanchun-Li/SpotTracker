import cv2
import time
from gstreamer_stream import start_gstreamer_stream
from point_tracking import point_tracker

def flip_image(frame):
    return cv2.flip(frame, -1)

# 启动GStreamer RTSP流传输
start_gstreamer_stream()
time.sleep(5) 

# 连接到RTSP流
url = "rtsp://raspberry_pi_ip_address:8554/"
cap = cv2.VideoCapture(url)

# 初始化变量
points = [(100, 100), (200, 100), (200, 200), (100, 200), (150, 150), (50, 50)]  # 示例点，替换为实际值
selected_point = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # 图像反转
    flipped_frame = flip_image(frame)

    # 点跟踪
    tracked_frame = point_tracker(flipped_frame, points, selected_point)

    # 显示处理后的帧
    cv2.imshow('Tracked Frame', tracked_frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
