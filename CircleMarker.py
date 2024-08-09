import cv2
import numpy as np
import threading
from collections import deque

class KalmanTracker:
    def __init__(self):
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32)
        self.history = deque(maxlen=5)

    def predict(self):
        return self.kalman.predict()

    def correct(self, midpoint):
        self.kalman.correct(midpoint)
        self.history.append(midpoint)
        if len(self.history) == self.history.maxlen:
            median = np.median(np.array(self.history), axis=0)
            self.history.popleft()
            return median
        return midpoint

# 打开摄像头
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# 记录上次更新的中点坐标
trackers = []
detected_centers = []
frame_processed = None

def process_frame():
    global trackers, detected_centers, frame_processed
    while True:
        if frame_processed is None:
            continue

        frame = frame_processed.copy()
        # 转换为灰度图像
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 使用HoughCircles检测圆形
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20, param1=50, param2=30, minRadius=5, maxRadius=50)
        
        detected_centers = []

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")

            for (x, y, r) in circles:
                midpoint = np.array([x, y], dtype=np.float32)

                if len(trackers) < len(detected_centers) + 1:
                    trackers.append(KalmanTracker())

                # 使用卡尔曼滤波器
                tracker = trackers[len(detected_centers)]
                corrected_point = tracker.correct(midpoint)
                predicted = tracker.predict()
                detected_center = (int(predicted[0]), int(predicted[1]))

                detected_centers.append(detected_center)

        # 输出检测到的圆形个数和中点
        print(f"Detected {len(detected_centers)} white circle(s). Centers:", detected_centers)
        for center in detected_centers:
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

# 启动图像处理线程
processing_thread = threading.Thread(target=process_frame)
processing_thread.daemon = True
processing_thread.start()

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法接收帧 (stream end?). Exiting ...")
        break

    frame_processed = frame.copy()

    for center in detected_centers:
        cv2.circle(frame, center, 5, (255, 0, 0), -1)

    cv2.imshow('Detected Circles', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
