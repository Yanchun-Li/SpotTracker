import cv2
import numpy as np
import threading

def get_triangle_center(contour):
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)
    return None

def is_triangle(contour):
    epsilon = 0.04 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    return len(approx) == 3

def triangle_similarity(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return float('inf')
    return area / (0.5 * perimeter * perimeter)

def distance(pt1, pt2):
    return np.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

# 初始化卡尔曼滤波器
kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03

# 打开摄像头
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# 记录上次更新的中点坐标
last_midpoint = None
detected_center = None
frame_processed = None

def process_frame():
    global last_midpoint, detected_center, frame_processed
    while True:
        if frame_processed is None:
            continue

        frame = frame_processed.copy()
        # 转换为灰度图像
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 转换为HSV图像以进行颜色检测
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 蓝色范围的阈值
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        # 蓝色掩码
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        triangles = []

        for contour in contours:
            if is_triangle(contour):
                area = cv2.contourArea(contour)
                if area > 1000:  # 过滤掉小面积轮廓
                    # 检查该三角形是否为蓝色
                    mask_contour = np.zeros_like(mask)
                    cv2.drawContours(mask_contour, [contour], -1, 255, -1)
                    mask_and = cv2.bitwise_and(mask, mask_contour)
                    non_zero = cv2.countNonZero(mask_and)
                    if non_zero > 0.5 * area:  # 如果掩码中的非零像素超过轮廓面积的50%
                        similarity = triangle_similarity(contour)
                        center = get_triangle_center(contour)
                        if center:
                            triangles.append((similarity, center, contour))

        triangles = sorted(triangles, key=lambda x: x[0])[:2]

        if len(triangles) == 2:
            center1 = triangles[0][1]
            center2 = triangles[1][1]
            if distance(center1, center2) > 50:  # 检查两个三角形重心之间的距离，阈值为50像素
                midpoint = np.array([(center1[0] + center2[0]) / 2, (center1[1] + center2[1]) / 2], dtype=np.float32)

                # 使用卡尔曼滤波器
                kalman.correct(midpoint)
                predicted = kalman.predict()
                detected_center = (int(predicted[0]), int(predicted[1]))

                last_midpoint = detected_center
            else:
                detected_center = last_midpoint
        else:
            if last_midpoint is not None:
                detected_center = last_midpoint
            else:
                detected_center = (0, 0)

        # 输出检测到的三角形个数和中点
        print(f"Detected {len(triangles)} blue triangle(s). Midpoint of the two triangles:", detected_center)
        for _, center, contour in triangles:
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
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

    if detected_center is not None:
        cv2.circle(frame, detected_center, 5, (255, 0, 0), -1)

    cv2.imshow('Detected Triangles', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()



# ######################## without Kalman filter ################################
# import cv2
# import numpy as np
# import threading

# def get_triangle_center(contour):
#     M = cv2.moments(contour)
#     if M["m00"] != 0:
#         cX = int(M["m10"] / M["m00"])
#         cY = int(M["m01"] / M["m00"])
#         return (cX, cY)
#     return None

# def is_triangle(contour):
#     epsilon = 0.04 * cv2.arcLength(contour, True)
#     approx = cv2.approxPolyDP(contour, epsilon, True)
#     return len(approx) == 3

# def triangle_similarity(contour):
#     area = cv2.contourArea(contour)
#     perimeter = cv2.arcLength(contour, True)
#     if perimeter == 0:
#         return float('inf')
#     return area / (0.5 * perimeter * perimeter)

# def distance(pt1, pt2):
#     return np.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)


# # 打开摄像头
# cap = cv2.VideoCapture(0)

# if not cap.isOpened():
#     print("无法打开摄像头")
#     exit()

# # 记录上次更新的中点坐标
# last_midpoint = None
# detected_center = None
# frame_processed = None

# def process_frame():
#     global last_midpoint, detected_center, frame_processed
#     while True:
#         if frame_processed is None:
#             continue

#         frame = frame_processed.copy()
#         # 转换为灰度图像
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         blurred = cv2.GaussianBlur(gray, (5, 5), 0)
#         edges = cv2.Canny(blurred, 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#         # 转换为HSV图像以进行颜色检测
#         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#         # 蓝色范围的阈值
#         lower_blue = np.array([100, 150, 50])
#         upper_blue = np.array([140, 255, 255])
#         # 蓝色掩码
#         mask = cv2.inRange(hsv, lower_blue, upper_blue)

#         triangles = []

#         for contour in contours:
#             if is_triangle(contour):
#                 area = cv2.contourArea(contour)
#                 if area > 1000:  # 过滤掉小面积轮廓
#                     # 检查该三角形是否为蓝色
#                     mask_contour = np.zeros_like(mask)
#                     cv2.drawContours(mask_contour, [contour], -1, 255, -1)
#                     mask_and = cv2.bitwise_and(mask, mask_contour)
#                     non_zero = cv2.countNonZero(mask_and)
#                     if non_zero > 0.5 * area:  # 如果掩码中的非零像素超过轮廓面积的50%
#                         similarity = triangle_similarity(contour)
#                         center = get_triangle_center(contour)
#                         if center:
#                             triangles.append((similarity, center, contour))

#         triangles = sorted(triangles, key=lambda x: x[0])[:2]

#         if len(triangles) == 2:
#             center1 = triangles[0][1]
#             center2 = triangles[1][1]
#             if distance(center1, center2) > 50:  # 检查两个三角形重心之间的距离，阈值为50像素
#                 midpoint = np.array([(center1[0] + center2[0]) / 2, (center1[1] + center2[1]) / 2], dtype=np.float32)

#                 detected_center = (int(midpoint[0]), int(midpoint[1]))

#                 last_midpoint = detected_center
#             else:
#                 detected_center = last_midpoint
#         else:
#             if last_midpoint is not None:
#                 detected_center = last_midpoint
#             else:
#                 detected_center = (0, 0)

#         for _, _, contour in triangles:
#             cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
#         # 输出检测到的三角形个数和中点
#         print(f"Detected {len(triangles)} blue triangle(s). Midpoint of the two triangles:", detected_center)

# # 启动图像处理线程
# processing_thread = threading.Thread(target=process_frame)
# processing_thread.daemon = True
# processing_thread.start()

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("无法接收帧 (stream end?). Exiting ...")
#         break

#     frame_processed = frame.copy()

#     if detected_center is not None:
#         cv2.circle(frame, detected_center, 5, (255, 0, 0), -1)

#     cv2.imshow('Detected Triangles', frame)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()