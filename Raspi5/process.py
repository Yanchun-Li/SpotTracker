import cv2
import numpy as np
import math
import json
import socket
from picamera2 import Picamera2

# Parameter definitions
H, W = 300, 400     # height and width of the shootable area
l = 150             # distance between the object and the laser (mirror)
c = 50              # distance between the camera and the mirror
d = 50              # depth between the camera and the mirror
MIN_DISTANCE_BETWEEN_MARKERS = 50  # 设定A和B之间的最小距离阈值
MIN_CONTOUR_AREA = 100  # 最小轮廓面积阈值
MAX_CONTOUR_AREA = 10000  # 最大轮廓面积阈值


# def phi(x_rel, frame_shape):
#     x = W * (x_rel / (frame_shape[1] / 2))
#     phi_value = math.degrees(math.atan((c - x) / l))
#     return max(-50, min(50, phi_value))

# def theta(y_rel, frame_shape):
#     y = H * (y_rel / (frame_shape[0] / 2))
#     theta_value = math.degrees(math.atan(y / l))
#     return max(-50, min(50, theta_value))

def convert_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours  

#--------------------------Radar Center-----------------------------------
def detect_radar_center(contours):
    largest_contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest_contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)    


#--------------------------- Marker Center ------------------------
def RectMarkerCenter_Contour(marker_num, contours):
    detected_centers = []
    A_detected = False
    B_detected = False
    
    for contour in contours:
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
        
        # 如果逼近的多边形有4个顶点，可能是矩形
        if len(approx) == 4:
            rect = cv2.boundingRect(approx)
            (x, y, w, h) = rect
            aspect_ratio = float(w) / h

            # 根据宽高比过滤近似矩形的区域，通常我们认为宽高比在一定范围内(near to square and w,h are larger than 10 pixels)
            if 0.8 <= aspect_ratio <= 1.2 and w > 10 and h > 10:
                detected_center = (int(x + w / 2), int(y + h / 2))

                # 根据之前检测到的位置判断当前检测到的是A还是B
                if np.linalg.norm(np.array(detected_center) - np.array(last_detected_centers[0])) < \
                   np.linalg.norm(np.array(detected_center) - np.array(last_detected_centers[1])):
                    if not A_detected:
                        detected_centers.append(detected_center)
                        A_detected = True  # 标记A被检测到
                    elif np.linalg.norm(np.array(detected_center) - np.array(detected_centers[0])) > MIN_DISTANCE_BETWEEN_MARKERS:
                        detected_centers.append(detected_center)
                        B_detected = True  # 标记B被检测到
                else:
                    if not B_detected:
                        detected_centers.append(detected_center)
                        B_detected = True  # 标记B被检测到
                    elif np.linalg.norm(np.array(detected_center) - np.array(detected_centers[0])) > MIN_DISTANCE_BETWEEN_MARKERS:
                        detected_centers.append(detected_center)
                        A_detected = True  # 标记A被检测到

    # 限制最多检测到marker_num个矩形中心
    detected_centers = detected_centers[:marker_num-1]
    if marker_num == 2:
        # 如果检测到的矩形少于2个，使用上次检测到的位置补全
        if not A_detected and len(last_detected_centers) > 0:
            detected_centers.insert(0, last_detected_centers[0])  # 补充A的位置
        if not B_detected and len(last_detected_centers) > 1:
            detected_centers.append(last_detected_centers[1])  # 补充B的位置

    return detected_centers


#--------------------------- Marker Center ------------------------
def CirMarkerCenter_Contour(marker_num, contours):

    detected_centers = []
    A_detected = False
    B_detected = False

    # Fit ellipses
    for contour in contours:
        if len(contour) >= 5:  # At least 5 points needed to fit an ellipse
            ellipse = cv2.fitEllipse(contour)
            (x, y), (MA, ma), angle = ellipse  # Center, Major axis, Minor axis, Angle of rotation

            if MA == 0:  # Avoid division by zero
                continue

            # Filter based on the aspect ratio and size
            aspect_ratio = ma / MA  # Ratio of minor to major axis
            if 0.8 <= aspect_ratio <= 1.2 and 10 < MA < 100:  # Adjust thresholds as needed
                detected_center = (int(x), int(y))

                # 根据上次记录的位置，判断当前检测到的是A还是B
                if np.linalg.norm(np.array(detected_center) - np.array(last_detected_centers[0])) < np.linalg.norm(np.array(detected_center) - np.array(last_detected_centers[1])):
                    if not A_detected:
                        detected_centers.append(detected_center)
                        A_detected = True  # 标记A被检测到
                    elif np.linalg.norm(np.array(detected_center) - np.array(detected_centers[0])) > MIN_DISTANCE_BETWEEN_MARKERS:
                        detected_centers.append(detected_center)
                        B_detected = True  # 标记B被检测到
                else:
                    if not B_detected:
                        detected_centers.append(detected_center)
                        B_detected = True  # 标记B被检测到
                    elif np.linalg.norm(np.array(detected_center) - np.array(detected_centers[0])) > MIN_DISTANCE_BETWEEN_MARKERS:
                        detected_centers.append(detected_center)
                        A_detected = True  # 标记A被检测到

    # 限制最多检测到marker_num个圆心
    detected_centers = detected_centers[:marker_num-1]
    if marker_num == 2:
        # 如果检测到的圆少于2个，使用上次检测到的圆心位置补全
        if not A_detected and len(last_detected_centers) > 0:
            detected_centers.insert(0, last_detected_centers[0])  # 补充A的位置
        if not B_detected and len(last_detected_centers) > 1:
            detected_centers.append(last_detected_centers[1])  # 补充B的位置

    return detected_centers

#--------------------------Calculate Error------------------------
def calculate_error(point_id, last_armmarker_center, radar_center):
    marker_center = last_armmarker_center[point_id]
    error_x = radar_center[0] - marker_center[0]
    error_y = radar_center[1] - marker_center[1]
    return (error_x, error_y)

#--------------------------Send to PC-----------------------------------------
def send2pc(value, server_ip, server_port=5005):
    """
    server_ip:    Windows PC IP ADDRESS
    server_port:  5005 (freely definable)
    """
    message = json.dumps(value)                                 # convert into json

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((server_ip, server_port))                      # connect server in Windows
    sock.sendall(message.encode())                              # send signals
    sock.close()


def display(point_id, ArmMarker_centers, Radar_center):
    # 计算和输出选中点的角度，并同时显示两个圆
    for i, center in enumerate(ArmMarker_centers):
        # 设置颜色：红色表示选中的点，绿色表示未选中的点
        color = (0, 0, 255) if i == point_id else (0, 255, 0)
        cv2.circle(frame, (center[0], center[1]), 3, color, -1)
    
    cv2.circle(frame, (Radar_center[0], Radar_center[1]), 3, (0, 0, 255), -1)  
    # draw a line between selected point and Radar center
    cv2.line(frame, (Radar_center[0], Radar_center[1]), (ArmMarker_centers[point_id][0], ArmMarker_centers[point_id][1]), (0, 0, 255), 1)


# def mirrorAngles(detected_centers):
#     for i, center in enumerate(detected_centers):
#         # Calculate angles of selected point
#         if i == selected_point:
#             x_rel = center[0] - frame.shape[1] // 2
#             y_rel = frame.shape[0] // 2 - center[1]
#             phi_value = phi(x_rel, frame.shape)
#             theta_value = theta(y_rel, frame.shape)
#             return phi_value, theta_value

##### NOT USING THIS ######################
def CirMarkerCenter_Hough(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Use HoughCircles to detect circles
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20, param1=50, param2=30, minRadius=5, maxRadius=50)
    
    detected_centers = []
    A_detected = False
    B_detected = False

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        # Filter out circles based on a minimum radius and limit to a maximum of 2 circles
        valid_circles = [c for c in circles if c[2] >= 3]
        valid_circles = valid_circles[:1]   # only save two circles

        for (x, y, r) in valid_circles:
            detected_center = (x, y)

            # 根据上次记录的位置，判断当前检测到的是A还是B
            if np.linalg.norm(np.array(detected_center) - np.array(last_detected_centers[0])) < np.linalg.norm(np.array(detected_center) - np.array(last_detected_centers[1])):
                detected_centers.append(detected_center)
                A_detected = True  # Marker A is detected
            else:
                detected_centers.append(detected_center)
                B_detected = True  # Marker B is detected

    # 如果检测到的圆少于2个，使用上次检测到的圆心位置补全
    if not A_detected:
        detected_centers.insert(0, last_detected_centers[0])
    if not B_detected:
        detected_centers.append(last_detected_centers[1])

    return detected_centers

if __name__ == "__main__":
    # Initialize Picamera2
    picam2 = Picamera2()
    preview_config = picam2.create_preview_configuration()
    picam2.configure(preview_config)
    picam2.start()

    selected_point = 0  # 初始化 selected_point
    last_detected_centers = [(0, 0), (0, 0)]  # 记录上次检测到的两个圆心位置

    while True:
        frame = picam2.capture_array()
        # Using contours to detect circle markers
        contours = convert_frame(frame)
        detected_centers = CirMarkerCenter_Contour(contours)
        # Using Hough circle method
        # detected_centers = CirMarkerCenter_Hough(frame)
        last_detected_centers = detected_centers.copy() # update marker centers
        
        display(detected_centers)
        # phi_value, theta_value = mirrorAngles(detected_centers)
        # print(f"Selected Point[{selected_point}]: phi={phi_value}, theta={theta_value}")

        cv2.imshow('Detected Circles', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('1'):
            selected_point = 0
        elif key == ord('2'):
            selected_point = 1
        elif key == ord('q'):
            break

    picam2.stop()
    cv2.destroyAllWindows()