import cv2
import numpy as np
import math
from picamera2 import Picamera2

# Parameter definitions
H, W = 300, 400     # height and width of the shootable area
l = 150             # distance between the object and the laser (mirror)
c = 50              # distance between the camera and the mirror
d = 50              # depth between the camera and the mirror
MIN_DISTANCE_BETWEEN_MARKERS = 50  # 设定A和B之间的最小距离阈值
MIN_CONTOUR_AREA = 100  # 最小轮廓面积阈值
MAX_CONTOUR_AREA = 10000  # 最大轮廓面积阈值


def phi(x_rel, frame_shape):
    x = W * (x_rel / (frame_shape[1] / 2))
    phi_value = math.degrees(math.atan((c - x) / l))
    return max(-50, min(50, phi_value))

def theta(y_rel, frame_shape):
    y = H * (y_rel / (frame_shape[0] / 2))
    theta_value = math.degrees(math.atan(y / l))
    return max(-50, min(50, theta_value))

def HoughCircleMarker_get_angles():
    # Initialize Picamera2
    picam2 = Picamera2()
    preview_config = picam2.create_preview_configuration()
    picam2.configure(preview_config)
    picam2.start()

    selected_point = 0 
    last_detected_centers = [(0, 0), (0, 0)] 

    def process_frame(frame):
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

    while True:
        frame = picam2.capture_array()
        detected_centers = process_frame(frame)
        last_detected_centers = detected_centers.copy()

        # 显示两个圆并计算选中点的角度
        for i, center in enumerate(detected_centers):
            color = (0, 0, 255) if i == selected_point else (0, 255, 0)
            cv2.circle(frame, (center[0], center[1]), 3, color, -1)

            # Calculate angles of selected point
            if i == selected_point:
                x_rel = center[0] - frame.shape[1] // 2
                y_rel = frame.shape[0] // 2 - center[1]
                phi_value = phi(x_rel, frame.shape)
                theta_value = theta(y_rel, frame.shape)
                yield selected_point, phi_value, theta_value

        # show the image
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
    yield phi, theta

def ContourCircleMarker_get_angles():
    # Initialize Picamera2
    picam2 = Picamera2()
    preview_config = picam2.create_preview_configuration()
    picam2.configure(preview_config)
    picam2.start()

    selected_point = 0  # 初始化 selected_point
    last_detected_centers = [(0, 0), (0, 0)]  # 记录上次检测到的两个圆心位置

    def process_frame(frame):
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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

        # 限制最多检测到两个圆心
        detected_centers = detected_centers[:2]

        # 如果检测到的圆少于2个，使用上次检测到的圆心位置补全
        if not A_detected and len(last_detected_centers) > 0:
            detected_centers.insert(0, last_detected_centers[0])  # 补充A的位置
        if not B_detected and len(last_detected_centers) > 1:
            detected_centers.append(last_detected_centers[1])  # 补充B的位置

        return detected_centers

    while True:
        frame = picam2.capture_array()
        detected_centers = process_frame(frame)

        # 更新记录的圆心位置
        last_detected_centers = detected_centers.copy()

        # 计算和输出选中点的角度，并同时显示两个圆
        for i, center in enumerate(detected_centers):
            # 设置颜色：红色表示选中的点，绿色表示未选中的点
            color = (0, 0, 255) if i == selected_point else (0, 255, 0)
            cv2.circle(frame, (center[0], center[1]), 3, color, -1)

            # 只计算和输出选中的点的角度
            if i == selected_point:
                x_rel = center[0] - frame.shape[1] // 2
                y_rel = frame.shape[0] // 2 - center[1]
                phi_value = phi(x_rel, frame.shape)
                theta_value = theta(y_rel, frame.shape)
                yield phi_value, theta_value  # 实时返回角度值

        # 显示图像
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



if __name__ == "__main__":
    for selected_point, phi_value, theta_value in HoughCircleMarker_get_angles():
        print(f"Selected Point[{selected_point + 1}] phi={phi_value}, theta={theta_value}")
    # for phi_value, theta_value in ContourCircleMarker_get_angles():
    #     print(f"Selected Point: phi={phi_value}, theta={theta_value}")
