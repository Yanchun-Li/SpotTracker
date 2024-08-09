from picamera2 import Picamera2
import cv2
import numpy as np
import threading

# Initialize Picamera2
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration()
picam2.configure(preview_config)
picam2.start()

# Variables to store detected centers
detected_centers = []
last_detected_centers = [(0, 0), (0, 0)]  # 记录上次检测到的两个圆心位置
frame_processed = None
A_detected = False  # 标记是否检测到A
B_detected = False  # 标记是否检测到B

def process_frame():
    global detected_centers, last_detected_centers, frame_processed, A_detected, B_detected
    while True:
        if frame_processed is None:
            continue

        frame = frame_processed.copy()
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
            valid_circles = valid_circles[:1]  # max two circles

            for (x, y, r) in valid_circles:
                detected_center = (x, y)

                # 根据上次记录的位置，判断当前检测到的是A还是B
                if np.linalg.norm(np.array(detected_center) - np.array(last_detected_centers[0])) < np.linalg.norm(np.array(detected_center) - np.array(last_detected_centers[1])):
                    detected_centers.append(detected_center)
                    A_detected = True  # 标记A被检测到
                else:
                    detected_centers.append(detected_center)
                    B_detected = True  # 标记B被检测到

        # 如果检测到的圆少于2个，使用上次检测到的圆心位置补全
        if not A_detected:
            detected_centers.insert(0, last_detected_centers[0])  # 补充A的位置
        if not B_detected:
            detected_centers.append(last_detected_centers[1])  # 补充B的位置

        # Update last detected centers
        last_detected_centers = detected_centers.copy()

        # Print detected circles and their centers
        print(f"Detected {len(detected_centers)} valid circle(s). Centers:", detected_centers)
        for center in detected_centers:
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

# Start image processing thread
processing_thread = threading.Thread(target=process_frame)
processing_thread.daemon = True
processing_thread.start()

while True:
    frame = picam2.capture_array()

    frame_processed = frame.copy()

    for center in detected_centers:
        cv2.circle(frame, center, 5, (255, 0, 0), -1)

    cv2.imshow('Detected Circles', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
