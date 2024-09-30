from picamera2 import Picamera2
import cv2
import numpy as np
import socket
import json
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

#--------------------------Radar Center-----------------------------------
def detect_radar_center_from_threshold(blurred_image):
    _, thresh = cv2.threshold(blurred_image, 200, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)    
    return None

#--------------------------Calculate Error------------------------
def calculate_error(marker_center, radar_center):
    error_x = radar_center[0] - marker_center[0]
    error_y = radar_center[1] - marker_center[1]
    return (error_x, error_y)

#--------------------------Send to PC-----------------------------------------
def send2pc(detected_centers):
    server_ip = "WINDOWS_PC_IP_ADDRESS"                         # Windows PC's IP Address
    server_port = 5005                                          # server port (自由定义)
    message = json.dumps(detected_centers)                      # convert into json

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((server_ip, server_port))                      # connect server in Windows
    sock.sendall(message.encode())                              # send signals
    sock.close()

#-----------------------Main Processing----------------------------------------------
def process_frame():
    global trackers, detected_centers, last_detected_centers, frame_processed, A_detected, B_detected
    while True:
        if frame_processed is None:
            continue

        frame = frame_processed.copy()
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
            valid_circles = valid_circles[:1]  # the upper limit of circle number is 2

            for (x, y, r) in valid_circles:
                midpoint = np.array([x, y], dtype=np.float32)
                if len(trackers) < len(detected_centers) + 1:
                    trackers.append(KalmanTracker())

                # Use Kalman filter
                tracker = trackers[len(detected_centers)]
                corrected_point = tracker.correct(midpoint) # the most accurate position estimate based on both the model and the new measurement
                predicted = tracker.predict()               # predict where the object will be in the next frame (e.g., for smooth motion tracking)
                detected_center = (int(corrected_point[0]), int(corrected_point[1]))

                # Determine whether the current detection is A or B based on the last recorded position
                if np.linalg.norm(np.array(detected_center) - np.array(last_detected_centers[0])) < np.linalg.norm(np.array(detected_center) - np.array(last_detected_centers[1])):
                    detected_centers.append(detected_center)
                    A_detected = True  # 
                else:
                    detected_centers.append(detected_center)
                    B_detected = True  # 

        # If fewer than 2 circles are detected, the center of the last detected circle is used to complete the list
        if not A_detected:
            detected_centers.insert(0, last_detected_centers[0])  # append A's center
        if not B_detected:
            detected_centers.append(last_detected_centers[1])  # append B's center


        # Update last detected centers
        last_detected_centers = detected_centers.copy()
        radar_center = detect_radar_center_from_threshold(blurred)
        
        if radar_center:
            # If radar point is detected, calculate the error between radar center and one of the marker centers
            error = calculate_error(detected_centers[0], radar_center)  # Assuming detected_centers[0] is the relevant marker
            print(f"Marker Center: {detected_centers[0]}, Radar Center: {radar_center}, Error: {error}")
            
            # Send the error to the Windows computer
            send2pc(error)

        # Print detected circles and their centers
        print(f"Detected {len(detected_centers)} valid circle(s). Centers:", detected_centers)
        for center in detected_centers:
            cv2.circle(frame, center, 5, (0, 0, 255), -1)



##################################################################
# Initialize Picamera2
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration()
picam2.configure(preview_config)
picam2.start()

# Variables to store trackers and detected centers
trackers = []
detected_centers = []
last_detected_centers = [(0, 0), (0, 0)]  # Record the location of the last two detected circle centers
frame_processed = None
A_detected = False  # Whether A is detected
B_detected = False  # Whether B is detected

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