import cv2
import socket
import json
import numpy as np
from picamera2 import Picamera2

# Initialize Picamera2
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration()
picam2.configure(preview_config)
picam2.start()

# Initialize selected_point
point_id = 0
marker_num = 1
tolerance = 300
initial_skip_frames = 24
frame_counter = 0

last_ArmMarker_centers = [(0, 0)] if marker_num == 1 else [(0,0), (0,0)]
last_Radar_center = (0, 0)
start_tracking = False

while True:
    frame = picam2.capture_array()
    cv2.imshow("Press 'Enter' to start tracking", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == 13:
        start_tracking = True
        cv2.destroyAllWindows()
        break
    elif key == ord('q'):
        picam2.stop()
        cv2.destroyAllWindows()
        exit()

server_ip = '192.168.100.29'
server_port = 5005

while start_tracking:
    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # adaptive_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    edges = cv2.Canny(blurred, 25, 70)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cv2.namedWindow("grayimage", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("grayimage", 960, 720)
    cv2.imshow("grayimage", edges)
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
    cv2.imshow("contours", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

socket.close()
picam2.stop()
cv2.destroyAllWindows()