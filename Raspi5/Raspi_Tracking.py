import cv2
import numpy as np
from picamera2 import Picamera2

from process import convert_frame, RectMarkerCenter_Contour, detect_radar_center, calculate_error, send2pc, display

# Initialize Picamera2
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration()
picam2.configure(preview_config)
picam2.start()

# Initialize selected_point
point_id = 0
marker_num = 1
tolerance = 600
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

while start_tracking:
    frame = picam2.capture_array()
    contours = convert_frame(frame)

    ArmMarker_centers = RectMarkerCenter_Contour(marker_num, contours, last_ArmMarker_centers)                       # center of markers on arms (2 centers)
    Radar_center = detect_radar_center(contours, last_Radar_center)                                # center of radar point

    if frame_counter < initial_skip_frames:
        frame_counter += 1
        last_ArmMarker_centers = ArmMarker_centers.copy()
        last_Radar_center = Radar_center
    else:
        arm_moved_dist = sum(abs(ArmMarker_centers[i][j] - last_ArmMarker_centers[i][j]) for i in range(marker_num) for j in range(2))
        if len(ArmMarker_centers) > 0 and (arm_moved_dist < tolerance):
            last_ArmMarker_centers = ArmMarker_centers.copy()                           # update marker centers
        rad_moved_dist = sum(abs(Radar_center[i] - last_Radar_center[i]) for i in range(2))
        if len(Radar_center) > 0 and rad_moved_dist < tolerance:
            last_Radar_center = Radar_center

    print(f"ARM{last_ArmMarker_centers}, RADAR{last_Radar_center}")

    key = cv2.waitKey(1) & 0xFF
    if key in  [ord('1'), ord('2')]:
        point_id = key - ord('0') - 1                                               # choose tracking which point via keyboard input
    # error = calculate_error(point_id, last_ArmMarker_centers, Radar_center)     # calculate the error distance (x, y)[pixel]
    # send2pc(error, server_ip='WINDOWS PC IP ADDRESS', server_port=5005)         # send error value to PC

    display(frame, point_id, last_ArmMarker_centers, last_Radar_center)                          # display 2 marker centers and 1 radar centers

    if key == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()