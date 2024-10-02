import cv2
import numpy as np
from picamera2 import Picamera2

from process import convert_frame, CirMarkerCenter_Contour, detect_radar_center, calculate_error, send2pc, display

# Initialize Picamera2
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration()
picam2.configure(preview_config)
picam2.start()

# Initialize selected_point
point_id = 0               

while True:
    frame = picam2.capture_array()
    contours = convert_frame(contours)

    ArmMarker_centers = CirMarkerCenter_Contour(contours)                       # center of markers on arms (2 centers)
    last_ArmMarker_centers = ArmMarker_centers.copy()                           # update marker centers
    Radar_center = detect_radar_center(contours)                                # center of radar point

    key = cv2.waitKey(1) & 0xFF
    point_id = key - ord('0') - 1                                               # choose tracking which point via keyboard input
    error = calculate_error(point_id, last_ArmMarker_centers, Radar_center)     # calculate the error distance (x, y)[pixel]
    send2pc(error, server_ip='WINDOWS PC IP ADDRESS', server_port=5005)         # send error value to PC

    display(point_id, ArmMarker_centers, Radar_center)                          # display 2 marker centers and 1 radar centers

    if key == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()