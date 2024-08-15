import cv2
import numpy as np
from picamera2 import Picamera2

from Raspi_Marker_withoutKalmanFilter import convert_frame, CirMarkerCenter_Contour, display, mirrorAngles
from Raspi_MoveStage import TriangleMarker_position

# Initialize Picamera2
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration()
picam2.configure(preview_config)
picam2.start()

selected_point = 0  # Initialize selected_point

while True:
    frame = picam2.capture_array()
    contours = convert_frame(contours)

    ArmMarker_centers = CirMarkerCenter_Contour(contours)       # center of markers on arms
    last_ArmMarker_centers = ArmMarker_centers.copy()           # update marker centers
    mirror_phi, mirror_theta = mirrorAngles(ArmMarker_centers)  # mirror rotation angle

    tri_x, tri_y = TriangleMarker_position(contours)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()