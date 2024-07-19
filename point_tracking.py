import cv2
import numpy as np
import math

def point_tracker(frame, points, selected_point=0):
    H, W = 300, 400
    l, c = 150, 50

    def phi(x_rel, frame_shape):
        x = W * (x_rel / (frame_shape[1] / 2))
        return math.degrees(math.atan((c - x) / l))

    def theta(y_rel, frame_shape):
        y = H * (y_rel / (frame_shape[0] / 2))
        return math.degrees(math.atan(y / l))

    tracking_points = calculate_tracking_points(points)
    old_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    p0 = np.array(tracking_points, dtype=np.float32).reshape(-1, 1, 2)

    ret, frame = cap.read()
    if not ret:
        print("Unable to get the frame from the camera")
        return

    # Optical flow tracking
    new_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, new_gray, p0, None)
    for i, (new, old) in enumerate(zip(p1, p0)):
        a, b = new.ravel()
        a, b = int(a), int(b)
        color = (0, 0, 255) if i == selected_point else (0, 255, 0)
        cv2.circle(frame, (a, b), 5, color, -1)
        if i == selected_point:
            x_rel = a - frame.shape[1] // 2
            y_rel = frame.shape[0] // 2 - b
            phi_value = phi(x_rel, frame.shape)
            theta_value = theta(y_rel, frame.shape)
            print(f"Selected Point: {i + 1}, phi={phi_value}, theta={theta_value}")

    return frame

def calculate_tracking_points(points):
    top_mid = points[1]
    bottom_mid = points[4]
    right_mid = ((top_mid[0] + bottom_mid[0]) // 2, (top_mid[1] + bottom_mid[1]) // 2)

    left_up = points[0]
    left_bottom = points[5]
    left_middle_mid = ((left_bottom[0] + left_up[0] + right_mid[0]) // 2, (left_up[1] + left_bottom[1]) // 2)

    return [right_mid, left_middle_mid]
