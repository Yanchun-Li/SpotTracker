import cv2
import numpy as np
import math

# Parameter definitions
H, W = 300, 400     # height and width of the shootable area
l = 150             # distance between the object and the laser (mirror)
c = 50              # distance between the camera and the mirror
d = 50              # depth between the camera and the mirror

def phi(x_rel, frame_shape):
    x = W * (x_rel / (frame_shape[1] / 2))
    return math.degrees(math.atan((c - x) / l))

def theta(y_rel, frame_shape):
    y = H * (y_rel / (frame_shape[0] / 2))
    return math.degrees(math.atan(y / l))

def calculate_tracking_points(points):
    # Calculate the trisection points on the middle vertical line and the midpoint with the left edge
    top_mid = points[1]
    bottom_mid = points[4]
    right_mid = ((top_mid[0] + bottom_mid[0]) // 2, (top_mid[1] + bottom_mid[1]) // 2)

    left_up = points[0]
    left_bottom = points[5]
    left_middle_mid = ((left_bottom[0] + left_up[0] + right_mid[0]) // 2, (left_up[1] + left_bottom[1]) // 2)

    return [right_mid, left_middle_mid]

def get_phi_theta():
    # Initialize variables
    points = []
    tracking_points = []
    selected_point = 0
    fixed_frame = None

    # Mouse callback function
    def draw_points(event, x, y, flags, param):
        nonlocal points, fixed_frame

        if event == cv2.EVENT_LBUTTONDOWN:
            if len(points) < 6:
                points.append((x, y))
                if len(points) > 1:
                    cv2.line(fixed_frame, points[-2], points[-1], (255, 0, 0), 2)
                if len(points) == 6:
                    cv2.line(fixed_frame, points[-1], points[0], (255, 0, 0), 2)
                    nonlocal tracking_points
                    tracking_points = calculate_tracking_points(points)

    # Initialize the camera
    cap = cv2.VideoCapture(0)

    # Capture a frame and fix it
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Unable to get the frame from the camera")
            break

        cv2.imshow('Frame', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            fixed_frame = frame.copy()
            break

    cv2.imshow('Fixed Frame', fixed_frame)
    cv2.setMouseCallback('Fixed Frame', draw_points)

    # Wait for the user to complete the hexagon drawing
    while len(points) < 6:
        cv2.imshow('Fixed Frame', fixed_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            return

    # Close the fixed frame window
    cv2.destroyWindow('Fixed Frame')

    # Initialize optical flow tracking parameters
    old_gray = cv2.cvtColor(fixed_frame, cv2.COLOR_BGR2GRAY)
    p0 = np.array(tracking_points, dtype=np.float32).reshape(-1, 1, 2)

    # Continue processing the real-time video stream
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Unable to get the frame from the camera")
            break

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
                yield phi_value, theta_value

        # Update optical flow tracking parameters
        old_gray = new_gray.copy()
        p0 = p1.reshape(-1, 1, 2)

        cv2.imshow('Frame', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('1'):
            selected_point = 0
        elif key == ord('2'):
            selected_point = 1
        elif key == ord('3'):
            selected_point = 2
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    for phi_value, theta_value in get_phi_theta():
        print(f"phi={phi_value}, theta={theta_value}")
