import cv2
import numpy as np
import math
import mss
import pygetwindow as gw

# Parameter definitions
H, W = 300, 400  # height and width of the shootable area
l = 150  # distance between the object and the laser (mirror)
c = 50  # distance between the camera and the mirror
d = 50  # depth between the camera and the mirror

def phi(x_rel, frame_shape):
    x = W * (x_rel / (frame_shape[1] / 2))
    return math.degrees(math.atan((c - x) / l))

def theta(y_rel, frame_shape):
    y = H * (y_rel / (frame_shape[0] / 2))
    return math.degrees(math.atan(y / l))

# Initialize variables
points = []
tracking_points = []
fixed_frame = None

# Mouse callback function
def draw_points(event, x, y, flags, param):
    global points, fixed_frame

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(points) < 6:
            points.append((x, y))
            if len(points) > 1:
                cv2.line(fixed_frame, points[-2], points[-1], (255, 0, 0), 2)
            if len(points) == 6:
                cv2.line(fixed_frame, points[-1], points[0], (255, 0, 0), 2)
                calculate_tracking_points()

def calculate_tracking_points():
    global tracking_points

    # Calculate the trisection points on the middle vertical line and the midpoint with the left edge
    top_mid = points[0]
    bottom_mid = points[3]
    middle_mid = ((top_mid[0] + bottom_mid[0]) // 2, (top_mid[1] + bottom_mid[1]) // 2)
    
    top_third = ((2 * top_mid[0] + bottom_mid[0]) // 3, (2 * top_mid[1] + bottom_mid[1]) // 3)
    bottom_third = ((top_mid[0] + 2 * bottom_mid[0]) // 3, (top_mid[1] + 2 * bottom_mid[1]) // 3)
    
    left_mid = points[1]
    left_middle_mid = ((left_mid[0] + middle_mid[0]) // 2, (left_mid[1] + middle_mid[1]) // 2)

    tracking_points = [top_third, bottom_third, left_middle_mid]

# Function to capture the screen
def capture_screen(bbox=None):
    with mss.mss() as sct:
        screenshot = sct.grab(bbox) if bbox else sct.grab(sct.monitors[1])
        img = np.array(screenshot)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img

# Function to get the bounding box of Achcam Camera Measure 3.0 window
def get_achcam_window_bbox():
    windows = gw.getWindowsWithTitle('Achcam Camera Measure 3.0')
    if not windows:
        raise Exception("Achcam Camera Measure 3.0 window not found")
    window = windows[0]
    return (window.left, window.top, window.left + window.width, window.top + window.height)

def main():
    global selected_point  # Ensure selected_point is referenced correctly
    selected_point = 0  # Initialize selected_point
    
    # Get the bounding box of Achcam Camera Measure 3.0 window
    try:
        bbox = get_achcam_window_bbox()
        print(f"Detected Achcam Camera Measure 3.0 window bbox: {bbox}")
    except Exception as e:
        print(e)
        return
    
    # Manually adjust the bbox to match the window more accurately
    # These values may need to be adjusted based on actual observation
    left_adjustment = 10
    top_adjustment = 10
    right_adjustment = -10
    bottom_adjustment = -10
    bbox = (bbox[0] + left_adjustment, bbox[1] + top_adjustment, bbox[2] + right_adjustment, bbox[3] + bottom_adjustment)
    print(f"Adjusted bbox for screen capture: {bbox}")

    # Capture a fixed frame from the Achcam Camera Measure 3.0 window
    while True:
        frame = capture_screen(bbox)
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
            cv2.destroyAllWindows()
            return

    # Close the fixed frame window
    cv2.destroyWindow('Fixed Frame')

    # Initialize optical flow tracking parameters
    old_gray = cv2.cvtColor(fixed_frame, cv2.COLOR_BGR2GRAY)
    p0 = np.array(tracking_points, dtype=np.float32).reshape(-1, 1, 2)

    # Continue processing the real-time video stream
    while True:
        frame = capture_screen(bbox)
        new_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Optical flow tracking
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

    # Release the resources
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
