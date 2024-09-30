import cv2
import numpy as np
from picamera2 import Picamera2

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error_x = 0
        self.prev_error_y = 0
        self.intergral_x = 0
        self.intergral_y = 0
    
    def compute(self, error_x, error_y, dt):
        self.intergral_x += error_x * dt
        self.intergral_y += error_y * dt

        derivative_x = (error_x - self.prev_error_x) / dt
        derivative_y = (error_y - self.prev_error_y) / dt

        control_x = self.kp * error_x + self.ki * self.intergral_x + self.kd * derivative_x
        control_y = self.kp * error_y + self.ki * self.intergral_y + self.kd * derivative_y

        self.prev_error_x = error_x
        self.prev_error_y = error_y

        return control_x, control_y

def is_triangle(contour):
    epsilon = 0.02 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    return len(approx) == 3

def triangle_similarity(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return float('inf')
    return area / (0.5 * perimeter * perimeter)

def convert_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3,3), 0)
    edges = cv2.Canny(blurred, 30, 100)
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours  

def get_triangle_center(contour):
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return np.array([cX, cY])
    return None

def TriangleMarker_position(contours):
    triangles = []
    for contour in contours:
        if is_triangle(contour):
            area = cv2.contourArea(contour)
            if area > 500: # Filter out small contours
                similarity = triangle_similarity(contour)
                center = get_triangle_center(contour)
                if center is not None:
                    triangles.append((similarity, center))

    triangles = sorted(triangles, key=lambda x: x[0])[:1]   # [(simlarity, (x, y))]

    if len(triangles) == 1:
        Center = triangles[0][1]
        return Center
    else:
        return None

if __name__ == "__main__":
    # Initialize Picamera2
    picam2 = Picamera2()
    preview_config = picam2.create_preview_configuration()
    picam2.configure(preview_config)
    picam2.start()
    frame = picam2.capture_array()                                            # (height, width)

    target_position = np.array([frame.shape[1] // 2, frame.shape[0] // 2])    # (0, 0) at top-left corner

    while True:
        frame = picam2.capture_array()
        contours = convert_frame(frame)
        TriMarker_center = TriangleMarker_position(contours)
        
        if TriMarker_center is not None:
            delta = target_position - TriMarker_center
            print(f"Delta Distance: [{delta[0]}, {delta[1]}]")
            cv2.circle(frame, tuple(TriMarker_center), 3, (255, 0, 0), -1)

        cv2.imshow("Detected Triangle", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    picam2.stop()
    cv2.destroyAllWindows()