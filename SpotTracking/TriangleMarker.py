import cv2
import np as numpy
import threading

def get_triangle_center(contour):
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)
    return None

def is_triangle(contour):
    epsilon = 0.04 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    return len(approx) == 3

def triangle_similarity(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return float('inf')
    return area / (0.5 * perimeter * perimeter)

def distance(pt1, pt2):
    return np.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

# Initialize Kalman filter
kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03

# Open the camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Unable to open the camera")
    exit()

# Record the last updated midpoint coordinates
last_midpoint = None
detected_center = None
frame_processed = None

def process_frame():
    global last_midpoint, detected_center, frame_processed
    while True:
        if frame_processed is None:
            continue

        frame = frame_processed.copy()
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Convert to HSV image for color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Thresholds for the blue color range
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        # Blue mask
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        triangles = []

        for contour in contours:
            if is_triangle(contour):
                area = cv2.contourArea(contour)
                if area > 1000:  # Filter out small contours
                    # Check if the triangle is blue
                    mask_contour = np.zeros_like(mask)
                    cv2.drawContours(mask_contour, [contour], -1, 255, -1)
                    mask_and = cv2.bitwise_and(mask, mask_contour)
                    non_zero = cv2.countNonZero(mask_and)
                    if non_zero > 0.5 * area:  # If more than 50% of the contour area has non-zero pixels in the mask
                        similarity = triangle_similarity(contour)
                        center = get_triangle_center(contour)
                        if center:
                            triangles.append((similarity, center, contour))

        triangles = sorted(triangles, key=lambda x: x[0])[:2]

        if len(triangles) == 2:
            center1 = triangles[0][1]
            center2 = triangles[1][1]
            if distance(center1, center2) > 50:  # Check the distance between the centers of the two triangles, threshold is 50 pixels
                midpoint = np.array([(center1[0] + center2[0]) / 2, (center1[1] + center2[1]) / 2], dtype=np.float32)

                # Use Kalman filter
                kalman.correct(midpoint)
                predicted = kalman.predict()
                detected_center = (int(predicted[0]), int(predicted[1]))

                last_midpoint = detected_center
            else:
                detected_center = last_midpoint
        else:
            if last_midpoint is not None:
                detected_center = last_midpoint
            else:
                detected_center = (0, 0)

        # Output the number of detected triangles and the midpoint
        print(f"Detected {len(triangles)} blue triangle(s). Midpoint of the two triangles:", detected_center)
        for _, center, contour in triangles:
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

# Start the image processing thread
processing_thread = threading.Thread(target=process_frame)
processing_thread.daemon = True
processing_thread.start()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Unable to receive frame (stream end?). Exiting ...")
        break

    frame_processed = frame.copy()

    if detected_center is not None:
        cv2.circle(frame, detected_center, 5, (255, 0, 0), -1)

    cv2.imshow('Detected Triangles', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


######################## without Kalman filter ################################
# import cv2
# import numpy as np
# import threading

# def get_triangle_center(contour):
#     M = cv2.moments(contour)
#     if M["m00"] != 0:
#         cX = int(M["m10"] / M["m00"])
#         cY = int(M["m01"] / M["m00"])
#         return (cX, cY)
#     return None

# def is_triangle(contour):
#     epsilon = 0.04 * cv2.arcLength(contour, True)
#     approx = cv2.approxPolyDP(contour, epsilon, True)
#     return len(approx) == 3

# def triangle_similarity(contour):
#     area = cv2.contourArea(contour)
#     perimeter = cv2.arcLength(contour, True)
#     if perimeter == 0:
#         return float('inf')
#     return area / (0.5 * perimeter * perimeter)

# def distance(pt1, pt2):
#     return np.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)


# # Open the camera
# cap = cv2.VideoCapture(0)

# if not cap.isOpened():
#     print("Unable to open the camera")
#     exit()

# # Record the last updated midpoint coordinates
# last_midpoint = None
# detected_center = None
# frame_processed = None

# def process_frame():
#     global last_midpoint, detected_center, frame_processed
#     while True:
#         if frame_processed is None:
#             continue

#         frame = frame_processed.copy()
#         # Convert to grayscale
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         blurred = cv2.GaussianBlur(gray, (5, 5), 0)
#         edges = cv2.Canny(blurred, 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#         # Convert to HSV image for color detection
#         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#         # Thresholds for the blue color range
#         lower_blue = np.array([100, 150, 50])
#         upper_blue = np.array([140, 255, 255])
#         # Blue mask
#         mask = cv2.inRange(hsv, lower_blue, upper_blue)

#         triangles = []

#         for contour in contours:
#             if is_triangle(contour):
#                 area = cv2.contourArea(contour)
#                 if area > 1000:  # Filter out small contours
#                     # Check if the triangle is blue
#                     mask_contour = np.zeros_like(mask)
#                     cv2.drawContours(mask_contour, [contour], -1, 255, -1)
#                     mask_and = cv2.bitwise_and(mask, mask_contour)
#                     non_zero = cv2.countNonZero(mask_and)
#                     if non_zero > 0.5 * area:  # If more than 50% of the contour area has non-zero pixels in the mask
#                         similarity = triangle_similarity(contour)
#                         center = get_triangle_center(contour)
#                         if center:
#                             triangles.append((similarity, center, contour))

#         triangles = sorted(triangles, key=lambda x: x[0])[:2]

#         if len(triangles) == 2:
#             center1 = triangles[0][1]
#             center2 = triangles[1][1]
#             if distance(center1, center2) > 50:  # Check the distance between the centers of the two triangles, threshold is 50 pixels
#                 midpoint = np.array([(center1[0] + center2[0]) / 2, (center1[1] + center2[1]) / 2], dtype=np.float32)

#                 detected_center = (int(midpoint[0]), int(midpoint[1]))

#                 last_midpoint = detected_center
#             else:
#                 detected_center = last_midpoint
#         else:
#             if last_midpoint is not None:
#                 detected_center = last_midpoint
#             else:
#                 detected_center = (0, 0)

#         for _, _, contour in triangles:
#             cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
#         # Output the number of detected triangles and the midpoint
#         print(f"Detected {len(triangles)} blue triangle(s). Midpoint of the two triangles:", detected_center)

# # Start the image processing thread
# processing_thread = threading.Thread(target=process_frame)
# processing_thread.daemon = True
# processing_thread.start()

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("Unable to receive frame (stream end?). Exiting ...")
#         break

#     frame_processed = frame.copy()

#     if detected_center is not None:
#         cv2.circle(frame, detected_center, 5, (255, 0, 0), -1)

#     cv2.imshow('Detected Triangles', frame)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()
