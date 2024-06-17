import cv2
import numpy as np
import math

# Initialize the parameters
H, W = 200, 400     # height and width of shootable area
l = 150             # distance between object and laser (mirror)
c = 50              # distance between camera and mirror
d = 50              # depth between camera and mirror

def phi(x_rel, frame_shape):
    x = W * (x_rel / (frame_shape[1] / 2))
    # return math.atan((c - x) / l)
    return math.degrees(math.atan((c - x) / l))

def theta(y_rel, frame_shape):
    y = H * (y_rel / (frame_shape[0] / 2))
    # return math.atan(y / l)
    return math.degrees(math.atan(y / l))


# Initialize the camera
cap = cv2.VideoCapture(0)

# Set Lucas-Kanade optical flow parameters
lk_params = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Get the first frame
ret, old_frame = cap.read()
if not ret:
    print("Unable to get the initial frame from the camera")
    cap.release()
    cv2.destroyAllWindows()
    exit()

old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)

# Manually select the initial tracking point
cv2.imshow("Select Tracking Point", old_frame)
roi = cv2.selectROI("Select Tracking Point", old_frame, fromCenter=False, showCrosshair=True)
cv2.destroyWindow("Select Tracking Point")

# Calculate the center of the initial point
initial_point = (int(roi[0] + roi[2] / 2), int(roi[1] + roi[3] / 2))

# Initialize the tracking point
p0 = np.array([[initial_point]], np.float32)

# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)

# Get the image center
frame_center = (old_frame.shape[1] // 2, old_frame.shape[0] // 2)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Unable to get the frame from the camera")
        break
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

    # Select good tracking points
    if p1 is not None and len(p1[st == 1]) > 0:
        good_new = p1[st == 1]
        good_old = p0[st == 1]


        # Draw the tracks
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            # mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)  # Line to display the tracking trajectory
            frame = cv2.circle(frame, (int(a), int(b)), 5, (0, 255, 0), -1)

            x_rel = a - frame_center[0]
            y_rel = frame_center[1] - b

            phi_value = phi(x_rel, old_frame.shape)
            theta_value = theta(y_rel, old_frame.shape)

            print(f"phi={phi_value}, theta={theta_value}")

        img = cv2.add(frame, mask)

        # Update the previous frame and previous points
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1, 1, 2)
    else:
        print("Optical flow calculation failed, reinitializing tracking points")
        p0 = np.array([[initial_point]], np.float32)
        mask = np.zeros_like(frame)

    cv2.imshow('Frame', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
