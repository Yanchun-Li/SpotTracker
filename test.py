import cv2

def list_available_cameras(max_tested=10):
    available_cameras = []
    for index in range(max_tested):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            available_cameras.append(index)
            cap.release()
    return available_cameras

# 列出最多10个可用摄像头
available_cameras = list_available_cameras()

print("Available camera indices:", available_cameras)

if available_cameras:
    print(f"Number of available cameras: {len(available_cameras)}")
    for cam_index in available_cameras:
        cap = cv2.VideoCapture(cam_index)
        ret, frame = cap.read()
        if ret:
            cv2.imshow(f'Camera {cam_index}', frame)
        cap.release()
else:
    print("No cameras available")

cv2.waitKey(0)
cv2.destroyAllWindows()
