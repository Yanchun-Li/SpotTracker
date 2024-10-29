import cv2
import numpy as np
from picamera2 import Picamera2

# ��ʼ�� Picamera2
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration()
picam2.configure(preview_config)
picam2.start()

while True:
    # ����ͼ�񣨲��ı�ԭʼframe�ṹ��
    frame = picam2.capture_array()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 1. �ҵ��������ֵ��������ֵ������
    minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(gray)
    print(f"�������ֵ: {maxVal}��λ��: {maxLoc}")

    # 2. ��ֵ��ͼ��ֻ���������Ĳ���
    _, threshold = cv2.threshold(gray, maxVal - 10, 255, cv2.THRESH_BINARY)

    # 3. ʹ����̬ѧ����ȥ������
    kernel = np.ones((3, 3), np.uint8)
    cleaned = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel, iterations=2)

    # 4. ��������
    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # ��ʼ����ߵ�����
    laser_center = None

    # �����������ҵ������������
    max_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                laser_center = (cx, cy)

    # ����ҵ�������ģ�������λ��
    if laser_center:
        cx, cy = laser_center
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)  # ��������
        print(f"�������: ({cx}, {cy}), ���: {max_area}")

    # ��ʾ������Ľ��
    cv2.namedWindow("Thresholded Image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Thresholded Image", 640, 480)
    cv2.namedWindow("Detected Laser Spot", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Detected Laser Spot", 640, 480)

    cv2.imshow("Thresholded Image", cleaned)  # ��ʾ������Ķ�ֵͼ
    cv2.imshow("Detected Laser Spot", frame)  # ��ʾ�����

    # �� 'q' ���˳�
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# �ر���Դ
picam2.stop()
cv2.destroyAllWindows()
