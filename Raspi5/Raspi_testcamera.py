import cv2
import socket
import json
import numpy as np
from picamera2 import Picamera2

# ��ʼ�� Picamera2
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration()
picam2.configure(preview_config)
picam2.start()

# ��ʼ������
server_ip = '192.168.100.29'
server_port = 5005
start_tracking = False

# ����ʼǰ�Ĵ�����ʾ
while True:
    frame = picam2.capture_array()

    # ��תͼ��180��
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    cv2.imshow("Press 'Enter' to start tracking", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == 13:  # Enter ��
        start_tracking = True
        cv2.destroyAllWindows()
        break
    elif key == ord('q'):  # �� 'q' �˳�
        picam2.stop()
        cv2.destroyAllWindows()
        exit()

while start_tracking:
    frame = picam2.capture_array()

    # ��תͼ��180��
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    # ͼ��Ԥ����
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ��ǿ�Աȶȣ�����Ӧֱ��ͼ���⻯��
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced_gray = clahe.apply(gray)

    # ��ֵ�˲�ȥ��
    blurred = cv2.medianBlur(enhanced_gray, 5)

    # Canny ��Ե���
    edges = cv2.Canny(blurred, 30, 150)

    # ʹ����̬ѧ�������Ӷ��ѱ�Ե
    kernel = np.ones((5, 5), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)

    # ������ HoughCircles ���Բ��
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
        param1=50, param2=30, minRadius=10, maxRadius=100
    )

    # �����⵽Բ�Σ�����Բ��
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

    # ��ʾ�������ͼ��
    cv2.imshow("Enhanced Gray", enhanced_gray)
    cv2.imshow("Edge Image", edges)
    cv2.imshow("Contours and Circles", frame)

    # �ȴ������¼�
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# ֹͣ����͹رմ���
picam2.stop()
cv2.destroyAllWindows()
