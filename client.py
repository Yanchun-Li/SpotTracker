import socket
import numpy as np
import cv2
import struct

HOST = "192.168.100.29"  # 替换为你的 Raspberry Pi 的有线 IP 地址
PORT = 5001

# 创建 socket 连接
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

data = b""
payload_size = struct.calcsize(">L")

while True:
    while len(data) < payload_size:
        packet = sock.recv(4096)
        if not packet: break
        data += packet

    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack(">L", packed_msg_size)[0]

    while len(data) < msg_size:
        data += sock.recv(4096)

    frame_data = data[:msg_size]
    data = data[msg_size:]

    frame = np.frombuffer(frame_data, dtype=np.uint8)
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

    # 显示视频帧
    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

sock.close()
cv2.destroyAllWindows()

