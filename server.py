import socketserver
import cv2
import sys

HOST = "192.168.100.13"  # ここはRaspberryPiのIPアドレスを入力
PORT = 5569


class TCPHandler(socketserver.BaseRequestHandler):
    videoCap = ''

    def handle(self):
        ret, frame = videoCap.read()
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 100]  # この値100は映像の質の値
        jpegs_byte = cv2.imencode('.jpeg', frame, encode_param)[1]
        self.request.send(jpegs_byte)


videoCap = cv2.VideoCapture(0)
socketserver.TCPServer.allow_reuse_address = True
server = socketserver.TCPServer((HOST, PORT), TCPHandler)

try:
    server.serve_forever()
except KeyboardInterrupt:
    server.shutdown()
    sys.exit()
