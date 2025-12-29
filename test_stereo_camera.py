#!/usr/bin/env python3
import cv2
import http.server
import socketserver
import numpy as np
PORT = 8080
# 開啟兩顆相機
pipeline0 = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
pipeline1 = "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
cap0 = cv2.VideoCapture(pipeline0, cv2.CAP_GSTREAMER)
cap1 = cv2.VideoCapture(pipeline1, cv2.CAP_GSTREAMER)
if not cap0.isOpened() or not cap1.isOpened():
    print("無法開啟相機")
    exit(1)
print("雙相機已開啟，正在啟動伺服器...")
class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    ret0, frame0 = cap0.read()
                    ret1, frame1 = cap1.read()
                    if ret0 and ret1:
                        # 左右並排顯示
                        combined = np.hstack((frame0, frame1))
                        _, jpeg = cv2.imencode('.jpg', combined)
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                        self.wfile.write(jpeg.tobytes())
                        self.wfile.write(b'\r\n')
            except:
                pass
print(f"雙鏡頭 MJPEG 串流: http://192.168.0.15:{PORT}/")
with socketserver.TCPServer(("", PORT), Handler) as httpd:
    httpd.serve_forever()
