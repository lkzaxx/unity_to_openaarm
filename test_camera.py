#!/usr/bin/env python3
import cv2
import http.server
import socketserver
PORT = 8080
pipeline = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("無法開啟相機")
    exit(1)
print("相機已開啟，正在啟動伺服器...")
class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    ret, frame = cap.read()
                    if ret:
                        _, jpeg = cv2.imencode('.jpg', frame)
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                        self.wfile.write(jpeg.tobytes())
                        self.wfile.write(b'\r\n')
            except:
                pass
print(f"MJPEG 串流: http://192.168.0.15:{PORT}/")
with socketserver.TCPServer(("", PORT), Handler) as httpd:
    httpd.serve_forever()
