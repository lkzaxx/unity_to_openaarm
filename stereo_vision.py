"""
IMX219-83 Stereo Vision Test
支援多種立體視覺顯示模式
"""
import cv2
import http.server
import socketserver
import numpy as np
from urllib.parse import urlparse, parse_qs
PORT = 8080
# 相機初始化
pipeline0 = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=640,height=480,framerate=20/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
pipeline1 = "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM),width=640,height=480,framerate=20/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
cap0 = cv2.VideoCapture(pipeline0, cv2.CAP_GSTREAMER)
cap1 = cv2.VideoCapture(pipeline1, cv2.CAP_GSTREAMER)
if not cap0.isOpened() or not cap1.isOpened():
    print("無法開啟相機")
    exit(1)
print("雙相機已開啟")
def create_anaglyph(left, right):
    """紅藍 3D 效果（需要紅藍眼鏡）"""
    # 左眼 = 紅色通道，右眼 = 藍綠通道
    anaglyph = np.zeros_like(left)
    anaglyph[:,:,2] = left[:,:,2]  # Red from left
    anaglyph[:,:,0] = right[:,:,0]  # Blue from right
    anaglyph[:,:,1] = right[:,:,1]  # Green from right
    return anaglyph
def create_sbs(left, right):
    """左右並排（適合 VR 眼鏡）"""
    return np.hstack((left, right))
def create_sbs_half(left, right):
    """左右並排（壓縮寬度，適合手機 VR）"""
    h, w = left.shape[:2]
    left_half = cv2.resize(left, (w//2, h))
    right_half = cv2.resize(right, (w//2, h))
    return np.hstack((left_half, right_half))
def create_over_under(left, right):
    """上下排列"""
    return np.vstack((left, right))
def create_interlaced(left, right):
    """交錯模式（需要特殊顯示器）"""
    h, w = left.shape[:2]
    interlaced = np.zeros_like(left)
    interlaced[0::2] = left[0::2]  # 偶數行 = 左眼
    interlaced[1::2] = right[1::2]  # 奇數行 = 右眼
    return interlaced
def create_difference(left, right):
    """差異圖（用於調試對齊）"""
    gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(gray_l, gray_r)
    return cv2.cvtColor(diff, cv2.COLOR_GRAY2BGR)
class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # 關閉日誌
    
    def do_GET(self):
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        mode = query.get('mode', ['sbs'])[0]
        
        if parsed.path == '/':
            # 主頁：顯示模式選單
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>IMX219-83 Stereo Vision</title>
                <meta name="viewport" content="width=device-width, initial-scale=1">
                <style>
                    body { font-family: Arial; background: #1a1a2e; color: #eee; padding: 20px; }
                    h1 { color: #00d9ff; }
                    .modes { display: flex; flex-wrap: wrap; gap: 10px; margin: 20px 0; }
                    a { background: #16213e; color: #00d9ff; padding: 15px 25px; text-decoration: none; border-radius: 8px; }
                    a:hover { background: #0f3460; }
                    img { max-width: 100%; border: 2px solid #00d9ff; border-radius: 8px; }
                    .current { border: 3px solid #00ff88; }
                </style>
            </head>
            <body>
                <h1>🎥 IMX219-83 Stereo Vision</h1>
                <div class="modes">
                    <a href="/stream?mode=sbs">👓 左右並排 (VR)</a>
                    <a href="/stream?mode=sbs_half">📱 VR 手機模式</a>
                    <a href="/stream?mode=anaglyph">🔴🔵 紅藍 3D</a>
                    <a href="/stream?mode=over_under">⬆️⬇️ 上下排列</a>
                    <a href="/stream?mode=left">👁️ 僅左眼</a>
                    <a href="/stream?mode=right">👁️ 僅右眼</a>
                    <a href="/stream?mode=diff">🔍 差異圖</a>
                </div>
                <p>選擇一個模式開始觀看</p>
            </body>
            </html>
            """
            self.wfile.write(html.encode())
            
        elif parsed.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            try:
                while True:
                    ret0, left = cap0.read()
                    ret1, right = cap1.read()
                    
                    if not ret0 or not ret1:
                        continue
                    
                    # 根據模式生成輸出
                    if mode == 'sbs':
                        output = create_sbs(left, right)
                    elif mode == 'sbs_half':
                        output = create_sbs_half(left, right)
                    elif mode == 'anaglyph':
                        output = create_anaglyph(left, right)
                    elif mode == 'over_under':
                        output = create_over_under(left, right)
                    elif mode == 'interlaced':
                        output = create_interlaced(left, right)
                    elif mode == 'left':
                        output = left
                    elif mode == 'right':
                        output = right
                    elif mode == 'diff':
                        output = create_difference(left, right)
                    else:
                        output = create_sbs(left, right)
                    
                    _, jpeg = cv2.imencode('.jpg', output, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(jpeg.tobytes())
                    self.wfile.write(b'\r\n')
            except:
                pass
print(f"""
╔══════════════════════════════════════════════╗
║     IMX219-83 Stereo Vision Server          ║
╠══════════════════════════════════════════════╣
║  主頁：http://192.168.0.15:{PORT}/              ║
║                                              ║
║  可用模式：                                   ║
║    /stream?mode=sbs       左右並排 (VR)      ║
║    /stream?mode=sbs_half  VR 手機模式        ║
║    /stream?mode=anaglyph  紅藍 3D 眼鏡       ║
║    /stream?mode=left      僅左眼             ║
║    /stream?mode=right     僅右眼             ║
║    /stream?mode=diff      差異圖（調試用）    ║
╚══════════════════════════════════════════════╝
""")
with socketserver.TCPServer(("", PORT), Handler) as httpd:
    httpd.serve_forever()