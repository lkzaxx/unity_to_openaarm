#!/usr/bin/env python3
"""
RealSense D435i Web Viewer
在網頁上即時顯示 RGB / Depth / RGB+Depth 疊合 / 點雲俯視圖

啟動: python3 ~/realsense/realsense_web_viewer.py
瀏覽: http://192.168.0.15:8081/
"""
import pyrealsense2 as rs
import cv2
import numpy as np
import http.server
import socketserver
import threading
import time
import json
from urllib.parse import urlparse, parse_qs

PORT = 8081

# ── RealSense Pipeline ──────────────────────────────────────
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 嘗試啟用 IMU
imu_enabled = False
try:
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    imu_enabled = True
except:
    pass

align = rs.align(rs.stream.color)
colorizer = rs.colorizer()
colorizer.set_option(rs.option.color_scheme, 0)  # Jet

profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# 確認 IMU 是否真的啟動
if imu_enabled:
    try:
        for _ in range(5):
            f = pipeline.wait_for_frames(1000)
            if f.first_or_default(rs.stream.accel):
                break
        else:
            imu_enabled = False
    except:
        imu_enabled = False

print(f"Depth scale: {depth_scale:.6f} m/unit")
print(f"IMU: {'啟用' if imu_enabled else '未啟用'}")

# 穩定串流
for _ in range(30):
    pipeline.wait_for_frames()

# ── 共享狀態 ────────────────────────────────────────────────
lock = threading.Lock()
latest = {
    "color": None,
    "depth_colormap": None,
    "depth_raw": None,
    "accel": (0, 0, 0),
    "gyro": (0, 0, 0),
    "center_depth_m": 0,
    "fps": 0,
}


def capture_loop():
    """背景執行緒持續擷取影像"""
    frame_count = 0
    fps_start = time.time()

    while True:
        try:
            frames = pipeline.wait_for_frames(1000)
        except:
            continue

        aligned = align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_color = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        cy, cx = depth_image.shape[0] // 2, depth_image.shape[1] // 2
        center_d = depth_image[cy, cx] * depth_scale

        accel = (0, 0, 0)
        gyro = (0, 0, 0)
        if imu_enabled:
            af = frames.first_or_default(rs.stream.accel)
            gf = frames.first_or_default(rs.stream.gyro)
            if af:
                a = af.as_motion_frame().get_motion_data()
                accel = (a.x, a.y, a.z)
            if gf:
                g = gf.as_motion_frame().get_motion_data()
                gyro = (g.x, g.y, g.z)

        frame_count += 1
        elapsed = time.time() - fps_start
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            fps_start = time.time()
        else:
            fps = latest["fps"]

        with lock:
            latest["color"] = color_image
            latest["depth_colormap"] = depth_color
            latest["depth_raw"] = depth_image
            latest["accel"] = accel
            latest["gyro"] = gyro
            latest["center_depth_m"] = center_d
            latest["fps"] = fps


# ── 影像生成 ────────────────────────────────────────────────
def render_frame(mode):
    with lock:
        color = latest["color"]
        depth_cm = latest["depth_colormap"]
        depth_raw = latest["depth_raw"]
        accel = latest["accel"]
        gyro = latest["gyro"]
        center_d = latest["center_depth_m"]
        fps = latest["fps"]

    if color is None:
        return None

    if mode == "color":
        output = color.copy()
    elif mode == "depth":
        output = depth_cm.copy()
    elif mode == "overlay":
        output = cv2.addWeighted(color, 0.6, depth_cm, 0.4, 0)
    elif mode == "sbs":
        output = np.hstack((color, depth_cm))
    else:
        output = color.copy()

    # 在畫面上疊加資訊
    h, w = output.shape[:2]
    # 中心十字標記 (只在非 sbs 模式)
    if mode != "sbs":
        cy, cx = h // 2, w // 2
        cv2.drawMarker(output, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
        cv2.putText(output, f"{center_d:.2f}m", (cx + 15, cy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 資訊列
    info = f"FPS:{fps:.0f} | Depth:{center_d:.2f}m"
    if imu_enabled:
        info += f" | Accel:({accel[0]:+.1f},{accel[1]:+.1f},{accel[2]:+.1f})"
    cv2.putText(output, info, (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    return output


# ── HTTP Handler ────────────────────────────────────────────
class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        mode = query.get("mode", ["color"])[0]

        if parsed.path == "/":
            self._serve_index()
        elif parsed.path == "/stream":
            self._serve_stream(mode)
        elif parsed.path == "/snapshot":
            self._serve_snapshot(mode)
        elif parsed.path == "/imu":
            self._serve_imu()
        else:
            self.send_error(404)

    def _serve_index(self):
        imu_section = ""
        if imu_enabled:
            imu_section = """
            <div class="imu-panel" id="imu">
              <h3>IMU</h3>
              <div>Accel: <span id="accel">-</span></div>
              <div>Gyro: <span id="gyro">-</span></div>
              <div>Depth: <span id="depth">-</span></div>
            </div>
            <script>
            setInterval(()=>{
              fetch('/imu').then(r=>r.json()).then(d=>{
                document.getElementById('accel').textContent=
                  `x:${d.accel[0].toFixed(2)} y:${d.accel[1].toFixed(2)} z:${d.accel[2].toFixed(2)} m/s²`;
                document.getElementById('gyro').textContent=
                  `x:${d.gyro[0].toFixed(3)} y:${d.gyro[1].toFixed(3)} z:${d.gyro[2].toFixed(3)} rad/s`;
                document.getElementById('depth').textContent=d.center_depth.toFixed(3)+' m';
              });
            }, 200);
            </script>
            """

        html = f"""<!DOCTYPE html>
<html>
<head>
  <title>RealSense D435i Viewer</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * {{ margin:0; padding:0; box-sizing:border-box; }}
    body {{ font-family:Arial,sans-serif; background:#111; color:#eee; padding:16px; }}
    h1 {{ color:#0cf; margin-bottom:12px; font-size:1.4em; }}
    .modes {{ display:flex; flex-wrap:wrap; gap:8px; margin-bottom:12px; }}
    .modes a {{
      background:#1a2a3a; color:#0cf; padding:10px 18px;
      text-decoration:none; border-radius:6px; border:1px solid #234;
    }}
    .modes a:hover, .modes a.active {{ background:#0cf; color:#111; }}
    #viewer {{ width:100%; max-width:1280px; border:2px solid #0cf; border-radius:6px; }}
    .imu-panel {{
      margin-top:12px; padding:12px; background:#1a2a3a;
      border-radius:6px; font-family:monospace; font-size:0.9em;
      display:inline-block;
    }}
    .imu-panel h3 {{ color:#0cf; margin-bottom:6px; }}
  </style>
</head>
<body>
  <h1>RealSense D435i Web Viewer</h1>
  <div class="modes">
    <a href="#" onclick="setMode('color')">RGB</a>
    <a href="#" onclick="setMode('depth')">Depth</a>
    <a href="#" onclick="setMode('overlay')">RGB+Depth</a>
    <a href="#" onclick="setMode('sbs')">Side-by-Side</a>
    <a href="/snapshot?mode=color" target="_blank">Snapshot</a>
  </div>
  <img id="viewer" src="/stream?mode=color">
  {imu_section}
  <script>
    function setMode(m) {{
      document.getElementById('viewer').src = '/stream?mode=' + m + '&t=' + Date.now();
    }}
  </script>
</body>
</html>"""
        self.send_response(200)
        self.send_header("Content-type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(html.encode())

    def _serve_stream(self, mode):
        self.send_response(200)
        self.send_header("Content-type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        try:
            while True:
                frame = render_frame(mode)
                if frame is None:
                    time.sleep(0.03)
                    continue
                _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                self.wfile.write(jpeg.tobytes())
                self.wfile.write(b"\r\n")
                time.sleep(0.033)  # ~30fps cap
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _serve_snapshot(self, mode):
        frame = render_frame(mode)
        if frame is None:
            self.send_error(503, "No frame")
            return
        _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        self.send_response(200)
        self.send_header("Content-type", "image/jpeg")
        self.send_header("Content-Disposition", f"inline; filename=realsense_{mode}.jpg")
        self.end_headers()
        self.wfile.write(jpeg.tobytes())

    def _serve_imu(self):
        with lock:
            data = {
                "accel": list(latest["accel"]),
                "gyro": list(latest["gyro"]),
                "center_depth": latest["center_depth_m"],
                "fps": latest["fps"],
            }
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())


# ── Main ────────────────────────────────────────────────────
if __name__ == "__main__":
    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()

    print(f"""
╔══════════════════════════════════════════════╗
║      RealSense D435i Web Viewer             ║
╠══════════════════════════════════════════════╣
║  http://192.168.0.15:{PORT}/                    ║
║                                              ║
║  模式:                                       ║
║    /stream?mode=color    RGB 彩色            ║
║    /stream?mode=depth    深度圖              ║
║    /stream?mode=overlay  RGB+Depth 疊合      ║
║    /stream?mode=sbs      左右並排            ║
║    /snapshot?mode=color  高畫質截圖          ║
║    /imu                  IMU JSON            ║
║                                              ║
║  IMU: {"啟用" if imu_enabled else "未啟用"}                                    ║
╚══════════════════════════════════════════════╝
""")

    server = socketserver.ThreadingTCPServer(("", PORT), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        server.server_close()
        print("\n已停止")
