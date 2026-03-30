#!/usr/bin/env python3
"""
RealSense D435i + YOLO Web Viewer
基於 realsense_web_viewer.py，新增 YOLO 追蹤 + 計數模式

啟動: python3 ~/ros2_ws/yolo_model/realsense_yolo_viewer.py
瀏覽: http://192.168.0.15:8081/

新增模式:
  - yolo:  RGB + YOLO 偵測框 + ByteTrack ID + 計數線
  - yolo_depth: YOLO + Depth 疊合
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

# ── YOLO 設定 ───────────────────────────────────────────────
YOLO_MODEL = "/home/idaka/ros2_ws/yolo_model/ehand_16kpt_sam_v4.engine"
CAN_MODEL = "/home/idaka/ros2_ws/yolo_model/can_pose_v1_best.engine"
# YOLO_MODEL = "/home/idaka/ros2_ws/yolo_model/ehand_16kpt_sam_v4.engine"
CAN_MODEL = "/home/idaka/ros2_ws/yolo_model/can_pose_v1_best.engine"
CONF_THRESHOLD = 0.5
TRACKER = "botsort.yaml"  # ByteTrack 跟 pose 有相容性問題，改用 BoT-SORT

# Pose 關鍵點設定 - 16 keypoints for eHand
KP_NAMES = ["palm", "th_mcp", "th_ip", "th_tip",
            "ix_mcp", "ix_pip", "ix_tip", "md_mcp", "md_pip", "md_tip",
            "rg_mcp", "rg_pip", "rg_tip", "pk_mcp", "pk_pip", "pk_tip"]
KP_COLORS = [
    (255, 255, 255),  # palm - white
    (68, 68, 255), (34, 34, 255), (0, 0, 255),        # thumb
    (102, 255, 102), (68, 255, 68), (0, 255, 0),      # index
    (102, 102, 255), (68, 68, 255), (0, 0, 255),      # middle
    (102, 255, 255), (0, 255, 255), (0, 204, 204),    # ring
    (255, 102, 255), (255, 68, 255), (255, 0, 255),   # pinky
]
KP_SKELETON = [
    (0, 1, (100, 100, 255)),  (1, 2, (100, 100, 255)),  (2, 3, (0, 0, 255)),
    (0, 4, (100, 255, 100)),  (4, 5, (100, 255, 100)),  (5, 6, (0, 255, 0)),
    (0, 7, (255, 100, 100)),  (7, 8, (255, 100, 100)),  (8, 9, (255, 0, 0)),
    (0, 10, (100, 255, 255)), (10, 11, (100, 255, 255)), (11, 12, (0, 255, 255)),
    (0, 13, (255, 100, 255)), (13, 14, (255, 100, 255)), (14, 15, (255, 0, 255)),
]

# 計數線
LINE_Y_RATIO = 0.5    # 畫面高度的比例 (0.5 = 中間)
COUNT_DIRECTION = "down"  # down / up / both

# ── RealSense Pipeline ──────────────────────────────────────
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

imu_enabled = False
try:
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    imu_enabled = True
except:
    pass

align = rs.align(rs.stream.color)
colorizer = rs.colorizer()
colorizer.set_option(rs.option.color_scheme, 0)

profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

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

for _ in range(30):
    pipeline.wait_for_frames()

# ── YOLO 載入 ───────────────────────────────────────────────
print(f"Loading eHand model: {YOLO_MODEL}")
from ultralytics import YOLO
yolo_model = YOLO(YOLO_MODEL, task="pose")
print("eHand model loaded!")
# Single model mode - v4 handles both
print("Single model mode: v4 handles eHand + Can")

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

# YOLO 追蹤狀態
yolo_lock = threading.Lock()
yolo_state = {
    "annotated": None,       # YOLO 標註過的畫面
    "count": 0,              # 計數
    "tracked_objects": {},    # {id: last_cy}
    "active_ids": [],        # 當前畫面中的 ID 列表
    "detections": 0,         # 當前幀偵測數
    "inference_ms": 0,       # 推論時間
}


def capture_loop():
    """背景執行緒持續擷取影像 + YOLO 追蹤"""
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

        # --- YOLO 追蹤 ---
        h, w = color_image.shape[:2]
        line_y = int(h * LINE_Y_RATIO)

        # Run both models
        try:
            results = yolo_model.track(
                color_image,
                persist=True,
                tracker=TRACKER,
                conf=CONF_THRESHOLD,
                verbose=False,
            )
        except Exception:
            results = yolo_model.predict(
                color_image,
                conf=CONF_THRESHOLD,
                verbose=False,
            )
        # Run can model separately (better accuracy for RealSense angle)
        try:
            can_results = can_model.predict(
                color_image,
                conf=0.87,
                verbose=False,
            )
        except Exception:
            can_results = None

        annotated = color_image.copy()
        active_ids = []
        det_count = 0

        with yolo_lock:
            tracked = yolo_state["tracked_objects"]
            count = yolo_state["count"]

        if results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            ids = results[0].boxes.id.cpu().numpy().astype(int)
            confs = results[0].boxes.conf.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy().astype(int)
            det_count = len(ids)

            for idx, (box, obj_id, conf, obj_cls) in enumerate(zip(boxes, ids, confs, classes)):
                x1, y1, x2, y2 = box.astype(int)
                obj_cy = (y1 + y2) / 2
                active_ids.append(int(obj_id))

                # 越線計數
                if obj_id in tracked:
                    prev_cy = tracked[obj_id]
                    crossed = False
                    if COUNT_DIRECTION == "down":
                        crossed = prev_cy < line_y and obj_cy >= line_y
                    elif COUNT_DIRECTION == "up":
                        crossed = prev_cy > line_y and obj_cy <= line_y
                    elif COUNT_DIRECTION == "both":
                        crossed = (prev_cy < line_y and obj_cy >= line_y) or \
                                  (prev_cy > line_y and obj_cy <= line_y)
                    if crossed:
                        count += 1

                tracked[obj_id] = obj_cy

                # 畫框 - 用不同顏色區分 class
                cls_name = results[0].names.get(obj_cls, "?")
                if obj_cls == 0:  # can
                    color_box = (0, 165, 255)  # orange (BGR)
                elif obj_cls == 1:  # ehand
                    color_box = (0, 255, 0)  # green
                else:
                    color_box = (200, 200, 200)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color_box, 2)

                # 標籤: class + ID + 信心度
                label = f"{cls_name} ID:{obj_id} {conf:.2f}"
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated, (x1, y1 - th - 8), (x1 + tw + 4, y1), color_box, -1)
                cv2.putText(annotated, label, (x1 + 2, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                # 取深度
                obj_cx = int((x1 + x2) / 2)
                obj_cy_int = int(obj_cy)
                if 0 <= obj_cx < w and 0 <= obj_cy_int < h:
                    d = depth_image[obj_cy_int, obj_cx] * depth_scale
                    if d > 0:
                        cv2.putText(annotated, f"{d:.2f}m", (x1, y2 + 18),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # 畫 Pose 關鍵點 + 連線 (根據 class 不同繪製)
            if results[0].keypoints is not None:
                all_kps = results[0].keypoints.xy.cpu().numpy()  # (N, 16, 2)
                all_confs = results[0].keypoints.conf.cpu().numpy() if results[0].keypoints.conf is not None else None

                for obj_idx in range(len(all_kps)):
                    obj_cls_kp = classes[obj_idx] if obj_idx < len(classes) else -1
                    kps = all_kps[obj_idx]  # (16, 2)

                    if obj_cls_kp == 0:
                        # Can: only draw first 3 keypoints (top, center, bottom)
                        can_colors = [(0, 0, 255), (0, 255, 255), (255, 0, 0)]
                        can_names = ["top", "center", "bottom"]
                        can_pts = []
                        for ki in range(min(3, len(kps))):
                            kx, ky = int(kps[ki][0]), int(kps[ki][1])
                            if kx > 0 or ky > 0:
                                cv2.circle(annotated, (kx, ky), 6, can_colors[ki], -1)
                                cv2.circle(annotated, (kx, ky), 8, (255, 255, 255), 1)
                                can_pts.append((kx, ky))
                            else:
                                can_pts.append(None)
                        # Draw line top→center→bottom
                        for i in range(len(can_pts) - 1):
                            if can_pts[i] and can_pts[i+1]:
                                cv2.line(annotated, can_pts[i], can_pts[i+1], (0, 165, 255), 2)
                    else:
                        # eHand: draw all 16 keypoints + skeleton
                        kp_valid = []
                        for ki in range(len(kps)):
                            kx, ky = int(kps[ki][0]), int(kps[ki][1])
                            visible = kx > 0 or ky > 0
                            kp_valid.append(visible)
                            if visible:
                                color = KP_COLORS[ki] if ki < len(KP_COLORS) else (255,255,255)
                                cv2.circle(annotated, (kx, ky), 5, color, -1)
                                cv2.circle(annotated, (kx, ky), 7, (255,255,255), 1)
                        for (si, ei, line_color) in KP_SKELETON:
                            if si < len(kps) and ei < len(kps) and kp_valid[si] and kp_valid[ei]:
                                pt1 = (int(kps[si][0]), int(kps[si][1]))
                                pt2 = (int(kps[ei][0]), int(kps[ei][1]))
                                cv2.line(annotated, pt1, pt2, line_color, 2)

        # 畫 Can 偵測結果 (from can_model)
        if can_results is not None and len(can_results) > 0:
            cr = can_results[0]
            if cr.boxes is not None and len(cr.boxes) > 0:
                for bi in range(len(cr.boxes)):
                    bx = cr.boxes[bi]
                    x1c, y1c, x2c, y2c = bx.xyxy[0].cpu().numpy().astype(int)
                    conf_c = float(bx.conf)
                    cv2.rectangle(annotated, (x1c, y1c), (x2c, y2c), (255, 165, 0), 2)
                    label_c = f"can {conf_c:.2f}"
                    (tw, th), _ = cv2.getTextSize(label_c, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    cv2.rectangle(annotated, (x1c, y1c - th - 8), (x1c + tw + 4, y1c), (255, 165, 0), -1)
                    cv2.putText(annotated, label_c, (x1c + 2, y1c - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                    # Draw can keypoints if available
                    if cr.keypoints is not None and bi < len(cr.keypoints.xy):
                        ckps = cr.keypoints.xy[bi].cpu().numpy()
                        can_kp_colors = [(0,0,255),(255,0,255),(0,255,0),(0,255,0),(255,255,0),(0,0,255)]
                        for ki in range(min(len(ckps), 6)):
                            kx, ky = int(ckps[ki][0]), int(ckps[ki][1])
                            if kx > 0 or ky > 0:
                                cv2.circle(annotated, (kx, ky), 5, can_kp_colors[ki], -1)
                                cv2.circle(annotated, (kx, ky), 7, (255, 255, 255), 1)

        # 畫計數線
        cv2.line(annotated, (0, line_y), (w, line_y), (0, 0, 255), 2)
        cv2.putText(annotated, "COUNT LINE", (w - 150, line_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 計數顯示
        cv2.putText(annotated, f"Count: {count}", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

        # 推論時間
        inf_ms = results[0].speed.get("inference", 0)
        cv2.putText(annotated, f"{inf_ms:.0f}ms | {det_count} det", (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        with yolo_lock:
            yolo_state["annotated"] = annotated
            yolo_state["count"] = count
            yolo_state["tracked_objects"] = tracked
            yolo_state["active_ids"] = active_ids
            yolo_state["detections"] = det_count
            yolo_state["inference_ms"] = inf_ms

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
    elif mode == "yolo":
        with yolo_lock:
            output = yolo_state["annotated"]
        if output is None:
            output = color.copy()
        return output
    elif mode == "yolo_depth":
        with yolo_lock:
            yolo_frame = yolo_state["annotated"]
        if yolo_frame is not None:
            output = cv2.addWeighted(yolo_frame, 0.7, depth_cm, 0.3, 0)
        else:
            output = cv2.addWeighted(color, 0.6, depth_cm, 0.4, 0)
        return output
    else:
        output = color.copy()

    h, w = output.shape[:2]
    if mode != "sbs":
        cy, cx = h // 2, w // 2
        cv2.drawMarker(output, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
        cv2.putText(output, f"{center_d:.2f}m", (cx + 15, cy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    info = f"FPS:{fps:.0f} | Depth:{center_d:.2f}m"
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
        elif parsed.path == "/yolo_status":
            self._serve_yolo_status()
        elif parsed.path == "/reset_count":
            self._reset_count()
        else:
            self.send_error(404)

    def _serve_index(self):
        imu_js = ""
        imu_html = ""
        if imu_enabled:
            imu_html = """
            <div class="panel" id="imu-panel">
              <div>Accel: <span id="accel">-</span></div>
              <div>Gyro: <span id="gyro">-</span></div>
            </div>"""
            imu_js = """
            fetch('/imu').then(r=>r.json()).then(d=>{
              document.getElementById('accel').textContent=
                `x:${d.accel[0].toFixed(2)} y:${d.accel[1].toFixed(2)} z:${d.accel[2].toFixed(2)}`;
              document.getElementById('gyro').textContent=
                `x:${d.gyro[0].toFixed(3)} y:${d.gyro[1].toFixed(3)} z:${d.gyro[2].toFixed(3)}`;
            });"""

        html = f"""<!DOCTYPE html>
<html>
<head>
  <title>RealSense + YOLO Viewer</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * {{ margin:0; padding:0; box-sizing:border-box; }}
    body {{ font-family:Arial,sans-serif; background:#111; color:#eee; padding:16px; }}
    h1 {{ color:#0cf; margin-bottom:12px; font-size:1.4em; }}
    .modes {{ display:flex; flex-wrap:wrap; gap:8px; margin-bottom:12px; }}
    .modes a {{
      background:#1a2a3a; color:#0cf; padding:10px 18px;
      text-decoration:none; border-radius:6px; border:1px solid #234;
      cursor:pointer;
    }}
    .modes a:hover, .modes a.active {{ background:#0cf; color:#111; }}
    .modes a.yolo-btn {{ border-color:#0f0; color:#0f0; }}
    .modes a.yolo-btn:hover, .modes a.yolo-btn.active {{ background:#0f0; color:#111; }}
    .modes a.reset-btn {{ border-color:#f44; color:#f44; }}
    .modes a.reset-btn:hover {{ background:#f44; color:#111; }}
    #viewer {{ width:100%; max-width:1280px; border:2px solid #0cf; border-radius:6px; }}
    .panel {{
      margin-top:12px; padding:12px; background:#1a2a3a;
      border-radius:6px; font-family:monospace; font-size:0.9em;
      display:inline-block; margin-right:12px;
    }}
    .panel h3 {{ color:#0cf; margin-bottom:6px; }}
    #yolo-panel {{ border-color:#0f0; border:1px solid #0f0; }}
    #yolo-panel h3 {{ color:#0f0; }}
    .count-big {{ font-size:2em; color:#0f0; font-weight:bold; }}
  </style>
</head>
<body>
  <h1>RealSense D435i + YOLO Tracker</h1>
  <div class="modes">
    <a onclick="setMode('color')">RGB</a>
    <a onclick="setMode('depth')">Depth</a>
    <a onclick="setMode('overlay')">RGB+Depth</a>
    <a onclick="setMode('sbs')">Side-by-Side</a>
    <a class="yolo-btn" onclick="setMode('yolo')">YOLO Track</a>
    <a class="yolo-btn" onclick="setMode('yolo_depth')">YOLO+Depth</a>
    <a href="/snapshot?mode=yolo" target="_blank">Snapshot</a>
    <a class="reset-btn" onclick="resetCount()">Reset Count</a>
  </div>
  <img id="viewer" src="/stream?mode=yolo">
  <div style="margin-top:12px; display:flex; flex-wrap:wrap;">
    <div class="panel" id="yolo-panel">
      <h3>YOLO Tracker</h3>
      <div>Count: <span class="count-big" id="yolo-count">0</span></div>
      <div>Detections: <span id="yolo-det">-</span></div>
      <div>Inference: <span id="yolo-ms">-</span></div>
      <div>Active IDs: <span id="yolo-ids">-</span></div>
    </div>
    {imu_html}
  </div>
  <script>
    let currentMode = 'yolo';
    function setMode(m) {{
      currentMode = m;
      document.getElementById('viewer').src = '/stream?mode=' + m + '&t=' + Date.now();
    }}
    function resetCount() {{
      fetch('/reset_count').then(r=>r.json()).then(d=>{{
        document.getElementById('yolo-count').textContent = '0';
      }});
    }}
    setInterval(()=>{{
      fetch('/yolo_status').then(r=>r.json()).then(d=>{{
        document.getElementById('yolo-count').textContent = d.count;
        document.getElementById('yolo-det').textContent = d.detections;
        document.getElementById('yolo-ms').textContent = d.inference_ms.toFixed(0) + 'ms';
        document.getElementById('yolo-ids').textContent = d.active_ids.join(', ') || '-';
      }});
      {imu_js}
    }}, 300);
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
                time.sleep(0.033)
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

    def _serve_yolo_status(self):
        with yolo_lock:
            data = {
                "count": yolo_state["count"],
                "detections": yolo_state["detections"],
                "inference_ms": yolo_state["inference_ms"],
                "active_ids": yolo_state["active_ids"],
            }
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _reset_count(self):
        with yolo_lock:
            yolo_state["count"] = 0
            yolo_state["tracked_objects"] = {}
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({"status": "ok", "count": 0}).encode())


# ── Main ────────────────────────────────────────────────────
if __name__ == "__main__":
    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()

    print(f"""
╔══════════════════════════════════════════════╗
║   RealSense D435i + YOLO Web Viewer         ║
╠══════════════════════════════════════════════╣
║  http://192.168.0.15:{PORT}/                    ║
║                                              ║
║  模式:                                       ║
║    RGB / Depth / RGB+Depth / Side-by-Side    ║
║    YOLO Track    ← 罐子追蹤 + 計數           ║
║    YOLO+Depth    ← 追蹤 + 深度疊合           ║
║                                              ║
║  API:                                        ║
║    /yolo_status  YOLO 狀態 JSON              ║
║    /reset_count  重置計數器                   ║
║                                              ║
║  Model: {YOLO_MODEL.split('/')[-1]:30s}      ║
║  IMU: {"啟用" if imu_enabled else "未啟用":30s}                  ║
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
        print(f"\n已停止")
