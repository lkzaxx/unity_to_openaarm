"""
罐子即時追蹤 + 計數
====================
RealSense D435i → YOLOv11 TensorRT → ByteTrack → 越線計數

使用方式 (在 Jetson 上):
    cd ~/ros2_ws/yolo_model
    python3 can_tracker.py

按 'q' 結束, 's' 截圖
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# ============================================================
# 設定
# ============================================================
MODEL_PATH = "can_v4_best.engine"  # TensorRT (最快)
# MODEL_PATH = "can_v4_best.pt"   # PyTorch (備用)

CONF_THRESHOLD = 0.4        # 偵測信心度閾值
TRACKER = "bytetrack.yaml"  # 追蹤器

# RealSense 設定
RS_WIDTH = 640
RS_HEIGHT = 480
RS_FPS = 30

# 計數線 (Y 座標, 從上往下數)
# 物體中心越過這條線就計數
LINE_Y = RS_HEIGHT // 2     # 預設畫面中間, 可調整
LINE_COLOR = (0, 0, 255)    # 紅色
LINE_THICKNESS = 2

# 計數方向: "down" = 從上往下越線計數, "up" = 從下往上, "both" = 雙向
COUNT_DIRECTION = "down"


# ============================================================
# 主程式
# ============================================================
def main():
    # --- RealSense 初始化 ---
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, RS_WIDTH, RS_HEIGHT, rs.format.bgr8, RS_FPS)

    print("Starting RealSense...")
    try:
        pipeline.start(config)
    except Exception as e:
        print(f"RealSense 啟動失敗: {e}")
        print("請確認 D435i 已接上 Jetson")
        return

    # --- YOLO 載入 ---
    print(f"Loading model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)

    # --- 計數狀態 ---
    count = 0
    tracked_objects = {}  # {id: last_cy}  記錄每個 ID 上一幀的 Y 座標

    print(f"Running! Count line at Y={LINE_Y}, direction={COUNT_DIRECTION}")
    print("Press 'q' to quit, 's' to screenshot")

    try:
        while True:
            # 取幀
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())

            # YOLO 追蹤
            results = model.track(
                frame,
                persist=True,
                tracker=TRACKER,
                conf=CONF_THRESHOLD,
                verbose=False,
            )

            # 處理追蹤結果
            if results[0].boxes.id is not None:
                boxes = results[0].boxes.xyxy.cpu().numpy()
                ids = results[0].boxes.id.cpu().numpy().astype(int)
                confs = results[0].boxes.conf.cpu().numpy()

                for box, obj_id, conf in zip(boxes, ids, confs):
                    x1, y1, x2, y2 = box.astype(int)
                    cy = (y1 + y2) / 2  # 物體中心 Y

                    # 檢查是否越線
                    if obj_id in tracked_objects:
                        prev_cy = tracked_objects[obj_id]

                        crossed = False
                        if COUNT_DIRECTION == "down":
                            crossed = prev_cy < LINE_Y and cy >= LINE_Y
                        elif COUNT_DIRECTION == "up":
                            crossed = prev_cy > LINE_Y and cy <= LINE_Y
                        elif COUNT_DIRECTION == "both":
                            crossed = (prev_cy < LINE_Y and cy >= LINE_Y) or \
                                      (prev_cy > LINE_Y and cy <= LINE_Y)

                        if crossed:
                            count += 1
                            print(f"  [COUNT] ID={obj_id} crossed line → total: {count}")

                    tracked_objects[obj_id] = cy

                    # 畫框 + ID
                    color = (0, 255, 0) if cy < LINE_Y else (255, 165, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    label = f"ID:{obj_id} {conf:.2f}"
                    cv2.putText(frame, label, (x1, y1 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # 畫計數線
            cv2.line(frame, (0, LINE_Y), (RS_WIDTH, LINE_Y), LINE_COLOR, LINE_THICKNESS)

            # 畫計數數字
            cv2.putText(frame, f"Count: {count}", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

            # FPS
            fps_text = f"FPS: {results[0].speed['inference']:.0f}ms"
            cv2.putText(frame, fps_text, (RS_WIDTH - 180, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # 顯示
            cv2.imshow("Can Tracker", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"screenshot_{count}.jpg"
                cv2.imwrite(filename, frame)
                print(f"  Screenshot saved: {filename}")

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f"\nFinal count: {count}")


if __name__ == "__main__":
    main()
