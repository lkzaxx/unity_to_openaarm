#!/usr/bin/env python3
"""Random finger movement + RealSense recording (120s)"""
import subprocess, random, time, signal, sys, os, threading

DURATION = 120  # seconds
FINGER_INTERVAL = 1.5  # seconds between random finger moves
VIDEO_PATH = os.path.expanduser("~/.openclaw/canvas/ehand_random_120s.mp4")
ROS_SETUP = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"

stop_flag = threading.Event()

def signal_handler(sig, frame):
    stop_flag.set()
signal.signal(signal.SIGINT, signal_handler)

def random_finger_loop():
    """Randomly move R_F1-R_F6 (0.0~1.0) via ROS2 topic"""
    while not stop_flag.is_set():
        # Generate random finger positions
        fingers = [round(random.uniform(0.0, 1.0), 2) for _ in range(6)]
        pos_str = ", ".join(str(f) for f in fingers)
        cmd = f"""{ROS_SETUP} && ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
            "{{name: ['R_F1','R_F2','R_F3','R_F4','R_F5','R_F6'], position: [{pos_str}]}}" --once"""
        subprocess.run(["bash", "-c", cmd], capture_output=True, timeout=5)
        print(f"[finger] R_F1-6 = [{pos_str}]", flush=True)
        # Wait with check
        for _ in range(int(FINGER_INTERVAL * 10)):
            if stop_flag.is_set():
                return
            time.sleep(0.1)

def record_realsense():
    """Record color stream to MP4 for DURATION seconds"""
    import pyrealsense2 as rs
    import numpy as np
    import cv2

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    print("[realsense] Starting pipeline...", flush=True)
    pipeline.start(config)

    # Wait for auto-exposure to settle
    for _ in range(30):
        pipeline.wait_for_frames()

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(VIDEO_PATH, fourcc, 30.0, (640, 480))
    print(f"[realsense] Recording {DURATION}s to {VIDEO_PATH}", flush=True)

    start = time.time()
    frame_count = 0
    while time.time() - start < DURATION and not stop_flag.is_set():
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        img = np.asanyarray(color_frame.get_data())
        out.write(img)
        frame_count += 1
        elapsed = time.time() - start
        if frame_count % 300 == 0:
            print(f"[realsense] {elapsed:.0f}s / {DURATION}s, {frame_count} frames", flush=True)

    out.release()
    pipeline.stop()
    elapsed = time.time() - start
    print(f"[realsense] Done! {frame_count} frames in {elapsed:.1f}s → {VIDEO_PATH}", flush=True)
    stop_flag.set()  # signal finger thread to stop too

if __name__ == "__main__":
    print(f"=== Starting random finger + RealSense record ({DURATION}s) ===", flush=True)
    # Hand home first
    cmd = f"""{ROS_SETUP} && ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
        "{{name: ['R_HAND_HOME'], position: []}}" --once"""
    subprocess.run(["bash", "-c", cmd], capture_output=True, timeout=10)
    time.sleep(2)

    finger_thread = threading.Thread(target=random_finger_loop, daemon=True)
    finger_thread.start()
    record_realsense()
    finger_thread.join(timeout=5)
    print("=== All done! ===", flush=True)
