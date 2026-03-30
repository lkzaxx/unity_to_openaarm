#!/usr/bin/env python3
"""Infinite random finger + 1080p30 RealSense recording"""
import subprocess, random, time, signal, sys, os, threading

RECORD_DURATION = 60
VIDEO_PATH = os.path.expanduser("~/.openclaw/canvas/ehand_random_1080p_60s.mp4")
ROS_SETUP = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"
FINGER_INTERVAL = 2.0

stop_flag = threading.Event()
signal.signal(signal.SIGINT, lambda s,f: stop_flag.set())

def record_realsense():
    import pyrealsense2 as rs
    import numpy as np
    import cv2

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    print("[realsense] Starting 1920x1080@30fps...", flush=True)
    pipeline.start(config)
    for _ in range(60): pipeline.wait_for_frames()  # longer warmup for 1080p

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(VIDEO_PATH, fourcc, 30.0, (1920, 1080))
    print(f"[realsense] Recording {RECORD_DURATION}s", flush=True)

    start = time.time()
    fc = 0
    while time.time() - start < RECORD_DURATION and not stop_flag.is_set():
        frames = pipeline.wait_for_frames()
        cf = frames.get_color_frame()
        if not cf: continue
        out.write(np.asanyarray(cf.get_data()))
        fc += 1
        if fc % 150 == 0:
            print(f"[realsense] {time.time()-start:.0f}s, {fc} frames", flush=True)

    out.release()
    pipeline.stop()
    sz = os.path.getsize(VIDEO_PATH) / 1024 / 1024
    print(f"[realsense] Done! {fc} frames, {sz:.1f}MB -> {VIDEO_PATH}", flush=True)

def run_fingers():
    script = """
import rclpy, random, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

class F(Node):
    def __init__(self):
        super().__init__("finger_inf")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.ap = self.create_publisher(JointState, "/unity/joint_commands", qos)
        self.hp = self.create_publisher(JointState, "/unity/ehand_commands", qos)
        self.ready = False
        self.create_timer(0.1, self.wait_subs)

    def wait_subs(self):
        if self.ap.get_subscription_count() > 0 and self.hp.get_subscription_count() > 0:
            if not self.ready:
                self.ready = True
                self.arm_pos = [-0.02, 0.006, -0.014, 1.526, -0.007, 0.008, 0.0]
                self.create_timer(0.2, self.ka)
                m = JointState(); m.name = ["R_HAND_ENABLE"]; m.position = []
                self.hp.publish(m)
                self.get_logger().info("ENABLE + start")
                self.create_timer(2.0, self.sf_once)

    def sf_once(self):
        if hasattr(self, _started): return
        self._started = True
        self.create_timer(INTERVAL, self.sf)

    def ka(self):
        m = JointState()
        m.name = ["R_J1","R_J2","R_J3","R_J4","R_J5","R_J6","R_J7"]
        m.position = self.arm_pos[:]
        self.ap.publish(m)

    def sf(self):
        f = [round(random.uniform(0.0, 1.0), 2) for _ in range(6)]
        m = JointState()
        m.name = ["R_F1","R_F2","R_F3","R_F4","R_F5","R_F6"]
        m.position = f
        self.hp.publish(m)

INTERVAL = """ + str(FINGER_INTERVAL) + """
rclpy.init()
n = F()
try:
    rclpy.spin(n)
except (SystemExit, KeyboardInterrupt):
    pass
n.destroy_node(); rclpy.shutdown()
"""
    with open(os.path.expanduser("~/finger_inf_node.py"), "w") as f:
        f.write(script)
    os.system(f"{ROS_SETUP} && python3 ~/finger_inf_node.py")

if __name__ == "__main__":
    print("=== Infinite finger + 1080p30 record 60s ===", flush=True)
    ft = threading.Thread(target=run_fingers, daemon=True)
    ft.start()
    time.sleep(5)  # let fingers start first
    record_realsense()
    print("Recording done. Fingers still running in background.", flush=True)
    print("Press Ctrl+C to stop fingers.", flush=True)
    try:
        ft.join()
    except KeyboardInterrupt:
        pass
