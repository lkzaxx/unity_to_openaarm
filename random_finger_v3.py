#!/usr/bin/env python3
"""Random finger movement v3 - uses rclpy directly for continuous publishing"""
import subprocess, random, time, signal, sys, os

DURATION = 20
FINGER_INTERVAL = 2.0
ROS_SETUP = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"

# Read current arm position first
print("[init] Reading current arm position...", flush=True)
result = subprocess.run(["bash", "-c",
    f'{ROS_SETUP} && ros2 topic echo /openarm/joint_states --once'],
    capture_output=True, text=True, timeout=10)

lines = result.stdout.split('\n')
positions = []
in_position = False
for line in lines:
    if 'position:' in line:
        in_position = True
        continue
    if in_position:
        line = line.strip()
        if line.startswith('- '):
            positions.append(float(line[2:]))
        else:
            break

if len(positions) < 14:
    print("[ERROR] Could not read arm position!", flush=True)
    sys.exit(1)

arm_pos = positions[7:14]
print(f"[init] Right arm: {[round(x,3) for x in arm_pos]}", flush=True)

# Now use rclpy for continuous publishing
script = f'''
import rclpy, random, time, signal, threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState

ARM_POS = {arm_pos}
DURATION = {DURATION}
FINGER_INTERVAL = {FINGER_INTERVAL}

class FingerRandomizer(Node):
    def __init__(self):
        super().__init__("finger_randomizer")
        vol_qos = QoSProfile(depth=1)
        vol_qos.durability = DurabilityPolicy.VOLATILE
        self.arm_pub = self.create_publisher(JointState, "/unity/joint_commands", vol_qos)
        self.hand_pub = self.create_publisher(JointState, "/unity/ehand_commands", vol_qos)
        self.start_time = time.time()
        self.count = 0

        # Arm keepalive at 2 Hz
        self.arm_timer = self.create_timer(0.5, self.send_arm)
        # Finger random at FINGER_INTERVAL
        self.finger_timer = self.create_timer(FINGER_INTERVAL, self.send_finger)
        # Home first
        self.home_sent = False
        self.create_timer(1.0, self.send_home_once)

    def send_arm(self):
        if time.time() - self.start_time > DURATION:
            raise SystemExit
        msg = JointState()
        msg.name = ["R_J1","R_J2","R_J3","R_J4","R_J5","R_J6","R_J7"]
        msg.position = ARM_POS[:]
        self.arm_pub.publish(msg)

    def send_home_once(self):
        if not self.home_sent:
            self.home_sent = True
            msg = JointState()
            msg.name = ["R_HAND_HOME"]
            msg.position = []
            self.hand_pub.publish(msg)
            self.get_logger().info("Hand HOME sent")

    def send_finger(self):
        if time.time() - self.start_time > DURATION:
            raise SystemExit
        if time.time() - self.start_time < 3.0:
            return  # wait for home to finish
        fingers = [round(random.uniform(0.0, 1.0), 2) for _ in range(6)]
        msg = JointState()
        msg.name = ["R_F1","R_F2","R_F3","R_F4","R_F5","R_F6"]
        msg.position = fingers
        self.hand_pub.publish(msg)
        self.count += 1
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"[{{elapsed:.0f}}s] #{{self.count}} fingers={{fingers}}")

rclpy.init()
node = FingerRandomizer()
try:
    rclpy.spin(node)
except (SystemExit, KeyboardInterrupt):
    pass
node.destroy_node()
rclpy.shutdown()
print(f"[done] Finished", flush=True)
'''

# Write and run
with open(os.path.expanduser("~/finger_random_node.py"), "w") as f:
    f.write(script)
print("[init] Starting rclpy node...", flush=True)
os.execvp("bash", ["bash", "-c", f"{ROS_SETUP} && python3 ~/finger_random_node.py"])
