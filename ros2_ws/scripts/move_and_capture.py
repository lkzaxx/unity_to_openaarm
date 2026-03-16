#!/usr/bin/env python3
"""Move joint incrementally and capture images for verification."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import subprocess
import time
import math

# Mapping: openarm name -> unity command name
NAME_MAP = {
    "openarm_left_joint1": "L_J1", "openarm_left_joint2": "L_J2",
    "openarm_left_joint3": "L_J3", "openarm_left_joint4": "L_J4",
    "openarm_left_joint5": "L_J5", "openarm_left_joint6": "L_J6",
    "openarm_left_joint7": "L_J7",
    "openarm_right_joint1": "R_J1", "openarm_right_joint2": "R_J2",
    "openarm_right_joint3": "R_J3", "openarm_right_joint4": "R_J4",
    "openarm_right_joint5": "R_J5", "openarm_right_joint6": "R_J6",
    "openarm_right_joint7": "R_J7",
}

class MoveAndCapture(Node):
    def __init__(self):
        super().__init__("move_and_capture")
        self.pub = self.create_publisher(JointState, "/unity/joint_commands", 10)
        self.sub = self.create_subscription(JointState, "/openarm/joint_states", self.state_cb, 10)
        self.current_positions = {}
        
    def state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_positions[name] = pos
    
    def wait_for_state(self, timeout=3.0):
        start = time.time()
        while not self.current_positions and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return bool(self.current_positions)
    
    def capture_image(self, filename):
        cmd = f'gst-launch-1.0 -e nvarguscamerasrc sensor-id=1 num-buffers=1 ! "video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=30/1" ! nvjpegenc ! filesink location={filename}'
        subprocess.run(cmd, shell=True, capture_output=True, timeout=10)
    
    def send_command(self, positions_dict):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for openarm_name, pos in positions_dict.items():
            unity_name = NAME_MAP.get(openarm_name)
            if unity_name:
                msg.name.append(unity_name)
                msg.position.append(pos)
        
        self.pub.publish(msg)
        self.get_logger().info(f"Sent: {list(zip(msg.name, [round(p,3) for p in msg.position]))}")
    
    def move_incremental(self, joint_name, delta_rad, steps=5, delay=1.2):
        if not self.wait_for_state():
            print("Failed to get joint state")
            return
        
        current_val = self.current_positions.get(joint_name, 0)
        step_size = delta_rad / steps
        
        print(f"Moving {joint_name} ({NAME_MAP.get(joint_name)})")
        print(f"  From: {current_val:.4f} rad ({math.degrees(current_val):.1f} deg)")
        print(f"  To:   {current_val + delta_rad:.4f} rad ({math.degrees(current_val + delta_rad):.1f} deg)")
        print(f"  Step: {step_size:.4f} rad ({math.degrees(step_size):.1f} deg)")
        print()
        
        for i in range(steps):
            rclpy.spin_once(self, timeout_sec=0.1)
            
            target_val = current_val + step_size * (i + 1)
            cmd_positions = dict(self.current_positions)
            cmd_positions[joint_name] = target_val
            
            self.send_command(cmd_positions)
            time.sleep(delay)
            
            img_path = f"/home/idaka/.openclaw/canvas/step_{i+1}.jpg"
            self.capture_image(img_path)
            
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            actual = self.current_positions.get(joint_name, 0)
            print(f"Step {i+1}/{steps}: target={target_val:.3f} actual={actual:.3f} rad ({math.degrees(actual):.1f} deg)")

def main():
    rclpy.init()
    node = MoveAndCapture()
    delta = math.radians(10)
    node.move_incremental("openarm_right_joint4", delta, steps=5, delay=1.2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
