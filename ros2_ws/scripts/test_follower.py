#!/usr/bin/env python3
"""
Unity Follower Interface 測試腳本

使用方式：
  python3 test_follower.py                    # 測試右臂
  python3 test_follower.py left               # 測試左臂
  python3 test_follower.py right 1.0 0 0 1.5  # 自訂關節角度
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import time

class FollowerTestNode(Node):
    def __init__(self):
        super().__init__('follower_test')
        
        self.pub = self.create_publisher(
            JointState, '/unity/joint_commands', 10
        )
        
        # 訂閱關節狀態以顯示當前位置
        self.joint_states = {}
        self.sub = self.create_subscription(
            JointState, '/openarm/joint_states',
            self.state_callback, 10
        )
    
    def state_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.joint_states[name] = msg.position[i]
    
    def send_command(self, side: str, positions: list):
        """發送關節命令"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        prefix = 'L_J' if side == 'left' else 'R_J'
        
        for i, pos in enumerate(positions):
            msg.name.append(f'{prefix}{i+1}')
            msg.position.append(pos)
        
        self.pub.publish(msg)
        self.get_logger().info(f"Sent {side} command: {[f'{p:.2f}' for p in positions]}")
    
    def print_current_positions(self, side: str):
        """顯示當前位置"""
        prefix = 'openarm_left_' if side == 'left' else 'openarm_right_'
        print("\nCurrent positions:")
        for i in range(1, 8):
            name = f'{prefix}joint{i}'
            pos = self.joint_states.get(name, 0.0)
            print(f"  Joint {i}: {pos:.3f} rad ({pos * 180/3.14159:.1f}°)")


def main():
    rclpy.init()
    node = FollowerTestNode()
    
    # 解析命令列參數
    side = 'right'
    if len(sys.argv) > 1:
        if sys.argv[1] in ['left', 'right']:
            side = sys.argv[1]
    
    # 預設測試位置（右臂抬起）
    # Joint 1: 1.57 rad (90°) - 向前抬起
    # Joint 4: 1.5 rad (86°) - 手肘彎曲
    default_positions = [1.57, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]
    
    # 如果提供了自訂位置
    if len(sys.argv) > 2:
        try:
            positions = [float(x) for x in sys.argv[2:9]]
            while len(positions) < 7:
                positions.append(0.0)
        except ValueError:
            print("Error: Invalid position values")
            return
    else:
        positions = default_positions
    
    print("=" * 50)
    print("Unity Follower Interface Test")
    print("=" * 50)
    print(f"Side: {side}")
    print(f"Target: {[f'{p:.2f}' for p in positions]}")
    print()
    
    # 等待一下讓訂閱生效
    time.sleep(0.5)
    rclpy.spin_once(node, timeout_sec=0.5)
    
    # 顯示當前位置
    node.print_current_positions(side)
    
    # 發送命令
    print(f"\nSending command to {side} arm...")
    node.send_command(side, positions)
    
    # 持續發送 3 秒（模擬 Unity 持續發送）
    print("Sending for 3 seconds...")
    start = time.time()
    while time.time() - start < 3.0:
        node.send_command(side, positions)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.016)  # ~60Hz
    
    # 顯示最終位置
    rclpy.spin_once(node, timeout_sec=0.5)
    node.print_current_positions(side)
    
    print("\nDone!")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
