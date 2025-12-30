#!/usr/bin/env python3
"""
Unity Follower Interface 測試腳本

使用方式：
  python3 test_follower.py                    # 測試右臂
  python3 test_follower.py left               # 測試左臂
  python3 test_follower.py right 1.0 0 0 1.5  # 自訂關節角度
  python3 test_follower.py stop               # 停止（歸零）
  python3 test_follower.py dance              # 跳舞動作
  python3 test_follower.py shake              # 揮手動作
  python3 test_follower.py soda               # 搖汽水動作
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
    
    def send_both_arms(self, left_positions: list, right_positions: list):
        """同時發送左右臂命令"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 左臂
        for i, pos in enumerate(left_positions):
            msg.name.append(f'L_J{i+1}')
            msg.position.append(pos)
        
        # 右臂
        for i, pos in enumerate(right_positions):
            msg.name.append(f'R_J{i+1}')
            msg.position.append(pos)
        
        self.pub.publish(msg)
    
    def print_current_positions(self, side: str):
        """顯示當前位置"""
        prefix = 'openarm_left_' if side == 'left' else 'openarm_right_'
        print("\nCurrent positions:")
        for i in range(1, 8):
            name = f'{prefix}joint{i}'
            pos = self.joint_states.get(name, 0.0)
            print(f"  Joint {i}: {pos:.3f} rad ({pos * 180/3.14159:.1f}°)")


def run_stop(node):
    """停止動作 - 雙臂歸零"""
    print("=== STOP: 雙臂歸零 ===")
    left_pos = [0, 0, 0, 0, 0, 0, 0]
    right_pos = [0, 0, 0, 0, 0, 0, 0]
    
    duration = 3.0
    start = time.time()
    while time.time() - start < duration:
        node.send_both_arms(left_pos, right_pos)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.016)
    
    print("Done!")


def run_dance(node):
    """跳舞動作 - 雙臂協調舞動"""
    print("=== DANCE: 跳舞動作 ===")
    
    # 舞蹈動作序列 [(left_pos, right_pos, duration)]
    # 注意：左臂 Joint 1, 2 方向與右臂相反
    dance_sequence = [
        # 開始姿勢
        ([0, -1.5, -1.5, 0.5, 0, 0, -0.2], [0, 1.5, 1.5, 0.5, 0, 0, -0.2], 3.0),
        # 舞動 1
        ([0, -1.7, -1.5, 0.1, 0, 0, 0.5], [0, 1.7, 1.5, 0.1, 0, 0, 0.5], 1.0),
        # 舞動 2
        ([0, -1.5, -1.5, 0.5, 0, 0, -0.2], [0, 1.5, 1.5, 0.5, 0, 0, -0.2], 1.0),
        # 舞動 3
        ([0, -1.7, -1.5, 0.1, 0, 0, 0.5], [0, 1.7, 1.5, 0.1, 0, 0, 0.5], 1.0),
        # 舞動 4
        ([0, -1.5, -1.5, 0.5, 0, 0, -0.2], [0, 1.5, 1.5, 0.5, 0, 0, -0.2], 1.0),
        # 舞動 5
        ([0, -1.7, -1.5, 0.1, 0, 0, 0.5], [0, 1.7, 1.5, 0.1, 0, 0, 0.5], 1.0),
        # 回到零位
        ([0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], 3.0),
    ]
    
    for i, (left_pos, right_pos, duration) in enumerate(dance_sequence):
        print(f"  Step {i+1}/{len(dance_sequence)}...")
        start = time.time()
        while time.time() - start < duration:
            node.send_both_arms(left_pos, right_pos)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.016)
    
    print("Dance complete!")


def run_shake(node):
    """揮手動作 - 右臂揮手"""
    print("=== SHAKE: 右臂揮手 ===")
    
    # 左臂保持歸零
    left_pos = [0, 0, 0, 0, 0, 0, 0]
    
    # 揮手動作序列 [(right_pos, duration)]
    shake_sequence = [
        # 抬起手臂
        ([0, 1.5, 1.2, 1.8, 0, 0, 0], 3.0),
        # 揮手 1
        ([0, 1.5, 1.2, 1.2, 0, 0, 0], 1.0),
        # 揮手 2
        ([0, 1.5, 1.2, 1.8, 0, 0, 0], 1.0),
        # 揮手 3
        ([0, 1.5, 1.2, 1.2, 0, 0, 0], 1.0),
        # 揮手 4
        ([0, 1.5, 1.2, 1.8, 0, 0, 0], 1.0),
        # 揮手 5
        ([0, 1.5, 1.2, 1.2, 0, 0, 0], 1.0),
        # 揮手 6
        ([0, 1.5, 1.2, 1.8, 0, 0, 0], 1.0),
        # 回到零位
        ([0, 0, 0, 0, 0, 0, 0], 3.0),
    ]
    
    for i, (right_pos, duration) in enumerate(shake_sequence):
        print(f"  Step {i+1}/{len(shake_sequence)}...")
        start = time.time()
        while time.time() - start < duration:
            node.send_both_arms(left_pos, right_pos)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.016)
    
    print("Shake complete!")


def run_soda(node):
    """搖汽水動作 - 雙臂往前抬升做上下搖擺"""
    print("=== SODA: 搖汽水動作 ===")
    
    # 搖汽水動作序列 [(left_pos, right_pos, duration)]
    # 只動 Joint 1 (肩膀前後) 和 Joint 4 (手肘)
    # 左臂 Joint 1 方向與右臂相反
    # 左右手錯開相位：左手往上時右手往下
    soda_sequence = [
        # 準備姿勢：雙手往前抬 + 手肘彎曲
        ([-1.0, 0, 0, 0.7, 0, 0, 0], [1.0, 0, 0, 0.7, 0, 0, 0], 2.0),
        # 左上右下（錯開）
        ([-1.3, 0, 0, 0.0, 0, 0, 0], [0.7, 0, 0, 1.5, 0, 0, 0], 1),
        # 左下右上
        ([-0.7, 0, 0, 1.5, 0, 0, 0], [1.3, 0, 0, 0.0, 0, 0, 0], 1),
        # 左上右下
        ([-1.3, 0, 0, 0.0, 0, 0, 0], [0.7, 0, 0, 1.5, 0, 0, 0], 1),
        # 左下右上
        ([-0.7, 0, 0, 1.5, 0, 0, 0], [1.3, 0, 0, 0.0, 0, 0, 0], 1),
        # 左上右下
        ([-1.3, 0, 0, 0.0, 0, 0, 0], [0.7, 0, 0, 1.5, 0, 0, 0], 1),
        # 左下右上
        ([-0.7, 0, 0, 1.5, 0, 0, 0], [1.3, 0, 0, 0.0, 0, 0, 0], 1),
        # 左上右下
        ([-1.3, 0, 0, 0.0, 0, 0, 0], [0.7, 0, 0, 1.5, 0, 0, 0], 1),
        # 左下右上
        ([-0.7, 0, 0, 1.5, 0, 0, 0], [1.3, 0, 0, 0.0, 0, 0, 0], 1),
        # 回到零位
        ([0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], 2.0),
    ]
    
    for i, (left_pos, right_pos, duration) in enumerate(soda_sequence):
        print(f"  Step {i+1}/{len(soda_sequence)}...")
        start = time.time()
        while time.time() - start < duration:
            node.send_both_arms(left_pos, right_pos)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.016)
    
    print("Soda shake complete!")


def run_custom(node, side: str, positions: list):
    """自訂位置測試"""
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


def main():
    rclpy.init()
    node = FollowerTestNode()
    
    # 等待一下讓訂閱生效
    time.sleep(0.5)
    rclpy.spin_once(node, timeout_sec=0.5)
    
    # 解析命令列參數
    if len(sys.argv) > 1:
        cmd = sys.argv[1].lower()
        
        if cmd == 'stop':
            run_stop(node)
        elif cmd == 'dance':
            run_dance(node)
        elif cmd == 'shake':
            run_shake(node)
        elif cmd == 'soda':
            run_soda(node)
        elif cmd in ['left', 'right']:
            side = cmd
            # 如果提供了自訂位置
            if len(sys.argv) > 2:
                try:
                    positions = [float(x) for x in sys.argv[2:9]]
                    while len(positions) < 7:
                        positions.append(0.0)
                except ValueError:
                    print("Error: Invalid position values")
                    node.destroy_node()
                    rclpy.shutdown()
                    return
            else:
                # 預設測試位置
                positions = [1.57, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]
            run_custom(node, side, positions)
        else:
            print(f"Unknown command: {cmd}")
            print("Usage:")
            print("  python3 test_follower.py stop     # 雙臂歸零")
            print("  python3 test_follower.py dance    # 跳舞動作")
            print("  python3 test_follower.py shake    # 揮手動作")
            print("  python3 test_follower.py soda     # 搖汽水動作")
            print("  python3 test_follower.py [left|right] [j1 j2 j3 j4 j5 j6 j7]")
    else:
        # 預設：右臂測試
        run_custom(node, 'right', [1.57, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0])
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
