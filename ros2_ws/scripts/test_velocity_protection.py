#!/usr/bin/env python3
"""
測試腳本：驗證 Unity Interface 的速度保護機制
發送大角度變化命令，檢查是否正確計算安全的執行時間
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class VelocityProtectionTester(Node):
    def __init__(self):
        super().__init__('velocity_protection_tester')
        
        self.publisher = self.create_publisher(
            JointState,
            '/unity/joint_commands',
            10
        )
        
        self.get_logger().info('Velocity Protection Tester started')
        
    def send_test_command(self, joint_angles_deg, description):
        """發送測試命令"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 左臂關節
        msg.name = ['L_J1', 'L_J2', 'L_J3', 'L_J4', 'L_J5', 'L_J6', 'L_J7']
        # 將角度轉換為弧度
        msg.position = [math.radians(angle) for angle in joint_angles_deg]
        
        self.get_logger().info(f'\n=== {description} ===')
        self.get_logger().info(f'Sending angles (deg): {joint_angles_deg}')
        self.get_logger().info(f'Sending angles (rad): {[f"{p:.3f}" for p in msg.position]}')
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    tester = Node('velocity_protection_tester')
    
    publisher = tester.create_publisher(JointState, '/unity/joint_commands', 10)
    
    # 等待連接建立
    time.sleep(1)
    
    print("\n" + "="*60)
    print("速度保護機制測試")
    print("="*60)
    
    # 測試 1: 小角度變化（應該使用最小時間 50ms）
    print("\n測試 1: 小角度變化 (5度)")
    msg = JointState()
    msg.header.stamp = tester.get_clock().now().to_msg()
    msg.name = ['L_J1', 'L_J2', 'L_J3', 'L_J4', 'L_J5', 'L_J6', 'L_J7']
    msg.position = [math.radians(5)] * 7  # 所有關節 5 度
    publisher.publish(msg)
    print(f"  發送: 所有關節 5° (0.087 rad)")
    print(f"  預期時間: ~0.05s (最小閾值)")
    time.sleep(2)
    
    # 測試 2: Joint 3 大角度變化（DM4340，最慢的關節）
    print("\n測試 2: Joint 3 大角度變化 (90度)")
    msg.position = [0.0, 0.0, math.radians(90), 0.0, 0.0, 0.0, 0.0]
    publisher.publish(msg)
    print(f"  發送: Joint 3 = 90° (1.571 rad)")
    print(f"  速度限制: 5.6 rad/s")
    print(f"  預期時間: 1.571 / 5.6 = ~0.28s")
    time.sleep(3)
    
    # 測試 3: Joint 1 大角度變化（DM8009，較快的關節）
    print("\n測試 3: Joint 1 大角度變化 (180度)")
    msg.position = [math.radians(-180), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    publisher.publish(msg)
    print(f"  發送: Joint 1 = 180° (3.142 rad)")
    print(f"  速度限制: 31.5 rad/s")
    print(f"  預期時間: 3.142 / 31.5 = ~0.10s")
    time.sleep(3)
    
    # 測試 4: 混合多關節大角度變化
    print("\n測試 4: 混合多關節移動")
    msg.position = [
        math.radians(-90),   # J1: 90度
        math.radians(45),   # J2: 45度
        math.radians(90),   # J3: 90度 (最慢)
        math.radians(60),   # J4: 60度
        math.radians(30),   # J5: 30度
        math.radians(30),   # J6: 30度
        math.radians(30)    # J7: 30度
    ]
    publisher.publish(msg)
    print(f"  發送: J1=90°, J2=45°, J3=90°, J4=60°, J5-7=30°")
    print(f"  預期時間: 由 Joint 3 決定 (~0.28s)")
    time.sleep(3)
    
    # 測試 5: 回到零位
    print("\n測試 5: 回到零位")
    msg.position = [0.0] * 7
    publisher.publish(msg)
    print(f"  發送: 所有關節回零")
    time.sleep(3)
    
    print("\n" + "="*60)
    print("測試完成！請檢查 ROS2 log 確認計算的軌跡時間")
    print("="*60)
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
