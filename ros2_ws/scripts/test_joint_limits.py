#!/usr/bin/env python3
"""
測試腳本：驗證每個關節的正負最大值
目的：根據 JOINT_LIMITS_FIX_PROPOSAL.md 檢查左右手關節方向是否正確

測試邏輯：
1. 逐個關節測試正負極限值
2. 觀察實際運動方向
3. 驗證左右手是否需要鏡像

預期結果（從機器人後方觀察）：
- Joint 1, 2, 3, 5, 6, 7: 左右手方向相反（需要鏡像）
- Joint 4: 左右手方向相同（不需要鏡像）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
import sys


class JointLimitTester(Node):
    def __init__(self):
        super().__init__('joint_limit_tester')
        
        # 左手發布器
        self.left_publisher = self.create_publisher(
            JointState,
            '/unity/joint_commands',
            10
        )
        
        # 右手發布器（如果需要測試右手）
        self.right_publisher = self.create_publisher(
            JointState,
            '/unity/joint_commands',
            10
        )
        
        # 關節限制定義（90% 安全範圍）
        self.joint_limits = {
            'joint1': {
                'lower': -1.1538,   # -80° * 0.9
                'upper': 3.1416,    # 180°（保守值）
                'lower_deg': -66,
                'upper_deg': 180
            },
            'joint2': {
                'lower': -1.5708,   # -90°
                'upper': 1.5708,    # 90°
                'lower_deg': -90,
                'upper_deg': 90
            },
            'joint3': {
                'lower': -1.4137,   # -81°
                'upper': 1.4137,    # 81°
                'lower_deg': -81,
                'upper_deg': 81
            },
            'joint4': {
                'lower': 0.1745,    # 10°（保守值）
                'upper': 2.2689,    # 130°（保守值）
                'lower_deg': 10,
                'upper_deg': 130
            },
            'joint5': {
                'lower': -1.4137,   # -81°
                'upper': 1.4137,    # 81°
                'lower_deg': -81,
                'upper_deg': 81
            },
            'joint6': {
                'lower': -0.7069,   # -40.5°
                'upper': 0.7069,    # 40.5°
                'lower_deg': -40,
                'upper_deg': 40
            },
            'joint7': {
                'lower': -1.4137,   # -81°
                'upper': 1.4137,    # 81°
                'lower_deg': -81,
                'upper_deg': 81
            }
        }
        
        self.get_logger().info('Joint Limit Tester started')
    
    def send_joint_command(self, arm_side, joint_positions, description):
        """
        發送關節命令
        
        Args:
            arm_side: 'left' 或 'right'
            joint_positions: 7個關節的位置列表（弧度）
            description: 測試描述
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        if arm_side == 'left':
            msg.name = ['L_J1', 'L_J2', 'L_J3', 'L_J4', 'L_J5', 'L_J6', 'L_J7']
            publisher = self.left_publisher
        else:
            msg.name = ['R_J1', 'R_J2', 'R_J3', 'R_J4', 'R_J5', 'R_J6', 'R_J7']
            publisher = self.right_publisher
        
        msg.position = joint_positions
        
        self.get_logger().info(f'\n=== {description} ===')
        self.get_logger().info(f'手臂: {arm_side.upper()}')
        self.get_logger().info(f'位置 (rad): {[f"{p:.3f}" for p in msg.position]}')
        self.get_logger().info(f'位置 (deg): {[f"{math.degrees(p):.1f}" for p in msg.position]}')
        
        publisher.publish(msg)


def test_single_joint(tester, arm_side, joint_num, wait_time=3):
    """
    測試單一關節的正負極限
    
    Args:
        tester: JointLimitTester 節點
        arm_side: 'left' 或 'right'
        joint_num: 關節編號 (1-7)
        wait_time: 每個動作的等待時間（秒）
    """
    joint_key = f'joint{joint_num}'
    limits = tester.joint_limits[joint_key]
    
    print("\n" + "="*70)
    print(f"測試 Joint {joint_num} - {arm_side.upper()} 手")
    print("="*70)
    
    # 1. 回零位
    zero_positions = [0.0] * 7
    tester.send_joint_command(
        arm_side, 
        zero_positions,
        f"Joint {joint_num} - 步驟 1: 回零位"
    )
    print(f">>> 觀察: 所有關節應回到零位")
    time.sleep(wait_time)
    
    # 2. 測試正最大值
    positive_positions = [0.0] * 7
    positive_positions[joint_num - 1] = limits['upper']
    tester.send_joint_command(
        arm_side,
        positive_positions,
        f"Joint {joint_num} - 步驟 2: 正最大值 ({limits['upper_deg']}°)"
    )
    print(f">>> 觀察: Joint {joint_num} 移動到 +{limits['upper_deg']}°")
    print(f">>> 記錄: 運動方向 = __________")
    time.sleep(wait_time)
    
    # 3. 回零位
    tester.send_joint_command(
        arm_side,
        zero_positions,
        f"Joint {joint_num} - 步驟 3: 回零位"
    )
    time.sleep(wait_time)
    
    # 4. 測試負最小值
    negative_positions = [0.0] * 7
    negative_positions[joint_num - 1] = limits['lower']
    tester.send_joint_command(
        arm_side,
        negative_positions,
        f"Joint {joint_num} - 步驟 4: 負最小值 ({limits['lower_deg']}°)"
    )
    print(f">>> 觀察: Joint {joint_num} 移動到 {limits['lower_deg']}°")
    print(f">>> 記錄: 運動方向 = __________")
    time.sleep(wait_time)
    
    # 5. 回零位
    tester.send_joint_command(
        arm_side,
        zero_positions,
        f"Joint {joint_num} - 步驟 5: 回零位"
    )
    time.sleep(wait_time)
    
    print(f"\n{'='*70}")
    print(f"Joint {joint_num} 測試完成！請確認並記錄運動方向")
    print(f"{'='*70}\n")


def test_all_joints(tester, arm_side):
    """測試所有關節"""
    print("\n" + "╔"+"="*68+"╗")
    print(f"║  開始測試 {arm_side.upper()} 手所有關節的正負極限值")
    print("║  請從機器人後方觀察，記錄每個關節的運動方向")
    print("╚"+"="*68+"╝")
    
    for joint_num in range(1, 8):
        test_single_joint(tester, arm_side, joint_num, wait_time=4)
        
        if joint_num < 7:
            print("\n按 Enter 繼續下一個關節測試...")
            input()


def test_comparison_mode(tester):
    """
    對比模式：同時顯示左右手相同數值的運動
    用於驗證哪些關節需要鏡像
    """
    print("\n" + "╔"+"="*68+"╗")
    print("║  對比模式：測試左右手在相同數值時的運動方向")
    print("║  預期：Joint 1,2,3,5,6,7 方向相反，Joint 4 方向相同")
    print("╚"+"="*68+"╝")
    
    test_angle = 1.5  # 使用 1.5 rad (約 86°) 作為測試角度
    
    for joint_num in range(1, 8):
        print("\n" + "="*70)
        print(f"對比測試 Joint {joint_num}")
        print("="*70)
        
        # 左手
        positions = [0.0] * 7
        positions[joint_num - 1] = test_angle
        tester.send_joint_command(
            'left',
            positions,
            f"左手 Joint {joint_num} = {math.degrees(test_angle):.1f}°"
        )
        print(f">>> 觀察左手 Joint {joint_num} 的運動方向")
        time.sleep(3)
        
        # 回零
        tester.send_joint_command('left', [0.0]*7, "左手回零")
        time.sleep(2)
        
        # 右手
        tester.send_joint_command(
            'right',
            positions,
            f"右手 Joint {joint_num} = {math.degrees(test_angle):.1f}°"
        )
        print(f">>> 觀察右手 Joint {joint_num} 的運動方向")
        time.sleep(3)
        
        # 回零
        tester.send_joint_command('right', [0.0]*7, "右手回零")
        time.sleep(2)
        
        print(f"\n>>> Joint {joint_num} 左右手方向是否相反？ [是/否]: __________")
        
        if joint_num < 7:
            print("\n按 Enter 繼續下一個關節...")
            input()


def main(args=None):
    rclpy.init(args=args)
    tester = JointLimitTester()
    
    # 等待連接建立
    time.sleep(1)
    
    print("\n" + "╔"+"="*68+"╗")
    print("║           OpenArm 關節限制測試工具")
    print("║")
    print("║  用途：測試每個關節的正負極限值，驗證左右手方向")
    print("║")
    print("║  測試模式：")
    print("║    1. 左手完整測試（測試所有關節的正負極限）")
    print("║    2. 右手完整測試（測試所有關節的正負極限）")
    print("║    3. 對比模式（同時測試左右手，驗證是否需要鏡像）")
    print("║    4. 單一關節測試")
    print("║    5. 退出")
    print("╚"+"="*68+"╝\n")
    
    while True:
        try:
            choice = input("請選擇測試模式 (1-5): ").strip()
            
            if choice == '1':
                test_all_joints(tester, 'left')
            elif choice == '2':
                test_all_joints(tester, 'right')
            elif choice == '3':
                test_comparison_mode(tester)
            elif choice == '4':
                arm = input("選擇手臂 (left/right): ").strip().lower()
                if arm not in ['left', 'right']:
                    print("錯誤：請輸入 'left' 或 'right'")
                    continue
                joint = input("選擇關節編號 (1-7): ").strip()
                try:
                    joint_num = int(joint)
                    if 1 <= joint_num <= 7:
                        test_single_joint(tester, arm, joint_num)
                    else:
                        print("錯誤：關節編號必須在 1-7 之間")
                except ValueError:
                    print("錯誤：請輸入有效的數字")
            elif choice == '5':
                print("\n測試結束，感謝使用！")
                break
            else:
                print("錯誤：請輸入 1-5 之間的數字\n")
                
        except KeyboardInterrupt:
            print("\n\n測試被中斷")
            break
        except Exception as e:
            print(f"\n錯誤: {e}")
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

