#!/usr/bin/env python3
# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
靈巧手獨立測試腳本

用法:
    # 設定 CAN FD (如果尚未設定)
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
    sudo ip link set can0 up
    
    # 執行測試
    python3 test_dexterous_hand.py [can_interface] [can_id]
    
    # 範例
    python3 test_dexterous_hand.py can0 1
    python3 test_dexterous_hand.py can1 0x01

注意:
    - 執行前請確認 CAN 介面已設定為 CAN FD 模式
    - 確認靈巧手已連接並供電
    - 測試時手指不要碰到障礙物
"""

import sys
import time
import argparse

# 添加 Python 模組路徑
sys.path.insert(0, '/home/idaka/openarm_can/python')

from openarm.can.dexterous_hand import DexterousHand


def test_basic_commands(hand: DexterousHand):
    """測試基本命令: 回零、開、合"""
    print("\n" + "=" * 60)
    print("測試 1: 基本命令 (回零 → 開 → 合)")
    print("=" * 60)
    
    print("[1.1] 發送回零命令...")
    hand.home()
    print("      等待 2 秒...")
    time.sleep(2.0)
    
    print("[1.2] 發送伸展（開）命令...")
    hand.open()
    print("      等待 2 秒...")
    time.sleep(2.0)
    
    print("[1.3] 發送收緊（合）命令...")
    hand.close()
    print("      等待 2 秒...")
    time.sleep(2.0)
    
    print("[1.4] 回到伸展狀態...")
    hand.open()
    time.sleep(1.0)
    
    print("✅ 基本命令測試完成！")


def test_position_mode(hand: DexterousHand):
    """測試位置模式"""
    print("\n" + "=" * 60)
    print("測試 2: 位置模式 (漸進開合)")
    print("=" * 60)
    
    grip_values = [0.0, 0.25, 0.5, 0.75, 1.0, 0.75, 0.5, 0.25, 0.0]
    
    for grip in grip_values:
        print(f"[2.x] grip = {grip:.2f} (position = {int(grip * 255)})")
        hand.set_grip(grip, speed=128, torque=128)
        time.sleep(0.8)
    
    print("✅ 位置模式測試完成！")


def test_individual_fingers(hand: DexterousHand):
    """測試單一手指控制"""
    print("\n" + "=" * 60)
    print("測試 3: 單一手指控制")
    print("=" * 60)
    
    finger_names = [
        "拇指旋轉 (Motor1)",
        "拇指伸縮 (Motor2)",
        "食指 (Motor3)",
        "中指 (Motor4)",
        "無名指 (Motor5)",
        "尾指 (Motor6)",
    ]
    
    # 先全部打開
    print("[3.0] 先打開所有手指...")
    hand.open()
    time.sleep(1.0)
    
    # 逐一測試每根手指
    for i, name in enumerate(finger_names):
        print(f"[3.{i+1}] 測試 {name}...")
        
        # 收緊
        print(f"       收緊...")
        hand.set_finger(i, 200, speed=100, torque=100)
        time.sleep(0.8)
        
        # 放鬆
        print(f"       放鬆...")
        hand.set_finger(i, 50, speed=100, torque=100)
        time.sleep(0.8)
    
    # 最後全部打開
    print("[3.7] 恢復到打開狀態...")
    hand.open()
    time.sleep(1.0)
    
    print("✅ 單一手指控制測試完成！")


def test_speed_torque(hand: DexterousHand):
    """測試速度和力矩參數"""
    print("\n" + "=" * 60)
    print("測試 4: 速度/力矩參數")
    print("=" * 60)
    
    print("[4.1] 低速低力矩 (speed=50, torque=50)...")
    hand.set_grip(1.0, speed=50, torque=50)
    time.sleep(2.0)
    
    hand.set_grip(0.0, speed=50, torque=50)
    time.sleep(2.0)
    
    print("[4.2] 高速高力矩 (speed=200, torque=200)...")
    hand.set_grip(1.0, speed=200, torque=200)
    time.sleep(1.0)
    
    hand.set_grip(0.0, speed=200, torque=200)
    time.sleep(1.0)
    
    print("✅ 速度/力矩測試完成！")


def interactive_mode(hand: DexterousHand):
    """互動模式"""
    print("\n" + "=" * 60)
    print("互動模式")
    print("=" * 60)
    print("命令:")
    print("  h = 回零 (home)")
    print("  o = 伸展 (open)")
    print("  c = 收緊 (close)")
    print("  g <0~100> = 設定握合百分比")
    print("  f <0~5> <0~255> = 設定單一手指位置")
    print("  q = 離開")
    print("-" * 60)
    
    while True:
        try:
            cmd = input("\n指令> ").strip().lower()
            
            if cmd == 'q':
                print("離開互動模式...")
                break
            elif cmd == 'h':
                print("執行: 回零")
                hand.home()
            elif cmd == 'o':
                print("執行: 伸展")
                hand.open()
            elif cmd == 'c':
                print("執行: 收緊")
                hand.close()
            elif cmd.startswith('g '):
                try:
                    percent = float(cmd[2:])
                    grip = percent / 100.0
                    print(f"執行: 握合 {percent}% (grip={grip:.2f})")
                    hand.set_grip(grip)
                except ValueError:
                    print("錯誤: 請輸入數字 (0~100)")
            elif cmd.startswith('f '):
                try:
                    parts = cmd[2:].split()
                    finger = int(parts[0])
                    pos = int(parts[1])
                    print(f"執行: 手指 {finger} 位置 {pos}")
                    hand.set_finger(finger, pos)
                except (ValueError, IndexError):
                    print("錯誤: 格式為 f <手指索引 0~5> <位置 0~255>")
            else:
                print("未知命令，請輸入 h/o/c/g/f/q")
                
        except KeyboardInterrupt:
            print("\n中斷...")
            break


def main():
    parser = argparse.ArgumentParser(description="靈巧手測試腳本")
    parser.add_argument("can_interface", nargs="?", default="can0",
                        help="CAN 介面名稱 (預設: can0)")
    parser.add_argument("can_id", nargs="?", default="0x01",
                        help="靈巧手 CAN ID (預設: 0x01)")
    parser.add_argument("-i", "--interactive", action="store_true",
                        help="進入互動模式")
    parser.add_argument("-t", "--test", type=int, default=0,
                        help="只執行指定測試 (1=基本, 2=位置, 3=單指, 4=速度力矩)")
    
    args = parser.parse_args()
    
    # 解析 CAN ID
    can_id = int(args.can_id, 0)  # 支援 0x 格式
    
    print("=" * 60)
    print("靈巧手測試腳本")
    print("=" * 60)
    print(f"CAN 介面: {args.can_interface}")
    print(f"CAN ID: 0x{can_id:02X}")
    print("=" * 60)
    
    # 連接靈巧手
    print(f"\n[INFO] 連接靈巧手於 {args.can_interface}...")
    
    try:
        hand = DexterousHand(args.can_interface, can_id=can_id)
        print("[INFO] ✅ 連接成功！")
    except Exception as e:
        print(f"[ERROR] ❌ 連接失敗: {e}")
        print("\n請確認:")
        print("  1. CAN 介面已設定為 CAN FD 模式")
        print("  2. 靈巧手已連接並供電")
        print("  3. CAN ID 正確")
        sys.exit(1)
    
    try:
        if args.interactive:
            # 互動模式
            interactive_mode(hand)
        elif args.test > 0:
            # 執行單一測試
            if args.test == 1:
                test_basic_commands(hand)
            elif args.test == 2:
                test_position_mode(hand)
            elif args.test == 3:
                test_individual_fingers(hand)
            elif args.test == 4:
                test_speed_torque(hand)
            else:
                print(f"未知測試編號: {args.test}")
        else:
            # 執行所有測試
            test_basic_commands(hand)
            test_position_mode(hand)
            # test_individual_fingers(hand)  # 可選：單指測試較慢
            # test_speed_torque(hand)        # 可選
            
            print("\n" + "=" * 60)
            print("🎉 所有測試完成！")
            print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n\n[INFO] 測試中斷")
    except Exception as e:
        print(f"\n[ERROR] 測試失敗: {e}")
    finally:
        # 關閉前恢復到安全狀態
        print("\n[INFO] 恢復到打開狀態...")
        try:
            hand.open()
            time.sleep(0.5)
        except:
            pass
        
        print("[INFO] 斷開連線...")
        hand.disconnect()
        print("[INFO] 完成！")


if __name__ == "__main__":
    main()
