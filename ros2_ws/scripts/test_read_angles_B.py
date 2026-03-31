#!/usr/bin/env python3
"""
方式 B：openarm_can 函式庫讀取馬達角度

原理：用 openarm_can C++ binding，呼叫 refresh_all() + recv_all()
自動解碼 CAN 回應為 position/velocity/torque

用法：python3 test_read_angles_B.py [can1|can2]
"""

import sys
import math

try:
    import openarm_can as oa
except ImportError:
    print("❌ 無法 import openarm_can，請確認已安裝")
    print("   cd ~/openarm_can && pip install .")
    sys.exit(1)

# CAN bus（預設 can1 = 右臂）
CAN_BUS = sys.argv[1] if len(sys.argv) > 1 else "can1"

# V10 馬達配置
MOTOR_TYPES = [
    oa.MotorType.DM8009,  # J1
    oa.MotorType.DM8009,  # J2
    oa.MotorType.DM4340,  # J3
    oa.MotorType.DM4340,  # J4
    oa.MotorType.DM4310,  # J5
    oa.MotorType.DM4310,  # J6
    oa.MotorType.DM4310,  # J7
]
SEND_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
RECV_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]

print(f"=== 方式 B: openarm_can 函式庫讀取角度 ===")
print(f"CAN bus: {CAN_BUS}")
print()

try:
    arm = oa.OpenArm(CAN_BUS, True)
    arm.init_arm_motors(MOTOR_TYPES, SEND_IDS, RECV_IDS)
    print("✅ OpenArm 初始化成功")
except Exception as e:
    print(f"❌ OpenArm 初始化失敗: {e}")
    sys.exit(1)

# 讀取角度（不 enable，不控制，只讀）
try:
    arm.refresh_all()
    arm.recv_all()
    print()
    print("--- 馬達角度 ---")
    for i, motor in enumerate(arm.get_arm().get_motors()):
        pos = motor.get_position()
        vel = motor.get_velocity()
        tau = motor.get_torque()
        print(f"  J{i+1}: {pos:+.4f} rad ({math.degrees(pos):+.2f}°)  vel={vel:+.3f}  tau={tau:+.3f}")
    print()
    print("=== 方式 B 完成 ===")
except Exception as e:
    print(f"❌ 讀取失敗: {e}")
    sys.exit(1)
