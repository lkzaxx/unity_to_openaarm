#!/usr/bin/env python3
"""eHand 快速指令工具

用法：
  python3 ehand_cmd.py home [R|L|B]        # 歸零
  python3 ehand_cmd.py open [R|L|B]        # 張開
  python3 ehand_cmd.py close [R|L|B]       # 全握
  python3 ehand_cmd.py disable [R|L|B]     # 禁用
  python3 ehand_cmd.py enable [R|L|B]      # 啟用
  python3 ehand_cmd.py grip [R|L|B] 0.5    # 指定握合度 (0~1)
  python3 ehand_cmd.py status [R|L|B]      # 讀狀態

B = 雙手, R = 右手(預設), L = 左手
"""

import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "home"
    side = sys.argv[2].upper() if len(sys.argv) > 2 else "R"
    value = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0

    # 構建訊息
    if cmd in ("home", "open", "close", "disable", "enable", "status"):
        action = cmd.upper()
        if side == "B":
            names = [f"HAND_{action}"]
        else:
            names = [f"{side}_HAND_{action}"]
        positions = [0.0]
    elif cmd == "grip":
        if side == "B":
            names = [f"{s}_F{i}" for s in ("L", "R") for i in range(1, 7)]
            positions = [value] * 12
        else:
            names = [f"{side}_F{i}" for i in range(1, 7)]
            positions = [value] * 6
    else:
        print(f"Unknown command: {cmd}")
        print(__doc__)
        sys.exit(1)

    print(f">> {cmd} {side} → {names}")

    rclpy.init()
    node = rclpy.create_node("ehand_cmd")
    pub = node.create_publisher(JointState, "/unity/ehand_commands", 10)

    msg = JointState()
    msg.name = names
    msg.position = positions

    # 等 DDS discovery
    time.sleep(2)

    # grip 需要先 ENABLE（DISABLE 後位置指令會被忽略）
    if cmd == "grip":
        en = JointState()
        if side == "B":
            en.name = ["HAND_ENABLE"]
        else:
            en.name = [f"{side}_HAND_ENABLE"]
        en.position = [0.0]
        for _ in range(10):
            pub.publish(en)
            time.sleep(0.05)
        time.sleep(0.5)

    # 發送指令
    for _ in range(20):
        pub.publish(msg)
        time.sleep(0.05)

    print("<< done")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
