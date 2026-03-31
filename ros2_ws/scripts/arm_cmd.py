#!/usr/bin/env python3
"""手臂快速指令工具（繞過 VOLATILE QoS 限制）

用法：
  python3 arm_cmd.py R 0.13 0.05 0.01 0.61 0.03 0 1.20   # 右臂 7 關節
  python3 arm_cmd.py L -0.09 0.02 0.01 0.38 -0.03 -0.04 -1.15  # 左臂
  python3 arm_cmd.py HOME       # 雙臂歸零+禁用
  python3 arm_cmd.py R_HOME     # 右臂歸零+禁用
  python3 arm_cmd.py L_HOME     # 左臂歸零+禁用
  python3 arm_cmd.py ENABLE     # 雙臂重新啟用
  python3 arm_cmd.py R_ENABLE   # 右臂重新啟用
"""

import sys
import time
import rclpy
from sensor_msgs.msg import JointState

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    cmd = sys.argv[1]

    rclpy.init()
    node = rclpy.create_node("arm_cmd")

    # 使用 VOLATILE QoS 匹配 follower 的訂閱
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    volatile_qos = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE
    )
    pub = node.create_publisher(JointState, "/unity/joint_commands", volatile_qos)

    msg = JointState()

    # 特殊指令
    if cmd.upper() in ('HOME', 'L_HOME', 'R_HOME', 'ENABLE', 'L_ENABLE', 'R_ENABLE'):
        msg.name = [cmd.upper()]
        msg.position = [0.0]
        print(f">> {cmd.upper()}")
    # 關節指令
    elif cmd.upper() in ('R', 'L') and len(sys.argv) >= 9:
        side = cmd.upper()
        positions = [float(x) for x in sys.argv[2:9]]
        msg.name = [f"{side}_J{i+1}" for i in range(7)]
        msg.position = positions
        print(f">> {side} arm: {positions}")
    else:
        print(f"Error: unknown command or missing arguments")
        print(__doc__)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # 等 DDS discovery
    time.sleep(2)

    # 發送
    for _ in range(20):
        pub.publish(msg)
        time.sleep(0.05)

    print("<< done")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
