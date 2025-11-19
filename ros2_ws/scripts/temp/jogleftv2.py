#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
import sys, termios, tty

# 左臂 7 个关节名称
JOINTS = [
    "openarm_left_joint1",
    "openarm_left_joint2",
    "openarm_left_joint3",
    "openarm_left_joint4",
    "openarm_left_joint5",
    "openarm_left_joint6",
    "openarm_left_joint7"
]

STEP = 0.1  # 每次按键增加/减少角度
ACTION = "/left_joint_trajectory_controller/follow_joint_trajectory"
current_idx = 0  # 默认控制 joint1

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        c = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return c

class JogNode(Node):
    def __init__(self):
        super().__init__("jog_left_arm_final")
        self.joint_pos = {}
        self.sub = self.create_subscription(
            JointState, "/joint_states", self.cb_joint_states, 10
        )
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)

    def cb_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = pos

    def send_goal(self, positions):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100_000_000  # 0.1秒快速响应

        goal.trajectory.points = [point]
        self.client.wait_for_server()
        self.client.send_goal_async(goal)

def main():
    global current_idx
    rclpy.init()
    node = JogNode()

    print("数字键 1~7 切换关节, w/s 增加/减少角度, q 退出")
    print(f"当前控制关节: {JOINTS[current_idx]}")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            # 确保 joint_states 已准备好
            if not all(j in node.joint_pos for j in JOINTS):
                print("\r等待 joint_states...", end="", flush=True)
                continue

            current_positions = [node.joint_pos[j] for j in JOINTS]
            cur_joint = JOINTS[current_idx]
            current = current_positions[current_idx]
            print(f"\r控制: {cur_joint} 当前角度 {current:.3f} rad", end="", flush=True)

            c = getch()
            if c == "q":
                print("\n退出")
                break

            if c in "1234567":
                current_idx = int(c) - 1
                print(f"\n切换控制关节: {JOINTS[current_idx]}")
                continue

            # jog 单次按键
            target_positions = current_positions.copy()
            if c == "w":
                target_positions[current_idx] += STEP
                node.send_goal(target_positions)
                print(f"\n{cur_joint} -> {target_positions[current_idx]:.3f}")
            elif c == "s":
                target_positions[current_idx] -= STEP
                node.send_goal(target_positions)
                print(f"\n{cur_joint} -> {target_positions[current_idx]:.3f}")

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
