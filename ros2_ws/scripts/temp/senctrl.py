#!/usr/bin/env python3
import sys
import subprocess

# 检查参数数量
if len(sys.argv) != 8:
    print("用法: python3 send_openarm_goal.py j1 j2 j3 j4 j5 j6 j7")
    print("示例: python3 send_openarm_goal.py 0.1 -0.5 0 0 0.3 0 0")
    sys.exit(1)

# 读取并检查范围
positions = []
for i, arg in enumerate(sys.argv[1:], start=1):
    try:
        value = float(arg)
    except ValueError:
        print(f"❌ 第 {i} 个参数 '{arg}' 不是数字")
        sys.exit(1)
    if not -10.0 <= value <= 10.0:
        print(f"❌ 第 {i} 个关节值 {value} 超出范围 [-1, 1]")
        sys.exit(1)
    positions.append(value)

# 构造 ROS2 action 命令
joint_names = [
    "openarm_joint1", "openarm_joint2", "openarm_joint3",
    "openarm_joint4", "openarm_joint5", "openarm_joint6", "openarm_joint7"
]

ros_cmd = (
    "ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory "
    "control_msgs/action/FollowJointTrajectory "
    f"\"{{trajectory: {{joint_names: {joint_names}, "
    f"points: [{{positions: {positions}, time_from_start: {{sec: 3, nanosec: 0}}}}]}}}}\""
)

print("✅ 将执行命令：")
print(ros_cmd)
print()

# 执行命令
try:
    subprocess.run(ros_cmd, shell=True, check=True)
except subprocess.CalledProcessError as e:
    print(f"❌ ROS2 命令执行失败: {e}")
