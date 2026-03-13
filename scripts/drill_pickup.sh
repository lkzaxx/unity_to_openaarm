#!/bin/bash
# 電鑽拿取腳本
# 注意：執行前需確保 follower 已啟動

set -e

# ROS2 環境
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 位置定義 (弧度)
INIT_POS="[-0.13, 0.05, 0.01, 0.61, 0.03, -0.02, 1.20]"
DRILL_POS="[0.2643, 0.0, -0.3, 1.0127, 0.0524, -0.2, 0.5]"

# 中間位置 (用於平滑移動)
MID1_POS="[-0.13, 0.03, -0.1, 0.85, 0.03, -0.1, 0.85]"
MID2_POS="[0.1, 0.0, -0.2, 1.0127, 0.03, -0.15, 0.65]"

# 抬起位置
LIFT1_POS="[0.613, 0.0, -0.3, 1.0127, 0.0524, -0.2, 0.5]"   # J1 +20°
LIFT2_POS="[0.788, 0.0, -0.3, 0.926, 0.0524, -0.2, 0.5]"    # J1 +10°, J4 -5°

echo "=== 電鑽拿取腳本 ==="
echo ""

# ============================================
# 步驟 1: 從初始位置移動到電鑽位置 (約3秒)
# ============================================
echo "[步驟 1] 移動到電鑽位置..."
echo "  J4 先移動..."
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: [R_J1,R_J2,R_J3,R_J4,R_J5,R_J6,R_J7], position: [-0.13, 0.05, 0.01, 1.0127, 0.03, -0.02, 1.20]}" --once
sleep 0.8

echo "  J1 + J7 移動..."
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: [R_J1,R_J2,R_J3,R_J4,R_J5,R_J6,R_J7], position: [0.2643, 0.0, 0.01, 1.0127, 0.03, -0.1, 0.85]}" --once
sleep 0.8

echo "  J3 + J6 + J7 移動..."
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: [R_J1,R_J2,R_J3,R_J4,R_J5,R_J6,R_J7], position: [0.2643, 0.0, -0.3, 1.0127, 0.0524, -0.2, 0.5]}" --once
sleep 1.4

echo "  ✓ 到達電鑽位置"
echo ""

# ============================================
# 步驟 2: 手握緊 (除了食指)
# ============================================
echo "[步驟 2] 手握緊 (食指除外)..."
# 先啟用手部控制
ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
  "{name: [R_HAND_ENABLE], position: [0.0]}" --once
sleep 0.5

# 握緊 (食指 = 0.0 = 張開, 其他 = 高值 = 握緊)
timeout 3 ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
  "{name: [R_F1,R_F2,R_F3,R_F4,R_F5,R_F6], position: [0.78, 0.84, 0.0, 0.82, 0.85, 0.83]}" --rate 5
sleep 0.5
echo "  ✓ 手握緊完成"
echo ""

# ============================================
# 步驟 3: J1 往上 20 度
# ============================================
echo "[步驟 3] J1 往上 20°..."
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: [R_J1,R_J2,R_J3,R_J4,R_J5,R_J6,R_J7], position: [0.613, 0.0, -0.3, 1.0127, 0.0524, -0.2, 0.5]}" --once
sleep 1.5
echo "  ✓ J1 已抬起"
echo ""

# ============================================
# 步驟 4: 同時 J4 下降 5 度 + J1 往上 10 度 (拿起動作)
# ============================================
echo "[步驟 4] 電鑽拿起動作 (J1+10°, J4-5°)..."
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: [R_J1,R_J2,R_J3,R_J4,R_J5,R_J6,R_J7], position: [0.788, 0.0, -0.3, 0.926, 0.0524, -0.2, 0.5]}" --once
sleep 1.5
echo "  ✓ 電鑽已拿起"
echo ""

# ============================================
# 步驟 5: 保持 10 秒
# ============================================
echo "[步驟 5] 保持 10 秒..."
for i in {10..1}; do
  echo "  剩餘 $i 秒..."
  sleep 1
done
echo "  ✓ 保持完成"
echo ""

# ============================================
# 步驟 6: 沿原路徑放回電鑽位置
# ============================================
echo "[步驟 6] 放回電鑽..."
echo "  J4 + J1 復原..."
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: [R_J1,R_J2,R_J3,R_J4,R_J5,R_J6,R_J7], position: [0.613, 0.0, -0.3, 1.0127, 0.0524, -0.2, 0.5]}" --once
sleep 1.5

echo "  J1 復原..."
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: [R_J1,R_J2,R_J3,R_J4,R_J5,R_J6,R_J7], position: [0.2643, 0.0, -0.3, 1.0127, 0.0524, -0.2, 0.5]}" --once
sleep 1.5
echo "  ✓ 已放回電鑽位置"
echo ""

# ============================================
# 步驟 7: 放開手 (偵測位置，沒放開就禁止+歸零)
# ============================================
echo "[步驟 7] 放開手..."
# 先嘗試張開
ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
  "{name: [R_HAND_OPEN], position: [0.0]}" --once
sleep 2

# 讀取位置
ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
  "{name: [R_HAND_STATUS], position: [0.0]}" --once
sleep 1

# 如果沒放開，使用禁止+歸零
echo "  備用方案：禁止 + 歸零..."
ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
  "{name: [R_HAND_DISABLE], position: [0.0]}" --once
sleep 1
ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
  "{name: [R_HAND_HOME], position: [0.0]}" --once
sleep 2
echo "  ✓ 手已放開"
echo ""

# ============================================
# 步驟 8: 手臂回到初始位置 (反向順序)
# ============================================
echo "[步驟 8] 手臂回到初始位置..."
echo "  J6 復原..."
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: [R_J1,R_J2,R_J3,R_J4,R_J5,R_J6,R_J7], position: [0.2643, 0.0, -0.3, 1.0127, 0.0524, -0.02, 0.5]}" --once
sleep 0.8

echo "  J3 復原..."
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: [R_J1,R_J2,R_J3,R_J4,R_J5,R_J6,R_J7], position: [0.2643, 0.0, 0.01, 1.0127, 0.0524, -0.02, 0.5]}" --once
sleep 0.8

echo "  J7 復原..."
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: [R_J1,R_J2,R_J3,R_J4,R_J5,R_J6,R_J7], position: [0.2643, 0.0, 0.01, 1.0127, 0.0524, -0.02, 1.20]}" --once
sleep 0.8

echo "  J1 復原..."
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: [R_J1,R_J2,R_J3,R_J4,R_J5,R_J6,R_J7], position: [-0.13, 0.0, 0.01, 1.0127, 0.03, -0.02, 1.20]}" --once
sleep 0.8

echo "  J4 + 其他 復原..."
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: [R_J1,R_J2,R_J3,R_J4,R_J5,R_J6,R_J7], position: [-0.13, 0.05, 0.01, 0.61, 0.03, -0.02, 1.20]}" --once
sleep 1.5

echo "  ✓ 已回到初始位置"
echo ""

echo "=== 腳本完成 ==="
