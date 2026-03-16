#!/bin/bash
# 電鑽拿取腳本 v2 - 平滑移動版
# 注意：執行前需確保 follower 已啟動

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ROS2 環境
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# 位置定義 (弧度)
INIT_POS='[-0.13, 0.05, 0.01, 0.61, 0.03, -0.02, 1.20]'
DRILL_POS='[0.2643, 0.0, -0.3, 1.0127, 0.0524, -0.2, 0.5]'
LIFT1_POS='[0.613, 0.0, -0.3, 1.0127, 0.0524, -0.2, 0.5]'   # J1 +20°
LIFT2_POS='[0.788, 0.0, -0.3, 0.926, 0.0524, -0.2, 0.5]'    # J1 +10°, J4 -5°

echo "=== 電鑽拿取腳本 v2 (平滑版) ==="
echo ""

# ============================================
# 步驟 1: 從初始位置移動到電鑽位置 (平滑)
# ============================================
echo "[步驟 1] 平滑移動到電鑽位置..."
python3 "$SCRIPT_DIR/smooth_move.py" \
    --start "$INIT_POS" \
    --end "$DRILL_POS" \
    --duration 2.0 \
    --rate 50
echo "  ✓ 到達電鑽位置"
echo ""

# ============================================
# 步驟 2: 手握緊 (除了食指)
# ============================================
echo "[步驟 2] 手握緊 (食指除外)..."
ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
    "{name: [R_HAND_ENABLE], position: [0.0]}" --once
sleep 0.5

timeout 3 ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
    "{name: [R_F1,R_F2,R_F3,R_F4,R_F5,R_F6], position: [0.78, 0.84, 0.0, 0.82, 0.85, 0.83]}" --rate 5 || true
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
# 步驟 4: 拿起動作 (J1+10°, J4-5°)
# ============================================
echo "[步驟 4] 電鑽拿起動作..."
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
    "{name: [R_J1,R_J2,R_J3,R_J4,R_J5,R_J6,R_J7], position: [0.788, 0.0, -0.3, 0.926, 0.0524, -0.2, 0.5]}" --once
sleep 1.5
echo "  ✓ 電鑽已拿起"
echo ""

# ============================================
# 步驟 5: 保持 5 秒
# ============================================
echo "[步驟 5] 保持 5 秒..."
for i in 5 4 3 2 1; do
    echo "  剩餘 $i 秒..."
    sleep 1
done
echo "  ✓ 保持完成"
echo ""

# ============================================
# 步驟 6: 平滑放回電鑽位置
# ============================================
echo "[步驟 6] 平滑放回電鑽..."
python3 "$SCRIPT_DIR/smooth_move.py" \
    --start "$LIFT2_POS" \
    --end "$DRILL_POS" \
    --duration 1.5 \
    --rate 50
echo "  ✓ 已放回電鑽位置"
echo ""

# ============================================
# 步驟 7: 放開手
# ============================================
echo "[步驟 7] 放開手..."
ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
    "{name: [R_HAND_OPEN], position: [0.0]}" --once
sleep 2
echo "  ✓ 手已放開"
echo ""

# ============================================
# 步驟 8: 平滑回到初始位置
# ============================================
echo "[步驟 8] 平滑回到初始位置..."
python3 "$SCRIPT_DIR/smooth_move.py" \
    --start "$DRILL_POS" \
    --end "$INIT_POS" \
    --duration 2.0 \
    --rate 50
echo "  ✓ 已回到初始位置"
echo ""

echo "=== 腳本完成 ==="
