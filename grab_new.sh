#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

pub() {
  ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState "{name: [$1], position: [$2]}" --once 2>/dev/null
}

echo "=== 夾取新位置方塊 ==="

echo "1. J4 舉起 90°"
pub "\"R_J4\"" "1.5708"
sleep 0.8

echo "2. 打開夾爪"
pub "\"R_EE\"" "0.0425"
sleep 0.5

echo "3. J1 旋轉到新位置 (往左調整)"
# 原本 J1=0.087, X 往左移 13.2mm, J1 減少約 0.04
pub "\"R_J1\"" "0.047"
sleep 0.5

echo "4. J2, J3 伸展"
pub "\"R_J2\", \"R_J3\"" "0.62, 0.667"
sleep 0.5

echo "5. J4, J6 放下對準"
pub "\"R_J4\", \"R_J6\"" "0.277, 0.577"
sleep 1.0

echo "6. 夾取方塊"
pub "\"R_EE\"" "0.0"
echo "   >>> 等待 3 秒穩定..."
sleep 3

echo "7. J4 舉起方塊"
pub "\"R_J4\"" "1.5708"
sleep 1.0

echo "8. J4 放下方塊"
pub "\"R_J4\"" "0.277"
sleep 1.0

echo "   >>> 等待 3 秒穩定..."
sleep 3

echo "9. 放開夾爪"
pub "\"R_EE\"" "0.0425"
sleep 0.5

echo "=== 開始歸零 ==="

echo "10. J4 舉起"
pub "\"R_J4\"" "1.5708"
sleep 0.5

echo "11. J6 歸 0"
pub "\"R_J6\"" "0.0"
sleep 0.5

echo "12. J1 歸 0"
pub "\"R_J1\"" "0.0"
sleep 0.5

echo "13. J2, J3, J5, J7 歸 0"
pub "\"R_J2\", \"R_J3\", \"R_J5\", \"R_J7\"" "0.0, 0.0, 0.0, 0.0"
sleep 0.5

echo "14. J4 歸 0"
pub "\"R_J4\"" "0.0"
sleep 0.5

echo "15. 夾爪歸 0"
pub "\"R_EE\"" "0.0"

echo "=== 完成！ ==="
