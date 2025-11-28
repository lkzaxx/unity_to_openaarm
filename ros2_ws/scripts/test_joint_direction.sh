#!/bin/bash
# 關節方向測試腳本
# 用於測試每個關節的正負方向

echo "=========================================="
echo "   機械手臂關節方向測試工具"
echo "=========================================="
echo ""

# 回到初始位置的函數
reset_position() {
    local arm=$1
    echo "將 ${arm} 回到初始位置..."
    
    ros2 action send_goal /${arm}_joint_trajectory_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {
          joint_names: [
            'openarm_${arm}_joint1',
            'openarm_${arm}_joint2', 
            'openarm_${arm}_joint3',
            'openarm_${arm}_joint4',
            'openarm_${arm}_joint5',
            'openarm_${arm}_joint6',
            'openarm_${arm}_joint7'
          ],
          points: [{
            positions: [0, 0, 0, 0, 0, 0, 0],
            time_from_start: {sec: 2}
          }]
        }}" > /dev/null 2>&1
    
    sleep 2.5
    echo "✓ 已回到初始位置"
}

# 測試單個關節的函數
test_joint() {
    local arm=$1
    local joint=$2
    local value=$3
    
    # 構建 7 個關節的位置數組
    local positions="0, 0, 0, 0, 0, 0, 0"
    
    # 將指定關節設為測試值
    case $joint in
        1) positions="$value, 0, 0, 0, 0, 0, 0" ;;
        2) positions="0, $value, 0, 0, 0, 0, 0" ;;
        3) positions="0, 0, $value, 0, 0, 0, 0" ;;
        4) positions="0, 0, 0, $value, 0, 0, 0" ;;
        5) positions="0, 0, 0, 0, $value, 0, 0" ;;
        6) positions="0, 0, 0, 0, 0, $value, 0" ;;
        7) positions="0, 0, 0, 0, 0, 0, $value" ;;
    esac
    
    echo ""
    echo "🎯 測試 ${arm} 手 Joint${joint} = ${value}"
    echo "   執行動作中..."
    
    ros2 action send_goal /${arm}_joint_trajectory_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {
          joint_names: [
            'openarm_${arm}_joint1',
            'openarm_${arm}_joint2',
            'openarm_${arm}_joint3',
            'openarm_${arm}_joint4',
            'openarm_${arm}_joint5',
            'openarm_${arm}_joint6',
            'openarm_${arm}_joint7'
          ],
          points: [{
            positions: [$positions],
            time_from_start: {sec: 2}
          }]
        }}" > /dev/null 2>&1
    
    sleep 2.5
    echo "✓ 動作完成"
}

# 主循環
while true; do
    echo ""
    echo "=========================================="
    echo "請選擇操作："
    echo "1) 測試左手關節"
    echo "2) 測試右手關節"
    echo "3) 左手回初始位置"
    echo "4) 右手回初始位置"
    echo "5) 雙手回初始位置"
    echo "0) 退出"
    echo "=========================================="
    read -p "請輸入選項 [0-5]: " choice
    
    case $choice in
        0)
            echo "退出測試工具"
            exit 0
            ;;
        1)
            arm="left"
            ;;
        2)
            arm="right"
            ;;
        3)
            reset_position "left"
            continue
            ;;
        4)
            reset_position "right"
            continue
            ;;
        5)
            reset_position "left" &
            reset_position "right" &
            wait
            continue
            ;;
        *)
            echo "❌ 無效選項，請重新選擇"
            continue
            ;;
    esac
    
    # 選擇關節
    echo ""
    echo "請選擇要測試的關節："
    echo "1) Joint 1 (基座旋轉 - 左右/前後)"
    echo "2) Joint 2 (肩部 - 上下)"
    echo "3) Joint 3 (上臂)"
    echo "4) Joint 4 (肘部)"
    echo "5) Joint 5 (手腕旋轉1)"
    echo "6) Joint 6 (手腕旋轉2)"
    echo "7) Joint 7 (手腕旋轉3)"
    read -p "請輸入關節編號 [1-7]: " joint
    
    if [[ ! $joint =~ ^[1-7]$ ]]; then
        echo "❌ 無效的關節編號"
        continue
    fi
    
    # 選擇測試值
    echo ""
    echo "請選擇測試方向和幅度："
    echo "1) +0.5 (小幅正向)"
    echo "2) +1.0 (中幅正向)"
    echo "3) +1.5 (大幅正向)"
    echo "4) -0.5 (小幅負向)"
    echo "5) -1.0 (中幅負向)"
    echo "6) -1.5 (大幅負向)"
    echo "7) 自訂數值"
    read -p "請輸入選項 [1-7]: " value_choice
    
    case $value_choice in
        1) value=0.5 ;;
        2) value=1.0 ;;
        3) value=1.5 ;;
        4) value=-0.5 ;;
        5) value=-1.0 ;;
        6) value=-1.5 ;;
        7)
            read -p "請輸入自訂數值 (例如: 0.8 或 -1.2): " value
            ;;
        *)
            echo "❌ 無效選項"
            continue
            ;;
    esac
    
    # 執行測試
    test_joint "$arm" "$joint" "$value"
    
    # 詢問是否回到初始位置
    echo ""
    read -p "是否將 ${arm} 手回到初始位置? [y/n]: " reset_choice
    if [[ $reset_choice == "y" || $reset_choice == "Y" ]]; then
        reset_position "$arm"
    fi
done
