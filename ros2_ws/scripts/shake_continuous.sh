#!/bin/bash
# 清理函數  
cleanup() {
    echo -e "\n停止動作，回到初始位置..."
    pkill -P $$
    
    # 回到初始位置
    ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {
          joint_names: [
            'openarm_right_joint1',
            'openarm_right_joint2', 
            'openarm_right_joint3',
            'openarm_right_joint4',
            'openarm_right_joint5',
            'openarm_right_joint6',
            'openarm_right_joint7'
          ],
          points: [{
            positions: [0, 0, 0, 0, 0, 0, 0],
            time_from_start: {sec: 3}
          }]
        }}" > /dev/null 2>&1
    
    sleep 3
    echo "已停止"
    exit 0
}
# 設置 Ctrl+C 的處理
trap cleanup SIGINT
echo "開始持續招手..."
echo "按 Ctrl+C 停止"

# 先舉起手臂到準備位置
echo "準備動作..."
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
      joint_names: [
        'openarm_right_joint1',
        'openarm_right_joint2',
        'openarm_right_joint3',
        'openarm_right_joint4',
        'openarm_right_joint5',
        'openarm_right_joint6',
        'openarm_right_joint7'
      ],
      points: [
      {
        positions: [0, 0.7, 0.6, 0.6, 0, 0, 0],
        time_from_start: {sec: 1, nanosec: 500000000}
      },
      {
        positions: [0, 1.5, 1.2, 1.2, 0, 0, 0],
        time_from_start: {sec: 3}
      }
      ]
    }}" > /dev/null 2>&1

sleep 3.5  # 等待準備動作完成

# 招手的函數（現在從已舉起的位置開始）
wave_once() {
    ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {
          joint_names: [
            'openarm_right_joint1',
            'openarm_right_joint2',
            'openarm_right_joint3',
            'openarm_right_joint4',
            'openarm_right_joint5',
            'openarm_right_joint6',
            'openarm_right_joint7'
          ],
          points: [
          {
            positions: [0, 1.5, 1.2, 1.8, 0, 0, 0],
            time_from_start: {sec: 0, nanosec: 800000000}
          },
          {
            positions: [0, 1.5, 1.2, 1.2, 0, 0, 0],
            time_from_start: {sec: 1, nanosec: 600000000}
          },
          {
            positions: [0, 1.5, 1.2, 1.8, 0, 0, 0],
            time_from_start: {sec: 2, nanosec: 400000000}
          },
          {
            positions: [0, 1.5, 1.2, 1.2, 0, 0, 0],
            time_from_start: {sec: 3, nanosec: 200000000}
          }
          ]
        }}" > /dev/null 2>&1
}

# 計數器
count=0
echo "開始招手循環..."

# 持續循環
while true; do
    wave_once
    ((count++))
    echo "招手次數: $count"
    sleep 1.6  # 等待當前動作完成
done