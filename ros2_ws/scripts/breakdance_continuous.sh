#!/bin/bash
# 霹靂舞動作腳本 - 雙手版本 (左手往前，右手跳舞)
# 清理函數  
cleanup() {
    echo -e "\n停止霹靂舞，回到初始位置..."
    pkill -P $$
    
    # 左手回到初始位置
    ros2 action send_goal /left_joint_trajectory_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {
          joint_names: [
            'openarm_left_joint1',
            'openarm_left_joint2', 
            'openarm_left_joint3',
            'openarm_left_joint4',
            'openarm_left_joint5',
            'openarm_left_joint6',
            'openarm_left_joint7'
          ],
          points: [{
            positions: [0, 0, 0, 0, 0, 0, 0],
            time_from_start: {sec: 3}
          }]
        }}" > /dev/null 2>&1 &
    
    # 右手回到初始位置
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
    echo "霹靂舞已停止"
    exit 0
}

# 設置 Ctrl+C 的處理
trap cleanup SIGINT

echo "🎵 開始霹靂舞表演..."
echo "按 Ctrl+C 停止"

# 準備姿勢
echo "準備動作..."

# 左手往前伸展 (joint1 正值 = 往前，其他關節與右手鏡像)
ros2 action send_goal /left_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
      joint_names: [
        'openarm_left_joint1',
        'openarm_left_joint2',
        'openarm_left_joint3',
        'openarm_left_joint4',
        'openarm_left_joint5',
        'openarm_left_joint6',
        'openarm_left_joint7'
      ],
      points: [
      {
        positions: [-1.0, -0.5, -0.5, -0.5, 0, 0, 0],
        time_from_start: {sec: 2}
      }
      ]
    }}" > /dev/null 2>&1 &

# 右手準備舞蹈
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
        positions: [0, 0.5, 0.5, 0.5, 0, 0, 0],
        time_from_start: {sec: 2}
      }
      ]
    }}" > /dev/null 2>&1

sleep 2.5

# 動作1: 波浪動作 (Wave)
wave_move() {
    echo "  🌊 波浪動作..."
    # 左手往前伸展波浪 (與右手鏡像)
    ros2 action send_goal /left_joint_trajectory_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {
          joint_names: [
            'openarm_left_joint1',
            'openarm_left_joint2',
            'openarm_left_joint3',
            'openarm_left_joint4',
            'openarm_left_joint5',
            'openarm_left_joint6',
            'openarm_left_joint7'
          ],
          points: [
          {
            positions: [-1.0, -0.8, -0.6, -0.4, -0.3, -0.2, 0],
            time_from_start: {sec: 0, nanosec: 500000000}
          },
          {
            positions: [-1.0, -1.2, -0.9, -0.8, -0.6, -0.4, -0.2],
            time_from_start: {sec: 1}
          },
          {
            positions: [-1.0, -1.5, -1.2, -1.2, -0.9, -0.6, -0.3],
            time_from_start: {sec: 1, nanosec: 500000000}
          },
          {
            positions: [-1.0, -1.2, -0.9, -0.8, -0.6, -0.4, -0.2],
            time_from_start: {sec: 2}
          },
          {
            positions: [-1.0, -0.8, -0.6, -0.4, -0.3, -0.2, 0],
            time_from_start: {sec: 2, nanosec: 500000000}
          }
          ]
        }}" > /dev/null 2>&1 &
    
    # 右手波浪動作
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
            positions: [0, 0.8, 0.6, 0.4, 0.3, 0.2, 0],
            time_from_start: {sec: 0, nanosec: 500000000}
          },
          {
            positions: [0, 1.2, 0.9, 0.8, 0.6, 0.4, 0.2],
            time_from_start: {sec: 1}
          },
          {
            positions: [0, 1.5, 1.2, 1.2, 0.9, 0.6, 0.3],
            time_from_start: {sec: 1, nanosec: 500000000}
          },
          {
            positions: [0, 1.2, 0.9, 0.8, 0.6, 0.4, 0.2],
            time_from_start: {sec: 2}
          },
          {
            positions: [0, 0.8, 0.6, 0.4, 0.3, 0.2, 0],
            time_from_start: {sec: 2, nanosec: 500000000}
          }
          ]
        }}" > /dev/null 2>&1
}

# 動作2: 旋轉動作 (Rotation)
rotation_move() {
    echo "  🌀 旋轉動作..."
    # 左手往前，手腕旋轉擺動 (與右手鏡像)
    ros2 action send_goal /left_joint_trajectory_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {
          joint_names: [
            'openarm_left_joint1',
            'openarm_left_joint2',
            'openarm_left_joint3',
            'openarm_left_joint4',
            'openarm_left_joint5',
            'openarm_left_joint6',
            'openarm_left_joint7'
          ],
          points: [
          {
            positions: [-1.0, -1.0, -0.8, -0.8, 0.5, 0.5, 0.5],
            time_from_start: {sec: 0, nanosec: 600000000}
          },
          {
            positions: [-1.0, -1.0, -0.8, -0.8, -0.5, -0.5, -0.5],
            time_from_start: {sec: 1, nanosec: 200000000}
          },
          {
            positions: [-1.0, -1.0, -0.8, -0.8, 0.5, 0.5, 0.5],
            time_from_start: {sec: 1, nanosec: 800000000}
          },
          {
            positions: [-1.0, -1.0, -0.8, -0.8, 0, 0, 0],
            time_from_start: {sec: 2, nanosec: 400000000}
          }
          ]
        }}" > /dev/null 2>&1 &
    
    # 右手旋轉動作
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
            positions: [0.5, 1.0, 0.8, 0.8, 0.5, 0.5, 0.5],
            time_from_start: {sec: 0, nanosec: 600000000}
          },
          {
            positions: [-0.5, 1.0, 0.8, 0.8, -0.5, -0.5, -0.5],
            time_from_start: {sec: 1, nanosec: 200000000}
          },
          {
            positions: [0.5, 1.0, 0.8, 0.8, 0.5, 0.5, 0.5],
            time_from_start: {sec: 1, nanosec: 800000000}
          },
          {
            positions: [0, 1.0, 0.8, 0.8, 0, 0, 0],
            time_from_start: {sec: 2, nanosec: 400000000}
          }
          ]
        }}" > /dev/null 2>&1
}

# 動作3: 上下擺動 (Bounce)
bounce_move() {
    echo "  ⬆️⬇️ 上下擺動..."
    # 左手往前上下擺動 (與右手鏡像)
    ros2 action send_goal /left_joint_trajectory_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {
          joint_names: [
            'openarm_left_joint1',
            'openarm_left_joint2',
            'openarm_left_joint3',
            'openarm_left_joint4',
            'openarm_left_joint5',
            'openarm_left_joint6',
            'openarm_left_joint7'
          ],
          points: [
          {
            positions: [-1.0, -1.5, -1.0, -1.0, 0, 0, 0],
            time_from_start: {sec: 0, nanosec: 400000000}
          },
          {
            positions: [-1.0, -0.3, -0.3, -0.3, 0, 0, 0],
            time_from_start: {sec: 0, nanosec: 800000000}
          },
          {
            positions: [-1.0, -1.5, -1.0, -1.0, 0, 0, 0],
            time_from_start: {sec: 1, nanosec: 200000000}
          },
          {
            positions: [-1.0, -0.3, -0.3, -0.3, 0, 0, 0],
            time_from_start: {sec: 1, nanosec: 600000000}
          },
          {
            positions: [-1.0, -1.0, -0.7, -0.7, 0, 0, 0],
            time_from_start: {sec: 2}
          }
          ]
        }}" > /dev/null 2>&1 &
    
    # 右手上下擺動
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
            positions: [0, 1.5, 1.0, 1.0, 0, 0, 0],
            time_from_start: {sec: 0, nanosec: 400000000}
          },
          {
            positions: [0, 0.3, 0.3, 0.3, 0, 0, 0],
            time_from_start: {sec: 0, nanosec: 800000000}
          },
          {
            positions: [0, 1.5, 1.0, 1.0, 0, 0, 0],
            time_from_start: {sec: 1, nanosec: 200000000}
          },
          {
            positions: [0, 0.3, 0.3, 0.3, 0, 0, 0],
            time_from_start: {sec: 1, nanosec: 600000000}
          },
          {
            positions: [0, 1.0, 0.7, 0.7, 0, 0, 0],
            time_from_start: {sec: 2}
          }
          ]
        }}" > /dev/null 2>&1
}

# 動作4: 手腕擺動 (Wrist Shake)
wrist_shake_move() {
    echo "  👋 手腕擺動..."
    # 左手往前手腕快速擺動 (與右手鏡像)
    ros2 action send_goal /left_joint_trajectory_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {
          joint_names: [
            'openarm_left_joint1',
            'openarm_left_joint2',
            'openarm_left_joint3',
            'openarm_left_joint4',
            'openarm_left_joint5',
            'openarm_left_joint6',
            'openarm_left_joint7'
          ],
          points: [
          {
            positions: [-1.0, -1.2, -0.9, -0.6, 0.8, 0.8, 0.8],
            time_from_start: {sec: 0, nanosec: 300000000}
          },
          {
            positions: [-1.0, -1.2, -0.9, -0.6, -0.8, -0.8, -0.8],
            time_from_start: {sec: 0, nanosec: 600000000}
          },
          {
            positions: [-1.0, -1.2, -0.9, -0.6, 0.8, 0.8, 0.8],
            time_from_start: {sec: 0, nanosec: 900000000}
          },
          {
            positions: [-1.0, -1.2, -0.9, -0.6, -0.8, -0.8, -0.8],
            time_from_start: {sec: 1, nanosec: 200000000}
          },
          {
            positions: [-1.0, -1.2, -0.9, -0.6, 0, 0, 0],
            time_from_start: {sec: 1, nanosec: 500000000}
          }
          ]
        }}" > /dev/null 2>&1 &
    
    # 右手手腕擺動
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
            positions: [0, 1.2, 0.9, 0.6, 0.8, 0.8, 0.8],
            time_from_start: {sec: 0, nanosec: 300000000}
          },
          {
            positions: [0, 1.2, 0.9, 0.6, -0.8, -0.8, -0.8],
            time_from_start: {sec: 0, nanosec: 600000000}
          },
          {
            positions: [0, 1.2, 0.9, 0.6, 0.8, 0.8, 0.8],
            time_from_start: {sec: 0, nanosec: 900000000}
          },
          {
            positions: [0, 1.2, 0.9, 0.6, -0.8, -0.8, -0.8],
            time_from_start: {sec: 1, nanosec: 200000000}
          },
          {
            positions: [0, 1.2, 0.9, 0.6, 0, 0, 0],
            time_from_start: {sec: 1, nanosec: 500000000}
          }
          ]
        }}" > /dev/null 2>&1
}

# 計數器
count=0
echo "🕺 開始霹靂舞循環表演..."
echo ""

# 持續循環
while true; do
    ((count++))
    echo "🎪 第 $count 輪霹靂舞表演"
    
    # 執行各種動作組合
    wave_move
    sleep 2.8
    
    rotation_move
    sleep 2.7
    
    bounce_move
    sleep 2.3
    
    wrist_shake_move
    sleep 1.8
    
    echo "  ✨ 第 $count 輪完成"
    echo ""
    sleep 0.5
done
