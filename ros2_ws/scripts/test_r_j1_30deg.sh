#!/bin/bash
# 測試右手 J1 舉起 30 度，2 秒後自動停止

LOG=/tmp/test_r_j1_30deg.log
echo "[西元2026年02月24日 (星期二) 15時30分30秒    ] Starting test..." > $LOG

# 設置環境
source /opt/ros/humble/setup.bash
source /home/idaka/ros2_ws/install/setup.bash
export PYTHONPATH="/home/idaka/openarm_can:$PYTHONPATH"

# 記錄開始時的記憶體和 CAN 狀態
echo "=== 開始前記憶體 ===" >> $LOG
free -h | grep -E 'total|Mem' >> $LOG
echo "=== 開始前 CAN dropped ===" >> $LOG
ip -s link show can1 2>&1 | grep dropped >> $LOG
ip -s link show can2 2>&1 | grep dropped >> $LOG

# 切換到腳本目錄
cd /home/idaka/ros2_ws/src/openarm_ros2/openarm_bringup/scripts

# 啟動 follower (背景執行，最多 15 秒)
echo "[西元2026年02月24日 (星期二) 15時30分30秒    ] Starting follower..." >> $LOG
timeout 15 python3 unity_interface_follower.py >> $LOG 2>&1 &
FOLLOWER_PID=$!
echo "[西元2026年02月24日 (星期二) 15時30分30秒    ] Follower PID: $FOLLOWER_PID" >> $LOG

# 等待 follower 初始化 (4 秒)
sleep 4

# 發送右手 J1 = 30 度指令 (只發 2 秒)
# 0.5236 rad = 30 度
echo "[西元2026年02月24日 (星期二) 15時30分30秒    ] Sending R_J1 = 30 deg (0.5236 rad) for 2 seconds..." >> $LOG
timeout 2 ros2 topic pub /unity/joint_commands std_msgs/msg/Float32MultiArray "{data: [0,0,0,0,0,0,0, 0.5236,0,0,0,0,0,0]}" --rate 100 >> $LOG 2>&1

echo "[西元2026年02月24日 (星期二) 15時30分30秒    ] Command sent. Stopping follower..." >> $LOG

# 停止 follower
kill $FOLLOWER_PID 2>/dev/null
sleep 1
kill -9 $FOLLOWER_PID 2>/dev/null

# 記錄結束時的狀態
echo "=== 結束後 CAN dropped ===" >> $LOG
ip -s link show can1 2>&1 | grep dropped >> $LOG
ip -s link show can2 2>&1 | grep dropped >> $LOG
echo "=== 結束後記憶體 ===" >> $LOG
free -h | grep -E 'total|Mem' >> $LOG

echo "[西元2026年02月24日 (星期二) 15時30分31秒    ] Test completed." >> $LOG
