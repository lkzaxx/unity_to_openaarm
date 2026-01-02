#!/bin/bash
# ============================================================================
# Unity Follower Interface 啟動腳本
# 
# 使用方式：
#   cd ~/ros2_ws/scripts
#   ./start_follower.sh
#
# 注意：不能同時執行 ros2 launch openarm_bringup openarm.bimanual.launch.py
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="$SCRIPT_DIR/.."
VENV_PATH="/home/idaka/openarm_can/python/venv"

echo "=============================================="
echo "Unity Follower Interface - 500Hz MIT Control"
echo "=============================================="

# 1. 設置 CAN
echo ""
echo "Setting up CAN interfaces..."
"$SCRIPT_DIR/cansetup.sh"

# 2. Source ROS2
echo ""
echo "Sourcing ROS2..."
source /opt/ros/humble/setup.bash
source "$ROS2_WS/install/setup.bash" 2>/dev/null || true

# 3. 啟動 ROS TCP Endpoint (背景執行)
echo ""
echo "Starting ROS TCP Endpoint..."
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 &
TCP_PID=$!
echo "TCP Endpoint PID: $TCP_PID"

# 4. 等待 TCP Endpoint 啟動
sleep 2

# 5. 啟用虛擬環境
echo ""
echo "Activating virtual environment: $VENV_PATH"
source "$VENV_PATH/bin/activate"

# 6. 啟動 Camera Publisher (背景執行)
# 預設使用實體相機，如果相機不可用會自動使用測試模式
echo ""
echo "Starting Camera Publisher (background)..."
python3 "$ROS2_WS/src/openarm_ros2/openarm_bringup/scripts/camera_publisher.py" \
    --ros-args -p use_test_pattern:=false -p jpeg_quality:=50 &
CAMERA_PID=$!
echo "Camera Publisher PID: $CAMERA_PID"

# 等待相機初始化
sleep 2

# 7. 啟動 Unity Follower Interface (前景執行)
echo ""
echo "Starting Unity Follower Interface..."
echo ""
python3 "$ROS2_WS/src/openarm_ros2/openarm_bringup/scripts/unity_interface_follower.py"

# 清理
echo ""
echo "Shutting down..."
deactivate 2>/dev/null || true
kill $CAMERA_PID 2>/dev/null || true
kill $TCP_PID 2>/dev/null || true
echo "Shutdown complete."
