#!/bin/bash
# ============================================================================
# Unity Follower Interface ???單
# 
# 雿輻?孵?嚗?#   cd ~/ros2_ws/scripts
#   ./start_follower.sh
#
# 瘜冽?嚗??賢??銵?ros2 launch openarm_bringup openarm.bimanual.launch.py
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="$SCRIPT_DIR/.."

echo "=============================================="
echo "Unity Follower Interface - 500Hz MIT Control"
echo "=============================================="

# 1. 閮剔蔭 CAN
echo ""
echo "Setting up CAN interfaces..."
"$SCRIPT_DIR/cansetup.sh"

# 2. Source ROS2
echo ""
echo "Sourcing ROS2..."
source /opt/ros/humble/setup.bash
source "$ROS2_WS/install/setup.bash" 2>/dev/null || true

# 3. ?? ROS TCP Endpoint (??瑁?)
echo ""
echo "Starting ROS TCP Endpoint..."
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 &
TCP_PID=$!
echo "TCP Endpoint PID: $TCP_PID"

# 4. 蝑? TCP Endpoint ??
sleep 2

# 5. ?? Follower Interface
echo ""
echo "Starting Unity Follower Interface..."
echo ""
python3 "$ROS2_WS/src/openarm_ros2/openarm_bringup/scripts/unity_interface_follower.py"

# 皜?
kill $TCP_PID 2>/dev/null || true
echo ""
echo "Shutdown complete."
