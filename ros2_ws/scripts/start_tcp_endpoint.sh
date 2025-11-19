#!/bin/bash
# 啟動 ROS2 TCP Endpoint (Port 10000)
# 用於 Unity 與 ROS2 通信

cd /home/idaka/ros2_ws
source install/setup.bash

echo "啟動 ROS2 TCP Endpoint..."
echo "IP: 0.0.0.0"
echo "Port: 10000"
echo "等待 Unity 連接..."

ros2 launch ros_tcp_endpoint endpoint.py

