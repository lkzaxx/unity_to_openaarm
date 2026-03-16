#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

pub() {
  ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState "{name: [$1], position: [$2]}" --once 2>/dev/null
}

snap() {
  gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 num-buffers=1 ! "video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=30/1" ! nvjpegenc ! filesink location=$HOME/.openclaw/canvas/cal_${1}_L.jpg 2>/dev/null &
  gst-launch-1.0 -e nvarguscamerasrc sensor-id=1 num-buffers=1 ! "video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=30/1" ! nvjpegenc ! filesink location=$HOME/.openclaw/canvas/cal_${1}_R.jpg 2>/dev/null &
  wait
}

echo "=== 校準位置 1: 歸零 ==="
pub "\"R_J1\", \"R_J2\", \"R_J3\", \"R_J4\", \"R_J5\", \"R_J6\", \"R_J7\"" "0, 0, 0, 0, 0, 0, 0"
sleep 1
snap "p1_zero"

echo "=== 校準位置 2: J4=90° ==="
pub "\"R_J4\"" "1.5708"
sleep 1
snap "p2_j4up"

echo "=== 校準位置 3: 抓取位置 ==="
pub "\"R_J1\", \"R_J2\", \"R_J3\", \"R_J4\", \"R_J6\"" "0.087, 0.62, 0.667, 0.277, 0.577"
sleep 1
snap "p3_grab"

echo "=== 歸零 ==="
pub "\"R_J4\"" "1.5708"
sleep 0.5
pub "\"R_J6\"" "0"
sleep 0.5
pub "\"R_J1\", \"R_J2\", \"R_J3\", \"R_J5\", \"R_J7\"" "0, 0, 0, 0, 0"
sleep 0.5
pub "\"R_J4\"" "0"

echo "=== 完成！ ==="
