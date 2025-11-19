ros2 action send_goal /right_gripper_controller/gripper_cmd \
control_msgs/action/GripperCommand \
"{command: {position: 0.03, max_effort: 0.1}}"
