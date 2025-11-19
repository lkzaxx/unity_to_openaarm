TIME_FACTOR=3
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
        positions: [-2.9, -0.85, 1.3, 1.5, 0, 0, -1.5],
        time_from_start: {sec: $((1 * TIME_FACTOR))}
      }]
    }}" &
    
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
        positions: [2.9, 0.85, -1.3, 1.5, 0, 0, 1.5],
        time_from_start: {sec: $((1 * TIME_FACTOR))}
      }]
    }}" &


