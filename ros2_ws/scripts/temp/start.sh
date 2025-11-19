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
        positions: [0, -0.2, 0.1, 0.1, 0.5, 0.8, 0.8],
        time_from_start: {sec: 2}
      }]
    }}" &

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
        positions: [0, 0.5, 0.1, 0.5, 0.5, 0.8, 0.8],
        time_from_start: {sec: 2}
      }]
    }}" &
