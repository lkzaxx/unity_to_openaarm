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
        positions: [0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
        time_from_start: {sec: 5}
      }]
    }}"
