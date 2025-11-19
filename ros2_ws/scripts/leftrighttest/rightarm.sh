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
        positions: [0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
        time_from_start: {sec: 5}
      }]
    }}" 
