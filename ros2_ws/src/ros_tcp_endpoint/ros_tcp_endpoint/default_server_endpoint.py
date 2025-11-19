#!/usr/bin/env python

import rclpy

from ros_tcp_endpoint import TcpServer
from ros_tcp_endpoint.publisher import RosPublisher

# 導入標準 ROS2 訊息類型
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    tcp_server = TcpServer("UnityEndpoint")

    # 預先註冊 Unity 會發布的 Publisher
    # Unity 會透過這些 topic 發布訊息到 ROS2
    publishers = {
        "/openarm/joint_states": RosPublisher(
            "/openarm/joint_states",
            JointState,
            queue_size=10
        ),
        "/openarm/end_effector_pose": RosPublisher(
            "/openarm/end_effector_pose",
            PoseStamped,
            queue_size=10
        ),
        "/openarm/status": RosPublisher(
            "/openarm/status",
            String,
            queue_size=10
        ),
        "/unity/heartbeat_echo": RosPublisher(
            "/unity/heartbeat_echo",
            String,
            queue_size=10
        ),
    }

    tcp_server.start(publishers=publishers)

    tcp_server.setup_executor()

    tcp_server.destroy_nodes()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
