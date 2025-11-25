#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, String

class UnityInterface(Node):
    def __init__(self):
        super().__init__('unity_interface')

        # 宣告參數
        self.declare_parameter('left_arm_prefix', 'openarm_left_')
        self.declare_parameter('right_arm_prefix', 'openarm_right_')
        
        self.left_prefix = self.get_parameter('left_arm_prefix').get_parameter_value().string_value
        self.right_prefix = self.get_parameter('right_arm_prefix').get_parameter_value().string_value

        # 訂閱 Unity 的 JointState
        self.subscription = self.create_subscription(
            JointState,
            '/unity/joint_commands',
            self.listener_callback,
            10)
        
        # 訂閱真實機器人的 JointState (用於回傳給 Unity)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # 訂閱 Unity 心跳 (用於維持連線)
        self.heartbeat_sub = self.create_subscription(
            String,
            '/unity/heartbeat',
            self.heartbeat_callback,
            10)

        # 發布給 ROS2 Controllers
        self.left_arm_pub = self.create_publisher(
            JointTrajectory,
            '/left_joint_trajectory_controller/joint_trajectory',
            10)
        
        self.right_arm_pub = self.create_publisher(
            JointTrajectory,
            '/right_joint_trajectory_controller/joint_trajectory',
            10)
            
        # 發布給 Unity 的狀態 (重新映射名稱後)
        self.unity_state_pub = self.create_publisher(
            JointState,
            '/openarm/joint_states',
            10)

        # 發布心跳回音
        self.heartbeat_pub = self.create_publisher(
            String,
            '/unity/heartbeat_echo',
            10)

        self.get_logger().info('Unity Interface Node has been started.')
        self.get_logger().info(f'Left Prefix: {self.left_prefix}, Right Prefix: {self.right_prefix}')
        
        self.joint_state_count = 0

    def listener_callback(self, msg: JointState):
        # 分離左右臂的關節資料
        left_joints = []
        left_positions = []
        right_joints = []
        right_positions = []

        for i, name in enumerate(msg.name):
            # 映射 Unity 名稱 (L_J1 -> openarm_left_joint1)
            if name.startswith('L_J'):
                joint_num = name.split('_J')[1]
                ros_name = f"{self.left_prefix}joint{joint_num}"
                left_joints.append(ros_name)
                left_positions.append(msg.position[i])
            elif name.startswith('R_J'):
                joint_num = name.split('_J')[1]
                ros_name = f"{self.right_prefix}joint{joint_num}"
                right_joints.append(ros_name)
                right_positions.append(msg.position[i])

        # 發布左臂命令
        if left_joints:
            traj_msg = self.create_trajectory_msg(left_joints, left_positions)
            self.left_arm_pub.publish(traj_msg)
            self.get_logger().info(f'Published Left Arm: {left_positions[0]:.3f}...')

        # 發布右臂命令
        if right_joints:
            traj_msg = self.create_trajectory_msg(right_joints, right_positions)
            self.right_arm_pub.publish(traj_msg)
            self.get_logger().info(f'Published Right Arm: {right_positions[0]:.3f}...')

    def create_trajectory_msg(self, joint_names, positions):
        msg = JointTrajectory()
        msg.header = Header()
        # msg.header.stamp = self.get_clock().now().to_msg() # 使用當前時間
        # 若不設定 stamp，controller 會立即執行
        
        msg.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000 # 200ms (放寬時間限制，避免因延遲被丟棄)
        
        msg.points.append(point)
        return msg

    def heartbeat_callback(self, msg: String):
        # 回傳心跳，讓 Unity 知道連線還活著
        echo_msg = String()
        echo_msg.data = msg.data
        self.heartbeat_pub.publish(echo_msg)

    def joint_state_callback(self, msg: JointState):
        # 將 ROS2 的 JointState 轉換回 Unity 格式 (可選，如果 Unity 需要顯示)
        # 目前直接轉發，Unity 端可能需要對應的名稱處理，或者我們在這裡改名
        # 為了簡單起見，我們這裡先直接轉發，Unity 端目前似乎是讀取 /openarm/joint_states
        # 如果 Unity 需要 L_J1 這樣的名稱，我們需要在這裡反向映射
        
        unity_msg = JointState()
        unity_msg.header = msg.header
        
        new_names = []
        
        for name in msg.name:
            if name.startswith(self.left_prefix):
                # openarm_left_joint1 -> L_J1
                suffix = name.replace(self.left_prefix, "") # joint1
                if suffix.startswith("joint"):
                    num = suffix.replace("joint", "")
                    new_names.append(f"L_J{num}")
                else:
                    new_names.append(name)
            elif name.startswith(self.right_prefix):
                # openarm_right_joint1 -> R_J1
                suffix = name.replace(self.right_prefix, "")
                if suffix.startswith("joint"):
                    num = suffix.replace("joint", "")
                    new_names.append(f"R_J{num}")
                else:
                    new_names.append(name)
            else:
                new_names.append(name)
                
        unity_msg.name = new_names
        unity_msg.position = msg.position
        unity_msg.velocity = msg.velocity
        unity_msg.effort = msg.effort
        
        self.unity_state_pub.publish(unity_msg)
        
        self.joint_state_count += 1
        if self.joint_state_count % 100 == 0:
             self.get_logger().info(f'Relayed JointState to Unity (Count: {self.joint_state_count})')

def main(args=None):
    rclpy.init(args=args)
    node = UnityInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
