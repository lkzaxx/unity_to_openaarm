#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, String

# ============================================================================
# 全域配置參數 - 方便快速調整保護機制
# ============================================================================
VELOCITY_SAFETY_FACTOR = 0.2    # 速度安全係數（硬體上限的百分比）30%
POSITION_SAFETY_FACTOR = 0.9    # 位置安全係數（使用範圍的百分比）90%
MIN_TRAJECTORY_TIME = 0.05      # 最小軌跡執行時間（秒）

# 硬體速度上限 (rad/s) - 基於馬達規格
HARDWARE_VELOCITY_LIMITS = {
    'DM8009': 45.0,   # Joint 1, 2
    'DM4340': 8.0,    # Joint 3, 4
    'DM4310': 30.0    # Joint 5, 6, 7
}

# 硬體位置限制 (rad) - 基於 joint_limits.yaml
HARDWARE_POSITION_LIMITS = {
    'joint1': {'lower': -1.396263, 'upper': 3.490659},
    'joint2': {'lower': -1.745329, 'upper': 1.745329},
    'joint3': {'lower': -1.570796, 'upper': 1.570796},
    'joint4': {'lower': 0.0, 'upper': 2.443461},
    'joint5': {'lower': -1.570796, 'upper': 1.570796},
    'joint6': {'lower': -0.785398, 'upper': 0.785398},
    'joint7': {'lower': -1.570796, 'upper': 1.570796}
}
# ============================================================================

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

        # 關節速度限制 - 使用全域配置自動計算
        # 當前設定：硬體上限的 30%
        self.joint_velocity_limits = self._calculate_velocity_limits()
        
        # 關節位置限制 - 使用全域配置自動計算
        # 當前設定：90% 安全範圍（兩端各留 5% 緩衝）
        self.joint_position_limits = self._calculate_position_limits()
        
        self.min_trajectory_time = MIN_TRAJECTORY_TIME
        self.current_positions = {}  # 追蹤當前關節位置
        
        self.get_logger().info('Unity Interface Node has been started.')
        self.get_logger().info(f'Left Prefix: {self.left_prefix}, Right Prefix: {self.right_prefix}')
        self.get_logger().info(f'Velocity Safety Factor: {VELOCITY_SAFETY_FACTOR * 100:.0f}%')
        self.get_logger().info(f'Position Safety Factor: {POSITION_SAFETY_FACTOR * 100:.0f}%')
        
        self.joint_state_count = 0
    
    def _calculate_velocity_limits(self):
        """根據全域安全係數計算速度限制"""
        return {
            'joint1': HARDWARE_VELOCITY_LIMITS['DM8009'] * VELOCITY_SAFETY_FACTOR,
            'joint2': HARDWARE_VELOCITY_LIMITS['DM8009'] * VELOCITY_SAFETY_FACTOR,
            'joint3': HARDWARE_VELOCITY_LIMITS['DM4340'] * VELOCITY_SAFETY_FACTOR,
            'joint4': HARDWARE_VELOCITY_LIMITS['DM4340'] * VELOCITY_SAFETY_FACTOR,
            'joint5': HARDWARE_VELOCITY_LIMITS['DM4310'] * VELOCITY_SAFETY_FACTOR,
            'joint6': HARDWARE_VELOCITY_LIMITS['DM4310'] * VELOCITY_SAFETY_FACTOR,
            'joint7': HARDWARE_VELOCITY_LIMITS['DM4310'] * VELOCITY_SAFETY_FACTOR
        }
    
    def _calculate_position_limits(self):
        """根據全域安全係數計算位置限制（縮小範圍兩端各留緩衝）"""
        limits = {}
        for joint, hw_limits in HARDWARE_POSITION_LIMITS.items():
            range_size = hw_limits['upper'] - hw_limits['lower']
            buffer = range_size * (1 - POSITION_SAFETY_FACTOR) / 2
            limits[joint] = {
                'lower': hw_limits['lower'] + buffer,
                'upper': hw_limits['upper'] - buffer
            }
        return limits

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
            # 限制位置在安全範圍內
            clamped_positions = self.clamp_joint_positions(left_joints, left_positions)
            traj_msg = self.create_trajectory_msg(left_joints, clamped_positions)
            self.left_arm_pub.publish(traj_msg)
            self.get_logger().info(f'Published Left Arm: {clamped_positions[0]:.3f}...')

        # 發布右臂命令
        if right_joints:
            # 限制位置在安全範圍內
            clamped_positions = self.clamp_joint_positions(right_joints, right_positions)
            traj_msg = self.create_trajectory_msg(right_joints, clamped_positions)
            self.right_arm_pub.publish(traj_msg)
            self.get_logger().info(f'Published Right Arm: {clamped_positions[0]:.3f}...')

    def calculate_safe_trajectory_time(self, joint_names, target_positions):
        """
        根據速度限制計算安全的軌跡執行時間
        
        Args:
            joint_names: 關節名稱列表
            target_positions: 目標位置列表
            
        Returns:
            float: 安全的軌跡執行時間（秒）
        """
        max_time = self.min_trajectory_time
        
        for i, joint_name in enumerate(joint_names):
            # 提取關節編號 (e.g., "openarm_left_joint1" -> "joint1")
            parts = joint_name.split('_')
            joint_key = parts[-1]  # 獲取最後一部分（joint1, joint2, etc.）
            
            # 獲取速度限制
            if joint_key not in self.joint_velocity_limits:
                self.get_logger().warn(f'No velocity limit found for {joint_key}, using default')
                continue
                
            max_velocity = self.joint_velocity_limits[joint_key]
            
            # 獲取當前位置（如果沒有記錄則假設為 0）
            current_pos = self.current_positions.get(joint_name, 0.0)
            target_pos = target_positions[i]
            
            # 計算位置變化和所需時間
            delta_pos = abs(target_pos - current_pos)
            required_time = delta_pos / max_velocity if max_velocity > 0 else self.min_trajectory_time
            
            # 記錄計算過程（僅在變化較大時）
            if delta_pos > 0.1:  # 只記錄大於 0.1 rad 的變化
                self.get_logger().info(
                    f'{joint_key}: Δ={delta_pos:.3f}rad, v_max={max_velocity:.1f}rad/s, t={required_time:.3f}s'
                )
            
            max_time = max(max_time, required_time)
        
        return max_time

    def clamp_joint_positions(self, joint_names, positions):
        """
        限制關節位置在安全範圍內，防止超出機械限制
        
        Args:
            joint_names: 關節名稱列表
            positions: 目標位置列表
            
        Returns:
            list: 限制後的安全位置列表
        """
        clamped_positions = []
        
        for i, joint_name in enumerate(joint_names):
            # 提取關節編號
            parts = joint_name.split('_')
            joint_key = parts[-1]
            
            target_pos = positions[i]
            
            # 檢查並限制位置
            if joint_key in self.joint_position_limits:
                limits = self.joint_position_limits[joint_key]
                original_pos = target_pos
                
                # 限制在上下限範圍內
                clamped_pos = max(limits['lower'], min(target_pos, limits['upper']))
                
                # 如果位置被限制了，記錄警告
                if abs(clamped_pos - original_pos) > 0.001:  # 超過 0.001 rad 才警告
                    self.get_logger().warn(
                        f'{joint_key}: Position clamped from {original_pos:.3f} to {clamped_pos:.3f} '
                        f'(limits: [{limits["lower"]:.3f}, {limits["upper"]:.3f}])'
                    )
                
                clamped_positions.append(clamped_pos)
            else:
                # 如果沒有找到限制，使用原始值並警告
                self.get_logger().warn(f'No position limits found for {joint_key}, using original position')
                clamped_positions.append(target_pos)
        
        return clamped_positions

    def create_trajectory_msg(self, joint_names, positions):
        msg = JointTrajectory()
        msg.header = Header()
        # msg.header.stamp = self.get_clock().now().to_msg() # 使用當前時間
        # 若不設定 stamp，controller 會立即執行
        
        msg.joint_names = joint_names
        
        # 計算安全的軌跡執行時間（基於速度限制）
        trajectory_time = self.calculate_safe_trajectory_time(joint_names, positions)
        
        point = JointTrajectoryPoint()
        point.positions = positions
        
        # 使用動態計算的時間而非固定的 200ms
        point.time_from_start.sec = int(trajectory_time)
        point.time_from_start.nanosec = int((trajectory_time % 1) * 1e9)
        
        msg.points.append(point)
        
        # 記錄使用的軌跡時間
        self.get_logger().info(f'Trajectory time: {trajectory_time:.3f}s')
        
        return msg

    def heartbeat_callback(self, msg: String):
        # 回傳心跳，讓 Unity 知道連線還活著
        echo_msg = String()
        echo_msg.data = msg.data
        self.heartbeat_pub.publish(echo_msg)
        self.get_logger().info(f'Echoed heartbeat: {msg.data}') # Debug log

    def joint_state_callback(self, msg: JointState):
        # 更新當前位置追蹤（用於速度保護計算）
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]
        
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
