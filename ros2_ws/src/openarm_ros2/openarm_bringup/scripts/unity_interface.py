#!/usr/bin/env python3
"""
Unity Interface Node - 即時追蹤控制版本

設計特點：
1. 平滑濾波：低通濾波器減少 Unity 輸入抖動
2. 速度限制：限制每個週期的位置變化量
3. 加速度限制：限制速度變化率，實現平滑加減速
4. 定時器驅動：50Hz 固定頻率控制迴圈
"""
import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ============================================================================
# 全域配置參數
# ============================================================================

# 控制迴圈參數
CONTROL_FREQUENCY = 50  # Hz，控制迴圈頻率
TRAJECTORY_HORIZON = 0.05  # 秒，每個軌跡的時間視野

# 平滑濾波參數
FILTER_ALPHA = 0.4  # 低通濾波係數 (0~1)，越小越平滑

# 速度限制 (rad/s) - 大關節慢、小關節快
MAX_VELOCITY = {
    "joint1": 1.5,   # 肩膀旋轉 - 最慢（慣性大）
    "joint2": 1.5,   # 肩膀抬舉 - 最慢
    "joint3": 2.5,   # 上臂旋轉
    "joint4": 2.5,   # 肘部彎曲
    "joint5": 4.0,   # 前臂旋轉
    "joint6": 4.0,   # 手腕彎曲
    "joint7": 5.0,   # 手腕旋轉 - 最快（慣性小）
}

# 加速度限制 (rad/s²) - 控制加減速平滑度
MAX_ACCELERATION = {
    "joint1": 2.0,   # 肩膀 - 緩慢加減速
    "joint2": 2.0,
    "joint3": 3.0,   # 肘部
    "joint4": 3.0,
    "joint5": 6.0,   # 手腕 - 快速加減速
    "joint6": 6.0,
    "joint7": 8.0,
}

# 位置安全係數
POSITION_SAFETY_FACTOR = 0.9  # 使用範圍的百分比（兩端各留 5% 緩衝）

# 硬體位置限制 (rad) - 基於 URDF
RIGHT_HARDWARE_POSITION_LIMITS = {
    "joint1": {"lower": -1.396263, "upper": 3.490659},   # -80° ~ 200°
    "joint2": {"lower": -0.174533, "upper": 3.316125},   # -10° ~ 190°
    "joint3": {"lower": -1.570796, "upper": 1.570796},   # -90° ~ 90°
    "joint4": {"lower": 0.0, "upper": 2.443461},         # 0° ~ 140°
    "joint5": {"lower": -1.570796, "upper": 1.570796},   # -90° ~ 90°
    "joint6": {"lower": -0.785398, "upper": 0.785398},   # -45° ~ 45°
    "joint7": {"lower": -1.570796, "upper": 1.570796},   # -90° ~ 90°
}

LEFT_HARDWARE_POSITION_LIMITS = {
    "joint1": {"lower": -3.490659, "upper": 1.396263},   # -200° ~ 80°
    "joint2": {"lower": -3.316125, "upper": 0.174533},   # -190° ~ 10°
    "joint3": {"lower": -1.570796, "upper": 1.570796},   # -90° ~ 90°
    "joint4": {"lower": 0.0, "upper": 2.443461},         # 0° ~ 140°
    "joint5": {"lower": -1.570796, "upper": 1.570796},   # -90° ~ 90°
    "joint6": {"lower": -0.785398, "upper": 0.785398},   # -45° ~ 45°
    "joint7": {"lower": -1.570796, "upper": 1.570796},   # -90° ~ 90°
}

# ============================================================================


class SmoothMotionController:
    """
    平滑運動控制器 - 整合濾波、速度限制、加速度限制
    
    每個關節一個實例，追蹤該關節的狀態並輸出平滑的目標位置。
    """
    
    def __init__(self, joint_name: str, max_vel: float, max_accel: float, 
                 filter_alpha: float = FILTER_ALPHA):
        self.joint_name = joint_name
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.filter_alpha = filter_alpha
        
        # 狀態變數
        self.filtered_target = None   # 濾波後的目標（Unity 期望位置）
        self.output_pos = None        # 實際輸出位置（發送給 controller）
        self.current_vel = 0.0        # 當前速度
        self.initialized = False
    
    def update(self, raw_target: float, dt: float) -> float:
        """
        更新控制器，返回平滑後的目標位置
        
        Args:
            raw_target: Unity 原始目標位置 (rad)
            dt: 時間步長 (秒)
        
        Returns:
            平滑後的目標位置 (rad)
        """
        # === Step 1: 低通濾波 (抗抖動) ===
        if self.filtered_target is None:
            self.filtered_target = raw_target
        else:
            self.filtered_target = (
                self.filter_alpha * raw_target + 
                (1.0 - self.filter_alpha) * self.filtered_target
            )
        
        # 初始化輸出位置
        if self.output_pos is None:
            self.output_pos = self.filtered_target
            self.initialized = True
            return self.output_pos
        
        # === Step 2: 計算期望速度 ===
        error = self.filtered_target - self.output_pos
        
        # 避免除以零
        if dt <= 0:
            return self.output_pos
        
        desired_vel = error / dt
        
        # === Step 3: 速度限制 ===
        desired_vel = max(-self.max_vel, min(desired_vel, self.max_vel))
        
        # === Step 4: 加速度限制 ===
        vel_change = desired_vel - self.current_vel
        max_vel_change = self.max_accel * dt
        vel_change = max(-max_vel_change, min(vel_change, max_vel_change))
        
        # 更新速度
        self.current_vel = self.current_vel + vel_change
        
        # === Step 5: 更新輸出位置 ===
        self.output_pos = self.output_pos + self.current_vel * dt
        
        return self.output_pos
    
    def reset(self, position: float = None):
        """重置控制器狀態"""
        self.filtered_target = position
        self.output_pos = position
        self.current_vel = 0.0
        self.initialized = position is not None


class UnityInterface(Node):
    def __init__(self):
        super().__init__("unity_interface")

        # 宣告參數
        self.declare_parameter("left_arm_prefix", "openarm_left_")
        self.declare_parameter("right_arm_prefix", "openarm_right_")

        self.left_prefix = (
            self.get_parameter("left_arm_prefix").get_parameter_value().string_value
        )
        self.right_prefix = (
            self.get_parameter("right_arm_prefix").get_parameter_value().string_value
        )

        # 訂閱 Unity 的 JointState
        self.subscription = self.create_subscription(
            JointState, "/unity/joint_commands", self.unity_command_callback, 10
        )

        # 訂閱真實機器人的 JointState (用於回傳給 Unity)
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # 訂閱 Unity 心跳
        self.heartbeat_sub = self.create_subscription(
            String, "/unity/heartbeat", self.heartbeat_callback, 10
        )

        # 發布給 ROS2 Controllers
        self.left_arm_pub = self.create_publisher(
            JointTrajectory, "/left_joint_trajectory_controller/joint_trajectory", 10
        )
        self.right_arm_pub = self.create_publisher(
            JointTrajectory, "/right_joint_trajectory_controller/joint_trajectory", 10
        )

        # 發布給 Unity 的狀態
        self.unity_state_pub = self.create_publisher(
            JointState, "/openarm/joint_states", 10
        )

        # 發布心跳回音
        self.heartbeat_pub = self.create_publisher(String, "/unity/heartbeat_echo", 10)

        # 夾爪 Action Client
        self.left_gripper_client = ActionClient(
            self, GripperCommand, "/left_gripper_controller/gripper_cmd"
        )
        self.right_gripper_client = ActionClient(
            self, GripperCommand, "/right_gripper_controller/gripper_cmd"
        )

        # === 位置限制 ===
        self.left_position_limits = self._calculate_position_limits(LEFT_HARDWARE_POSITION_LIMITS)
        self.right_position_limits = self._calculate_position_limits(RIGHT_HARDWARE_POSITION_LIMITS)

        # === 運動控制器 ===
        # 為每個關節創建一個 SmoothMotionController
        self.left_controllers = {}
        self.right_controllers = {}
        for i in range(1, 8):
            joint_key = f"joint{i}"
            left_name = f"{self.left_prefix}{joint_key}"
            right_name = f"{self.right_prefix}{joint_key}"
            
            max_vel = MAX_VELOCITY.get(joint_key, 2.0)
            max_accel = MAX_ACCELERATION.get(joint_key, 3.0)
            
            self.left_controllers[left_name] = SmoothMotionController(
                left_name, max_vel, max_accel
            )
            self.right_controllers[right_name] = SmoothMotionController(
                right_name, max_vel, max_accel
            )

        # === Unity 目標緩衝區 ===
        # 儲存 Unity 最新發來的目標（隨時會被覆蓋）
        self.latest_left_targets = {}   # {joint_name: raw_position}
        self.latest_right_targets = {}
        
        # 追蹤是否有新目標（避免沒有目標時發送空軌跡）
        self.has_left_target = False
        self.has_right_target = False

        # === 心跳檢測 ===
        self.enable_heartbeat_check = False
        self.heartbeat_timeout = 2.0
        self.last_heartbeat_time = 0.0
        self.heartbeat_timeout_warned = False

        # === 夾爪狀態 ===
        self.last_left_gripper_pos = None
        self.last_right_gripper_pos = None
        self.gripper_position_threshold = 0.001

        # === JointState Relay ===
        self.joint_state_count = 0
        self.enable_joint_state_log = False

        # === 定時器驅動控制迴圈 ===
        self.control_dt = 1.0 / CONTROL_FREQUENCY
        self.control_timer = self.create_timer(self.control_dt, self.control_loop_callback)

        # 啟動訊息
        self.get_logger().info("Unity Interface Node (Smooth Motion Control) started.")
        self.get_logger().info(f"Control frequency: {CONTROL_FREQUENCY} Hz")
        self.get_logger().info(f"Filter alpha: {FILTER_ALPHA}")
        self.get_logger().info(f"Max velocities: {MAX_VELOCITY}")

    def _calculate_position_limits(self, hw_limits):
        """計算帶有安全緩衝的位置限制"""
        limits = {}
        for joint, hw in hw_limits.items():
            range_size = hw["upper"] - hw["lower"]
            buffer = range_size * (1 - POSITION_SAFETY_FACTOR) / 2
            limits[joint] = {
                "lower": hw["lower"] + buffer,
                "upper": hw["upper"] - buffer,
            }
        return limits

    def _clamp_position(self, joint_name: str, position: float) -> float:
        """限制位置在安全範圍內"""
        joint_key = joint_name.split("_")[-1]
        
        if "left" in joint_name:
            limits = self.left_position_limits
        else:
            limits = self.right_position_limits
        
        if joint_key in limits:
            lim = limits[joint_key]
            clamped = max(lim["lower"], min(position, lim["upper"]))
            if abs(clamped - position) > 0.001:
                self.get_logger().debug(
                    f"{joint_name}: clamped {position:.3f} -> {clamped:.3f}"
                )
            return clamped
        return position

    def _map_unity_to_ros_name(self, unity_name: str) -> str:
        """將 Unity 關節名稱映射到 ROS 名稱"""
        if unity_name.startswith("L_J"):
            joint_num = unity_name.split("_J")[1]
            return f"{self.left_prefix}joint{joint_num}"
        elif unity_name.startswith("R_J"):
            joint_num = unity_name.split("_J")[1]
            return f"{self.right_prefix}joint{joint_num}"
        return unity_name

    def unity_command_callback(self, msg: JointState):
        """
        Unity 訊息回調：只更新最新目標，不發送軌跡
        
        軌跡發送由定時器控制迴圈負責。
        """
        left_gripper_pos = None
        right_gripper_pos = None

        for i, name in enumerate(msg.name):
            if name.startswith("L_J"):
                ros_name = self._map_unity_to_ros_name(name)
                clamped = self._clamp_position(ros_name, msg.position[i])
                self.latest_left_targets[ros_name] = clamped
                self.has_left_target = True
            elif name.startswith("R_J"):
                ros_name = self._map_unity_to_ros_name(name)
                clamped = self._clamp_position(ros_name, msg.position[i])
                self.latest_right_targets[ros_name] = clamped
                self.has_right_target = True
            elif name == "L_EE":
                left_gripper_pos = msg.position[i]
            elif name == "R_EE":
                right_gripper_pos = msg.position[i]

        # 夾爪命令立即發送（不需要平滑）
        if left_gripper_pos is not None:
            self.send_gripper_command("left", left_gripper_pos)
        if right_gripper_pos is not None:
            self.send_gripper_command("right", right_gripper_pos)

    def control_loop_callback(self):
        """
        定時器控制迴圈：更新控制器並發送軌跡
        
        固定 50Hz 執行，不受 Unity 訊息頻率影響。
        """
        # 心跳檢測
        if self.enable_heartbeat_check and not self._check_heartbeat():
            return

        dt = self.control_dt

        # === 左臂 ===
        if self.has_left_target and self.latest_left_targets:
            left_names = []
            left_positions = []
            
            for joint_name, raw_target in self.latest_left_targets.items():
                controller = self.left_controllers.get(joint_name)
                if controller:
                    smoothed_pos = controller.update(raw_target, dt)
                    left_names.append(joint_name)
                    left_positions.append(smoothed_pos)
            
            if left_names:
                self._publish_trajectory(left_names, left_positions, self.left_arm_pub)

        # === 右臂 ===
        if self.has_right_target and self.latest_right_targets:
            right_names = []
            right_positions = []
            
            for joint_name, raw_target in self.latest_right_targets.items():
                controller = self.right_controllers.get(joint_name)
                if controller:
                    smoothed_pos = controller.update(raw_target, dt)
                    right_names.append(joint_name)
                    right_positions.append(smoothed_pos)
            
            if right_names:
                self._publish_trajectory(right_names, right_positions, self.right_arm_pub)

    def _check_heartbeat(self) -> bool:
        """檢查 Unity 心跳是否正常"""
        if self.last_heartbeat_time == 0.0:
            return False
        
        now = self.get_clock().now().nanoseconds / 1e9
        time_since = now - self.last_heartbeat_time
        
        if time_since > self.heartbeat_timeout:
            if not self.heartbeat_timeout_warned:
                self.get_logger().warn(
                    f"⚠️ Unity heartbeat timeout! ({time_since:.1f}s)"
                )
                self.heartbeat_timeout_warned = True
            return False
        return True

    def _publish_trajectory(self, joint_names: list, positions: list, publisher):
        """發送短週期軌跡"""
        msg = JointTrajectory()
        msg.header = Header()
        msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(TRAJECTORY_HORIZON * 1e9)
        msg.points.append(point)

        publisher.publish(msg)

    def send_gripper_command(self, side: str, position: float):
        """發送夾爪命令"""
        position_clamped = max(0.0, min(position, 0.0425))
        position_scaled = position_clamped * (0.03 / 0.0425)

        if side == "left":
            if (self.last_left_gripper_pos is not None and
                abs(position_scaled - self.last_left_gripper_pos) < self.gripper_position_threshold):
                return
            self.last_left_gripper_pos = position_scaled
            client = self.left_gripper_client
        else:
            if (self.last_right_gripper_pos is not None and
                abs(position_scaled - self.last_right_gripper_pos) < self.gripper_position_threshold):
                return
            self.last_right_gripper_pos = position_scaled
            client = self.right_gripper_client

        if not client.wait_for_server(timeout_sec=0.1):
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position_scaled
        goal_msg.command.max_effort = 0.1

        future = client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f: self._gripper_response_callback(f, side)
        )

    def _gripper_response_callback(self, future, side: str):
        """夾爪 goal 回應回調"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"{side} gripper goal rejected")

    def heartbeat_callback(self, msg: String):
        """Unity 心跳回調"""
        self.last_heartbeat_time = self.get_clock().now().nanoseconds / 1e9

        if self.heartbeat_timeout_warned:
            self.get_logger().info("✅ Unity heartbeat restored!")
            self.heartbeat_timeout_warned = False

        echo_msg = String()
        echo_msg.data = msg.data
        self.heartbeat_pub.publish(echo_msg)

    def joint_state_callback(self, msg: JointState):
        """轉發 JointState 給 Unity"""
        unity_msg = JointState()
        unity_msg.header = msg.header

        new_names = []
        for name in msg.name:
            if name.startswith(self.left_prefix):
                suffix = name.replace(self.left_prefix, "")
                if suffix.startswith("joint"):
                    num = suffix.replace("joint", "")
                    new_names.append(f"L_J{num}")
                else:
                    new_names.append(name)
            elif name.startswith(self.right_prefix):
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
        if self.enable_joint_state_log and self.joint_state_count % 100 == 0:
            self.get_logger().info(f"JointState relayed (#{self.joint_state_count})")


def main(args=None):
    rclpy.init(args=args)
    node = UnityInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
