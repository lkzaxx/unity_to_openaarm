#!/usr/bin/env python3
import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ============================================================================
# 全域配置參數 - 方便快速調整保護機制
# ============================================================================
VELOCITY_SAFETY_FACTOR = 0.2  # 速度安全係數（硬體上限的百分比）30%
POSITION_SAFETY_FACTOR = 0.9  # 位置安全係數（使用範圍的百分比）90%
MIN_TRAJECTORY_TIME = 0.05  # 最小軌跡執行時間（秒）

# 硬體速度上限 (rad/s) - 基於馬達規格
HARDWARE_VELOCITY_LIMITS = {
    "DM8009": 45.0,  # Joint 1, 2
    "DM4340": 8.0,  # Joint 3, 4
    "DM4310": 30.0,  # Joint 5, 6, 7
}

# 硬體位置限制 (rad) - 基於 URDF (openarm_bimanual_control.urdf)
# 🔥 左右手的 Joint 1, 2 限制不同！

# 右手限制
RIGHT_HARDWARE_POSITION_LIMITS = {
    "joint1": {"lower": -1.396263, "upper": 3.490659},   # -80° ~ 200°
    "joint2": {"lower": -0.174533, "upper": 3.316125},   # -10° ~ 190° (可舉高)
    "joint3": {"lower": -1.570796, "upper": 1.570796},   # -90° ~ 90°
    "joint4": {"lower": 0.0, "upper": 2.443461},         # 0° ~ 140°
    "joint5": {"lower": -1.570796, "upper": 1.570796},   # -90° ~ 90°
    "joint6": {"lower": -0.785398, "upper": 0.785398},   # -45° ~ 45°
    "joint7": {"lower": -1.570796, "upper": 1.570796},   # -90° ~ 90°
}

# 左手限制（Joint 1, 2 與右手相反）
LEFT_HARDWARE_POSITION_LIMITS = {
    "joint1": {"lower": -3.490659, "upper": 1.396263},   # -200° ~ 80°
    "joint2": {"lower": -3.316125, "upper": 0.174533},   # -190° ~ 10° (可舉高)
    "joint3": {"lower": -1.570796, "upper": 1.570796},   # -90° ~ 90°
    "joint4": {"lower": 0.0, "upper": 2.443461},         # 0° ~ 140°
    "joint5": {"lower": -1.570796, "upper": 1.570796},   # -90° ~ 90°
    "joint6": {"lower": -0.785398, "upper": 0.785398},   # -45° ~ 45°
    "joint7": {"lower": -1.570796, "upper": 1.570796},   # -90° ~ 90°
}
# ============================================================================


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
            JointState, "/unity/joint_commands", self.listener_callback, 10
        )

        # 訂閱真實機器人的 JointState (用於回傳給 Unity)
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # 訂閱 Unity 心跳 (用於維持連線)
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

        # 發布給 Unity 的狀態 (重新映射名稱後)
        self.unity_state_pub = self.create_publisher(
            JointState, "/openarm/joint_states", 10
        )

        # 發布心跳回音
        self.heartbeat_pub = self.create_publisher(String, "/unity/heartbeat_echo", 10)

        # 🔥 新增：夾爪 Action Client
        self.left_gripper_client = ActionClient(
            self, GripperCommand, "/left_gripper_controller/gripper_cmd"
        )

        self.right_gripper_client = ActionClient(
            self, GripperCommand, "/right_gripper_controller/gripper_cmd"
        )

        # 關節速度限制 - 使用全域配置自動計算
        # 當前設定：硬體上限的 30%
        self.joint_velocity_limits = self._calculate_velocity_limits()

        # 關節位置限制 - 區分左右手，使用全域配置自動計算
        # 當前設定：90% 安全範圍（兩端各留 5% 緩衝）
        self.left_joint_position_limits = self._calculate_position_limits(LEFT_HARDWARE_POSITION_LIMITS)
        self.right_joint_position_limits = self._calculate_position_limits(RIGHT_HARDWARE_POSITION_LIMITS)

        self.min_trajectory_time = MIN_TRAJECTORY_TIME
        self.current_positions = {}  # 追蹤當前關節位置

        # 每一側最後一次軌跡所使用的「安全執行時間」（秒）
        # 之後會用它來判斷何時可以啟動下一個軌跡
        self.last_left_traj_duration = self.min_trajectory_time
        self.last_right_traj_duration = self.min_trajectory_time

        # === 目標平滑功能 ===
        # 位置平滑濾波器（指數移動平均，用於減少目標抖動）
        # key: 關節名稱（openarm_left_joint1 等）, value: 上一次平滑後的位置
        self.left_position_filter = {}
        self.right_position_filter = {}
        # 平滑係數 α：越小越平滑、反應越慢；越大越貼近原始輸入
        self.position_smoothing_factor = 0.3

        # === 軌跡保護與佇列功能 ===
        # 記錄最後一次已送出指令的時間（用於最小間隔保護）
        self.last_left_send_time = 0.0
        self.last_right_send_time = 0.0
        # 兩次指令最小間隔（秒），避免過高頻率更新目標
        self.min_send_interval = 0.1  # 降低至 0.1 秒，讓第一筆指令更快發出

        # 目前已送出的「上一次目標」（用來判斷是否已經到位）
        # key: 關節名稱, value: 目標位置（rad）
        self.last_left_target = {}
        self.last_right_target = {}

        # Unity 不斷更新時，只保留「最後一筆」待發目標（關節名稱 + 目標位置）
        # 型別: Optional[Tuple[List[str], List[float]]]
        self.pending_left_target = None
        self.pending_right_target = None

        # （用於目標變化死區判斷，差異小於此值時不再送出新軌跡）
        self.motion_done_epsilon = 0.3  # rad
        # === Unity 心跳檢測功能 ===
        # 是否啟用心跳檢測（True: 必須有心跳才發送軌跡, False: 不檢查心跳）
        self.enable_heartbeat_check = False
        # 心跳超時時間（秒），超過此時間沒收到心跳視為 Unity 斷線
        self.heartbeat_timeout = 2.0
        # 最後一次收到心跳的時間戳
        self.last_heartbeat_time = 0.0
        # 是否已經警告過心跳超時（避免重複 log）
        self.heartbeat_timeout_warned = False

        # JointState Relay 訊息 log 開關（避免 log 過多）
        # True: 每 100 筆 JointState 會印出一次 Relayed JointState 訊息
        # False: 完全不印出這個統計訊息
        self.enable_joint_state_log = False

        # 夾爪狀態追蹤（避免頻繁發送相同命令）
        self.last_left_gripper_pos = None
        self.last_right_gripper_pos = None
        self.gripper_position_threshold = 0.001  # 1mm 變化才發送

        self.get_logger().info("Unity Interface Node has been started.")
        self.get_logger().info(
            f"Left Prefix: {self.left_prefix}, Right Prefix: {self.right_prefix}"
        )
        self.get_logger().info(
            f"Velocity Safety Factor: {VELOCITY_SAFETY_FACTOR * 100:.0f}%"
        )
        self.get_logger().info(
            f"Position Safety Factor: {POSITION_SAFETY_FACTOR * 100:.0f}%"
        )
        self.get_logger().info(
            f"Trajectory Queue: min_send_interval={self.min_send_interval}s, motion_done_epsilon={self.motion_done_epsilon}rad"
        )
        self.get_logger().info(
            f"Heartbeat Check: {'ENABLED' if self.enable_heartbeat_check else 'DISABLED'} (timeout={self.heartbeat_timeout}s)"
        )
        self.get_logger().info("Gripper action clients initialized")

        self.joint_state_count = 0

    def _calculate_velocity_limits(self):
        """根據全域安全係數計算速度限制"""
        return {
            "joint1": HARDWARE_VELOCITY_LIMITS["DM8009"] * VELOCITY_SAFETY_FACTOR,
            "joint2": HARDWARE_VELOCITY_LIMITS["DM8009"] * VELOCITY_SAFETY_FACTOR,
            "joint3": HARDWARE_VELOCITY_LIMITS["DM4340"] * VELOCITY_SAFETY_FACTOR,
            "joint4": HARDWARE_VELOCITY_LIMITS["DM4340"] * VELOCITY_SAFETY_FACTOR,
            "joint5": HARDWARE_VELOCITY_LIMITS["DM4310"] * VELOCITY_SAFETY_FACTOR,
            "joint6": HARDWARE_VELOCITY_LIMITS["DM4310"] * VELOCITY_SAFETY_FACTOR,
            "joint7": HARDWARE_VELOCITY_LIMITS["DM4310"] * VELOCITY_SAFETY_FACTOR,
        }

    def _calculate_position_limits(self, hw_position_limits):
        """根據全域安全係數計算位置限制（縮小範圍兩端各留緩衝）"""
        limits = {}
        for joint, hw_limits in hw_position_limits.items():
            range_size = hw_limits["upper"] - hw_limits["lower"]
            buffer = range_size * (1 - POSITION_SAFETY_FACTOR) / 2
            limits[joint] = {
                "lower": hw_limits["lower"] + buffer,
                "upper": hw_limits["upper"] - buffer,
            }
        return limits

    def is_unity_connected(self):
        """
        === 檢查 Unity 連線狀態 ===
        Returns:
            bool: True 如果 Unity 連線正常（心跳未超時），False 如果超時或未啟用檢測
        """
        if not self.enable_heartbeat_check:
            return True  # 未啟用檢測時，視為永遠連線

        if self.last_heartbeat_time == 0.0:
            return False  # 從未收到過心跳

        now = self.get_clock().now().nanoseconds / 1e9
        time_since_heartbeat = now - self.last_heartbeat_time
        return time_since_heartbeat <= self.heartbeat_timeout

    def listener_callback(self, msg: JointState):
        # 分離左右臂的關節資料
        left_joints = []
        left_positions = []
        right_joints = []
        right_positions = []

        # 🔥 新增：夾爪資料
        left_gripper_pos = None
        right_gripper_pos = None

        for i, name in enumerate(msg.name):
            # 映射 Unity 名稱 (L_J1 -> openarm_left_joint1)
            if name.startswith("L_J"):
                joint_num = name.split("_J")[1]
                ros_name = f"{self.left_prefix}joint{joint_num}"
                left_joints.append(ros_name)
                left_positions.append(msg.position[i])
            elif name.startswith("R_J"):
                joint_num = name.split("_J")[1]
                ros_name = f"{self.right_prefix}joint{joint_num}"
                right_joints.append(ros_name)
                right_positions.append(msg.position[i])
            # 🔥 新增：處理夾爪
            elif name == "L_EE":
                left_gripper_pos = msg.position[i]
            elif name == "R_EE":
                right_gripper_pos = msg.position[i]

        # === 更新左臂暫存目標（只保留最新一筆） ===
        if left_joints:
            # 限制位置在安全範圍內
            clamped_positions = self.clamp_joint_positions(left_joints, left_positions)
            # 🔧 位置平滑：用指數移動平均減少小抖動
            smoothed_positions = self.smooth_positions(
                left_joints, clamped_positions, self.left_position_filter
            )
            # 僅更新暫存目標，不立即發送，避免覆蓋正在執行的軌跡
            self.pending_left_target = (left_joints, smoothed_positions)
            self.get_logger().debug(
                f"Updated left pending target: {smoothed_positions[0]:.3f}..."
            )

        # === 更新右臂暫存目標（只保留最新一筆） ===
        if right_joints:
            # 限制位置在安全範圍內
            clamped_positions = self.clamp_joint_positions(
                right_joints, right_positions
            )
            # 🔧 位置平滑：用指數移動平均減少小抖動
            smoothed_positions = self.smooth_positions(
                right_joints, clamped_positions, self.right_position_filter
            )
            self.pending_right_target = (right_joints, smoothed_positions)
            self.get_logger().debug(
                f"Updated right pending target: {smoothed_positions[0]:.3f}..."
            )

        # === 嘗試啟動下一條軌跡（若上一筆已完成） ===
        self.try_start_next_trajectory("left")
        self.try_start_next_trajectory("right")

        # 🔥 新增：發送夾爪命令
        if left_gripper_pos is not None:
            self.send_gripper_command("left", left_gripper_pos)

        if right_gripper_pos is not None:
            self.send_gripper_command("right", right_gripper_pos)

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
            parts = joint_name.split("_")
            joint_key = parts[-1]  # 獲取最後一部分（joint1, joint2, etc.）

            # 獲取速度限制
            if joint_key not in self.joint_velocity_limits:
                self.get_logger().warn(
                    f"No velocity limit found for {joint_key}, using default"
                )
                continue

            max_velocity = self.joint_velocity_limits[joint_key]

            # 獲取當前位置（如果沒有記錄則假設為 0）
            current_pos = self.current_positions.get(joint_name, 0.0)
            target_pos = target_positions[i]

            # 計算位置變化和所需時間
            delta_pos = abs(target_pos - current_pos)
            required_time = (
                delta_pos / max_velocity
                if max_velocity > 0
                else self.min_trajectory_time
            )

            # 記錄計算過程（僅在變化較大時）
            if delta_pos > 0.1:  # 只記錄大於 0.1 rad 的變化
                self.get_logger().info(
                    f"{joint_key}: Δ={delta_pos:.3f}rad, v_max={max_velocity:.1f}rad/s, t={required_time:.3f}s"
                )

            max_time = max(max_time, required_time)

        return max_time

    def clamp_joint_positions(self, joint_names, positions):
        """
        限制關節位置在安全範圍內，防止超出機械限制
        🔥 區分左右手使用不同的限制！

        Args:
            joint_names: 關節名稱列表
            positions: 目標位置列表

        Returns:
            list: 限制後的安全位置列表
        """
        clamped_positions = []

        for i, joint_name in enumerate(joint_names):
            # 提取關節編號和判斷左右手
            parts = joint_name.split("_")
            joint_key = parts[-1]  # joint1, joint2, etc.
            
            # 🔥 根據關節名稱選擇左右手的限制
            if "left" in joint_name:
                position_limits = self.left_joint_position_limits
            else:
                position_limits = self.right_joint_position_limits

            target_pos = positions[i]

            # 檢查並限制位置
            if joint_key in position_limits:
                limits = position_limits[joint_key]
                original_pos = target_pos

                # 限制在上下限範圍內
                clamped_pos = max(limits["lower"], min(target_pos, limits["upper"]))

                # 如果位置被限制了，記錄警告
                if abs(clamped_pos - original_pos) > 0.001:  # 超過 0.001 rad 才警告
                    self.get_logger().warn(
                        f"{joint_name}: Position clamped from {original_pos:.3f} to {clamped_pos:.3f} "
                        f'(limits: [{limits["lower"]:.3f}, {limits["upper"]:.3f}])'
                    )

                clamped_positions.append(clamped_pos)
            else:
                # 如果沒有找到限制，使用原始值並警告
                self.get_logger().warn(
                    f"No position limits found for {joint_key}, using original position"
                )
                clamped_positions.append(target_pos)

        return clamped_positions

    def smooth_positions(self, joint_names, new_positions, filter_dict):
        """
        使用指數移動平均平滑位置命令，減少目標位置的高頻抖動。

        Args:
            joint_names: 關節名稱列表
            new_positions: 新的目標位置列表（已做過安全 clamp）
            filter_dict: 濾波器字典（每個關節儲存上一個平滑後的位置）

        Returns:
            list: 平滑後的位置列表
        """
        alpha = self.position_smoothing_factor
        smoothed = []

        for i, joint_name in enumerate(joint_names):
            new_pos = new_positions[i]

            if joint_name in filter_dict:
                old_pos = filter_dict[joint_name]
                # 指數移動平均：smoothed = α * new + (1-α) * old
                smoothed_pos = alpha * new_pos + (1.0 - alpha) * old_pos
            else:
                # 第一次出現時，直接使用新的位置
                smoothed_pos = new_pos

            filter_dict[joint_name] = smoothed_pos
            smoothed.append(smoothed_pos)

        return smoothed

    def try_start_next_trajectory(self, side: str):
        """
        === 軌跡佇列功能 ===
        若上一條軌跡已「大致完成」，且有暫存的最新目標，則送出下一條軌跡。
        - 不會在上一筆尚未完成時覆蓋目標，避免實體手臂被中途打斷造成抖動。
        - Unity 持續更新時，只會保留「最後一筆」作為 pending。
        """
        now = self.get_clock().now().nanoseconds / 1e9

        if side == "left":
            pending = self.pending_left_target
            last_target = self.last_left_target
            last_send = self.last_left_send_time
            last_duration = self.last_left_traj_duration
        else:
            pending = self.pending_right_target
            last_target = self.last_right_target
            last_send = self.last_right_send_time
            last_duration = self.last_right_traj_duration

        # 沒有待發目標，直接返回
        if pending is None:
            self.get_logger().debug(f"[{side}] No pending target")
            return

        # === Unity 心跳檢測 ===
        if self.enable_heartbeat_check:
            # 如果從未收到過心跳（last_heartbeat_time == 0），不發送軌跡
            if self.last_heartbeat_time == 0.0:
                self.get_logger().debug(
                    f"[{side}] Waiting for first Unity heartbeat..."
                )
                return

            # 檢查心跳是否超時
            time_since_heartbeat = now - self.last_heartbeat_time
            if time_since_heartbeat > self.heartbeat_timeout:
                # 只在第一次超時時警告，避免重複 log
                if not self.heartbeat_timeout_warned:
                    self.get_logger().warn(
                        f"⚠️ Unity heartbeat timeout! Last heartbeat: {time_since_heartbeat:.1f}s ago (timeout: {self.heartbeat_timeout}s)"
                    )
                    self.heartbeat_timeout_warned = True
                return

        # 最小發送間隔保護
        time_since_last = now - last_send if last_send > 0.0 else 1e6
        if time_since_last < self.min_send_interval:
            self.get_logger().debug(
                f"[{side}] Too soon since last send: {time_since_last:.3f}s < {self.min_send_interval}s"
            )
            return

        # 如果還沒有送過任何目標（第一次），可以直接送出 pending
        if not last_target:
            joint_names, positions = pending
            traj_time = self.calculate_safe_trajectory_time(joint_names, positions)
            traj_msg = self.create_trajectory_msg(joint_names, positions, traj_time)

            if side == "left":
                self.left_arm_pub.publish(traj_msg)
                self.last_left_target = dict(zip(joint_names, positions))
                self.last_left_send_time = now
                self.last_left_traj_duration = traj_time
                self.pending_left_target = None
                self.get_logger().info(
                    f"✅ [{side}] First trajectory sent: {positions[0]:.3f}..."
                )
            else:
                self.right_arm_pub.publish(traj_msg)
                self.last_right_target = dict(zip(joint_names, positions))
                self.last_right_send_time = now
                self.last_right_traj_duration = traj_time
                self.pending_right_target = None
                self.get_logger().info(
                    f"✅ [{side}] First trajectory sent: {positions[0]:.3f}..."
                )

            return

        # 已經送過上一筆目標：
        # 使用「上一筆軌跡的安全執行時間」來判斷是否可以啟動下一筆
        if time_since_last < last_duration:
            self.get_logger().debug(
                f"[{side}] Still within safe trajectory time: {time_since_last:.3f}s < {last_duration:.3f}s"
            )
            return

        # 走到這裡代表：已經超過上一筆軌跡的安全執行時間
        # 可以送出 pending 中「最新」目標作為下一筆軌跡（若變化足夠大）
        joint_names, positions = pending

        # === 目標變化死區判斷 ===
        # 若新目標與上一個目標的最大差異小於 motion_done_epsilon，視為變化太小，不送新軌跡。
        if last_target:
            max_delta = 0.0
            for i, name in enumerate(joint_names):
                prev = last_target.get(name, positions[i])
                delta = abs(positions[i] - prev)
                if delta > max_delta:
                    max_delta = delta

            if max_delta < self.motion_done_epsilon:
                self.get_logger().debug(
                    f"[{side}] Skip trajectory: max_delta={max_delta:.4f} < epsilon={self.motion_done_epsilon:.4f}"
                )
                if side == "left":
                    self.pending_left_target = None
                else:
                    self.pending_right_target = None
                return

        traj_time = self.calculate_safe_trajectory_time(joint_names, positions)
        traj_msg = self.create_trajectory_msg(joint_names, positions, traj_time)

        if side == "left":
            self.left_arm_pub.publish(traj_msg)
            self.last_left_target = dict(zip(joint_names, positions))
            self.last_left_send_time = now
            self.last_left_traj_duration = traj_time
            self.pending_left_target = None
            self.get_logger().info(
                f"✅ [{side}] Next trajectory sent: {positions[0]:.3f}... (waited {time_since_last:.3f}s, last_duration={last_duration:.3f}s)"
            )
        else:
            self.right_arm_pub.publish(traj_msg)
            self.last_right_target = dict(zip(joint_names, positions))
            self.last_right_send_time = now
            self.last_right_traj_duration = traj_time
            self.pending_right_target = None
            self.get_logger().info(
                f"✅ [{side}] Next trajectory sent: {positions[0]:.3f}... (waited {time_since_last:.3f}s, last_duration={last_duration:.3f}s)"
            )

    def create_trajectory_msg(
        self, joint_names, positions, trajectory_time: float = None
    ):
        msg = JointTrajectory()
        msg.header = Header()
        # msg.header.stamp = self.get_clock().now().to_msg() # 使用當前時間
        # 若不設定 stamp，controller 會立即執行

        msg.joint_names = joint_names

        # 計算安全的軌跡執行時間（基於速度限制）
        if trajectory_time is None:
            trajectory_time = self.calculate_safe_trajectory_time(
                joint_names, positions
            )

        point = JointTrajectoryPoint()
        point.positions = positions

        # 使用動態計算的時間而非固定的 200ms
        point.time_from_start.sec = int(trajectory_time)
        point.time_from_start.nanosec = int((trajectory_time % 1) * 1e9)

        msg.points.append(point)

        # 記錄使用的軌跡時間
        self.get_logger().info(f"Trajectory time: {trajectory_time:.3f}s")

        return msg

    def send_gripper_command(self, side: str, position: float):
        """
        發送夾爪命令

        Args:
            side: 'left' 或 'right'
            position: 夾爪位置（公尺），Unity 範圍：0~0.0425m
        """
        # 限制範圍並轉換（Unity: 0~0.0425m, ROS2: 0~0.03m）
        # Unity 的 0.0425m 對應完全打開，我們映射到 ROS2 的 0.03m
        position_clamped = max(0.0, min(position, 0.0425))
        position_scaled = position_clamped * (0.03 / 0.0425)  # 轉換到 ROS2 範圍

        # 檢查是否需要發送（避免頻繁發送相同命令）
        if side == "left":
            if (
                self.last_left_gripper_pos is not None
                and abs(position_scaled - self.last_left_gripper_pos)
                < self.gripper_position_threshold
            ):
                return
            self.last_left_gripper_pos = position_scaled
            client = self.left_gripper_client
            controller_name = "left_gripper_controller"
        else:  # right
            if (
                self.last_right_gripper_pos is not None
                and abs(position_scaled - self.last_right_gripper_pos)
                < self.gripper_position_threshold
            ):
                return
            self.last_right_gripper_pos = position_scaled
            client = self.right_gripper_client
            controller_name = "right_gripper_controller"

        # 等待 action server
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"{controller_name} action server not available")
            return

        # 創建 GripperCommand goal
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position_scaled
        goal_msg.command.max_effort = 0.1  # 根據 righthand.sh

        # 發送 goal（非阻塞）
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f: self.gripper_goal_response_callback(f, side, position_scaled)
        )

        self.get_logger().info(
            f"{side.capitalize()} gripper: {position:.4f}m (Unity) -> {position_scaled:.4f}m (ROS2)"
        )

    def gripper_goal_response_callback(self, future, side: str, position: float):
        """夾爪 goal 回應回調"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"{side.capitalize()} gripper goal rejected")
            return

        self.get_logger().debug(
            f"{side.capitalize()} gripper goal accepted: {position:.4f}m"
        )

    def heartbeat_callback(self, msg: String):
        """
        === Unity 心跳回調 ===
        收到 Unity 心跳時更新時間戳，並回傳心跳確認。
        """
        # 更新最後一次心跳時間
        self.last_heartbeat_time = self.get_clock().now().nanoseconds / 1e9

        # 如果之前有超時警告，現在收到心跳了，重置標誌並 log
        if self.heartbeat_timeout_warned:
            self.get_logger().info("✅ Unity heartbeat restored!")
            self.heartbeat_timeout_warned = False

        # 回傳心跳，讓 Unity 知道連線還活著
        echo_msg = String()
        echo_msg.data = msg.data
        self.heartbeat_pub.publish(echo_msg)
        self.get_logger().debug(f"Echoed heartbeat: {msg.data}")

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
                suffix = name.replace(self.left_prefix, "")  # joint1
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
        if self.enable_joint_state_log and self.joint_state_count % 100 == 0:
            self.get_logger().info(
                f"Relayed JointState to Unity (Count: {self.joint_state_count})"
            )

        # 位置更新目前只用於計算安全執行時間（速度限制），
        # 是否啟動下一筆軌跡則改由 Unity 訊息觸發的 try_start_next_trajectory 控制


def main(args=None):
    rclpy.init(args=args)
    node = UnityInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
