#!/usr/bin/env python3
"""
Unity Follower Interface - 500Hz MIT 控制

模仿 OpenArm Teleop Follower 模式：
- 繞過 ros2_control，直接使用 openarm_can 控制馬達
- 500Hz MIT 控制迴圈，實現即時跟隨
- Unity ~60Hz 輸入，Linux 端高頻追蹤

注意：使用此腳本時，不能同時執行 ros2_control（CAN 會衝突）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import openarm_can as oa
import threading
import time
import os

# 靈巧手控制器（方案 A：模組化）
from dexterous_hand_controller import DexterousHandController

# Ruckig 軌跡平滑（條件 import）
try:
    from ruckig import Ruckig, InputParameter, OutputParameter, Result
    RUCKIG_AVAILABLE = True
except ImportError:
    RUCKIG_AVAILABLE = False

# ===== [JOINT_LOGGER] 關節數據記錄 - 開始 =====
# 設為 False 可完全停用記錄功能
ENABLE_JOINT_LOGGING = True
LOG_FREQUENCY = 50          # 記錄頻率 (Hz)
LOG_MAX_DURATION = 60.0     # 最大記錄時長 (秒)
AUTO_PLOT_ON_SHUTDOWN = True  # 關閉時自動繪圖

if ENABLE_JOINT_LOGGING:
    try:
        import sys
        import os
        _script_dir = os.path.dirname(os.path.abspath(__file__))
        if _script_dir not in sys.path:
            sys.path.insert(0, _script_dir)
        from utils.joint_data_logger import JointDataLogger
        JOINT_LOGGER_AVAILABLE = True
    except ImportError as e:
        print(f"[WARN] JointDataLogger 無法載入: {e}")
        JOINT_LOGGER_AVAILABLE = False
else:
    JOINT_LOGGER_AVAILABLE = False
# ===== [JOINT_LOGGER] 關節數據記錄 - 結束 =====

# ============================================================================
# 硬體配置
# ============================================================================

# CAN 介面
LEFT_CAN_INTERFACE = "can2"
RIGHT_CAN_INTERFACE = "can1"

# 馬達類型（V10 配置）
MOTOR_TYPES = [
    oa.MotorType.DM8009,  # Joint 1
    oa.MotorType.DM8009,  # Joint 2
    oa.MotorType.DM4340,  # Joint 3
    oa.MotorType.DM4340,  # Joint 4
    oa.MotorType.DM4310,  # Joint 5
    oa.MotorType.DM4310,  # Joint 6
    oa.MotorType.DM4310,  # Joint 7
]

# 馬達 CAN ID
SEND_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
RECV_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]

# 夾爪配置（舊夾爪 - 達妙 DM4310）
GRIPPER_MOTOR_TYPE = oa.MotorType.DM4310
GRIPPER_SEND_ID = 0x08
GRIPPER_RECV_ID = 0x18

# ============================================================================
# 末端執行器配置（並存模式）
# ============================================================================
# 夾爪和靈巧手可同時啟用，根據收到的命令自動選擇：
#   - L_EE / R_EE 命令 → 控制夾爪
#   - L_F1~L_F6 / R_F1~R_F6 命令 → 控制靈巧手
ENABLE_GRIPPER = True         # 啟用夾爪控制
ENABLE_DEXTEROUS_HAND = True  # 啟用靈巧手控制（需要 USB CANFD 設備）

# 靈巧手配置（使用 USB CANFD DEBUG 設備 via ZLGCAN SDK）
# 注意：左右靈巧手共用同一個 USB CANFD 設備，透過不同 CAN ID 區分
DEXTEROUS_HAND_LEFT_CAN_ID = 0x12   # 左靈巧手 CAN ID
DEXTEROUS_HAND_RIGHT_CAN_ID = 0x11  # 右靈巧手 CAN ID
DEXTEROUS_HAND_SPEED = 230          # 速度 (0~255)
DEXTEROUS_HAND_TORQUE = 230         # 力矩 (0~255)
DEXTEROUS_HAND_CONTROL_FREQ = 30    # 控制頻率 (Hz) - 時間估算機制會自動防塞車

# ============================================================================
# MIT 控制參數
# ============================================================================

# Kp, Kd 參數（沿用 ros2_control 配置）
# KP = [70.0, 70.0, 20.0, 20.0,  5.0,  5.0,  5.0]
# KP = [20.0, 20.0, 10.0, 10.0, 5.0, 5.0, 5.0]
KP = [30.0, 30.0, 20.0,20.0, 5.0, 5.0, 5.0]
KD = [2.75, 2.5, 0.7, 0.4, 0.7, 0.6, 0.5]

# 夾爪參數
GRIPPER_KP = 5.0
GRIPPER_KD = 0.1

# 控制頻率
CONTROL_FREQUENCY = 500  # Hz
CONTROL_PERIOD = 1.0 / CONTROL_FREQUENCY  # 2ms

# 動態重力補償參數 (Nm) - 從 v10_simple_hardware.cpp 移植
MAX_COMP_JOINT0 = 12.0   # Joint 0: 肩膀前後抬升最大補償
MAX_COMP_JOINT1 = 12.0   # Joint 1: 肩膀旋轉最大補償
MAX_COMP_JOINT3 = 5.0    # Joint 3: 手肘最大補償
FULL_COMP_THRESHOLD = 0.1  # 誤差大於此值時給全力補償

# 目標位置變化率限制 (rad/s) - 防止移動太快觸發馬達保護
MAX_POSITION_RATE = [1.2, 1.2, 1.5, 1.5, 2.0, 2.0, 2.0]  # 各關節最大速度 (rad/s)

# ============================================================================
# 優化開關（用於 A/B 測試）
# ============================================================================

# A. 狀態快取：避免 control_loop 和 state_timer 同時操作 CAN
#    True = 所有 CAN I/O 在 control_loop，Timer 只發布快取（推薦）
#    False = 原始行為，Timer 也會操作 CAN
USE_STATE_CACHE = False

# B. Deadline 時序：使用 perf_counter + deadline 避免時序漂移
#    True = 使用 deadline 模式（推薦）
#    False = 原始 sleep(period - elapsed) 模式
USE_DEADLINE_TIMING = False

# E. Ruckig 軌跡平滑：使用 Ruckig 產生 jerk-limited 軌跡
#    True = 使用 Ruckig 平滑 + 速度前饋（推薦）
#    False = 使用原始 rate limiting
USE_RUCKIG_SMOOTHING = True

# Ruckig 參數（只在 USE_RUCKIG_SMOOTHING = True 時使用）
# [2026-01-30] A+ 方案：根據馬達實際能力調整（約 30~50% 馬達能力）
# 舊值: [1.2, 1.2, 1.5, 1.5, 2.0, 2.0, 2.0]
RUCKIG_MAX_VELOCITY = [4.0, 4.0, 2.5, 2.5, 6.0, 6.0, 6.0]  # rad/s
# 舊值: [8.0, 8.0, 12.0, 12.0, 15.0, 15.0, 15.0]
RUCKIG_MAX_ACCELERATION = [20.0, 20.0, 15.0, 15.0, 30.0, 30.0, 30.0]  # rad/s²
# 舊值: [40.0, 40.0, 60.0, 60.0, 80.0, 80.0, 80.0]
RUCKIG_MAX_JERK = [100.0, 100.0, 80.0, 80.0, 150.0, 150.0, 150.0]  # rad/s³

# ============================================================================
# 安全限制
# ============================================================================

# 位置限制 (rad) - 右臂
RIGHT_POSITION_LIMITS = {
    0: {"lower": -1.396263, "upper": 3.490659},   # Joint 1
    1: {"lower": -0.174533, "upper": 3.316125},   # Joint 2
    2: {"lower": -1.570796, "upper": 1.570796},   # Joint 3
    3: {"lower": 0.0, "upper": 2.443461},         # Joint 4
    4: {"lower": -1.570796, "upper": 1.570796},   # Joint 5
    5: {"lower": -0.785398, "upper": 0.785398},   # Joint 6
    6: {"lower": -1.570796, "upper": 1.570796},   # Joint 7
}

# 位置限制 (rad) - 左臂
LEFT_POSITION_LIMITS = {
    0: {"lower": -3.490659, "upper": 1.396263},   # Joint 1
    1: {"lower": -3.316125, "upper": 0.174533},   # Joint 2
    2: {"lower": -1.570796, "upper": 1.570796},   # Joint 3
    3: {"lower": 0.0, "upper": 2.443461},         # Joint 4
    4: {"lower": -1.570796, "upper": 1.570796},   # Joint 5
    5: {"lower": -0.785398, "upper": 0.785398},   # Joint 6
    6: {"lower": -1.570796, "upper": 1.570796},   # Joint 7
}

# Unity 超時（秒）
UNITY_TIMEOUT = 2.0


class UnityFollowerInterface(Node):
    def __init__(self):
        super().__init__('unity_interface_follower')
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("Unity Follower Interface - 500Hz MIT Control")
        self.get_logger().info("=" * 50)
        
        # === 目標位置（執行緒安全） ===
        self.left_target = [0.0] * 7
        self.right_target = [0.0] * 7
        self.left_gripper_target = 0.0
        self.right_gripper_target = 0.0
        self.target_lock = threading.Lock()
        
        # === 平滑後的目標（用於限制變化率） ===
        self.left_smoothed = [0.0] * 7
        self.right_smoothed = [0.0] * 7
        
        # === Unity 連線狀態 ===
        self.last_unity_time = time.time()  # 初始化為當前時間，避免啟動時誤判
        self.unity_connected = False
        
        # === 狀態快取（用於 USE_STATE_CACHE 模式）===
        self.state_lock = threading.Lock()
        self.left_state = {
            "pos": [0.0]*7, "vel": [0.0]*7, "tau": [0.0]*7,
            "grip_pos": 0.0, "grip_vel": 0.0, "grip_tau": 0.0
        }
        self.right_state = {
            "pos": [0.0]*7, "vel": [0.0]*7, "tau": [0.0]*7,
            "grip_pos": 0.0, "grip_vel": 0.0, "grip_tau": 0.0
        }
        
        # === 初始化 OpenArm CAN ===
        self.get_logger().info(f"Initializing Left Arm on {LEFT_CAN_INTERFACE}...")
        self.left_arm = oa.OpenArm(LEFT_CAN_INTERFACE, True)
        self.left_arm.init_arm_motors(MOTOR_TYPES, SEND_IDS, RECV_IDS)
        self.left_arm.init_gripper_motor(GRIPPER_MOTOR_TYPE, GRIPPER_SEND_ID, GRIPPER_RECV_ID)
        
        self.get_logger().info(f"Initializing Right Arm on {RIGHT_CAN_INTERFACE}...")
        self.right_arm = oa.OpenArm(RIGHT_CAN_INTERFACE, True)
        self.right_arm.init_arm_motors(MOTOR_TYPES, SEND_IDS, RECV_IDS)
        self.right_arm.init_gripper_motor(GRIPPER_MOTOR_TYPE, GRIPPER_SEND_ID, GRIPPER_RECV_ID)
        
        # === 初始化靈巧手（使用 DexterousHandController 模組）===
        self.zlgcan = None  # 共用的 ZLGCAN 設備
        self.left_hand = None  # 左手控制器
        self.right_hand = None  # 右手控制器
        self.left_hand_target = [0.0] * 6  # 6 個手指 (0~1)
        self.right_hand_target = [0.0] * 6
        self.hand_target_lock = threading.Lock()
        
        # 初始化 ZLGCAN（如果啟用靈巧手）
        if ENABLE_DEXTEROUS_HAND:
            try:
                # 從 usbcanfd_scan.py 導入 ZLGCAN
                import sys
                sys.path.insert(0, '/home/idaka/openarm_can/setup')
                from usbcanfd_scan import ZLGCAN, MODE_NORMAL
                
                self.get_logger().info("Initializing USB CANFD for Dexterous Hand...")
                self.zlgcan = ZLGCAN()
                if self.zlgcan.open_device():
                    self.zlgcan.set_baudrate(0, 1000000, 5000000)  # 1Mbps/5Mbps
                    self.zlgcan.init_channel(0, MODE_NORMAL)
                    self.zlgcan.start_can()
                    
                    # 創建左右手控制器
                    self.left_hand = DexterousHandController(
                        zlgcan=self.zlgcan,
                        can_id=DEXTEROUS_HAND_LEFT_CAN_ID,
                        speed=DEXTEROUS_HAND_SPEED,
                        torque=DEXTEROUS_HAND_TORQUE
                    )
                    self.right_hand = DexterousHandController(
                        zlgcan=self.zlgcan,
                        can_id=DEXTEROUS_HAND_RIGHT_CAN_ID,
                        speed=DEXTEROUS_HAND_SPEED,
                        torque=DEXTEROUS_HAND_TORQUE
                    )
                    
                    self.get_logger().info("✅ USB CANFD initialized!")
                    self.get_logger().info(f"   {self.left_hand}")
                    self.get_logger().info(f"   {self.right_hand}")
                    
                    # 發送回零命令
                    self.left_hand.send_home()
                    self.right_hand.send_home()
                    time.sleep(2.0)  # 等待回零完成
                else:
                    self.get_logger().warn("⚠️ USB CANFD device not found (靈巧手功能停用)")
                    self.zlgcan = None
            except Exception as e:
                self.get_logger().warn(f"⚠️ ZLGCAN init failed: {e} (靈巧手功能停用)")
                self.zlgcan = None
        
        # 顯示末端執行器狀態
        self.get_logger().info(f"End effector mode: COEXISTENCE")
        self.get_logger().info(f"   Gripper: {'✅ Enabled' if ENABLE_GRIPPER else '❌ Disabled'}")
        ehand_ready = (self.left_hand is not None and self.left_hand.is_ready()) or (self.right_hand is not None and self.right_hand.is_ready())
        self.get_logger().info(f"   Dexterous Hand: {'✅ Ready' if ehand_ready else '⚠️ Not available'}")
        
        # === 啟用馬達 ===
        self.get_logger().info("Enabling all motors...")
        self.left_arm.set_callback_mode_all(oa.CallbackMode.STATE)
        self.right_arm.set_callback_mode_all(oa.CallbackMode.STATE)
        self.left_arm.enable_all()
        self.right_arm.enable_all()
        time.sleep(0.1)
        self.left_arm.recv_all()
        self.right_arm.recv_all()
        
        # === 讀取當前位置作為初始目標 ===
        self._read_initial_positions()
        
        # === ROS2 訂閱 ===
        self.unity_sub = self.create_subscription(
            JointState, '/unity/joint_commands',
            self.unity_callback, 10
        )
        
        self.heartbeat_sub = self.create_subscription(
            String, '/unity/heartbeat',
            self.heartbeat_callback, 10
        )
        
        # === 訂閱靈巧手命令（並存模式：總是訂閱，等待命令）===
        if ENABLE_DEXTEROUS_HAND:
            self.ehand_sub = self.create_subscription(
                JointState, '/unity/ehand_commands',
                self.ehand_callback, 10
            )
            self.get_logger().info("✓ Subscribed to /unity/ehand_commands")
        
        # === 發布 JointState 給 Unity ===
        self.joint_state_pub = self.create_publisher(
            JointState, '/openarm/joint_states', 10
        )
        
        # === Ruckig 初始化 ===
        if USE_RUCKIG_SMOOTHING and RUCKIG_AVAILABLE:
            self._init_ruckig()
        elif USE_RUCKIG_SMOOTHING and not RUCKIG_AVAILABLE:
            self.get_logger().warn("⚠️ Ruckig not available, falling back to rate limiting")
        
        # ===== [JOINT_LOGGER] 初始化 - 開始 =====
        # 注意：必須在 control_thread.start() 之前初始化，避免競爭條件
        self.joint_logger = None
        if ENABLE_JOINT_LOGGING and JOINT_LOGGER_AVAILABLE:
            try:
                utils_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'utils')
                self.joint_logger = JointDataLogger(
                    log_frequency=LOG_FREQUENCY,
                    control_frequency=CONTROL_FREQUENCY,
                    max_duration=LOG_MAX_DURATION,
                    save_dir=utils_dir
                )
                self.get_logger().info(f"✅ JointDataLogger 初始化完成")
                self.get_logger().info(f"   記錄頻率: {LOG_FREQUENCY} Hz, 最大時長: {LOG_MAX_DURATION} 秒")
            except Exception as e:
                self.get_logger().warn(f"⚠️ JointDataLogger 初始化失敗: {e}")
                self.joint_logger = None
        # ===== [JOINT_LOGGER] 初始化 - 結束 =====
        
        # === 控制迴圈（獨立執行緒） ===
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        
        # === 狀態發布 Timer (50Hz) ===
        self.state_timer = self.create_timer(0.02, self.publish_joint_states)
        
        self.get_logger().info("✅ Unity Follower Interface started!")
        self.get_logger().info(f"   Control frequency: {CONTROL_FREQUENCY} Hz")
        self.get_logger().info(f"   Left arm: {LEFT_CAN_INTERFACE}, Right arm: {RIGHT_CAN_INTERFACE}")
        self.get_logger().info(f"   USE_STATE_CACHE: {USE_STATE_CACHE}")
        self.get_logger().info(f"   USE_DEADLINE_TIMING: {USE_DEADLINE_TIMING}")
        self.get_logger().info(f"   USE_RUCKIG_SMOOTHING: {USE_RUCKIG_SMOOTHING and RUCKIG_AVAILABLE}")
    
    def _read_initial_positions(self):
        """讀取當前馬達位置作為初始目標（避免啟動時跳動）"""
        self.left_arm.refresh_all()
        self.right_arm.refresh_all()
        self.left_arm.recv_all()
        self.right_arm.recv_all()
        
        with self.target_lock:
            for i, motor in enumerate(self.left_arm.get_arm().get_motors()):
                pos = motor.get_position()
                self.left_target[i] = pos
                self.left_smoothed[i] = pos  # 同步初始化 smoothed
            for i, motor in enumerate(self.right_arm.get_arm().get_motors()):
                pos = motor.get_position()
                self.right_target[i] = pos
                self.right_smoothed[i] = pos  # 同步初始化 smoothed
        
        self.get_logger().info(f"Initial left positions: {[f'{p:.2f}' for p in self.left_target]}")
        self.get_logger().info(f"Initial right positions: {[f'{p:.2f}' for p in self.right_target]}")
    
    def _init_ruckig(self):
        """初始化 Ruckig 軌跡生成器"""
        self.get_logger().info("Initializing Ruckig trajectory generators...")
        
        # 左臂 Ruckig
        self.left_otg = Ruckig(7, CONTROL_PERIOD)
        self.left_ruckig_input = InputParameter(7)
        self.left_ruckig_output = OutputParameter(7)
        
        # 右臂 Ruckig
        self.right_otg = Ruckig(7, CONTROL_PERIOD)
        self.right_ruckig_input = InputParameter(7)
        self.right_ruckig_output = OutputParameter(7)
        
        # 設定限制
        for inp in [self.left_ruckig_input, self.right_ruckig_input]:
            inp.max_velocity = RUCKIG_MAX_VELOCITY
            inp.max_acceleration = RUCKIG_MAX_ACCELERATION
            inp.max_jerk = RUCKIG_MAX_JERK
        
        # 初始化當前狀態（使用讀取的初始位置）
        self.left_ruckig_input.current_position = self.left_smoothed[:]
        self.left_ruckig_input.current_velocity = [0.0] * 7
        self.left_ruckig_input.current_acceleration = [0.0] * 7
        
        self.right_ruckig_input.current_position = self.right_smoothed[:]
        self.right_ruckig_input.current_velocity = [0.0] * 7
        self.right_ruckig_input.current_acceleration = [0.0] * 7
        
        # 速度前饋快取
        self.left_velocity_ff = [0.0] * 7
        self.right_velocity_ff = [0.0] * 7
        
        self.get_logger().info("✅ Ruckig initialized!")
    
    # ===== [JOINT_LOGGER] 開始記錄輔助方法 - 開始 =====
    def _start_joint_logging(self):
        """開始關節數據記錄（避免重複開始）"""
        if self.joint_logger is not None and not self.joint_logger.is_recording:
            self.joint_logger.start()
            self.get_logger().info("📊 JointDataLogger 開始記錄")
    # ===== [JOINT_LOGGER] 開始記錄輔助方法 - 結束 =====
    
    def unity_callback(self, msg: JointState):
        """Unity 傳來目標時，只更新 target（不阻塞）"""
        self.last_unity_time = time.time()
        
        if not self.unity_connected:
            self.unity_connected = True
            self.get_logger().info("✅ Unity connected!")
            # ===== [JOINT_LOGGER] Unity 連線時開始記錄 =====
            self._start_joint_logging()
        
        with self.target_lock:
            for i, name in enumerate(msg.name):
                if name.startswith('L_J'):
                    joint_idx = int(name.split('_J')[1]) - 1
                    if 0 <= joint_idx < 7:
                        pos = self._clamp_position(msg.position[i], joint_idx, LEFT_POSITION_LIMITS)
                        self.left_target[joint_idx] = pos
                elif name.startswith('R_J'):
                    joint_idx = int(name.split('_J')[1]) - 1
                    if 0 <= joint_idx < 7:
                        pos = self._clamp_position(msg.position[i], joint_idx, RIGHT_POSITION_LIMITS)
                        self.right_target[joint_idx] = pos
                elif name == 'L_EE':
                    self.left_gripper_target = self._gripper_to_motor(msg.position[i])
                elif name == 'R_EE':
                    self.right_gripper_target = self._gripper_to_motor(msg.position[i])
    
    def heartbeat_callback(self, msg: String):
        """心跳回調"""
        self.last_unity_time = time.time()
        if not self.unity_connected:
            self.unity_connected = True
            self.get_logger().info("✅ Unity heartbeat received!")
            # ===== [JOINT_LOGGER] 心跳連線時開始記錄 =====
            self._start_joint_logging()
    
    def ehand_callback(self, msg: JointState):
        """靈巧手命令回調"""
        self.last_unity_time = time.time()
        
        # ===== [JOINT_LOGGER] ehand 連線時開始記錄 =====
        if not self.unity_connected:
            self.unity_connected = True
            self.get_logger().info("✅ Unity ehand connected!")
            self._start_joint_logging()
        
        with self.hand_target_lock:
            for i, name in enumerate(msg.name):
                if name.startswith('L_F'):
                    # L_F1 ~ L_F6
                    finger_idx = int(name.split('_F')[1]) - 1
                    if 0 <= finger_idx < 6:
                        self.left_hand_target[finger_idx] = max(0.0, min(1.0, msg.position[i]))
                elif name.startswith('R_F'):
                    # R_F1 ~ R_F6
                    finger_idx = int(name.split('_F')[1]) - 1
                    if 0 <= finger_idx < 6:
                        self.right_hand_target[finger_idx] = max(0.0, min(1.0, msg.position[i]))
        
        # Debug: 每 30 次輸出一次
        if not hasattr(self, '_ehand_log_count'):
            self._ehand_log_count = 0
        self._ehand_log_count += 1
        if self._ehand_log_count % 30 == 0:

            # 顯示所有 6 個手指數值，保留 2 位小數
            l_str = str([round(x, 2) for x in self.left_hand_target])
            r_str = str([round(x, 2) for x in self.right_hand_target])
            self.get_logger().info(f"[ehand] L={l_str}, R={r_str}")
    
    def _clamp_position(self, pos: float, joint_idx: int, limits: dict) -> float:
        """限制位置在安全範圍內"""
        if joint_idx in limits:
            lower = limits[joint_idx]["lower"]
            upper = limits[joint_idx]["upper"]
            return max(lower, min(pos, upper))
        return pos
    
    def _gripper_to_motor(self, joint_value: float) -> float:
        """夾爪位置轉換：Unity (0~0.0425m) → 馬達弧度"""
        # 參考 v10_simple_hardware.cpp
        GRIPPER_JOINT_0_POSITION = 0.044
        GRIPPER_MOTOR_1_RADIANS = -1.0472
        clamped = max(0.0, min(joint_value, 0.0425))
        return (clamped / GRIPPER_JOINT_0_POSITION) * GRIPPER_MOTOR_1_RADIANS
    
    def _calculate_gravity_compensation(self, arm, target_pos, side: str):
        """
        計算動態重力補償力矩
        簡化版本，完全模仿 v10_simple_hardware.cpp 的邏輯
        
        Args:
            arm: OpenArm 物件（用於讀取當前位置）
            target_pos: 目標位置列表
            side: "left" 或 "right"（目前未使用，保留以備將來擴展）
        
        Returns:
            list: 7 個關節的補償力矩
        """
        import math
        
        compensations = [0.0] * 7
        
        # 讀取當前位置
        try:
            motors = arm.get_arm().get_motors()
            current_pos = [m.get_position() for m in motors]
        except:
            return compensations
        
        for i in range(7):
            # 只有補償關節 0, 1, 3
            if i not in [0, 1, 3]:
                continue
            
            position_error = target_pos[i] - current_pos[i]
            abs_error = abs(position_error)
            
            # 🔧 簡化邏輯：完全模仿 C++ 版本
            # - 大誤差時 (>=0.1): 給全力補償（方向與誤差相同）
            # - 小誤差時 (<0.1): 平滑衰減到零
            if abs_error >= FULL_COMP_THRESHOLD:
                compensation_ratio = 1.0 if position_error > 0 else -1.0
            else:
                compensation_ratio = position_error / FULL_COMP_THRESHOLD
            
            # 計算實際補償力矩
            if i == 0:
                posture_factor = abs(math.sin(current_pos[i]))
                posture_factor = max(posture_factor, 0.3)
                compensations[i] = MAX_COMP_JOINT0 * posture_factor * compensation_ratio
            elif i == 1:
                arm_extension = abs(math.sin(current_pos[0]))
                arm_extension = max(arm_extension, 0.2)
                compensations[i] = MAX_COMP_JOINT1 * arm_extension * compensation_ratio
            elif i == 3:
                posture_factor = abs(math.sin(current_pos[i]))
                posture_factor = max(posture_factor, 0.3)
                compensations[i] = MAX_COMP_JOINT3 * posture_factor * compensation_ratio
        
        # 🐛 調試：顯示非零補償和誤差（已註解）
        # non_zero = [(i, round(c, 2)) for i, c in enumerate(compensations) if abs(c) > 0.01]
        # if non_zero:
        #     errors = [(i, round(target_pos[i] - current_pos[i], 3)) for i in [0, 1, 3]]
        #     print(f"[{side}] Errors: {errors}, Comp: {non_zero}")
        
        return compensations
    
    def control_loop(self):
        """500Hz MIT 控制迴圈"""
        self.get_logger().info("Control loop started")
        
        # Deadline 模式初始化
        if USE_DEADLINE_TIMING:
            next_deadline = time.perf_counter()
        
        # 靈巧手控制頻率計數器
        hand_control_interval = CONTROL_FREQUENCY // DEXTEROUS_HAND_CONTROL_FREQ  # 500/50=10
        loop_count = 0
        
        while self.running:
            loop_count += 1
            # === 時序控制：根據模式選擇 ===
            if USE_DEADLINE_TIMING:
                next_deadline += CONTROL_PERIOD
            else:
                loop_start = time.time()
            
            # 檢查 Unity 連線狀態
            current_time = time.time()
            if self.unity_connected and (current_time - self.last_unity_time) > UNITY_TIMEOUT:
                self.unity_connected = False
                self.get_logger().warn("⚠️ Unity connection timeout!")
            
            # 取得最新目標（執行緒安全）
            with self.target_lock:
                left_target = self.left_target.copy()
                right_target = self.right_target.copy()
                left_grip = self.left_gripper_target
                right_grip = self.right_gripper_target
            
            # === 目標平滑：根據模式選擇 ===
            if USE_RUCKIG_SMOOTHING and RUCKIG_AVAILABLE:
                # Ruckig 模式：使用 jerk-limited 軌跡
                self.left_ruckig_input.target_position = left_target
                self.right_ruckig_input.target_position = right_target
                
                # 更新左臂軌跡
                left_result = self.left_otg.update(self.left_ruckig_input, self.left_ruckig_output)
                left_pos = list(self.left_ruckig_output.new_position)
                self.left_velocity_ff = list(self.left_ruckig_output.new_velocity)
                
                # 更新 Ruckig 狀態
                self.left_ruckig_input.current_position = left_pos
                self.left_ruckig_input.current_velocity = self.left_velocity_ff
                self.left_ruckig_input.current_acceleration = list(self.left_ruckig_output.new_acceleration)
                
                # 更新右臂軌跡
                right_result = self.right_otg.update(self.right_ruckig_input, self.right_ruckig_output)
                right_pos = list(self.right_ruckig_output.new_position)
                self.right_velocity_ff = list(self.right_ruckig_output.new_velocity)
                
                # 更新 Ruckig 狀態
                self.right_ruckig_input.current_position = right_pos
                self.right_ruckig_input.current_velocity = self.right_velocity_ff
                self.right_ruckig_input.current_acceleration = list(self.right_ruckig_output.new_acceleration)
                
                # 同步 smoothed 變數（給其他功能使用）
                self.left_smoothed = left_pos[:]
                self.right_smoothed = right_pos[:]
            else:
                # 原始模式：使用 rate limiting
                for i in range(7):
                    max_delta = MAX_POSITION_RATE[i] * CONTROL_PERIOD
                    
                    # 左手
                    delta_left = left_target[i] - self.left_smoothed[i]
                    if abs(delta_left) > max_delta:
                        self.left_smoothed[i] += max_delta if delta_left > 0 else -max_delta
                    else:
                        self.left_smoothed[i] = left_target[i]
                    
                    # 右手
                    delta_right = right_target[i] - self.right_smoothed[i]
                    if abs(delta_right) > max_delta:
                        self.right_smoothed[i] += max_delta if delta_right > 0 else -max_delta
                    else:
                        self.right_smoothed[i] = right_target[i]
                
                left_pos = self.left_smoothed.copy()
                right_pos = self.right_smoothed.copy()
                # 原始模式沒有速度前饋
                self.left_velocity_ff = [0.0] * 7
                self.right_velocity_ff = [0.0] * 7
            
            # 計算重力補償力矩（簡化版本，模仿 C++ 邏輯）
            left_comp = self._calculate_gravity_compensation(self.left_arm, left_pos, "left")
            right_comp = self._calculate_gravity_compensation(self.right_arm, right_pos, "right")
            
            # 建立 MIT 命令（包含速度前饋和重力補償力矩）
            left_arm_cmds = [
                oa.MITParam(KP[i], KD[i], left_pos[i], self.left_velocity_ff[i], left_comp[i]) 
                for i in range(7)
            ]
            right_arm_cmds = [
                oa.MITParam(KP[i], KD[i], right_pos[i], self.right_velocity_ff[i], right_comp[i]) 
                for i in range(7)
            ]
            
            left_grip_cmds = [oa.MITParam(GRIPPER_KP, GRIPPER_KD, left_grip, 0.0, 0.0)]
            
            # 發送到馬達
            try:
                # === 左手臂 ===
                self.left_arm.get_arm().mit_control_all(left_arm_cmds)
                
                # === 左手末端執行器（並存模式）===
                # 夾爪：永遠控制（如果啟用）
                if ENABLE_GRIPPER:
                    self.left_arm.get_gripper().mit_control_all(left_grip_cmds)
                
                # 靈巧手：使用 DexterousHandController
                if self.left_hand and self.left_hand.is_ready() and loop_count % hand_control_interval == 0:
                    with self.hand_target_lock:
                        left_fingers = self.left_hand_target[:]
                    # 發送位置命令（防塞車機制已內建）
                    self.left_hand.send_positions(left_fingers)
                
                self.left_arm.recv_all(500)
                
                # === 右手臂 ===
                self.right_arm.get_arm().mit_control_all(right_arm_cmds)
                
                # === 右手末端執行器（並存模式）===
                # 夾爪：永遠控制（如果啟用）
                if ENABLE_GRIPPER:
                    right_grip_cmds = [oa.MITParam(GRIPPER_KP, GRIPPER_KD, right_grip, 0.0, 0.0)]
                    self.right_arm.get_gripper().mit_control_all(right_grip_cmds)
                
                # 靈巧手：使用 DexterousHandController
                if self.right_hand and self.right_hand.is_ready() and loop_count % hand_control_interval == 0:
                    with self.hand_target_lock:
                        right_fingers = self.right_hand_target[:]
                    # 發送位置命令（防塞車機制已內建）
                    self.right_hand.send_positions(right_fingers)
                
                self.right_arm.recv_all(500)
                
                # ===== [JOINT_LOGGER] 記錄數據 - 開始 =====
                if self.joint_logger is not None and self.joint_logger.is_recording:
                    try:
                        # 取得 Unity 原始目標（未經平滑）
                        with self.target_lock:
                            left_unity = self.left_target[:]
                            right_unity = self.right_target[:]
                        
                        # 取得 OpenArm 實際位置
                        left_motors = self.left_arm.get_arm().get_motors()
                        right_motors = self.right_arm.get_arm().get_motors()
                        left_actual = [m.get_position() for m in left_motors]
                        right_actual = [m.get_position() for m in right_motors]
                        
                        # 記錄
                        self.joint_logger.log(
                            left_unity_target=left_unity,
                            left_actual=left_actual,
                            right_unity_target=right_unity,
                            right_actual=right_actual
                        )
                    except Exception as e:
                        pass  # 記錄失敗不影響控制迴圈
                # ===== [JOINT_LOGGER] 記錄數據 - 結束 =====
                
                # === 狀態快取：在 recv_all 後更新快取 ===
                if USE_STATE_CACHE:
                    with self.state_lock:
                        # 左臂狀態
                        lm = self.left_arm.get_arm().get_motors()
                        self.left_state["pos"] = [m.get_position() for m in lm]
                        self.left_state["vel"] = [m.get_velocity() for m in lm]
                        self.left_state["tau"] = [m.get_torque() for m in lm]
                        # 左夾爪
                        try:
                            lg = self.left_arm.get_gripper().get_motors()[0]
                            self.left_state["grip_pos"] = lg.get_position()
                            self.left_state["grip_vel"] = lg.get_velocity()
                            self.left_state["grip_tau"] = lg.get_torque()
                        except:
                            pass
                        
                        # 右臂狀態
                        rm = self.right_arm.get_arm().get_motors()
                        self.right_state["pos"] = [m.get_position() for m in rm]
                        self.right_state["vel"] = [m.get_velocity() for m in rm]
                        self.right_state["tau"] = [m.get_torque() for m in rm]
                        # 右夾爪
                        try:
                            rg = self.right_arm.get_gripper().get_motors()[0]
                            self.right_state["grip_pos"] = rg.get_position()
                            self.right_state["grip_vel"] = rg.get_velocity()
                            self.right_state["grip_tau"] = rg.get_torque()
                        except:
                            pass
                
            except Exception as e:
                self.get_logger().error(f"Control error: {e}")
            
            # === 時序控制：維持控制頻率 ===
            if USE_DEADLINE_TIMING:
                # Deadline 模式：等待到下一個時間點
                now = time.perf_counter()
                sleep_time = next_deadline - now
                if sleep_time > 0:
                    time.sleep(sleep_time)
                # 如果超時，重置 deadline 避免追趕
                elif sleep_time < -CONTROL_PERIOD:
                    next_deadline = time.perf_counter()
            else:
                # 原始模式
                elapsed = time.time() - loop_start
                sleep_time = CONTROL_PERIOD - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
    
    def publish_joint_states(self):
        """發布當前關節狀態給 Unity (50Hz)"""
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            
            if USE_STATE_CACHE:
                # === 快取模式：不操作 CAN，只讀取快取 ===
                with self.state_lock:
                    lpos = self.left_state["pos"][:]
                    lvel = self.left_state["vel"][:]
                    ltau = self.left_state["tau"][:]
                    rpos = self.right_state["pos"][:]
                    rvel = self.right_state["vel"][:]
                    rtau = self.right_state["tau"][:]
                
                # 左臂
                for i in range(7):
                    msg.name.append(f'openarm_left_joint{i+1}')
                    msg.position.append(lpos[i])
                    msg.velocity.append(lvel[i])
                    msg.effort.append(ltau[i])
                
                # 右臂
                for i in range(7):
                    msg.name.append(f'openarm_right_joint{i+1}')
                    msg.position.append(rpos[i])
                    msg.velocity.append(rvel[i])
                    msg.effort.append(rtau[i])
            else:
                # === 原始模式：直接操作 CAN ===
                self.left_arm.refresh_all()
                self.right_arm.refresh_all()
                self.left_arm.recv_all(200)
                self.right_arm.recv_all(200)
                
                # 左臂
                for i, motor in enumerate(self.left_arm.get_arm().get_motors()):
                    msg.name.append(f'openarm_left_joint{i+1}')
                    msg.position.append(motor.get_position())
                    msg.velocity.append(motor.get_velocity())
                    msg.effort.append(motor.get_torque())
                
                # 右臂
                for i, motor in enumerate(self.right_arm.get_arm().get_motors()):
                    msg.name.append(f'openarm_right_joint{i+1}')
                    msg.position.append(motor.get_position())
                    msg.velocity.append(motor.get_velocity())
                    msg.effort.append(motor.get_torque())
            
            self.joint_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"State publish error: {e}")
    
    def shutdown(self):
        """關閉介面"""
        self.get_logger().info("Shutting down...")
        self.running = False
        
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
        
        # ===== [JOINT_LOGGER] 儲存數據並繪圖 - 開始 =====
        if self.joint_logger is not None:
            try:
                self.joint_logger.stop()
                stats = self.joint_logger.get_stats()
                
                if stats['count'] > 0:
                    # 儲存數據
                    data_file = self.joint_logger.save()
                    self.get_logger().info(f"📊 數據已儲存: {data_file}")
                    
                    # 自動繪圖
                    if AUTO_PLOT_ON_SHUTDOWN:
                        try:
                            from utils.plot_joint_comparison import plot_joint_comparison
                            self.get_logger().info("🎨 正在繪製比較圖...")
                            output_files = plot_joint_comparison(data_file, arm='both')
                            self.get_logger().info(f"✅ 已產生 {len(output_files)} 張圖")
                        except Exception as e:
                            self.get_logger().warn(f"⚠️ 繪圖失敗: {e}")
                else:
                    self.get_logger().info("📊 無數據需要儲存")
            except Exception as e:
                self.get_logger().warn(f"⚠️ JointDataLogger 儲存失敗: {e}")
        # ===== [JOINT_LOGGER] 儲存數據並繪圖 - 結束 =====
        
        # 關閉靈巧手（並存模式：張開兩隻手）
        if self.dexterous_hand_ready and self.zlgcan is not None:
            try:
                self.get_logger().info("Opening dexterous hands before shutdown...")
                # 發送張開命令 (0xFD 0x02)
                cmd_open = bytes([0xFD, 0x02] + [0xFF] * 30)
                self.zlgcan.transmit_fd(DEXTEROUS_HAND_LEFT_CAN_ID, cmd_open)
                self.zlgcan.transmit_fd(DEXTEROUS_HAND_RIGHT_CAN_ID, cmd_open)
                time.sleep(1.0)
                self.zlgcan.close()
                self.get_logger().info("USB CANFD closed")
            except Exception as e:
                self.get_logger().warn(f"Error closing dexterous hand: {e}")
        
        self.left_arm.disable_all()
        self.right_arm.disable_all()
        self.get_logger().info("Motors disabled")


def main(args=None):
    rclpy.init(args=args)
    node = UnityFollowerInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
