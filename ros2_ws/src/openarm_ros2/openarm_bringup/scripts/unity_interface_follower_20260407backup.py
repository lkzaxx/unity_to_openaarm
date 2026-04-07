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
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import openarm_can as oa
import threading
from concurrent.futures import ThreadPoolExecutor
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
AUTO_PLOT_ON_SHUTDOWN = False  # 關閉時自動繪圖

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

# CAN 介面：從 cansetup.sh 產生的 /tmp/can_arm_map 讀取
def _read_can_map():
    mapping = {}
    try:
        with open("/tmp/can_arm_map") as f:
            for line in f:
                line = line.strip()
                if "=" in line and not line.startswith("#"):
                    k, v = line.split("=", 1)
                    mapping[k] = v
    except FileNotFoundError:
        pass
    return mapping.get("LEFT_CAN", "can2"), mapping.get("RIGHT_CAN", "can1")

LEFT_CAN_INTERFACE, RIGHT_CAN_INTERFACE = _read_can_map()

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
ENABLE_DEXTEROUS_HAND = True  # 需保持 True 讓 ZLGCAN 初始化（SYNC_GRIPPER_TO_EHAND 需要）
ENABLE_EHAND_SUBSCRIPTION = False  # 訂閱 /unity/ehand_commands — 目前用 SYNC_GRIPPER_TO_EHAND 代替

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
# KP = [30.0, 30.0, 20.0,20.0, 5.0, 5.0, 5.0]
# KD = [3.5, 3.5, 0.7, 0.4, 0.7, 0.6, 0.5]
# [2026-02-05] 提高 Kp 替代重力補償，同時提高 Kd 保持阻尼比
# KP = [60.0, 60.0, 40.0, 40.0, 30.0, 30.0, 30.0]
# KD = [4.0, 4.0, 1.2, 1.0, 1.0, 1.0, 1.0]
# [2026-02-09] 針對 J1, J2, J4, J7 再提高 Kp（這些關節需抵抗重力）
# KP = [80.0, 80.0, 40.0, 60.0, 30.0, 30.0, 50.0]
# KD = [4.5, 4.5, 1.2, 1.5, 1.0, 1.0, 1.2]
# [2026-02-09] J1 再提高 Kp 改善追蹤
# KP = [100.0, 80.0, 40.0, 60.0, 30.0, 30.0, 50.0]
# KD = [5.0, 4.5, 1.2, 1.5, 1.0, 1.0, 1.2]

# [2026-02-10] 動態 Kp/Kd 方案（根據誤差大小調整）
USE_DYNAMIC_KP_KD = False  # 開啟動態 Kp/Kd

# [2026-02-11] 只對指定關節使用動態 Kp/Kd（其他關節出現震盪）
DYNAMIC_KP_KD_JOINTS = [0, 1]  # 只對 J1, J2 使用動態調整

# 基礎 Kp/Kd（動態調整的基準值）
# [2026-02-11] J1 提高 Kp 改善動態追蹤延遲
BASE_KP = [60.0, 60.0, 25.0, 25.0, 30.0, 30.0, 50.0]
BASE_KD = [5.0, 4.5, 1.2, 1.5, 1.0, 1.0, 1.2]

# 動態調整參數
LARGE_ERROR_THRESHOLD = 0.2   # 大誤差閾值 (rad)
SMALL_ERROR_THRESHOLD = 0.05  # 小誤差閾值 (rad)
LARGE_ERROR_KP_SCALE = 1.5    # 大誤差時 Kp 放大倍數
LARGE_ERROR_KD_SCALE = 0.8    # 大誤差時 Kd 縮小倍數
SMALL_ERROR_KP_SCALE = 0.7    # 小誤差時 Kp 縮小倍數
SMALL_ERROR_KD_SCALE = 1.2    # 小誤差時 Kd 放大倍數

# 靜態 Kp/Kd（USE_DYNAMIC_KP_KD = False 時使用）
KP = BASE_KP[:]
KD = BASE_KD[:]

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
# [2026-04-07] 提高到馬達能力的 ~20-30%（原值僅 3-7%，追蹤延遲過大）
# 舊值: [1.2, 1.2, 1.5, 1.5, 2.0, 2.0, 2.0]
MAX_POSITION_RATE = [6.0, 6.0, 2.5, 2.5, 8.0, 8.0, 8.0]  # 各關節最大速度 (rad/s)

# ============================================================================
# 優化開關（用於 A/B 測試）
# ============================================================================

# A. 狀態快取：避免 control_loop 和 state_timer 同時操作 CAN
#    True = 所有 CAN I/O 在 control_loop，Timer 只發布快取（推薦）
#    False = 原始行為，Timer 也會操作 CAN
USE_STATE_CACHE = True  # Must be True to avoid CAN race condition (50Hz timer vs 500Hz control loop)

# B. Deadline 時序：使用 perf_counter + deadline 避免時序漂移
#    True = 使用 deadline 模式（推薦）
#    False = 原始 sleep(period - elapsed) 模式
# [2026-04-07] 改為 True：使用 perf_counter + deadline 避免時序漂移
USE_DEADLINE_TIMING = True

# E. Ruckig 軌跡平滑：使用 Ruckig 產生 jerk-limited 軌跡
#    True = 使用 Ruckig 平滑 + 速度前饋（推薦）
#    False = 使用原始 rate limiting
# [2026-02-05] 測試關閉 Ruckig，直接追蹤 Unity 目標
USE_RUCKIG_SMOOTHING = False

# [TEMP] 夾爪連動靈巧手：夾爪 L_EE/R_EE 同步控制靈巧手張合
# True = 夾爪關閉時手也握，夾爪打開時手也張
# 不需要時改為 False 即可停用
SYNC_GRIPPER_TO_EHAND = True
SYNC_GRIPPER_THRESHOLD = 0.02  # 夾爪值 < 此值視為「關閉」(單位: meters, 0=關 0.0425=開)

# [TEMP DEMO] 鎖定特定關節到固定角度（不需要時改為空 dict）
# 格式：{(side, joint_index): lock_rad}  side="left"/"right", joint_index=0~6 (J1~J7)
# [2026-04-07] 暫時關閉關節鎖定，恢復右臂完整 7DOF
# LOCK_JOINTS = {
#     ("right", 2): 0.0,  # 右手 J3 鎖在 0 rad
#     ("right", 4): 0.0,  # 右手 J5 鎖在 0 rad
# }
LOCK_JOINTS = {}

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
        
        # === 追蹤模式：Unity 未送指令前，用馬達即時位置當目標 ===
        self.left_tracking_unity = False
        self.right_tracking_unity = False
        
        # === Home 指令狀態 ===
        self.left_homing = False   # 左臂正在回零
        self.right_homing = False  # 右臂正在回零
        self.left_disabled = False  # 左臂已 disable
        self.right_disabled = False # 右臂已 disable
        self.left_hand_disabled = False   # 左靈巧手已 disable
        self.right_hand_disabled = False  # 右靈巧手已 disable

        # === 靈巧手握合狀態追蹤（用於 disable→home→grip 流程）===
        self.left_hand_gripping = False   # 左手正在握
        self.right_hand_gripping = False  # 右手正在握
        self._hand_transition_threshold = 0.5  # 超過此值視為「握」
        
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
                    
                    # 探測靈巧手狀態
                    for label, hand in [("Left", self.left_hand), ("Right", self.right_hand)]:
                        status = hand.read_status()
                        if status:
                            fingers_str = ", ".join(
                                f"{f['name']}={f['position']}" for f in status['fingers']
                            )
                            self.get_logger().info(f"   {label} hand status: {status['hand_state']} | {fingers_str}")
                        else:
                            self.get_logger().warn(f"   {label} hand: no response")
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
        # [原始] self._read_initial_positions()  # refresh_all 在未 enable 時部分馬達讀不到
        # [改用] 從 start_follower.sh Step 2 寫入的暫存檔讀取（用 0xFC enable 指令取得的真實角度）
        self._load_initial_positions_from_file("/tmp/initial_joint_positions")

        # 比較 Step 2 角度和 enable 後的實際角度，檢測偏差
        self.left_arm.refresh_all()
        self.right_arm.refresh_all()
        self.left_arm.recv_all()
        self.right_arm.recv_all()
        for label, arm, target in [
            ("Left", self.left_arm, self.left_target),
            ("Right", self.right_arm, self.right_target)
        ]:
            actual = [m.get_position() for m in arm.get_arm().get_motors()]
            diffs = [abs(target[i] - actual[i]) for i in range(7)]
            max_diff = max(diffs)
            if max_diff > 0.1:
                self.get_logger().warn(
                    f"⚠️ {label} arm: Step2 vs actual max diff = {max_diff:.3f} rad ({max_diff*57.3:.1f}°) "
                    f"diffs={[f'{d:.3f}' for d in diffs]}"
                )
            else:
                self.get_logger().info(f"✅ {label} arm: Step2 vs actual max diff = {max_diff:.3f} rad — OK")

        # === ROS2 訂閱 ===
        # Use VOLATILE durability to ignore cached/stale messages from DDS.
        # Without this, restarting the follower would replay the last joint command,
        # potentially causing J7 to jump to 0 before J4 is raised (ground collision!).
        volatile_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.unity_sub = self.create_subscription(
            JointState, '/unity/joint_commands',
            self.unity_callback, volatile_qos
        )
        
        self.heartbeat_sub = self.create_subscription(
            String, '/unity/heartbeat',
            self.heartbeat_callback, 10
        )
        
        # === 訂閱靈巧手命令（並存模式：總是訂閱，等待命令）===
        # 使用預設 QoS (depth=10) 讓 ros2 topic pub --once 也能送達
        # joint_commands 保持 VOLATILE 避免重啟時 replay 舊的手臂指令（安全考量）
        if ENABLE_DEXTEROUS_HAND and ENABLE_EHAND_SUBSCRIPTION:
            self.ehand_sub = self.create_subscription(
                JointState, '/unity/ehand_commands',
                self.ehand_callback, 10
            )
            self.get_logger().info("✓ Subscribed to /unity/ehand_commands")
        elif ENABLE_DEXTEROUS_HAND:
            self.get_logger().info("ℹ ehand subscription disabled (using SYNC_GRIPPER_TO_EHAND)")
        
        # === 發布 JointState 給 Unity ===
        self.joint_state_pub = self.create_publisher(
            JointState, '/openarm/joint_states', 10
        )
        self.hand_state_pub = self.create_publisher(
            JointState, '/openarm/hand_states', 10
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
        
        # === 左右臂並行 CAN 控制用 ThreadPoolExecutor ===
        # [2026-04-07] 左右臂用不同 CAN interface（socket_fd 獨立），可安全並行
        self.arm_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="arm_can")

        # === 控制迴圈（獨立執行緒） ===
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        
        # === 狀態發布 Timer (50Hz) ===
        self.state_timer = self.create_timer(0.02, self.publish_joint_states)
        
        self.get_logger().info("✅ Unity Follower Interface started!")
        self.get_logger().info("⏳ Arms IDLE — waiting for first external command before sending MIT control")
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

    def _load_initial_positions_from_file(self, filepath):
        """從 start_follower.sh Step 2 產生的暫存檔讀取初始角度，並立即啟用追蹤"""
        left_pos = [0.0] * 7
        right_pos = [0.0] * 7
        loaded_count = 0

        try:
            with open(filepath) as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    parts = line.split()
                    if len(parts) != 3:
                        continue
                    side, idx_str, val_str = parts
                    idx = int(idx_str)
                    val = float(val_str)
                    if 0 <= idx < 7:
                        if side == 'LEFT':
                            left_pos[idx] = val
                            loaded_count += 1
                        elif side == 'RIGHT':
                            right_pos[idx] = val
                            loaded_count += 1

            self.get_logger().info(f"✅ Loaded {loaded_count} joint positions from {filepath}")
        except FileNotFoundError:
            self.get_logger().warn(f"⚠️ {filepath} not found, falling back to refresh_all")
            self._read_initial_positions()
            return
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to read {filepath}: {e}, falling back to refresh_all")
            self._read_initial_positions()
            return

        # 寫入 target 和 smoothed
        with self.target_lock:
            for i in range(7):
                self.left_target[i] = left_pos[i]
                self.left_smoothed[i] = left_pos[i]
                self.right_target[i] = right_pos[i]
                self.right_smoothed[i] = right_pos[i]

        # 立即啟用追蹤，控制迴圈開始後即維持此角度
        self.left_tracking_unity = True
        self.right_tracking_unity = True

        self.get_logger().info(f"Initial left positions:  {[f'{p:.2f}' for p in left_pos]}")
        self.get_logger().info(f"Initial right positions: {[f'{p:.2f}' for p in right_pos]}")
        self.get_logger().info("🔒 Arms will hold current positions until Unity takes over")

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
        
        # === 特殊指令處理 ===
        if msg.name and msg.name[0] in ('HOME', 'L_HOME', 'R_HOME', 'L_ENABLE', 'R_ENABLE', 'ENABLE'):
            self._handle_special_command(msg.name[0])
            return
        
        with self.target_lock:
            for i, name in enumerate(msg.name):
                if name.startswith('L_J'):
                    if not self.left_tracking_unity:
                        # 首次收到左臂指令：用當前實際位置初始化所有目標，避免跳動
                        self.left_tracking_unity = True
                        try:
                            for j, m in enumerate(self.left_arm.get_arm().get_motors()):
                                actual = m.get_position()
                                self.left_target[j] = actual
                                self.left_smoothed[j] = actual
                        except:
                            pass
                        self.get_logger().info("✅ Left arm: switched to Unity tracking")
                    joint_idx = int(name.split('_J')[1]) - 1
                    if 0 <= joint_idx < 7:
                        pos = self._clamp_position(msg.position[i], joint_idx, LEFT_POSITION_LIMITS)
                        self.left_target[joint_idx] = pos
                elif name.startswith('R_J'):
                    if not self.right_tracking_unity:
                        # 首次收到右臂指令：用當前實際位置初始化所有目標，避免跳動
                        self.right_tracking_unity = True
                        try:
                            for j, m in enumerate(self.right_arm.get_arm().get_motors()):
                                actual = m.get_position()
                                self.right_target[j] = actual
                                self.right_smoothed[j] = actual
                        except:
                            pass
                        self.get_logger().info("✅ Right arm: switched to Unity tracking")
                    joint_idx = int(name.split('_J')[1]) - 1
                    if 0 <= joint_idx < 7:
                        pos = self._clamp_position(msg.position[i], joint_idx, RIGHT_POSITION_LIMITS)
                        self.right_target[joint_idx] = pos
                elif name == 'L_EE':
                    self.left_gripper_target = self._gripper_to_motor(msg.position[i])
                    # [TEMP] 夾爪連動靈巧手：夾爪關=手握，夾爪開=手開
                    if SYNC_GRIPPER_TO_EHAND:
                        self._sync_gripper_to_ehand("left", msg.position[i])
                elif name == 'R_EE':
                    self.right_gripper_target = self._gripper_to_motor(msg.position[i])
                    # [TEMP] 夾爪連動靈巧手：夾爪關=手握，夾爪開=手開
                    if SYNC_GRIPPER_TO_EHAND:
                        self._sync_gripper_to_ehand("right", msg.position[i])
    
    def _handle_special_command(self, cmd: str):
        """處理特殊指令：HOME (回零+禁用) / ENABLE (重新啟用)"""
        if cmd in ('HOME', 'L_HOME'):
            self.get_logger().info("Left arm: homing...")
            self.left_homing = True
            with self.target_lock:
                self.left_target = [0.0] * 7
                self.left_gripper_target = 0.0
        
        if cmd in ('HOME', 'R_HOME'):
            self.get_logger().info("Right arm: homing...")
            self.right_homing = True
            with self.target_lock:
                self.right_target = [0.0] * 7
                self.right_gripper_target = 0.0
        
        if cmd in ('ENABLE', 'L_ENABLE'):
            if self.left_disabled:
                self.get_logger().info("Left arm: re-enabling...")
                self.left_arm.enable_all()
                time.sleep(0.05)
                self.left_arm.recv_all()
                self.left_disabled = False
                self.left_homing = False
                self.get_logger().info("Left arm re-enabled")
        
        if cmd in ('ENABLE', 'R_ENABLE'):
            if self.right_disabled:
                self.get_logger().info("Right arm: re-enabling...")
                self.right_arm.enable_all()
                time.sleep(0.05)
                self.right_arm.recv_all()
                self.right_disabled = False
                self.right_homing = False
                self.get_logger().info("Right arm re-enabled")
    
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
        
        # === 靈巧手特殊指令處理 ===
        if msg.name and msg.name[0] in ('L_HAND_HOME', 'R_HAND_HOME', 'HAND_HOME',
                                         'L_HAND_OPEN', 'R_HAND_OPEN', 'HAND_OPEN',
                                         'L_HAND_CLOSE', 'R_HAND_CLOSE', 'HAND_CLOSE',
                                         'L_HAND_DISABLE', 'R_HAND_DISABLE', 'HAND_DISABLE',
                                         'L_HAND_ENABLE', 'R_HAND_ENABLE', 'HAND_ENABLE',
                                         'L_HAND_STATUS', 'R_HAND_STATUS', 'HAND_STATUS'):
            self._handle_ehand_special_command(msg.name[0])
            return
        
        # 解析新的手指目標
        new_left = list(self.left_hand_target)
        new_right = list(self.right_hand_target)
        has_left = False
        has_right = False

        for i, name in enumerate(msg.name):
            if name.startswith('L_F'):
                finger_idx = int(name.split('_F')[1]) - 1
                if 0 <= finger_idx < 6:
                    new_left[finger_idx] = max(0.0, min(1.0, msg.position[i]))
                    has_left = True
            elif name.startswith('R_F'):
                finger_idx = int(name.split('_F')[1]) - 1
                if 0 <= finger_idx < 6:
                    new_right[finger_idx] = max(0.0, min(1.0, msg.position[i]))
                    has_right = True

        # 檢查握合狀態變化，執行 disable→home→等待→enable 流程
        if has_left:
            avg_left = sum(new_left) / 6.0
            want_grip = avg_left >= self._hand_transition_threshold
            if want_grip != self.left_hand_gripping:
                self._ehand_transition("left", want_grip)
                self.left_hand_gripping = want_grip

        if has_right:
            avg_right = sum(new_right) / 6.0
            want_grip = avg_right >= self._hand_transition_threshold
            if want_grip != self.right_hand_gripping:
                self._ehand_transition("right", want_grip)
                self.right_hand_gripping = want_grip

        # 更新目標
        with self.hand_target_lock:
            if has_left:
                self.left_hand_target = new_left
            if has_right:
                self.right_hand_target = new_right

        # Debug: 每 30 次輸出一次
        if not hasattr(self, '_ehand_log_count'):
            self._ehand_log_count = 0
        self._ehand_log_count += 1
        if self._ehand_log_count % 30 == 0:
            l_str = str([round(x, 2) for x in self.left_hand_target])
            r_str = str([round(x, 2) for x in self.right_hand_target])
            self.get_logger().info(f"[ehand] L={l_str}, R={r_str}")
    
    def _sync_gripper_to_ehand(self, side: str, gripper_meters: float):
        """[TEMP] 夾爪連動靈巧手：夾爪開=手開(0.0)，夾爪關=手握(1.0)

        gripper_meters: Unity 夾爪值 (0.0=關閉, 0.0425=打開)
        """
        # 夾爪值 < threshold → 關閉 → 手握(1.0)
        # 夾爪值 >= threshold → 打開 → 手開(0.0)
        grip_value = 0.0 if gripper_meters >= SYNC_GRIPPER_THRESHOLD else 1.0

        if side == "left":
            old_avg = sum(self.left_hand_target) / 6.0
            new_target = [grip_value] * 6
            want_grip = grip_value >= self._hand_transition_threshold
            if want_grip != self.left_hand_gripping:
                self._ehand_transition("left", want_grip)
                self.left_hand_gripping = want_grip
            with self.hand_target_lock:
                self.left_hand_target = new_target
        else:
            old_avg = sum(self.right_hand_target) / 6.0
            new_target = [grip_value] * 6
            want_grip = grip_value >= self._hand_transition_threshold
            if want_grip != self.right_hand_gripping:
                self._ehand_transition("right", want_grip)
                self.right_hand_gripping = want_grip
            with self.hand_target_lock:
                self.right_hand_target = new_target

    def _ehand_transition(self, side: str, to_grip: bool):
        """靈巧手狀態轉換：disable → home → 等待 → enable

        eHand 硬體特性：直接切換握/開會不穩定，
        需要先 disable + home 重置後再設定新目標。
        """
        if side == "left":
            hand = self.left_hand
            can_id = DEXTEROUS_HAND_LEFT_CAN_ID
            label = "Left"
        else:
            hand = self.right_hand
            can_id = DEXTEROUS_HAND_RIGHT_CAN_ID
            label = "Right"

        if not hand or not hand.is_ready() or not self.zlgcan:
            return

        action = "GRIP" if to_grip else "OPEN"
        self.get_logger().info(f"[ehand] {label} hand transition → {action}: disable→home→wait→enable")

        CMD_DISABLE = bytes([0xFD, 0x00] + [0x00] * 30)
        CMD_HOME = bytes([0xFD, 0x04] + [0x00] * 30)

        # 1. DISABLE
        self.zlgcan.transmit_fd(can_id, CMD_DISABLE)
        if side == "left":
            self.left_hand_disabled = True
        else:
            self.right_hand_disabled = True
        time.sleep(0.1)

        # 2. HOME
        self.zlgcan.transmit_fd(can_id, CMD_HOME)
        time.sleep(0.5)

        # 3. ENABLE（清除 disabled flag，讓控制迴圈重新發送位置指令）
        if side == "left":
            self.left_hand_disabled = False
        else:
            self.right_hand_disabled = False

        self.get_logger().info(f"[ehand] {label} hand transition done, ready for {action}")

    def _handle_ehand_special_command(self, cmd: str):
        """處理靈巧手特殊指令"""
        # 判斷左/右/雙手
        is_left = cmd.startswith('L_') or not cmd.startswith('R_')
        is_right = cmd.startswith('R_') or not cmd.startswith('L_')
        action = cmd.split('HAND_')[1]  # HOME / OPEN / CLOSE / DISABLE / ENABLE / STATUS
        
        # === STATUS: 讀取靈巧手狀態 ===
        if action == 'STATUS':
            for label, hand, do_it in [
                ("Left", self.left_hand, is_left),
                ("Right", self.right_hand, is_right)
            ]:
                if not do_it:
                    continue
                if hand and hand.is_ready():
                    status = hand.read_status()
                    if status:
                        fingers_str = ", ".join(
                            f"{f['name']}={f['position']}({f['position_percent']:.0f}%)" 
                            for f in status['fingers']
                        )
                        self.get_logger().info(
                            f"{label} hand: {status['hand_state']} | {fingers_str}"
                        )
                    else:
                        self.get_logger().warn(f"{label} hand: no response")
                else:
                    self.get_logger().warn(f"{label} hand: not available")
            # 發布到 /openarm/hand_states topic
            self._publish_hand_status(is_left, is_right)
            return
        
        # 構建 CAN FD 命令 (32 bytes)
        CMD_MAP = {
            'HOME':    bytes([0xFD, 0x04] + [0x00] * 30),
            'OPEN':    bytes([0xFD, 0x02] + [0x00] * 30),
            'CLOSE':   bytes([0xFD, 0x03] + [0x00] * 30),
            'DISABLE': bytes([0xFD, 0x00] + [0x00] * 30),
        }
        
        if action == 'ENABLE':
            # 重新啟用：發送回零命令恢復
            if is_left and self.left_hand_disabled:
                if self.left_hand and self.left_hand.is_ready():
                    self.left_hand.send_home()
                    self.left_hand_disabled = False
                    self.get_logger().info("Left hand: re-enabled (home)")
            if is_right and self.right_hand_disabled:
                if self.right_hand and self.right_hand.is_ready():
                    self.right_hand.send_home()
                    self.right_hand_disabled = False
                    self.get_logger().info("Right hand: re-enabled (home)")
            return
        
        if action == "STATUS":
            # Read hand status (0xFC command)
            import time
            FINGER_NAMES = ["M1", "M2", "M3", "M4", "M5", "M6"]
            cmd_read = bytes([0xFC] + [0x00] * 31)
            
            if is_right and self.zlgcan:
                from usbcanfd_scan import TYPE_CANFD
                # Clear receive buffer first
                old_num = self.zlgcan.get_receive_num(TYPE_CANFD)
                if old_num > 0:
                    self.zlgcan.receive_fd(old_num, 100)
                
                time.sleep(0.1)
                self.zlgcan.transmit_fd(DEXTEROUS_HAND_RIGHT_CAN_ID, cmd_read)
                time.sleep(0.5)
                
                num = self.zlgcan.get_receive_num(TYPE_CANFD)
                if num > 0:
                    msgs = self.zlgcan.receive_fd(num, 100)
                    for msg in msgs:
                        data = bytes([msg.frame.data[i] for i in range(32)])
                        if (data[0] & 0x03) == 2:
                            positions = []
                            for i in range(6):
                                pos = data[2 + i*5 + 1]
                                pct = pos / 255 * 100
                                positions.append(f"{FINGER_NAMES[i]}={pos}({pct:.0f}%)")
                            self.get_logger().info("右手: " + ", ".join(positions))
                            break
                    else:
                        self.get_logger().warn("Right hand STATUS: no read response found")
                else:
                    self.get_logger().warn("Right hand STATUS: no response")
            
            if is_left and self.zlgcan:
                from usbcanfd_scan import TYPE_CANFD
                old_num = self.zlgcan.get_receive_num(TYPE_CANFD)
                if old_num > 0:
                    self.zlgcan.receive_fd(old_num, 100)
                
                time.sleep(0.1)
                self.zlgcan.transmit_fd(DEXTEROUS_HAND_LEFT_CAN_ID, cmd_read)
                time.sleep(0.5)
                
                num = self.zlgcan.get_receive_num(TYPE_CANFD)
                if num > 0:
                    msgs = self.zlgcan.receive_fd(num, 100)
                    for msg in msgs:
                        data = bytes([msg.frame.data[i] for i in range(32)])
                        if (data[0] & 0x03) == 2:
                            positions = []
                            for i in range(6):
                                pos = data[2 + i*5 + 1]
                                pct = pos / 255 * 100
                                positions.append(f"{FINGER_NAMES[i]}={pos}({pct:.0f}%)")
                            self.get_logger().info("左手: " + ", ".join(positions))
                            break
                    else:
                        self.get_logger().warn("Left hand STATUS: no read response found")
                else:
                    self.get_logger().warn("Left hand STATUS: no response")
            return
        
        can_cmd = CMD_MAP.get(action)
        if can_cmd is None:
            return
        
        if is_left:
            if self.left_hand and self.left_hand.is_ready():
                self.zlgcan.transmit_fd(DEXTEROUS_HAND_LEFT_CAN_ID, can_cmd)
                if action == 'DISABLE':
                    self.left_hand_disabled = True
                elif action == 'HOME':
                    self.left_hand_disabled = True  # 回零後也 disable 控制迴圈
                self.get_logger().info(f"Left hand: {action}")
            else:
                self.get_logger().warn("Left hand not available")
        
        if is_right:
            if self.right_hand and self.right_hand.is_ready():
                self.zlgcan.transmit_fd(DEXTEROUS_HAND_RIGHT_CAN_ID, can_cmd)
                if action == 'DISABLE':
                    self.right_hand_disabled = True
                elif action == 'HOME':
                    self.right_hand_disabled = True
                self.get_logger().info(f"Right hand: {action}")
            else:
                self.get_logger().warn("Right hand not available")
    
    def _publish_hand_status(self, is_left: bool, is_right: bool):
        """讀取靈巧手狀態並發布到 /openarm/hand_states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for label, hand, do_it, prefix in [
            ("left", self.left_hand, is_left, "L_F"),
            ("right", self.right_hand, is_right, "R_F"),
        ]:
            if not do_it:
                continue
            if hand and hand.is_ready():
                status = hand.read_status()
                if status:
                    for f in status['fingers']:
                        msg.name.append(f"{prefix}{status['fingers'].index(f)+1}")
                        msg.position.append(f['position'] / 255.0)  # 0~1 normalized
                        msg.velocity.append(float(f['speed']))
                        msg.effort.append(f['position_percent'])  # 0~100%
        
        if msg.name:
            self.hand_state_pub.publish(msg)
    
    def _clamp_position(self, pos: float, joint_idx: int, limits: dict) -> float:
        """限制位置在安全範圍內"""
        if joint_idx in limits:
            lower = limits[joint_idx]["lower"]
            upper = limits[joint_idx]["upper"]
            return max(lower, min(pos, upper))
        return pos
    
    def _get_dynamic_kp_kd(self, joint_idx: int, position_error: float) -> tuple:
        """
        根據誤差大小動態調整 Kp/Kd
        
        策略：
        - 大誤差時：高 Kp 快速追蹤，低 Kd 減少阻力
        - 小誤差時：低 Kp 避免震盪，高 Kd 增加穩定性
        - 正常時：使用基礎值
        
        Args:
            joint_idx: 關節索引 (0-6)
            position_error: 位置誤差 (target - actual)
        
        Returns:
            tuple: (kp, kd)
        """
        # 只對指定關節使用動態調整，其他關節使用靜態值
        if joint_idx not in DYNAMIC_KP_KD_JOINTS:
            return BASE_KP[joint_idx], BASE_KD[joint_idx]
        
        abs_error = abs(position_error)
        
        if abs_error > LARGE_ERROR_THRESHOLD:
            # 大誤差：高剛度快速追蹤
            kp = BASE_KP[joint_idx] * LARGE_ERROR_KP_SCALE
            kd = BASE_KD[joint_idx] * LARGE_ERROR_KD_SCALE
        elif abs_error < SMALL_ERROR_THRESHOLD:
            # 小誤差：低剛度避免震盪
            kp = BASE_KP[joint_idx] * SMALL_ERROR_KP_SCALE
            kd = BASE_KD[joint_idx] * SMALL_ERROR_KD_SCALE
        else:
            # 正常範圍：使用基礎值
            kp = BASE_KP[joint_idx]
            kd = BASE_KD[joint_idx]
        
        # 限制在安全範圍內
        kp = min(kp, 500.0)
        kd = min(kd, 5.0)
        
        return kp, kd
    
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

    def _control_one_arm(self, side, arm, arm_cmds, grip_cmds, hand, hand_disabled,
                         hand_fingers, homing_flag, loop_count, hand_control_interval):
        """單臂 CAN 控制（送指令 + 收回應），設計為可被 ThreadPoolExecutor 並行呼叫。

        Returns:
            str or None: "home_reached" 如果回零完成，否則 None
        """
        arm.get_arm().mit_control_all(arm_cmds)

        if ENABLE_GRIPPER:
            arm.get_gripper().mit_control_all(grip_cmds)

        # 靈巧手（降頻控制）
        if hand and hand.is_ready() and not hand_disabled and loop_count % hand_control_interval == 0:
            hand.send_positions(hand_fingers)

        arm.recv_all(500)

        # 回零檢查
        if homing_flag:
            motors = arm.get_arm().get_motors()
            if all(abs(m.get_position()) < 0.05 for m in motors):
                arm.disable_all()
                return "home_reached"

        return None

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

            # [TEMP DEMO] 鎖定關節
            for (side, idx), lock_val in LOCK_JOINTS.items():
                if side == "left" and 0 <= idx < 7:
                    left_target[idx] = lock_val
                    self.left_smoothed[idx] = lock_val
                elif side == "right" and 0 <= idx < 7:
                    right_target[idx] = lock_val
                    self.right_smoothed[idx] = lock_val

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
            # [2026-02-05] 測試關閉重力補償，觀察是否消除「山峰」現象
            # left_comp = self._calculate_gravity_compensation(self.left_arm, left_pos, "left")
            # right_comp = self._calculate_gravity_compensation(self.right_arm, right_pos, "right")
            left_comp = [0.0] * 7
            right_comp = [0.0] * 7
            
            # 建立 MIT 命令（包含速度前饋和重力補償力矩）
            if USE_DYNAMIC_KP_KD:
                # 動態 Kp/Kd：根據誤差大小調整
                left_motors = self.left_arm.get_arm().get_motors()
                right_motors = self.right_arm.get_arm().get_motors()
                left_actual = [m.get_position() for m in left_motors]
                right_actual = [m.get_position() for m in right_motors]
                
                left_arm_cmds = []
                right_arm_cmds = []
                for i in range(7):
                    left_error = left_pos[i] - left_actual[i]
                    right_error = right_pos[i] - right_actual[i]
                    left_kp, left_kd = self._get_dynamic_kp_kd(i, left_error)
                    right_kp, right_kd = self._get_dynamic_kp_kd(i, right_error)
                    left_arm_cmds.append(oa.MITParam(left_kp, left_kd, left_pos[i], self.left_velocity_ff[i], left_comp[i]))
                    right_arm_cmds.append(oa.MITParam(right_kp, right_kd, right_pos[i], self.right_velocity_ff[i], right_comp[i]))
            else:
                # 靜態 Kp/Kd
                left_arm_cmds = [
                    oa.MITParam(KP[i], KD[i], left_pos[i], self.left_velocity_ff[i], left_comp[i]) 
                    for i in range(7)
                ]
                right_arm_cmds = [
                    oa.MITParam(KP[i], KD[i], right_pos[i], self.right_velocity_ff[i], right_comp[i]) 
                    for i in range(7)
                ]
            
            left_grip_cmds = [oa.MITParam(GRIPPER_KP, GRIPPER_KD, left_grip, 0.0, 0.0)]
            right_grip_cmds = [oa.MITParam(GRIPPER_KP, GRIPPER_KD, right_grip, 0.0, 0.0)]

            # 準備靈巧手手指目標（在主執行緒取得 lock，避免子執行緒搶 lock）
            with self.hand_target_lock:
                left_fingers = self.left_hand_target[:]
                right_fingers = self.right_hand_target[:]

            # 發送到馬達（左右臂並行）
            # [2026-04-07] 左右臂用不同 CAN socket，ThreadPoolExecutor 並行送收
            try:
                left_future = None
                right_future = None

                if not self.left_disabled and self.left_tracking_unity:
                    left_future = self.arm_executor.submit(
                        self._control_one_arm,
                        "left", self.left_arm, left_arm_cmds, left_grip_cmds,
                        self.left_hand, self.left_hand_disabled,
                        left_fingers, self.left_homing,
                        loop_count, hand_control_interval
                    )

                if not self.right_disabled and self.right_tracking_unity:
                    right_future = self.arm_executor.submit(
                        self._control_one_arm,
                        "right", self.right_arm, right_arm_cmds, right_grip_cmds,
                        self.right_hand, self.right_hand_disabled,
                        right_fingers, self.right_homing,
                        loop_count, hand_control_interval
                    )

                # 等待兩臂完成
                if left_future is not None:
                    left_result = left_future.result()
                    if left_result == "home_reached":
                        self.left_disabled = True
                        self.left_homing = False
                        self.get_logger().info("Left arm: home reached, disabled")

                if right_future is not None:
                    right_result = right_future.result()
                    if right_result == "home_reached":
                        self.right_disabled = True
                        self.right_homing = False
                        self.get_logger().info("Right arm: home reached, disabled")
                
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
                            from utils.plot_joint_comparison import plot_joint_comparison, plot_all_joints_combined
                            self.get_logger().info("🎨 正在繪製比較圖...")
                            output_files = plot_joint_comparison(data_file, arm='both')
                            # 繪製組合圖（所有關節在同一張圖）
                            left_combined = plot_all_joints_combined(data_file, arm='left')
                            right_combined = plot_all_joints_combined(data_file, arm='right')
                            output_files.extend([left_combined, right_combined])
                            self.get_logger().info(f"✅ 已產生 {len(output_files)} 張圖（含組合圖）")
                        except Exception as e:
                            self.get_logger().warn(f"⚠️ 繪圖失敗: {e}")
                else:
                    self.get_logger().info("📊 無數據需要儲存")
            except Exception as e:
                self.get_logger().warn(f"⚠️ JointDataLogger 儲存失敗: {e}")
        # ===== [JOINT_LOGGER] 儲存數據並繪圖 - 結束 =====
        
        # 關閉 ThreadPoolExecutor
        if hasattr(self, 'arm_executor'):
            self.arm_executor.shutdown(wait=False)

        # 關閉靈巧手（並存模式：張開兩隻手）
        if self.zlgcan is not None:
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
