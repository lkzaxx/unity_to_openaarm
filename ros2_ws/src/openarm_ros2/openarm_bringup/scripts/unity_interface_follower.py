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

# 夾爪配置
GRIPPER_MOTOR_TYPE = oa.MotorType.DM4310
GRIPPER_SEND_ID = 0x08
GRIPPER_RECV_ID = 0x18

# ============================================================================
# MIT 控制參數
# ============================================================================

# Kp, Kd 參數（沿用 ros2_control 配置）
# KP = [70.0, 70.0, 20.0, 20.0,  5.0,  5.0,  5.0]
KP = [20.0, 20.0, 10.0, 10.0, 5.0, 5.0, 5.0]
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
MAX_POSITION_RATE = [1.0, 1.0, 1.5, 1.5, 2.0, 2.0, 2.0]  # 各關節最大速度 (rad/s)

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
        self.last_unity_time = 0.0
        self.unity_connected = False
        
        # === 初始化 OpenArm CAN ===
        self.get_logger().info(f"Initializing Left Arm on {LEFT_CAN_INTERFACE}...")
        self.left_arm = oa.OpenArm(LEFT_CAN_INTERFACE, True)
        self.left_arm.init_arm_motors(MOTOR_TYPES, SEND_IDS, RECV_IDS)
        self.left_arm.init_gripper_motor(GRIPPER_MOTOR_TYPE, GRIPPER_SEND_ID, GRIPPER_RECV_ID)
        
        self.get_logger().info(f"Initializing Right Arm on {RIGHT_CAN_INTERFACE}...")
        self.right_arm = oa.OpenArm(RIGHT_CAN_INTERFACE, True)
        self.right_arm.init_arm_motors(MOTOR_TYPES, SEND_IDS, RECV_IDS)
        self.right_arm.init_gripper_motor(GRIPPER_MOTOR_TYPE, GRIPPER_SEND_ID, GRIPPER_RECV_ID)
        
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
        
        # === 發布 JointState 給 Unity ===
        self.joint_state_pub = self.create_publisher(
            JointState, '/openarm/joint_states', 10
        )
        
        # === 控制迴圈（獨立執行緒） ===
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        
        # === 狀態發布 Timer (50Hz) ===
        self.state_timer = self.create_timer(0.02, self.publish_joint_states)
        
        self.get_logger().info("✅ Unity Follower Interface started!")
        self.get_logger().info(f"   Control frequency: {CONTROL_FREQUENCY} Hz")
        self.get_logger().info(f"   Left arm: {LEFT_CAN_INTERFACE}, Right arm: {RIGHT_CAN_INTERFACE}")
    
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
    
    def unity_callback(self, msg: JointState):
        """Unity 傳來目標時，只更新 target（不阻塞）"""
        self.last_unity_time = time.time()
        
        if not self.unity_connected:
            self.unity_connected = True
            self.get_logger().info("✅ Unity connected!")
        
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
        
        # 🐛 調試：顯示非零補償和誤差
        non_zero = [(i, round(c, 2)) for i, c in enumerate(compensations) if abs(c) > 0.01]
        if non_zero:
            errors = [(i, round(target_pos[i] - current_pos[i], 3)) for i in [0, 1, 3]]
            print(f"[{side}] Errors: {errors}, Comp: {non_zero}")
        
        return compensations
    
    def control_loop(self):
        """500Hz MIT 控制迴圈"""
        self.get_logger().info("Control loop started")
        
        while self.running:
            loop_start = time.time()
            
            # 檢查 Unity 連線狀態
            if self.unity_connected and (time.time() - self.last_unity_time) > UNITY_TIMEOUT:
                self.unity_connected = False
                self.get_logger().warn("⚠️ Unity connection timeout!")
            
            # 取得最新目標（執行緒安全）
            with self.target_lock:
                left_target = self.left_target.copy()
                right_target = self.right_target.copy()
                left_grip = self.left_gripper_target
                right_grip = self.right_gripper_target
            
            # === 目標平滑：限制每個控制週期的目標變化量 ===
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
            
            # 使用平滑後的目標
            left_pos = self.left_smoothed.copy()
            right_pos = self.right_smoothed.copy()
            
            # 計算重力補償力矩（簡化版本，模仿 C++ 邏輯）
            left_comp = self._calculate_gravity_compensation(self.left_arm, left_pos, "left")
            right_comp = self._calculate_gravity_compensation(self.right_arm, right_pos, "right")
            
            # 建立 MIT 命令（包含重力補償力矩）
            left_arm_cmds = [
                oa.MITParam(KP[i], KD[i], left_pos[i], 0.0, left_comp[i]) 
                for i in range(7)
            ]
            right_arm_cmds = [
                oa.MITParam(KP[i], KD[i], right_pos[i], 0.0, right_comp[i]) 
                for i in range(7)
            ]
            
            left_grip_cmds = [oa.MITParam(GRIPPER_KP, GRIPPER_KD, left_grip, 0.0, 0.0)]
            right_grip_cmds = [oa.MITParam(GRIPPER_KP, GRIPPER_KD, right_grip, 0.0, 0.0)]
            
            # 發送到馬達
            try:
                self.left_arm.get_arm().mit_control_all(left_arm_cmds)
                self.left_arm.get_gripper().mit_control_all(left_grip_cmds)
                self.left_arm.recv_all(500)
                
                self.right_arm.get_arm().mit_control_all(right_arm_cmds)
                self.right_arm.get_gripper().mit_control_all(right_grip_cmds)
                self.right_arm.recv_all(500)
            except Exception as e:
                self.get_logger().error(f"Control error: {e}")
            
            # 維持控制頻率
            elapsed = time.time() - loop_start
            sleep_time = CONTROL_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def publish_joint_states(self):
        """發布當前關節狀態給 Unity (50Hz)"""
        try:
            self.left_arm.refresh_all()
            self.right_arm.refresh_all()
            self.left_arm.recv_all(200)
            self.right_arm.recv_all(200)
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            
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
