#!/usr/bin/env python3
"""
waypoint_collector.py — OpenArm 腳本式資料收集器

兩種模式：
  calibrate: 互動式標定 waypoints，存入 YAML
  collect:   依照 waypoints 自動執行 pick-and-place，錄製訓練資料

用法：
  # 標定模式：逐步調整手臂姿態並儲存 waypoints
  python3 waypoint_collector.py calibrate -a R -o waypoints/pick_and_place.yaml

  # 收集模式：自動跑 100 個 episodes
  python3 waypoint_collector.py collect \
    -w waypoints/pick_and_place.yaml \
    -n 100 \
    -o ~/datasets/pick_and_place/raw \
    --randomize

前提：
  Terminal 1 已執行 ./start_follower.sh
"""

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from threading import Thread, Lock

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState, CompressedImage

# ── 關節限制（來自 openarm_env.py）──────────────────────────
JOINT_LOWER = np.array(
    [-1.396263, -1.745329, -1.570796, 0.0, -1.570796, -0.785398, -1.570796],
    dtype=np.float32,
)
JOINT_UPPER = np.array(
    [3.490659, 1.745329, 1.570796, 2.443461, 1.570796, 0.785398, 1.570796],
    dtype=np.float32,
)
HOME_JOINTS = [0.0, 0.0, 0.0, 1.22, 0.0, 0.0, 0.0]


# ════════════════════════════════════════════════════════════
# ROS2 Robot Interface
# ════════════════════════════════════════════════════════════

class RobotInterface(Node):
    """ROS2 interface：發送關節指令、接收狀態與影像。"""

    def __init__(self, arm: str = "R"):
        super().__init__("waypoint_collector")
        self.arm = arm.upper()
        self._lock = Lock()

        # 最新狀態
        self._joint_positions = np.zeros(7, dtype=np.float32)
        self._gripper_state = 0.0
        self._color_frame = None
        self._depth_frame = None
        self._has_joint_state = False

        # Publisher: joint commands (VOLATILE QoS 匹配 follower)
        volatile_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.joint_pub = self.create_publisher(
            JointState, "/unity/joint_commands", volatile_qos
        )

        # Publisher: eHand commands
        self.hand_pub = self.create_publisher(
            JointState, "/unity/ehand_commands", 10
        )

        # Subscriber: joint states (50 Hz)
        self.create_subscription(
            JointState, "/openarm/joint_states", self._joint_state_cb, 10
        )

        # Subscriber: camera color (CompressedImage)
        self.create_subscription(
            CompressedImage, "/camera/color/compressed", self._color_cb, 10
        )

        # Subscriber: camera depth (CompressedImage)
        self.create_subscription(
            CompressedImage, "/camera/depth/compressed", self._depth_cb, 10
        )

        # 背景 spin
        self._spin_thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
        self._spin_thread.start()

        self.get_logger().info(f"RobotInterface ready (arm={self.arm})")

    # ── Callbacks ──

    def _joint_state_cb(self, msg: JointState):
        prefix = f"openarm_{'right' if self.arm == 'R' else 'left'}_joint"
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name.startswith(prefix):
                    idx = int(name[-1]) - 1  # joint1 → 0
                    if 0 <= idx < 7:
                        self._joint_positions[idx] = pos
                        self._has_joint_state = True

    def _color_cb(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is not None:
            with self._lock:
                self._color_frame = frame

    def _depth_cb(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is not None:
            with self._lock:
                self._depth_frame = frame

    # ── Getters ──

    def get_joint_positions(self) -> np.ndarray:
        with self._lock:
            return self._joint_positions.copy()

    def get_color_frame(self) -> np.ndarray:
        with self._lock:
            return self._color_frame.copy() if self._color_frame is not None else None

    def get_depth_frame(self) -> np.ndarray:
        with self._lock:
            return self._depth_frame.copy() if self._depth_frame is not None else None

    def has_joint_state(self) -> bool:
        with self._lock:
            return self._has_joint_state

    # ── Commands ──

    def send_joints(self, positions: list):
        """發送 7 個關節角度。"""
        msg = JointState()
        msg.name = [f"{self.arm}_J{i+1}" for i in range(7)]
        msg.position = [float(p) for p in positions]
        self.joint_pub.publish(msg)

    def send_gripper(self, value: float):
        """控制夾爪。value: 0.0=全閉, 1.0=全開。"""
        msg = JointState()
        msg.name = [f"{self.arm}_F{i}" for i in range(1, 7)]
        msg.position = [float(value)] * 6
        self.hand_pub.publish(msg)

    def send_gripper_command(self, command: str):
        """發送夾爪特殊指令：open / close / enable / disable。"""
        msg = JointState()
        msg.name = [f"{self.arm}_HAND_{command.upper()}"]
        msg.position = [0.0]
        self.hand_pub.publish(msg)

    def send_special(self, command: str):
        """發送特殊指令：HOME / ENABLE 等。"""
        msg = JointState()
        msg.name = [command.upper()]
        msg.position = [0.0]
        self.joint_pub.publish(msg)

    def wait_for_state(self, timeout: float = 5.0) -> bool:
        """等待至少收到一次 joint state。"""
        t0 = time.time()
        while not self.has_joint_state():
            if time.time() - t0 > timeout:
                return False
            time.sleep(0.05)
        return True

    def shutdown(self):
        self.destroy_node()


# ════════════════════════════════════════════════════════════
# Calibrate Mode
# ════════════════════════════════════════════════════════════

def run_calibrate(args):
    """互動式 waypoint 標定。"""
    rclpy.init()
    robot = RobotInterface(arm=args.arm)

    # 等待連線
    print("等待 joint_states...")
    if not robot.wait_for_state(timeout=10.0):
        print("ERROR: 未收到 joint_states。確認 start_follower.sh 已執行。")
        robot.shutdown()
        rclpy.shutdown()
        return

    print(f"已連線 ({args.arm} arm)")
    print()

    waypoints = []
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # 如果檔案已存在，載入
    if output_path.exists():
        with open(output_path) as f:
            existing = yaml.safe_load(f) or {}
        waypoints = existing.get("waypoints", [])
        print(f"載入 {len(waypoints)} 個已有 waypoints")

    def print_help():
        print()
        print("指令：")
        print("  read            讀取目前關節角度")
        print("  move J1 J2...J7 移動到指定角度 (rad)")
        print("  save NAME DUR   儲存目前姿態為 waypoint (DUR=到達時間秒)")
        print("  gripper VALUE   控制夾爪 (0=閉, 1=開)")
        print("  list            列出已存 waypoints")
        print("  delete IDX      刪除第 IDX 個 waypoint")
        print("  done            儲存並退出")
        print("  help            顯示此說明")
        print()

    print_help()

    try:
        while True:
            try:
                line = input(f"[{args.arm}] >> ").strip()
            except EOFError:
                break
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            if cmd == "help":
                print_help()

            elif cmd == "read":
                pos = robot.get_joint_positions()
                angles = " ".join(f"{p:.4f}" for p in pos)
                degs = " ".join(f"{math.degrees(p):.1f}" for p in pos)
                print(f"  rad: [{angles}]")
                print(f"  deg: [{degs}]")

            elif cmd == "move" and len(parts) == 8:
                try:
                    joints = [float(x) for x in parts[1:8]]
                except ValueError:
                    print("  ERROR: 角度需為數字")
                    continue
                # 安全檢查
                joints_np = np.array(joints, dtype=np.float32)
                clipped = np.clip(joints_np, JOINT_LOWER, JOINT_UPPER)
                if not np.allclose(joints_np, clipped, atol=1e-4):
                    print(f"  WARNING: 角度被裁剪到安全範圍")
                    joints = clipped.tolist()
                robot.send_joints(joints)
                print(f"  → 已發送: {[round(j, 4) for j in joints]}")

            elif cmd == "save" and len(parts) >= 2:
                name = parts[1]
                duration = float(parts[2]) if len(parts) >= 3 else 2.0
                pos = robot.get_joint_positions().tolist()
                wp = {
                    "name": name,
                    "joints": [round(p, 5) for p in pos],
                    "duration": duration,
                }
                waypoints.append(wp)
                print(f"  ✓ 已存 waypoint [{len(waypoints)-1}] '{name}': "
                      f"{[round(p, 4) for p in pos]}, duration={duration}s")

            elif cmd == "gripper":
                if len(parts) < 2:
                    print("  用法: gripper open / close / 0.5")
                    continue
                val = parts[1].lower()
                if val == "open":
                    robot.send_gripper_command("OPEN")
                    print("  → gripper OPEN")
                elif val == "close":
                    robot.send_gripper_command("CLOSE")
                    print("  → gripper CLOSE")
                else:
                    try:
                        robot.send_gripper_command("ENABLE")
                        time.sleep(0.3)
                        robot.send_gripper(float(val))
                        print(f"  → gripper {val}")
                    except ValueError:
                        print("  ERROR: 無效的 gripper 值")

            elif cmd == "list":
                if not waypoints:
                    print("  (無 waypoints)")
                for i, wp in enumerate(waypoints):
                    print(f"  [{i}] {wp['name']}: {wp['joints']} "
                          f"dur={wp.get('duration', 2.0)}s "
                          f"gripper={wp.get('gripper', '-')}")

            elif cmd == "delete" and len(parts) == 2:
                try:
                    idx = int(parts[1])
                    removed = waypoints.pop(idx)
                    print(f"  ✓ 已刪除 [{idx}] '{removed['name']}'")
                except (ValueError, IndexError):
                    print("  ERROR: 無效的索引")

            elif cmd == "done":
                break

            else:
                print(f"  未知指令: {line}")
                print_help()

    except KeyboardInterrupt:
        print()

    # 儲存
    if waypoints:
        data = {
            "arm": args.arm,
            "task": args.task,
            "record_fps": 30,
            "waypoints": waypoints,
        }
        with open(output_path, "w") as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True,
                      sort_keys=False)
        print(f"\n已儲存 {len(waypoints)} 個 waypoints → {output_path}")
    else:
        print("\n無 waypoints 需要儲存")

    robot.shutdown()
    rclpy.shutdown()


# ════════════════════════════════════════════════════════════
# Collect Mode
# ════════════════════════════════════════════════════════════

def load_waypoints(path: str) -> dict:
    """載入 waypoint YAML。"""
    with open(path) as f:
        data = yaml.safe_load(f)
    if not data or "waypoints" not in data:
        raise ValueError(f"Invalid waypoint file: {path}")
    return data


def apply_randomization(waypoints: list, rng: np.random.Generator) -> list:
    """對 waypoints 加入隨機擾動（深拷貝）。"""
    import copy
    result = copy.deepcopy(waypoints)
    for wp in result:
        rand_cfg = wp.get("randomize")
        if not rand_cfg:
            continue
        joints = np.array(wp["joints"], dtype=np.float32)
        # 關節擾動
        if "joints" in rand_cfg:
            ranges = np.array(rand_cfg["joints"], dtype=np.float32)
            perturbation = rng.uniform(-ranges, ranges)
            joints += perturbation
            joints = np.clip(joints, JOINT_LOWER, JOINT_UPPER)
        # 時間擾動
        if "duration" in rand_cfg:
            d = wp["duration"]
            r = rand_cfg["duration"]
            wp["duration"] = max(0.5, d + rng.uniform(-r, r))
        wp["joints"] = joints.tolist()
    return result


def interpolate_segment(start: np.ndarray, end: np.ndarray,
                        duration: float, fps: float) -> list:
    """線性插值產生軌跡點。"""
    n_steps = max(1, int(duration * fps))
    trajectory = []
    for i in range(n_steps + 1):
        alpha = i / n_steps
        point = start + (end - start) * alpha
        trajectory.append(point)
    return trajectory


def run_collect(args):
    """自動執行 waypoint 軌跡 + 錄製訓練資料。"""
    # 載入 waypoints
    wp_data = load_waypoints(args.waypoints)
    arm = wp_data.get("arm", "R")
    task = wp_data.get("task", args.task)
    record_fps = wp_data.get("record_fps", 30)
    waypoints = wp_data["waypoints"]

    print(f"載入 {len(waypoints)} 個 waypoints (arm={arm})")
    print(f"任務: {task}")
    print(f"Episodes: {args.episodes}")
    print(f"錄製 FPS: {record_fps}")
    print(f"隨機化: {'ON' if args.randomize else 'OFF'}")
    print()

    # 輸出目錄
    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    # 已有的 episode 數
    existing = list(output_dir.glob("episode_*"))
    episode_start = len(existing)
    print(f"輸出目錄: {output_dir} (已有 {episode_start} episodes)")

    # ROS2
    rclpy.init()
    robot = RobotInterface(arm=arm)

    print("等待 joint_states...")
    if not robot.wait_for_state(timeout=10.0):
        print("ERROR: 未收到 joint_states。確認 start_follower.sh 已執行。")
        robot.shutdown()
        rclpy.shutdown()
        return

    # 等待相機
    print("等待相機影像...")
    t0 = time.time()
    while robot.get_color_frame() is None:
        if time.time() - t0 > 10.0:
            print("WARNING: 未收到相機影像，繼續但不錄影")
            break
        time.sleep(0.1)

    print("已連線，準備開始收集")
    print()

    rng = np.random.default_rng()
    publish_hz = 20  # 發布頻率
    publish_dt = 1.0 / publish_hz
    record_dt = 1.0 / record_fps

    try:
        for ep_idx in range(episode_start, episode_start + args.episodes):
            print(f"{'='*50}")
            print(f"Episode {ep_idx:04d} / {episode_start + args.episodes - 1:04d}")

            # 隨機化 waypoints
            if args.randomize:
                ep_waypoints = apply_randomization(waypoints, rng)
            else:
                ep_waypoints = waypoints

            # ── 產生完整軌跡 ──
            trajectory = []   # list of (joints_7, gripper_action)
            gripper_events = []  # list of (frame_idx, gripper_value_or_command)

            current_pos = robot.get_joint_positions()
            frame_idx = 0

            for wp in ep_waypoints:
                target = np.array(wp["joints"], dtype=np.float32)
                target = np.clip(target, JOINT_LOWER, JOINT_UPPER)
                duration = wp.get("duration", 2.0)

                # 插值
                segment = interpolate_segment(current_pos, target, duration, record_fps)
                start_frame = frame_idx

                for pt in segment:
                    trajectory.append(pt)
                    frame_idx += 1

                current_pos = target

                # 夾爪事件（在到達 waypoint 時觸發）
                grip = wp.get("gripper")
                if grip is not None:
                    gripper_events.append((frame_idx - 1, grip))

            total_frames = len(trajectory)
            total_duration = total_frames / record_fps
            print(f"  軌跡: {total_frames} frames, {total_duration:.1f}s")

            # ── 執行軌跡 + 同步錄製 ──
            states = []
            images_color = []
            timestamps = []

            # 先回到第一個 waypoint（不錄製）
            first_target = trajectory[0]
            print("  移動到起始位置...")
            for _ in range(int(publish_hz * 2)):  # 2 秒
                robot.send_joints(first_target.tolist())
                time.sleep(publish_dt)

            # 處理起始夾爪狀態
            for fi, grip in gripper_events:
                if fi == 0:
                    _execute_gripper(robot, grip)

            time.sleep(0.5)
            print("  開始錄製...")

            record_start = time.time()
            last_publish = 0.0
            last_record = 0.0
            grip_event_idx = 0  # 下一個待觸發的 gripper event

            for frame_i in range(total_frames):
                now = time.time()
                elapsed = now - record_start
                target_time = frame_i * record_dt

                # 等待到正確的時間點
                wait = target_time - elapsed
                if wait > 0:
                    time.sleep(wait)

                # 發布關節目標
                robot.send_joints(trajectory[frame_i].tolist())

                # 檢查夾爪事件
                while grip_event_idx < len(gripper_events):
                    ge_frame, ge_val = gripper_events[grip_event_idx]
                    if frame_i >= ge_frame:
                        _execute_gripper(robot, ge_val)
                        grip_event_idx += 1
                    else:
                        break

                # 錄製
                actual_joints = robot.get_joint_positions()
                state = np.zeros(8, dtype=np.float32)
                state[:7] = actual_joints
                # 簡化：gripper state 用最後一次夾爪指令
                states.append(state)

                color = robot.get_color_frame()
                if color is not None:
                    images_color.append(color)
                else:
                    # 填充黑色幀
                    images_color.append(np.zeros((480, 640, 3), dtype=np.uint8))

                timestamps.append(time.time() - record_start)

            actual_duration = timestamps[-1] if timestamps else 0
            print(f"  錄製完成: {len(states)} frames, {actual_duration:.1f}s")

            # ── 儲存 episode ──
            ep_dir = output_dir / f"episode_{ep_idx:06d}"
            ep_dir.mkdir(exist_ok=True)

            # data.npz
            states_arr = np.array(states, dtype=np.float32)
            actions_arr = np.zeros_like(states_arr)
            actions_arr[:-1] = states_arr[1:]
            actions_arr[-1] = states_arr[-1]

            np.savez_compressed(
                ep_dir / "data.npz",
                states=states_arr,
                actions=actions_arr,
                timestamps=np.array(timestamps, dtype=np.float32),
            )

            # exterior.mp4
            if images_color and images_color[0] is not None:
                h, w = images_color[0].shape[:2]
                writer = cv2.VideoWriter(
                    str(ep_dir / "exterior.mp4"),
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    record_fps,
                    (w, h),
                )
                for frame in images_color:
                    writer.write(frame)
                writer.release()

            # meta.json
            meta = {
                "episode_idx": ep_idx,
                "task": task,
                "fps": record_fps,
                "num_frames": len(states),
                "duration_sec": actual_duration,
                "arm": arm,
                "randomized": args.randomize,
                "timestamp": datetime.now().isoformat(),
            }
            with open(ep_dir / "meta.json", "w") as f:
                json.dump(meta, f, indent=2)

            print(f"  已儲存: {ep_dir}")

            # Episode 間隔：回 home
            if ep_idx < episode_start + args.episodes - 1:
                print("  回到 home...")
                home = ep_waypoints[0]["joints"]
                for _ in range(int(publish_hz * 2)):
                    robot.send_joints(home)
                    time.sleep(publish_dt)
                time.sleep(args.interval)

    except KeyboardInterrupt:
        print(f"\n中斷。已完成 {ep_idx - episode_start} 個 episodes。")

    print()
    total_episodes = len(list(output_dir.glob("episode_*")))
    print(f"收集結束。共 {total_episodes} 個 episodes in {output_dir}")

    robot.shutdown()
    rclpy.shutdown()


def _execute_gripper(robot: RobotInterface, grip_value):
    """執行夾爪動作。"""
    if isinstance(grip_value, str):
        if grip_value.lower() == "open":
            robot.send_gripper_command("OPEN")
        elif grip_value.lower() == "close":
            robot.send_gripper_command("CLOSE")
        else:
            robot.send_gripper_command(grip_value.upper())
    elif isinstance(grip_value, (int, float)):
        robot.send_gripper_command("ENABLE")
        time.sleep(0.2)
        robot.send_gripper(float(grip_value))
    # 等待夾爪動作完成
    time.sleep(0.5)


# ════════════════════════════════════════════════════════════
# Main
# ════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="OpenArm Waypoint Collector — 腳本式資料收集器",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="mode", required=True)

    # ── calibrate ──
    cal = sub.add_parser("calibrate", help="互動式標定 waypoints")
    cal.add_argument("-a", "--arm", default="R", choices=["R", "L"],
                     help="手臂 (R=右, L=左)")
    cal.add_argument("-o", "--output", default="waypoints/pick_and_place.yaml",
                     help="輸出 YAML 檔案路徑")
    cal.add_argument("-t", "--task", default="pick up the object and place it in the box",
                     help="任務描述 (英文)")

    # ── collect ──
    col = sub.add_parser("collect", help="自動收集 episodes")
    col.add_argument("-w", "--waypoints", required=True,
                     help="Waypoint YAML 檔案路徑")
    col.add_argument("-n", "--episodes", type=int, default=100,
                     help="要收集的 episodes 數量")
    col.add_argument("-o", "--output-dir",
                     default=os.path.expanduser("~/datasets/pick_and_place/raw"),
                     help="輸出目錄")
    col.add_argument("-t", "--task", default="pick up the object and place it in the box",
                     help="任務描述 (英文，YAML 中的優先)")
    col.add_argument("--randomize", action="store_true",
                     help="啟用隨機擾動")
    col.add_argument("--interval", type=float, default=1.0,
                     help="Episodes 之間的間隔秒數")

    args = parser.parse_args()

    if args.mode == "calibrate":
        run_calibrate(args)
    elif args.mode == "collect":
        run_collect(args)


if __name__ == "__main__":
    main()
