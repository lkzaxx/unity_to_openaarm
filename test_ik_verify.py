#!/usr/bin/env python3
"""
OpenArm IK/FK 驗證腳本
在 Jetson 上執行，驗證 Unity C# 的 DLS IK 邏輯是否正確

用法: python3 test_ik_verify.py
"""
import math
import numpy as np
from dataclasses import dataclass

# ============================================================================
# 連桿參數（與 OpenArmKinematics.cs 一致）
# ============================================================================
SHOULDER_TO_ELBOW = 0.25  # m
ELBOW_TO_WRIST = 0.25    # m
WRIST_TO_EE = 0.10       # m

# 關節軸（局部座標，與 Unity C# 一致）
# 注意：J4 軸為 -X（肘部反向鉸鏈）
JOINT_AXES = [
    np.array([1, 0, 0]),   # J1: Pitch (+X)
    np.array([0, 0, 1]),   # J2: Roll  (+Z)
    np.array([0, 1, 0]),   # J3: Yaw   (+Y)
    np.array([-1, 0, 0]),  # J4: Pitch (-X)  ← 肘部反向
    np.array([0, 0, 1]),   # J5: Roll  (+Z)
    np.array([0, 1, 0]),   # J6: Yaw   (+Y)
    np.array([1, 0, 0]),   # J7: Pitch (+X)
]

# 連桿偏移
LINK_OFFSETS = [
    np.array([0, 0, 0]),                    # J1 在基座
    np.array([0, 0, 0]),                    # J2 與 J1 共位
    np.array([0, 0, 0]),                    # J3 與 J1 共位
    np.array([0, -SHOULDER_TO_ELBOW, 0]),   # J4: 向下到肘部
    np.array([0, -ELBOW_TO_WRIST, 0]),      # J5: 向下到手腕
    np.array([0, 0, 0]),                    # J6 與 J5 共位
    np.array([0, 0, 0]),                    # J7 與 J5 共位
]

# 關節限制（運動學空間 = 左臂馬達慣例，度）
MIN_DEG = [-186.0, -180.0, -81.0, 7.0, -81.0, -40.5, -81.0]
MAX_DEG = [66.0, 0.0, 81.0, 133.0, 81.0, 40.5, 81.0]

# ============================================================================
# 旋轉工具（模擬 Unity 的 Quaternion — 右手旋轉規則）
# ============================================================================

def rotation_matrix(axis, angle_deg):
    """繞任意軸旋轉 angle_deg 度（右手規則，與 Unity AngleAxis 一致）"""
    axis = axis / np.linalg.norm(axis)
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    x, y, z = axis
    return np.array([
        [c + x*x*(1-c),     x*y*(1-c) - z*s,   x*z*(1-c) + y*s],
        [y*x*(1-c) + z*s,   c + y*y*(1-c),      y*z*(1-c) - x*s],
        [z*x*(1-c) - y*s,   z*y*(1-c) + x*s,    c + z*z*(1-c)]
    ])


def forward_kinematics(angles_deg, base_pos=np.zeros(3)):
    """正向運動學：回傳 (ee_pos, joint_positions[0..6])"""
    rot = np.eye(3)
    pos = base_pos.copy()
    joint_positions = []

    for i in range(7):
        pos = pos + rot @ LINK_OFFSETS[i]
        joint_positions.append(pos.copy())
        world_axis = rot @ JOINT_AXES[i]
        rot = rotation_matrix(world_axis, angles_deg[i]) @ rot

    ee_pos = pos + rot @ np.array([0, -WRIST_TO_EE, 0])
    return ee_pos, joint_positions


def solve_dls(target_pos, base_pos, initial_deg, max_iter=50, tol=0.005,
              damping=0.02, max_step=15.0):
    """DLS 逆向運動學（與 OpenArmKinematics.SolveDLS 一致）"""
    result = np.array(initial_deg, dtype=float)
    base_lambda2 = damping * damping

    for iteration in range(max_iter):
        # FK 計算各關節位置/軸
        rot = np.eye(3)
        pos = base_pos.copy()
        jac_pos = []
        jac_axes = []

        for i in range(7):
            pos = pos + rot @ LINK_OFFSETS[i]
            jac_pos.append(pos.copy())
            world_axis = rot @ JOINT_AXES[i]
            jac_axes.append(world_axis.copy())
            rot = rotation_matrix(world_axis, result[i]) @ rot

        ee_pos = pos + rot @ np.array([0, -WRIST_TO_EE, 0])

        # 誤差
        error = target_pos - ee_pos
        err_mag = np.linalg.norm(error)
        if err_mag < tol:
            return result, err_mag, iteration, True

        # Jacobian (3×7)
        J = np.zeros((3, 7))
        for i in range(7):
            to_ee = ee_pos - jac_pos[i]
            J[:, i] = np.cross(jac_axes[i], to_ee)

        # Adaptive damping
        lambda2 = base_lambda2 * (1.0 + err_mag * 5.0)

        # (JJᵀ + λ²I)⁻¹
        JJT = J @ J.T + lambda2 * np.eye(3)
        x = np.linalg.solve(JJT, error)

        # Δθ = Jᵀ · x
        d_rad = J.T @ x
        d_deg = np.degrees(d_rad)
        d_deg = np.clip(d_deg, -max_step, max_step)

        result = np.clip(result + d_deg, MIN_DEG, MAX_DEG)

    ee_final, _ = forward_kinematics(result, base_pos)
    final_err = np.linalg.norm(ee_final - target_pos)
    return result, final_err, max_iter, final_err < tol


def mirror_angles(deg):
    """左右臂鏡像：J1,2,3,5,6,7 取反，J4 不變"""
    m = np.array(deg, dtype=float)
    m[0] = -m[0]
    m[1] = -m[1]
    m[2] = -m[2]
    # m[3] unchanged
    m[4] = -m[4]
    m[5] = -m[5]
    m[6] = -m[6]
    return m


# ============================================================================
# 測試
# ============================================================================

def test_fk_known_poses():
    """測試 FK 已知姿態"""
    print("=" * 60)
    print("TEST 1: FK 已知姿態")
    print("=" * 60)

    # 1a. 全零：手臂垂直往下掛
    angles = [0, 0, 0, 0, 0, 0, 0]
    ee, _ = forward_kinematics(angles)
    expected = np.array([0, -(SHOULDER_TO_ELBOW + ELBOW_TO_WRIST + WRIST_TO_EE), 0])
    err = np.linalg.norm(ee - expected)
    status = "PASS" if err < 0.001 else "FAIL"
    print(f"  全零（垂下）: EE={ee}, 預期={expected}, 誤差={err:.4f}m [{status}]")

    # 1b. J1=-90°：手臂往前伸直
    angles = [-90, 0, 0, 0, 0, 0, 0]
    ee, _ = forward_kinematics(angles)
    expected = np.array([0, 0, SHOULDER_TO_ELBOW + ELBOW_TO_WRIST + WRIST_TO_EE])
    err = np.linalg.norm(ee - expected)
    status = "PASS" if err < 0.001 else "FAIL"
    print(f"  J1=-90°（前伸）: EE={ee}, 預期={expected}, 誤差={err:.4f}m [{status}]")

    # 1c. J1=+90° (= 右臂 +90° 鏡像後的運動學)：手臂往上
    angles = [90, 0, 0, 0, 0, 0, 0]
    ee, _ = forward_kinematics(angles)
    expected_z = -(SHOULDER_TO_ELBOW + ELBOW_TO_WRIST + WRIST_TO_EE)
    print(f"  J1=+90°（後伸）: EE={ee}")

    # 1d. J1=-90°, J4=90°：前伸+肘彎（前臂應該往上）
    angles = [-90, 0, 0, 90, 0, 0, 0]
    ee, joints = forward_kinematics(angles)
    print(f"  J1=-90° J4=90°（前伸+肘彎上）: EE={ee}")
    print(f"    肘部位置={joints[3]}, 手腕位置={joints[4]}")
    # 肘彎後前臂應往上：EE.y 應該 > 0
    elbow_up = ee[1] > joints[3][1]
    status = "PASS" if elbow_up else "FAIL"
    print(f"    EE 在肘部上方? {elbow_up} [{status}]")

    # 1e. J4=90° 從垂下（前臂應往前）
    angles = [0, 0, 0, 90, 0, 0, 0]
    ee, joints = forward_kinematics(angles)
    print(f"  J4=90°（垂下+肘彎前）: EE={ee}")
    forearm_forward = ee[2] > 0.1
    status = "PASS" if forearm_forward else "FAIL"
    print(f"    前臂往前(+Z)? EE.z={ee[2]:.3f} [{status}]")
    print()


def test_fk_joint_directions():
    """測試每個關節 +1.5 rad 的旋轉方向（對照實際機器人）"""
    print("=" * 60)
    print("TEST 2: 關節方向驗證（左臂，各關節 +1.5 rad）")
    print("=" * 60)

    descriptions = [
        "J1: 預期=往後(-Z)",
        "J2: 預期=往身體靠(+X)",
        "J3: 預期=繞垂直軸順時",
        "J4: 預期=肘往前(+Z)/上(+Y)",
        "J5: 預期=繞手臂軸旋轉",
        "J6: 預期=手繞垂直軸轉",
        "J7: 預期=手往後(-Z)",
    ]

    test_angle = math.degrees(1.5)  # ≈ 85.9°
    zero = [0] * 7

    for j in range(7):
        angles = zero.copy()
        angles[j] = test_angle
        ee, joints = forward_kinematics(angles)
        ee_zero, _ = forward_kinematics(zero)
        delta = ee - ee_zero
        print(f"  {descriptions[j]}")
        print(f"    EE: {ee}  Δ=({delta[0]:+.3f}, {delta[1]:+.3f}, {delta[2]:+.3f})")

    print()


def test_dls_basic():
    """測試 DLS 基本收斂"""
    print("=" * 60)
    print("TEST 3: DLS IK 收斂測試")
    print("=" * 60)

    base = np.zeros(3)

    targets = [
        ("正前方 0.50m", np.array([0, 0, 0.50])),
        ("正前方 0.55m（近極限）", np.array([0, 0, 0.55])),
        ("前下方", np.array([0, -0.3, 0.4])),
        ("前上方", np.array([0, 0.2, 0.4])),
        ("側前方", np.array([0.15, -0.1, 0.45])),
        ("正下方 0.50m", np.array([0, -0.50, 0])),
    ]

    seed = [-90, 0, 0, 7, 0, 0, 0]

    for name, target in targets:
        result, err, iters, converged = solve_dls(target, base, seed)
        ee_verify, _ = forward_kinematics(result, base)
        verify_err = np.linalg.norm(ee_verify - target)

        status = "PASS" if converged else ("CLOSE" if err < 0.02 else "FAIL")
        print(f"  {name}:")
        print(f"    目標={target}, 誤差={err:.4f}m, 迭代={iters}, [{status}]")
        print(f"    解={[f'{a:.1f}' for a in result]}")
        print(f"    FK驗證={ee_verify}, 驗證誤差={verify_err:.4f}m")
    print()


def test_mirror_consistency():
    """測試左右臂鏡像一致性"""
    print("=" * 60)
    print("TEST 4: 左右臂鏡像一致性")
    print("=" * 60)

    base = np.zeros(3)

    # 左臂前伸
    left_angles = [-90, 0, 0, 7, 0, 0, 0]
    left_ee, _ = forward_kinematics(left_angles, base)

    # 右臂前伸（馬達角度 = 鏡像）
    right_motor = [90, 0, 0, 7, 0, 0, 0]
    # 轉到運動學空間
    right_kinematic = mirror_angles(right_motor)
    right_ee, _ = forward_kinematics(right_kinematic, base)

    print(f"  左臂 {left_angles} → EE={left_ee}")
    print(f"  右臂馬達 {list(right_motor)} → 運動學 {list(right_kinematic)} → EE={right_ee}")
    pos_match = np.allclose(left_ee, right_ee, atol=0.001)
    status = "PASS" if pos_match else "FAIL"
    print(f"  兩臂 EE 位置一致? {pos_match} [{status}]")

    # DLS 測試：同一目標，左右臂應得到鏡像解
    target = np.array([0, -0.1, 0.45])
    left_seed = [-90, 0, 0, 7, 0, 0, 0]
    right_seed_kin = mirror_angles([90, 0, 0, 7, 0, 0, 0])

    left_result, left_err, _, _ = solve_dls(target, base, left_seed)
    right_result_kin, right_err, _, _ = solve_dls(target, base, list(right_seed_kin))
    right_result_motor = mirror_angles(right_result_kin)

    print(f"\n  目標={target}")
    print(f"  左臂 IK 結果: {[f'{a:.1f}' for a in left_result]}, 誤差={left_err:.4f}m")
    print(f"  右臂 IK 結果（運動學）: {[f'{a:.1f}' for a in right_result_kin]}, 誤差={right_err:.4f}m")
    print(f"  右臂馬達角度: {[f'{a:.1f}' for a in right_result_motor]}")

    # 左臂角度和右臂運動學角度應該一致
    kin_match = np.allclose(left_result, right_result_kin, atol=1.0)
    status = "PASS" if kin_match else "WARN (可能因初始seed不同)"
    print(f"  左=右運動學? {kin_match} [{status}]")
    print()


def test_ros2_limits_match():
    """驗證我們的限制與 ROS2 端 unity_interface_follower.py 的限制一致"""
    print("=" * 60)
    print("TEST 5: 關節限制對照 ROS2")
    print("=" * 60)

    # ROS2 端的限制（rad）— 來自 unity_interface_follower.py
    ros2_left = [
        (-3.490659, 1.396263),   # J1
        (-3.316125, 0.174533),   # J2
        (-1.570796, 1.570796),   # J3
        (0.0, 2.443461),         # J4
        (-1.570796, 1.570796),   # J5
        (-0.785398, 0.785398),   # J6
        (-1.570796, 1.570796),   # J7
    ]

    ros2_right = [
        (-1.396263, 3.490659),   # J1
        (-0.174533, 3.316125),   # J2
        (-1.570796, 1.570796),   # J3
        (0.0, 2.443461),         # J4
        (-1.570796, 1.570796),   # J5
        (-0.785398, 0.785398),   # J6
        (-1.570796, 1.570796),   # J7
    ]

    print("  左臂（IK 運動學空間 vs ROS2 硬體限制）:")
    for i in range(7):
        ik_min_rad = math.radians(MIN_DEG[i])
        ik_max_rad = math.radians(MAX_DEG[i])
        ros_min, ros_max = ros2_left[i]
        # IK 安全範圍應在 ROS2 硬體範圍內
        within = ik_min_rad >= ros_min - 0.01 and ik_max_rad <= ros_max + 0.01
        status = "OK" if within else "WARN"
        print(f"    J{i+1}: IK=[{ik_min_rad:.3f}, {ik_max_rad:.3f}] "
              f"ROS2=[{ros_min:.3f}, {ros_max:.3f}] [{status}]")

    print("\n  右臂鏡像後 vs ROS2 限制:")
    for i in range(7):
        # 鏡像：取反 J1,2,3,5,6,7 的限制
        if i in [0, 1, 2, 4, 5, 6]:
            mirror_min = -math.radians(MAX_DEG[i])
            mirror_max = -math.radians(MIN_DEG[i])
        else:
            mirror_min = math.radians(MIN_DEG[i])
            mirror_max = math.radians(MAX_DEG[i])
        ros_min, ros_max = ros2_right[i]
        within = mirror_min >= ros_min - 0.01 and mirror_max <= ros_max + 0.01
        status = "OK" if within else "WARN"
        print(f"    J{i+1}: Mirror=[{mirror_min:.3f}, {mirror_max:.3f}] "
              f"ROS2=[{ros_min:.3f}, {ros_max:.3f}] [{status}]")
    print()


def test_workspace_coverage():
    """測試工作空間多個目標點的 IK 收斂率"""
    print("=" * 60)
    print("TEST 6: 工作空間覆蓋率")
    print("=" * 60)

    base = np.zeros(3)
    seed = [-90, 0, 0, 7, 0, 0, 0]

    # 球座標採樣
    total = 0
    converged = 0
    close = 0  # < 2cm

    for r in [0.20, 0.30, 0.40, 0.50]:
        for theta in range(0, 180, 30):  # 仰角
            for phi in range(-90, 91, 30):  # 方位角
                t_rad = math.radians(theta)
                p_rad = math.radians(phi)
                x = r * math.sin(t_rad) * math.sin(p_rad)
                y = -r * math.cos(t_rad)  # 向下為主
                z = r * math.sin(t_rad) * math.cos(p_rad)
                target = np.array([x, y, z])

                result, err, _, ok = solve_dls(target, base, seed)
                total += 1
                if ok:
                    converged += 1
                elif err < 0.02:
                    close += 1

    print(f"  總測試點: {total}")
    print(f"  完全收斂 (<5mm): {converged} ({100*converged/total:.1f}%)")
    print(f"  接近收斂 (<20mm): {converged+close} ({100*(converged+close)/total:.1f}%)")
    print(f"  失敗 (>20mm): {total-converged-close} ({100*(total-converged-close)/total:.1f}%)")
    print()


if __name__ == '__main__':
    print("OpenArm IK/FK 驗證腳本")
    print(f"臂長: 肩→肘={SHOULDER_TO_ELBOW}m, 肘→腕={ELBOW_TO_WRIST}m, 腕→EE={WRIST_TO_EE}m")
    print(f"總長: {SHOULDER_TO_ELBOW+ELBOW_TO_WRIST+WRIST_TO_EE}m")
    print()

    test_fk_known_poses()
    test_fk_joint_directions()
    test_dls_basic()
    test_mirror_consistency()
    test_ros2_limits_match()
    test_workspace_coverage()

    print("=" * 60)
    print("全部測試完成")
    print("=" * 60)
