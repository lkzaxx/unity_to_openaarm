#!/usr/bin/env python3
"""
雙 Tracker J5/J6/J7 驗證 — Swing-Twist 分解（無萬向鎖）

校準：兩顆 tracker 放好，靜止 3 秒，各自歸零
驗證：轉動 tracker，J5/J6/J7 獨立不耦合

python3 test_dual_tracker_j5.py
"""

import socket
import struct
import math
import time
import threading

SLIMEVR_PORT = 6969
TRACKER_1_IP = "192.168.0.54"  # 機器人靈巧手
TRACKER_2_IP = "192.168.0.48"  # 使用者手
MAGIC_HANDSHAKE = b"Hey OVR =D 5"

tracker_data = {
    TRACKER_1_IP: {"q": (0, 0, 0, 1), "ts": 0, "count": 0},
    TRACKER_2_IP: {"q": (0, 0, 0, 1), "ts": 0, "count": 0},
}
data_lock = threading.Lock()


# === 四元數工具 ===

def quat_inverse(x, y, z, w):
    return (-x, -y, -z, w)

def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )

def quat_normalize(x, y, z, w):
    l = math.sqrt(x*x + y*y + z*z + w*w)
    if l < 1e-9:
        return (0, 0, 0, 1)
    return (x/l, y/l, z/l, w/l)

def vec_dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def swing_twist_decompose(q, twist_axis):
    """
    Swing-Twist 分解：把四元數 q 分成繞 twist_axis 的旋轉和垂直的旋轉。

    q = swing * twist

    twist = 繞 twist_axis 的旋轉分量（J5）
    swing = 垂直於 twist_axis 的旋轉分量（J6/J7）

    回傳：(twist_quat, swing_quat, twist_angle_deg)
    """
    x, y, z, w = q
    ax, ay, az = twist_axis

    # 投影四元數的虛部到 twist_axis
    proj = vec_dot((x, y, z), (ax, ay, az))
    twist = quat_normalize(ax * proj, ay * proj, az * proj, w)

    # swing = q * twist_inv
    twist_inv = quat_inverse(*twist)
    swing = quat_multiply(q, twist_inv)
    swing = quat_normalize(*swing)

    # twist 角度
    tx, ty, tz, tw = twist
    if tw < 0:
        tx, ty, tz, tw = -tx, -ty, -tz, -tw
    twist_angle = 2 * math.atan2(math.sqrt(tx*tx + ty*ty + tz*tz), tw)
    # 判斷方向
    if vec_dot((tx, ty, tz), (ax, ay, az)) < 0:
        twist_angle = -twist_angle

    return twist, swing, math.degrees(twist_angle)

def swing_to_two_angles(swing, axis1, axis2):
    """
    從 swing 四元數提取繞兩個垂直軸的角度。

    swing 是垂直於 twist_axis 的旋轉，可以分解為繞 axis1 和 axis2 的分量。
    用 atan2 提取各軸分量。
    """
    sx, sy, sz, sw = swing
    if sw < 0:
        sx, sy, sz, sw = -sx, -sy, -sz, -sw

    # swing 的虛部在 axis1 和 axis2 上的投影
    p1 = vec_dot((sx, sy, sz), axis1)
    p2 = vec_dot((sx, sy, sz), axis2)

    # 角度 = 2 * atan2(projection, w)
    angle1 = math.degrees(2 * math.atan2(p1, sw))
    angle2 = math.degrees(2 * math.atan2(p2, sw))

    return angle1, angle2


# === 網路 ===

def send_handshake(sock, tracker_ip):
    pkt = struct.pack(">B", 3) + MAGIC_HANDSHAKE
    for _ in range(5):
        sock.sendto(pkt, (tracker_ip, SLIMEVR_PORT))
        time.sleep(0.3)

def parse_rotation(data):
    if len(data) < 4:
        return None
    pkt_type = struct.unpack(">I", data[0:4])[0]
    if pkt_type != 17:
        return None
    offset = 12
    if len(data) < offset + 18:
        return None
    x, y, z, w = struct.unpack_from(">ffff", data, offset + 2)
    return (x, y, z, w)

def receiver_thread(sock):
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            ip = addr[0]
            if len(data) < 4:
                continue
            pkt_type = struct.unpack(">I", data[0:4])[0]
            if pkt_type == 3:
                sock.sendto(struct.pack(">B", 3) + MAGIC_HANDSHAKE, addr)
                continue
            if ip not in tracker_data:
                continue
            rot = parse_rotation(data)
            if rot is None:
                continue
            with data_lock:
                tracker_data[ip]["q"] = rot
                tracker_data[ip]["ts"] = time.time()
                tracker_data[ip]["count"] += 1
        except socket.timeout:
            pass
        except:
            pass


# === 主程式 ===

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", SLIMEVR_PORT))
    sock.settimeout(1.0)

    print("=== Swing-Twist J5/J6/J7 測試 ===")
    print(f"T1 (機器人): {TRACKER_1_IP}")
    print(f"T2 (使用者): {TRACKER_2_IP}")
    print()

    send_handshake(sock, TRACKER_1_IP)
    send_handshake(sock, TRACKER_2_IP)
    print("握手已發送...\n")

    recv = threading.Thread(target=receiver_thread, args=(sock,), daemon=True)
    recv.start()

    start_time = time.time()
    cal_q1 = None
    cal_q2 = None
    calibrated = False
    cal_count = 0

    # 校準時定義的軸（tracker 本地座標系）
    # 校準時手臂伸直朝前：
    #   X(1,0,0) = tracker 長邊 = 前臂方向 → J5 twist 軸
    #   Y(0,1,0) = tracker 短邊 = 左右方向 → J6
    #   Z(0,0,1) = tracker 正面 = 上方向   → J7
    # 如果映射不對，改這三個值
    TWIST_AXIS = (0, 0, 1)   # J5 翻轉 (原 J7 位置)
    SWING_AXIS1 = (1, 0, 0)  # J6 左右 (原備用位置)
    SWING_AXIS2 = (0, 1, 0)  # J7 上下 (原 J6 位置)

    print("=== 校準 ===")
    print("兩顆 tracker 放好，靜止 3 秒...")
    print()

    try:
        while True:
            time.sleep(0.05)

            with data_lock:
                t1 = tracker_data[TRACKER_1_IP].copy()
                t2 = tracker_data[TRACKER_2_IP].copy()

            if t1["count"] == 0 or t2["count"] == 0:
                if time.time() - start_time > 5:
                    missing = []
                    if t1["count"] == 0:
                        missing.append("T1")
                        send_handshake(sock, TRACKER_1_IP)
                    if t2["count"] == 0:
                        missing.append("T2")
                        send_handshake(sock, TRACKER_2_IP)
                    print(f"  等待 {', '.join(missing)}...")
                    start_time = time.time()
                continue

            q1 = t1["q"]
            q2 = t2["q"]

            if not calibrated:
                cal_count += 1
                if cal_count % 20 == 0:
                    print(f"  {cal_count}/60...")
                if cal_count >= 60:
                    cal_q1 = q1
                    cal_q2 = q2
                    calibrated = True
                    print()
                    print("  校準完成 ✅")
                    print()
                    print("=== 即時監控（Swing-Twist 分解）===")
                    print("  翻轉 → 只有 J5 變")
                    print("  左右擺 → 只有 J6 變")
                    print("  上下擺 → 只有 J7 變")
                    print("  (Ctrl+C 停止)")
                    print()
                    print("  ┌─ T1(機器人) ─┐  ┌─ T2(使用者) ─┐  ┌── 差值 ──┐")
                    print("   J5    J6    J7     J5    J6    J7    J5    J6    J7")
                    print("  " + "-" * 60)
                continue

            # 各自相對校準的旋轉
            delta1 = quat_multiply(quat_inverse(*cal_q1), q1)
            delta2 = quat_multiply(quat_inverse(*cal_q2), q2)

            # Swing-Twist 分解
            _, swing1, t1_j5 = swing_twist_decompose(delta1, TWIST_AXIS)
            t1_j6, t1_j7 = swing_to_two_angles(swing1, SWING_AXIS1, SWING_AXIS2)

            _, swing2, t2_j5 = swing_twist_decompose(delta2, TWIST_AXIS)
            t2_j6, t2_j7 = swing_to_two_angles(swing2, SWING_AXIS1, SWING_AXIS2)

            # 差值 = 使用者 - 機器人
            d_j5 = t2_j5 - t1_j5
            d_j6 = t2_j6 - t1_j6
            d_j7 = t2_j7 - t1_j7

            hz1 = t1["count"] / max(0.1, time.time() - start_time)
            hz2 = t2["count"] / max(0.1, time.time() - start_time)

            print(
                f"\r  {t1_j5:+6.1f} {t1_j6:+6.1f} {t1_j7:+6.1f}   "
                f"{t2_j5:+6.1f} {t2_j6:+6.1f} {t2_j7:+6.1f}   "
                f"{d_j5:+6.1f} {d_j6:+6.1f} {d_j7:+6.1f}  "
                f"Hz:{hz1:.0f}/{hz2:.0f}",
                end="", flush=True
            )

    except KeyboardInterrupt:
        print("\n\n=== 結果 ===")
        print(f"T1: {t1['count']} 筆 ({hz1:.0f}Hz)")
        print(f"T2: {t2['count']} 筆 ({hz2:.0f}Hz)")
        print()
        print("軸映射（如果不對就交換 TWIST_AXIS / SWING_AXIS1 / SWING_AXIS2）：")
        print(f"  J5(翻轉) = TWIST_AXIS  = {TWIST_AXIS}")
        print(f"  J6(左右) = SWING_AXIS1 = {SWING_AXIS1}")
        print(f"  J7(上下) = SWING_AXIS2 = {SWING_AXIS2}")

    sock.close()


if __name__ == "__main__":
    main()
