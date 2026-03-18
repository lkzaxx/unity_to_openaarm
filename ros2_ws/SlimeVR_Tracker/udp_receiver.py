"""
SlimeVR Tracker UDP Receiver
直接接收 SlimeVR 追蹤器的 IMU 旋轉資料 (四元數)，不需要 SlimeVR Server。
可在 PC (Windows) 或 Jetson (Linux) 上運行。

Usage:
    python udp_receiver.py              # 接收並印出四元數
    python udp_receiver.py --euler      # 同時顯示歐拉角
    python udp_receiver.py --json       # JSON 格式輸出 (方便串接其他程式)

Protocol: SlimeVR UDP on port 6969
Reference: https://github.com/SlimeVR/SlimeVR-Tracker-ESP
"""

import socket
import struct
import math
import time
import argparse
import json

# SlimeVR UDP protocol constants
SLIMEVR_PORT = 6969
MAGIC_HANDSHAKE = b"Hey OVR =D 5"

# Packet types
PACKET_HEARTBEAT = 0
PACKET_HANDSHAKE = 3
PACKET_ACCEL = 4
PACKET_BATTERY = 12
PACKET_SENSOR_INFO = 15
PACKET_ROTATION_DATA = 17
PACKET_SIGNAL_STRENGTH = 19
PACKET_TEMPERATURE = 20

PACKET_NAMES = {
    0: "HEARTBEAT",
    3: "HANDSHAKE",
    4: "ACCEL",
    12: "BATTERY",
    13: "TAP",
    14: "ERROR",
    15: "SENSOR_INFO",
    17: "ROTATION_DATA",
    18: "MAG_ACCURACY",
    19: "SIGNAL_STRENGTH",
    20: "TEMPERATURE",
    22: "FEATURE_FLAGS",
}


def quat_to_euler(x, y, z, w):
    """四元數轉歐拉角 (degrees): roll, pitch, yaw"""
    # Roll (x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


def build_handshake_response():
    """建立 Server 端的握手回應封包"""
    # Packet type (1 byte) + magic string at offset 1
    # Firmware validates: strncmp(packet + 1, "Hey OVR =D 5", 12)
    pkt = struct.pack(">B", PACKET_HANDSHAKE)
    pkt += MAGIC_HANDSHAKE
    return pkt


def parse_handshake(data):
    """解析追蹤器送來的握手封包"""
    offset = 12  # skip type(4) + packet_num(8)
    info = {}
    if len(data) > offset + 16:
        board, imu, mcu, fw_build = struct.unpack_from(">IIII", data, offset)
        offset += 16
        info["board"] = board
        info["imu"] = imu
        info["mcu"] = mcu
        info["fw_build"] = fw_build

        # Firmware string (length-prefixed)
        if offset < len(data):
            str_len = data[offset]
            offset += 1
            if offset + str_len <= len(data):
                info["firmware"] = data[offset : offset + str_len].decode(
                    "utf-8", errors="replace"
                )
                offset += str_len

        # MAC address (6 bytes)
        if offset + 6 <= len(data):
            mac_bytes = data[offset : offset + 6]
            info["mac"] = ":".join(f"{b:02X}" for b in mac_bytes)
            offset += 6

    return info


def parse_rotation(data):
    """解析旋轉資料封包 (四元數)"""
    offset = 12  # skip type(4) + packet_num(8)
    if len(data) < offset + 18:
        return None

    sensor_id = data[offset]
    data_type = data[offset + 1]
    x, y, z, w = struct.unpack_from(">ffff", data, offset + 2)
    accuracy = data[offset + 18] if len(data) > offset + 18 else 0

    return {
        "sensor_id": sensor_id,
        "data_type": data_type,
        "quat": {"x": x, "y": y, "z": z, "w": w},
        "accuracy": accuracy,
    }


def parse_battery(data):
    """解析電池電量封包"""
    offset = 12
    if len(data) >= offset + 4:
        voltage = struct.unpack_from(">f", data, offset)[0]
        level = struct.unpack_from(">f", data, offset + 4)[0] if len(data) >= offset + 8 else -1
        return {"voltage": voltage, "level": level}
    return None


def parse_signal_strength(data):
    """解析訊號強度封包"""
    offset = 12
    if len(data) >= offset + 2:
        sensor_id = data[offset]
        rssi = struct.unpack_from(">b", data, offset + 1)[0]
        return {"sensor_id": sensor_id, "rssi": rssi}
    return None


def run_receiver(show_euler=False, json_output=False, verbose=False, tracker_ip=None):
    """啟動 UDP 接收器，模擬 SlimeVR Server 的握手並接收追蹤器資料"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("0.0.0.0", SLIMEVR_PORT))
    sock.settimeout(1.0)

    print(f"SlimeVR UDP Receiver listening on port {SLIMEVR_PORT}...")
    if tracker_ip:
        print(f"主動連線模式: 向追蹤器 {tracker_ip} 發送握手 (繞過 UDP 廣播)")
    print("等待追蹤器連線... (確保追蹤器和此電腦在同一個 WiFi 網路)")
    print("Ctrl+C 停止\n")

    # 主動向追蹤器發送握手 (解決路由器阻擋 UDP 廣播的問題)
    if tracker_ip:
        response = build_handshake_response()
        for _ in range(5):
            sock.sendto(response, (tracker_ip, SLIMEVR_PORT))
            time.sleep(0.3)
        print(f"已向 {tracker_ip} 發送握手請求\n")

    connected_trackers = {}
    frame_count = 0
    last_print_time = time.time()
    last_active_handshake = time.time()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                # 定期重發握手 (主動連線模式)
                if tracker_ip and time.time() - last_active_handshake > 5:
                    sock.sendto(build_handshake_response(), (tracker_ip, SLIMEVR_PORT))
                    last_active_handshake = time.time()
                continue

            if len(data) < 4:
                continue

            pkt_type = struct.unpack_from(">I", data, 0)[0]

            # Handle handshake - respond to tracker discovery
            if pkt_type == PACKET_HANDSHAKE:
                info = parse_handshake(data)
                mac = info.get("mac", addr[0])
                connected_trackers[addr] = info
                print(f"[HANDSHAKE] 追蹤器連線: {addr[0]}:{addr[1]}")
                if info:
                    print(f"  Board: {info.get('board')}, IMU: {info.get('imu')}, "
                          f"MAC: {info.get('mac', 'N/A')}")
                    print(f"  Firmware: {info.get('firmware', 'N/A')}")

                # Send handshake response
                response = build_handshake_response()
                sock.sendto(response, addr)
                print(f"  -> 已回應握手\n")

            elif pkt_type == PACKET_HEARTBEAT:
                # Respond to heartbeat
                sock.sendto(build_handshake_response(), addr)

            elif pkt_type == PACKET_ROTATION_DATA:
                rot = parse_rotation(data)
                if rot is None:
                    continue

                frame_count += 1
                q = rot["quat"]

                if json_output:
                    output = {
                        "type": "rotation",
                        "sensor_id": rot["sensor_id"],
                        "quaternion": q,
                        "timestamp": time.time(),
                    }
                    if show_euler:
                        r, p, y = quat_to_euler(q["x"], q["y"], q["z"], q["w"])
                        output["euler_deg"] = {
                            "roll": round(r, 2),
                            "pitch": round(p, 2),
                            "yaw": round(y, 2),
                        }
                    print(json.dumps(output), flush=True)

                else:
                    now = time.time()
                    if now - last_print_time >= 0.1:  # 10Hz 顯示
                        last_print_time = now
                        line = (f"[ROT] sensor={rot['sensor_id']} "
                                f"q=({q['x']:+.4f}, {q['y']:+.4f}, "
                                f"{q['z']:+.4f}, {q['w']:+.4f})")
                        if show_euler:
                            r, p, y = quat_to_euler(
                                q["x"], q["y"], q["z"], q["w"]
                            )
                            line += f" euler=({r:+.1f}, {p:+.1f}, {y:+.1f})"
                        print(f"\r{line}    ", end="", flush=True)

            elif pkt_type == PACKET_BATTERY:
                bat = parse_battery(data)
                if bat and verbose:
                    print(f"\n[BAT] voltage={bat['voltage']:.3f}V, "
                          f"level={bat['level']:.0f}%")

            elif pkt_type == PACKET_SIGNAL_STRENGTH:
                sig = parse_signal_strength(data)
                if sig and verbose:
                    print(f"\n[SIGNAL] sensor={sig['sensor_id']}, "
                          f"RSSI={sig['rssi']}dBm")

            elif verbose:
                name = PACKET_NAMES.get(pkt_type, f"UNKNOWN({pkt_type})")
                print(f"\n[{name}] {len(data)} bytes from {addr[0]}")

    except KeyboardInterrupt:
        print(f"\n\n停止接收。共收到 {frame_count} 筆旋轉資料。")
    finally:
        sock.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SlimeVR Tracker UDP Receiver")
    parser.add_argument("--euler", action="store_true", help="同時顯示歐拉角 (度)")
    parser.add_argument("--json", action="store_true", help="JSON 格式輸出")
    parser.add_argument("--verbose", "-v", action="store_true", help="顯示所有封包類型")
    parser.add_argument("--tracker-ip", type=str, default=None,
                        help="追蹤器 IP (主動連線，繞過 UDP 廣播，適用於 Jetson 等環境)")
    args = parser.parse_args()

    run_receiver(show_euler=args.euler, json_output=args.json,
                 verbose=args.verbose, tracker_ip=args.tracker_ip)
