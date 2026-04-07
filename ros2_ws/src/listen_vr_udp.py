#!/usr/bin/env python3
"""
OpenArmX VR APK UDP Listener (port 5100)
在 Jetson 上執行，監聽 VR 頭盔傳來的控制器資料。

用法:
    python3 listen_vr_udp.py
    python3 listen_vr_udp.py --port 5100 --verbose
"""

import socket
import argparse
import time
from datetime import datetime

FIELD_NAMES = [
    "hand",        # L or R
    "pos_x",       # position X
    "pos_y",       # position Y
    "pos_z",       # position Z
    "quat_x",      # quaternion X
    "quat_y",      # quaternion Y
    "quat_z",      # quaternion Z
    "quat_w",      # quaternion W
    "trigger",     # index trigger 0.0-1.0
    "grip",        # grip 0.0-1.0
    "btn_a",       # A button (right)
    "btn_b",       # B button (right)
    "btn_x",       # X button (left)
    "btn_y",       # Y button (left)
    "rate",        # speed rate (0.1 or 1.0)
    "timestamp_ns" # nanosecond timestamp
]


def parse_vr_message(data: str) -> dict:
    parts = data.strip().split()
    result = {}
    for i, name in enumerate(FIELD_NAMES):
        if i < len(parts):
            result[name] = parts[i]
        else:
            result[name] = None
    if len(parts) > len(FIELD_NAMES):
        result["_extra"] = parts[len(FIELD_NAMES):]
    return result


def format_parsed(parsed: dict, verbose: bool = False) -> str:
    hand = parsed.get("hand", "?")
    px = parsed.get("pos_x", "?")
    py = parsed.get("pos_y", "?")
    pz = parsed.get("pos_z", "?")
    trigger = parsed.get("trigger", "?")
    grip = parsed.get("grip", "?")

    if verbose:
        lines = []
        for name in FIELD_NAMES:
            val = parsed.get(name)
            if val is not None:
                lines.append(f"  {name:>14s}: {val}")
        extra = parsed.get("_extra")
        if extra:
            lines.append(f"  {'_extra':>14s}: {' '.join(extra)}")
        return "\n".join(lines)
    else:
        return (f"[{hand}] pos=({px}, {py}, {pz}) "
                f"trigger={trigger} grip={grip}")


def main():
    parser = argparse.ArgumentParser(description="Listen for OpenArmX VR UDP packets")
    parser.add_argument("--port", type=int, default=5100, help="UDP port (default: 5100)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Show all fields")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    print(f"=== OpenArmX VR UDP Listener ===")
    print(f"Listening on 0.0.0.0:{args.port}")
    print(f"Waiting for VR data... (Ctrl+C to stop)\n")

    count = 0
    last_time = time.time()

    try:
        while True:
            data, addr = sock.recvfrom(512)
            count += 1
            now = time.time()

            try:
                msg = data.decode("utf-8")
            except UnicodeDecodeError:
                print(f"[{count}] RAW BYTES from {addr}: {data.hex()}")
                continue

            parsed = parse_vr_message(msg)
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]

            if args.verbose:
                print(f"--- #{count} [{ts}] from {addr[0]}:{addr[1]} ---")
                print(f"  raw: {msg.strip()}")
                print(format_parsed(parsed, verbose=True))
                print()
            else:
                hz = ""
                if now - last_time >= 1.0:
                    hz = f" (~{count}Hz)"
                    count = 0
                    last_time = now
                print(f"[{ts}] {addr[0]:>15s} | {format_parsed(parsed)}{hz}")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
