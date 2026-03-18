"""
SlimeVR Tracker Serial Monitor / Configuration Tool
Usage: python serial_monitor.py [command]

Commands:
  info     - Get tracker info
  wifi     - Set WiFi credentials
  monitor  - Monitor serial output continuously
  (none)   - Interactive mode
"""

import serial
import time
import sys


COM_PORT = "COM7"
BAUD_RATE = 115200


def get_serial():
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    time.sleep(0.5)
    ser.reset_input_buffer()
    return ser


def get_info():
    ser = get_serial()
    ser.write(b"GET INFO\r\n")
    time.sleep(2)
    resp = ser.read(4096)
    print(resp.decode("utf-8", errors="replace"))
    ser.close()


def set_wifi(ssid=None, password=None):
    if ssid is None:
        ssid = input("WiFi SSID: ")
    if password is None:
        password = input("WiFi Password: ")

    ser = get_serial()
    cmd = f'SET WIFI "{ssid}" "{password}"\r\n'
    print(f"Sending: {cmd.strip()}")
    ser.write(cmd.encode())

    print("Waiting for connection...")
    start = time.time()
    while time.time() - start < 20:
        data = ser.read(1024)
        if data:
            text = data.decode("utf-8", errors="replace")
            print(text, end="", flush=True)
            if "Connected successfully" in text:
                print("\n--- WiFi Connected! ---")
                break

    ser.write(b"GET INFO\r\n")
    time.sleep(2)
    resp = ser.read(4096)
    print(resp.decode("utf-8", errors="replace"))
    ser.close()


def monitor():
    ser = get_serial()
    print(f"Monitoring {COM_PORT} at {BAUD_RATE} baud... (Ctrl+C to stop)")
    try:
        while True:
            data = ser.read(1024)
            if data:
                text = data.decode("utf-8", errors="replace")
                print(text, end="", flush=True)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


def main():
    if len(sys.argv) > 1:
        cmd = sys.argv[1].lower()
        if cmd == "info":
            get_info()
        elif cmd == "wifi":
            ssid = sys.argv[2] if len(sys.argv) > 2 else None
            pwd = sys.argv[3] if len(sys.argv) > 3 else None
            set_wifi(ssid, pwd)
        elif cmd == "monitor":
            monitor()
        else:
            print(f"Unknown command: {cmd}")
            print(__doc__)
    else:
        print("SlimeVR Tracker Tool")
        print("1. Get Info")
        print("2. Set WiFi")
        print("3. Monitor")
        choice = input("Choice (1-3): ")
        if choice == "1":
            get_info()
        elif choice == "2":
            set_wifi()
        elif choice == "3":
            monitor()


if __name__ == "__main__":
    main()
