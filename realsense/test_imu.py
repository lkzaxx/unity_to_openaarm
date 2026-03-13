#!/usr/bin/env python3
"""RealSense D435i IMU 測試腳本 - 測試加速度計與陀螺儀"""
import pyrealsense2 as rs
import time
import sys

def main():
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("錯誤: 未找到 RealSense 裝置")
        sys.exit(1)

    dev = devices[0]
    has_imu = False
    for sensor in dev.query_sensors():
        name = sensor.get_info(rs.camera_info.name)
        if "Motion" in name:
            has_imu = True
            print(f"找到 IMU sensor: {name}")
            for p in sensor.get_stream_profiles():
                print(f"  {p.stream_type()} {p.format()} @ {p.fps()}fps")
            break

    if not has_imu:
        print("未找到 Motion Module (IMU)")
        print("請確認 hid-sensor-hub 等 kernel 模組已載入:")
        print("  lsmod | grep hid_sensor")
        sys.exit(1)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

    print("\n啟動 IMU 串流 (Accel@200Hz, Gyro@200Hz)...")
    try:
        pipeline.start(config)
    except Exception as e:
        print(f"錯誤: 無法啟動 IMU 串流 - {e}")
        sys.exit(1)

    print("讀取 IMU 資料 (5 秒)...\n")

    accel_count = 0
    gyro_count = 0
    start_time = time.time()

    while time.time() - start_time < 5:
        frames = pipeline.wait_for_frames()

        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)

        if accel_frame:
            accel_data = accel_frame.as_motion_frame().get_motion_data()
            accel_count += 1
            if accel_count % 100 == 0:
                print(f"  Accel: x={accel_data.x:+8.3f}  y={accel_data.y:+8.3f}  z={accel_data.z:+8.3f} m/s²")

        if gyro_frame:
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()
            gyro_count += 1
            if gyro_count % 100 == 0:
                print(f"  Gyro:  x={gyro_data.x:+8.4f}  y={gyro_data.y:+8.4f}  z={gyro_data.z:+8.4f} rad/s")

    elapsed = time.time() - start_time
    pipeline.stop()

    print(f"\n=== IMU 測試結果 ===")
    print(f"加速度計: {accel_count} 筆 ({accel_count/elapsed:.1f} Hz)")
    print(f"陀螺儀:   {gyro_count} 筆 ({gyro_count/elapsed:.1f} Hz)")
    print(f"\nIMU 測試通過!")

if __name__ == "__main__":
    main()
