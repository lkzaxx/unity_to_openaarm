#!/usr/bin/env python3
"""RealSense D435i 基本測試腳本 - 測試 RGB + Depth 串流"""
import pyrealsense2 as rs
import numpy as np
import sys

def main():
    ctx = rs.context()
    devices = ctx.query_devices()
    print(f"偵測到 {len(devices)} 台 RealSense 裝置")
    
    if len(devices) == 0:
        print("錯誤: 未找到 RealSense 裝置")
        sys.exit(1)
    
    for i, dev in enumerate(devices):
        print(f"\n裝置 {i}:")
        print(f"  名稱: {dev.get_info(rs.camera_info.name)}")
        print(f"  序號: {dev.get_info(rs.camera_info.serial_number)}")
        print(f"  韌體: {dev.get_info(rs.camera_info.firmware_version)}")

    # 測試 Depth + Color 串流
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    print("\n啟動 Depth + Color 串流 (640x480@30fps)...")
    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"深度比例: {depth_scale:.6f} m/unit")

    # 擷取 30 幀來穩定
    print("等待串流穩定 (30 幀)...")
    for _ in range(30):
        pipeline.wait_for_frames()

    # 擷取並分析一幀
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        print("錯誤: 無法取得影像幀")
        pipeline.stop()
        sys.exit(1)

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    print(f"\n=== 測試結果 ===")
    print(f"Color 影像: {color_image.shape} (H x W x C)")
    print(f"Depth 影像: {depth_image.shape} (H x W)")
    print(f"Depth 範圍: {depth_image.min()} ~ {depth_image.max()} (raw)")
    print(f"Depth 範圍: {depth_image.min() * depth_scale:.3f} ~ {depth_image.max() * depth_scale:.3f} m")
    
    center_y, center_x = depth_image.shape[0] // 2, depth_image.shape[1] // 2
    center_depth = depth_image[center_y, center_x] * depth_scale
    print(f"中心點深度: {center_depth:.3f} m")

    pipeline.stop()
    print("\nRealSense D435i 測試通過!")

if __name__ == "__main__":
    main()
