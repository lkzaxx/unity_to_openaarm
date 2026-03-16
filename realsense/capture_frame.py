#!/usr/bin/env python3
"""擷取一幀 RGB + Depth 影像並儲存為 PNG/NPY"""
import pyrealsense2 as rs
import numpy as np
import os
import sys

def main():
    save_dir = os.path.dirname(os.path.abspath(__file__))

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    align = rs.align(rs.stream.color)

    print("啟動串流...")
    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # 穩定串流
    for _ in range(30):
        pipeline.wait_for_frames()

    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)

    depth_frame = aligned.get_depth_frame()
    color_frame = aligned.get_color_frame()

    if not depth_frame or not color_frame:
        print("錯誤: 無法取得影像幀")
        pipeline.stop()
        sys.exit(1)

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # 儲存
    try:
        import cv2
        cv2.imwrite(os.path.join(save_dir, "color.png"), color_image)
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )
        cv2.imwrite(os.path.join(save_dir, "depth_colormap.png"), depth_colormap)
        print(f"已儲存 color.png 和 depth_colormap.png")
    except ImportError:
        print("OpenCV 未安裝，跳過 PNG 儲存")

    np.save(os.path.join(save_dir, "depth_raw.npy"), depth_image)
    np.save(os.path.join(save_dir, "color_raw.npy"), color_image)
    print(f"已儲存 depth_raw.npy 和 color_raw.npy")
    print(f"深度比例: {depth_scale} m/unit")
    print(f"Color: {color_image.shape}, Depth: {depth_image.shape}")

    # 取得相機內參
    intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
    print(f"\n相機內參:")
    print(f"  fx={intrinsics.fx:.2f}, fy={intrinsics.fy:.2f}")
    print(f"  cx={intrinsics.ppx:.2f}, cy={intrinsics.ppy:.2f}")
    print(f"  寬={intrinsics.width}, 高={intrinsics.height}")

    pipeline.stop()
    print("\n擷取完成!")

if __name__ == "__main__":
    main()
