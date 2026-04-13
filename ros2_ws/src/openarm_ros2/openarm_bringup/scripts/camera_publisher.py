#!/usr/bin/env python3
"""
RealSense D435i Camera Publisher for ROS2

透過 pyrealsense2 擷取 RealSense D435i 的 RGB + Depth 影像，
並發佈為 CompressedImage 訊息。

Topics:
    /camera/color/compressed - RGB 彩色影像 (sensor_msgs/CompressedImage)
    /camera/depth/compressed - Depth 偽彩圖 (sensor_msgs/CompressedImage)

Author: OpenArm Project
Date: 2025-12-30
Updated: 2026-04-13 — 從 IMX219-83 GStreamer 改為 RealSense D435i pyrealsense2
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import threading
import time


class CameraPublisher(Node):
    """ROS2 節點：發佈 RealSense D435i 的 RGB + Depth 壓縮影像"""

    def __init__(self):
        super().__init__('camera_publisher')

        # 宣告參數
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('enable_color', True)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('use_test_pattern', False)
        self.declare_parameter('align_depth', True)  # 將 depth 對齊到 color

        # 取得參數
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.enable_color = self.get_parameter('enable_color').value
        self.enable_depth = self.get_parameter('enable_depth').value
        self.use_test_pattern = self.get_parameter('use_test_pattern').value
        self.align_depth = self.get_parameter('align_depth').value

        # 建立 QoS Profile - 使用 RELIABLE 以匹配 ros_tcp_endpoint
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 建立 Publishers
        if self.enable_color:
            self.pub_color = self.create_publisher(
                CompressedImage, '/camera/color/compressed', qos_profile
            )
            # 向後相容：同時發佈到舊 left/right topics
            self.pub_left = self.create_publisher(
                CompressedImage, '/camera/left/compressed', qos_profile
            )
            self.pub_right = self.create_publisher(
                CompressedImage, '/camera/right/compressed', qos_profile
            )
            self.get_logger().info(
                'Color publisher: /camera/color/compressed '
                '+ /camera/left/compressed + /camera/right/compressed'
            )

        if self.enable_depth:
            self.pub_depth = self.create_publisher(
                CompressedImage, '/camera/depth/compressed', qos_profile
            )
            self.get_logger().info('Depth publisher: /camera/depth/compressed')

        # RealSense 物件
        self.rs_pipeline = None
        self.rs_align = None
        self.rs_colorizer = None
        self.depth_scale = 0.0

        # 背景擷取執行緒
        self._lock = threading.Lock()
        self._color_frame = None
        self._depth_colormap = None
        self.running = True
        self.frame_count = 0

        # 初始化相機
        if not self.use_test_pattern:
            self._init_realsense()
        else:
            self.get_logger().warn('Using test pattern mode (no physical camera)')

        # 建立定時器發佈影像
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f'RealSense D435i Publisher started: {self.width}x{self.height}@{self.fps}fps, '
            f'JPEG quality={self.jpeg_quality}%'
        )

    def _init_realsense(self) -> None:
        """初始化 RealSense D435i"""
        try:
            import pyrealsense2 as rs
        except ImportError:
            self.get_logger().error(
                'pyrealsense2 not installed. '
                'Install: pip3 install pyrealsense2'
            )
            return

        # 檢查裝置
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            self.get_logger().error('No RealSense device found!')
            return

        dev = devices[0]
        self.get_logger().info(
            f'Found: {dev.get_info(rs.camera_info.name)} '
            f'SN:{dev.get_info(rs.camera_info.serial_number)} '
            f'FW:{dev.get_info(rs.camera_info.firmware_version)}'
        )

        # 設定串流
        self.rs_pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(
            rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps
        )
        if self.enable_depth:
            config.enable_stream(
                rs.stream.depth, self.width, self.height, rs.format.z16, self.fps
            )

        # 啟動 pipeline
        profile = self.rs_pipeline.start(config)

        # 深度比例
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        self.get_logger().info(f'Depth scale: {self.depth_scale:.6f} m/unit')

        # 深度對齊到 color
        if self.align_depth:
            self.rs_align = rs.align(rs.stream.color)

        # 深度上色器
        self.rs_colorizer = rs.colorizer()
        self.rs_colorizer.set_option(rs.option.color_scheme, 0)  # Jet

        # 等待串流穩定
        self.get_logger().info('Waiting for stream to stabilize (30 frames)...')
        for _ in range(30):
            self.rs_pipeline.wait_for_frames()

        # 啟動背景擷取執行緒
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self.get_logger().info('RealSense D435i initialized successfully')

    def _capture_loop(self) -> None:
        """背景執行緒：持續擷取 RealSense 影像"""
        while self.running:
            try:
                frames = self.rs_pipeline.wait_for_frames(1000)
            except Exception:
                continue

            # 深度對齊
            if self.rs_align is not None:
                frames = self.rs_align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame() if self.enable_depth else None

            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            depth_colormap = None
            if depth_frame and self.rs_colorizer is not None:
                depth_colormap = np.asanyarray(
                    self.rs_colorizer.colorize(depth_frame).get_data()
                )

            with self._lock:
                self._color_frame = color_image
                self._depth_colormap = depth_colormap

    def _create_test_frame(self, camera_type: str) -> np.ndarray:
        """建立測試用影像（無實體相機時使用）"""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        if camera_type == 'color':
            frame[:, :, 1] = 150  # 綠色調
            frame[:, :, 2] = 100
        else:
            frame[:, :, 0] = 200  # 藍色調（模擬 Jet depth）
            frame[:, :, 1] = 80

        text = f"RealSense {camera_type.upper()} - Frame: {self.frame_count}"
        cv2.putText(
            frame, text,
            (50, self.height // 2),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
        )

        timestamp = time.strftime("%H:%M:%S")
        cv2.putText(
            frame, timestamp,
            (50, self.height // 2 + 40),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2
        )

        return frame

    def _compress_and_publish(
        self,
        frame: np.ndarray,
        publishers: list,
        frame_id: str,
        stamp_msg
    ) -> None:
        """壓縮影像並發佈到一或多個 publisher"""
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        success, encoded = cv2.imencode('.jpg', frame, encode_param)

        if not success:
            self.get_logger().warn(f'Failed to encode {frame_id} frame')
            return

        msg = CompressedImage()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = frame_id
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()

        for pub in publishers:
            pub.publish(msg)

    def _timer_callback(self) -> None:
        """定時器回調：發佈影像"""
        self.frame_count += 1

        stamp = self.get_clock().now().to_msg()

        if self.use_test_pattern:
            if self.enable_color:
                frame = self._create_test_frame('color')
                self._compress_and_publish(
                    frame, [self.pub_color, self.pub_left, self.pub_right], 'camera_color', stamp
                )
            if self.enable_depth:
                frame = self._create_test_frame('depth')
                self._compress_and_publish(
                    frame, [self.pub_depth], 'camera_depth', stamp
                )
        else:
            with self._lock:
                color = self._color_frame
                depth = self._depth_colormap

            if self.enable_color and color is not None:
                self._compress_and_publish(
                    color, [self.pub_color, self.pub_left, self.pub_right], 'camera_color', stamp
                )

            if self.enable_depth and depth is not None:
                self._compress_and_publish(
                    depth, [self.pub_depth], 'camera_depth', stamp
                )

        # 定期輸出狀態
        if self.frame_count % (self.fps * 10) == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')

    def destroy_node(self) -> None:
        """清理資源"""
        self.running = False

        if self.rs_pipeline is not None:
            self.rs_pipeline.stop()
            self.get_logger().info('RealSense pipeline stopped')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
