#!/usr/bin/env python3
"""
IMX219-83 Stereo Camera Publisher for ROS2

透過 GStreamer 擷取 IMX219-83 Stereo 相機的影像，
並發佈為 CompressedImage 訊息供 Unity 接收。

Topics:
    /camera/left/compressed  - 左眼影像 (sensor_msgs/CompressedImage)
    /camera/right/compressed - 右眼影像 (sensor_msgs/CompressedImage)

Author: OpenArm Project
Date: 2025-12-30
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from typing import Optional
import threading
import time


class CameraPublisher(Node):
    """ROS2 節點：發佈 IMX219-83 Stereo 相機的壓縮影像"""

    def __init__(self):
        super().__init__('camera_publisher')

        # 宣告參數
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('enable_left', True)
        self.declare_parameter('enable_right', True)
        self.declare_parameter('use_test_pattern', False)  # 用於沒有實體相機時的測試

        # 取得參數
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.enable_left = self.get_parameter('enable_left').value
        self.enable_right = self.get_parameter('enable_right').value
        self.use_test_pattern = self.get_parameter('use_test_pattern').value

        # 建立 QoS Profile - 使用 RELIABLE 以匹配 ros_tcp_endpoint
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 建立 Publishers
        if self.enable_left:
            self.pub_left = self.create_publisher(
                CompressedImage, '/camera/left/compressed', qos_profile
            )
            self.get_logger().info('Left camera publisher created (RELIABLE QoS)')

        if self.enable_right:
            self.pub_right = self.create_publisher(
                CompressedImage, '/camera/right/compressed', qos_profile
            )
            self.get_logger().info('Right camera publisher created (RELIABLE QoS)')

        # 相機擷取物件
        self.cap_left: Optional[cv2.VideoCapture] = None
        self.cap_right: Optional[cv2.VideoCapture] = None

        # 執行緒控制
        self.running = True
        self.frame_count = 0

        # 初始化相機
        if not self.use_test_pattern:
            self._init_cameras()
        else:
            self.get_logger().warn('Using test pattern mode (no physical camera)')

        # 建立定時器發佈影像
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f'Camera Publisher started: {self.width}x{self.height}@{self.fps}fps, '
            f'JPEG quality={self.jpeg_quality}%'
        )

    def _build_gstreamer_pipeline(self, sensor_id: int) -> str:
        """
        建立 GStreamer pipeline 字串

        Args:
            sensor_id: 相機 sensor ID (0=左, 1=右)

        Returns:
            GStreamer pipeline 字串
        """
        pipeline = (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM),width={self.width},height={self.height},"
            f"framerate={self.fps}/1 ! "
            "nvvidconv ! "
            "video/x-raw,format=BGRx ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=1"
        )
        return pipeline

    def _init_cameras(self) -> None:
        """初始化相機擷取"""
        if self.enable_left:
            pipeline_left = self._build_gstreamer_pipeline(0)
            self.cap_left = cv2.VideoCapture(pipeline_left, cv2.CAP_GSTREAMER)
            if not self.cap_left.isOpened():
                self.get_logger().error('Failed to open left camera (sensor_id=0)')
                self.cap_left = None
            else:
                self.get_logger().info('Left camera initialized successfully')

        if self.enable_right:
            pipeline_right = self._build_gstreamer_pipeline(1)
            self.cap_right = cv2.VideoCapture(pipeline_right, cv2.CAP_GSTREAMER)
            if not self.cap_right.isOpened():
                self.get_logger().error('Failed to open right camera (sensor_id=1)')
                self.cap_right = None
            else:
                self.get_logger().info('Right camera initialized successfully')

    def _create_test_frame(self, camera_side: str) -> np.ndarray:
        """
        建立測試用影像（無實體相機時使用）

        Args:
            camera_side: 'left' 或 'right'

        Returns:
            測試影像 (numpy array)
        """
        # 建立漸層背景
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # 左眼藍色系，右眼紅色系
        if camera_side == 'left':
            frame[:, :, 0] = 200  # Blue channel
            frame[:, :, 1] = 100
        else:
            frame[:, :, 2] = 200  # Red channel
            frame[:, :, 1] = 100

        # 添加文字資訊
        text = f"{camera_side.upper()} CAM - Frame: {self.frame_count}"
        cv2.putText(
            frame, text,
            (50, self.height // 2),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2
        )

        # 添加時間戳
        timestamp = time.strftime("%H:%M:%S")
        cv2.putText(
            frame, timestamp,
            (50, self.height // 2 + 50),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
        )

        return frame

    def _compress_and_publish(
        self,
        frame: np.ndarray,
        publisher,
        camera_side: str
    ) -> None:
        """
        壓縮影像並發佈

        Args:
            frame: 原始影像
            publisher: ROS2 publisher
            camera_side: 'left' 或 'right'（用於 log）
        """
        # JPEG 壓縮
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        success, encoded = cv2.imencode('.jpg', frame, encode_param)

        if not success:
            self.get_logger().warn(f'Failed to encode {camera_side} frame')
            return

        # 建立並發佈訊息
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'camera_{camera_side}'
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()

        publisher.publish(msg)

    def _timer_callback(self) -> None:
        """定時器回調：擷取並發佈影像"""
        self.frame_count += 1

        # 處理左眼相機
        if self.enable_left:
            if self.use_test_pattern:
                frame_left = self._create_test_frame('left')
                self._compress_and_publish(frame_left, self.pub_left, 'left')
            elif self.cap_left is not None:
                ret, frame = self.cap_left.read()
                if ret:
                    self._compress_and_publish(frame, self.pub_left, 'left')
                else:
                    self.get_logger().warn('Failed to read left camera frame', throttle_duration_sec=5.0)

        # 處理右眼相機
        if self.enable_right:
            if self.use_test_pattern:
                frame_right = self._create_test_frame('right')
                self._compress_and_publish(frame_right, self.pub_right, 'right')
            elif self.cap_right is not None:
                ret, frame = self.cap_right.read()
                if ret:
                    self._compress_and_publish(frame, self.pub_right, 'right')
                else:
                    self.get_logger().warn('Failed to read right camera frame', throttle_duration_sec=5.0)

        # 定期輸出狀態
        if self.frame_count % (self.fps * 10) == 0:  # 每 10 秒一次
            self.get_logger().info(f'Published {self.frame_count} frames')

    def destroy_node(self) -> None:
        """清理資源"""
        self.running = False

        if self.cap_left is not None:
            self.cap_left.release()
            self.get_logger().info('Left camera released')

        if self.cap_right is not None:
            self.cap_right.release()
            self.get_logger().info('Right camera released')

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
