# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
慧靈科技靈巧手 CAN FD 控制模組

通訊規格:
- CAN FD (非 classical CAN)
- 仲裁波特率: 1 Mbps
- 數據波特率: 5 Mbps
- 封包大小: 32 bytes 固定

封包格式:
- Byte1: 馬達選擇 + 讀寫命令
- Byte2: 控制模式
- Byte3~32: 6 顆馬達參數 (每顆 5 bytes)
"""

import socket
import struct
from typing import List, Optional
from dataclasses import dataclass


@dataclass
class MotorParam:
    """單顆馬達參數"""
    position: int = 0    # 0~255
    speed: int = 128     # 0~255
    torque: int = 128    # 0~255


class DexterousHand:
    """
    慧靈科技靈巧手控制類別
    
    使用方式:
        hand = DexterousHand("can0", can_id=0x01)
        hand.home()        # 回零
        hand.open()        # 伸展
        hand.close()       # 收緊
        hand.set_grip(0.5) # 位置控制
        hand.disconnect()
    """
    
    # =========================================================================
    # 控制模式常數
    # =========================================================================
    MODE_POSITION = 0x01   # 位置模式
    MODE_OPEN = 0x02       # 伸展（打開）
    MODE_CLOSE = 0x03      # 收緊（握緊）
    MODE_HOME = 0x04       # 初始化回零
    
    # =========================================================================
    # Byte1 馬達選擇常數
    # =========================================================================
    MASK_ALL_WRITE = 0xFD      # 全選 6 顆馬達，寫入
    MASK_ALL_READ = 0xFE       # 全選 6 顆馬達，讀出
    MASK_THUMB_ROT = 0x05      # 拇指旋轉 (M1)
    MASK_THUMB_EXT = 0x09      # 拇指伸縮 (M2)
    MASK_INDEX = 0x11          # 食指 (M3)
    MASK_MIDDLE = 0x21         # 中指 (M4)
    MASK_RING = 0x41           # 無名指 (M5)
    MASK_PINKY = 0x81          # 尾指 (M6)
    
    # =========================================================================
    # 馬達索引
    # =========================================================================
    MOTOR_THUMB_ROT = 0   # 拇指旋轉
    MOTOR_THUMB_EXT = 1   # 拇指伸縮
    MOTOR_INDEX = 2       # 食指
    MOTOR_MIDDLE = 3      # 中指
    MOTOR_RING = 4        # 無名指
    MOTOR_PINKY = 5       # 尾指
    
    # =========================================================================
    # CAN FD 常數
    # =========================================================================
    CANFD_BRS = 0x01      # Bit Rate Switch flag
    PACKET_SIZE = 32      # 固定封包大小
    
    def __init__(self, can_interface: str, can_id: int = 0x01):
        """
        初始化靈巧手
        
        Args:
            can_interface: CAN 介面名稱 (e.g., "can0", "can1")
            can_id: 靈巧手 CAN ID (待確認，預設 0x01)
        """
        self.can_interface = can_interface
        self.can_id = can_id
        self.sock: Optional[socket.socket] = None
        self._connected = False
        
        self._connect()
    
    def _connect(self):
        """建立 CAN FD Socket 連線"""
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            # 啟用 CAN FD
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            self.sock.bind((self.can_interface,))
            self._connected = True
        except Exception as e:
            self._connected = False
            raise RuntimeError(f"Failed to connect to {self.can_interface}: {e}")
    
    def _build_packet(self, mode: int, positions: List[int], 
                      speed: int = 128, torque: int = 128,
                      motor_mask: int = MASK_ALL_WRITE) -> bytes:
        """
        組裝 32 bytes 封包
        
        Args:
            mode: 控制模式 (0x01~0x04)
            positions: 6 個馬達位置 [0~255]
            speed: 速度 (0~255)
            torque: 力矩 (0~255)
            motor_mask: 馬達選擇 + 讀寫命令
        
        Returns:
            32 bytes 封包
        """
        packet = bytearray(self.PACKET_SIZE)
        packet[0] = motor_mask
        packet[1] = mode
        
        # 填充 6 組馬達參數
        for i in range(6):
            offset = 2 + i * 5
            pos = positions[i] if i < len(positions) else 0
            packet[offset + 0] = pos & 0xFF      # Position
            packet[offset + 1] = speed & 0xFF    # Speed
            packet[offset + 2] = torque & 0xFF   # Torque
            packet[offset + 3] = 0x00            # Reserved
            packet[offset + 4] = 0x00            # Reserved
        
        return bytes(packet)
    
    def _send(self, data: bytes):
        """
        發送 CAN FD 封包
        
        Args:
            data: 要發送的資料 (最多 64 bytes)
        """
        if not self._connected or self.sock is None:
            raise RuntimeError("Not connected to CAN interface")
        
        # CAN FD frame 格式:
        # struct canfd_frame {
        #     canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
        #     __u8    len;     /* frame payload length in bytes */
        #     __u8    flags;   /* additional flags for CAN FD */
        #     __u8    __res0;  /* reserved / padding */
        #     __u8    __res1;  /* reserved / padding */
        #     __u8    data[64];
        # };
        
        # 組裝 CAN FD frame
        can_id = self.can_id
        length = len(data)
        flags = self.CANFD_BRS  # Bit Rate Switch
        
        # struct format: I=uint32, B=uint8, 2x=2 bytes padding, 64s=64 bytes data
        frame = struct.pack("=IBB2x", can_id, length, flags)
        frame += data.ljust(64, b'\x00')
        
        self.sock.send(frame)
    
    def _send_command(self, mode: int, positions: List[int] = None,
                      speed: int = 255, torque: int = 255):
        """
        發送控制命令
        
        Args:
            mode: 控制模式
            positions: 馬達位置列表 (None 時使用預設 0xFF)
            speed: 速度
            torque: 力矩
        """
        if positions is None:
            positions = [0xFF] * 6
        
        packet = self._build_packet(mode, positions, speed, torque)
        self._send(packet)
    
    # =========================================================================
    # 公開 API
    # =========================================================================
    
    def home(self):
        """
        初始化回零
        
        建議在上電後首先執行此命令。
        """
        self._send_command(self.MODE_HOME)
    
    def open(self):
        """伸展（全開）"""
        self._send_command(self.MODE_OPEN)
    
    def close(self):
        """收緊（全握）"""
        self._send_command(self.MODE_CLOSE)
    
    def set_positions(self, positions: List[int], 
                      speed: int = 128, torque: int = 128):
        """
        設定各手指位置（位置模式）
        
        Args:
            positions: [thumb_rot, thumb_ext, index, middle, ring, pinky]
                       每個值範圍 0~255
            speed: 速度 (0~255)
            torque: 力矩 (0~255)
        """
        # 確保有 6 個值
        pos = list(positions)
        while len(pos) < 6:
            pos.append(0)
        
        packet = self._build_packet(self.MODE_POSITION, pos[:6], speed, torque)
        self._send(packet)
    
    def set_grip(self, grip_value: float, speed: int = 128, torque: int = 128):
        """
        簡易握合控制（Unity 用）
        
        將單一開合值映射到 6 指位置。
        
        Args:
            grip_value: 0.0 (全開) ~ 1.0 (全握)
            speed: 速度 (0~255)
            torque: 力矩 (0~255)
        """
        # Clamp 到有效範圍
        grip_value = max(0.0, min(1.0, grip_value))
        
        # 轉換為 0~255
        pos = int(grip_value * 255)
        
        # 映射到 6 指
        # Motor1 (拇指旋轉): 暫時不動，設為 0
        # Motor2~6: 跟隨 grip_value
        positions = [
            0,    # Motor1: 拇指旋轉（暫時不動）
            pos,  # Motor2: 拇指伸縮
            pos,  # Motor3: 食指
            pos,  # Motor4: 中指
            pos,  # Motor5: 無名指
            pos,  # Motor6: 尾指
        ]
        
        self.set_positions(positions, speed, torque)
    
    def set_finger(self, finger_index: int, position: int, 
                   speed: int = 128, torque: int = 128):
        """
        設定單一手指位置
        
        Args:
            finger_index: 手指索引 (0=拇指旋轉, 1=拇指伸縮, 2=食指, ...)
            position: 位置 (0~255)
            speed: 速度 (0~255)
            torque: 力矩 (0~255)
        """
        if finger_index < 0 or finger_index > 5:
            raise ValueError(f"Invalid finger index: {finger_index}")
        
        # 根據手指索引選擇對應的 mask
        masks = [
            self.MASK_THUMB_ROT,
            self.MASK_THUMB_EXT,
            self.MASK_INDEX,
            self.MASK_MIDDLE,
            self.MASK_RING,
            self.MASK_PINKY,
        ]
        
        positions = [0] * 6
        positions[finger_index] = position
        
        packet = self._build_packet(
            self.MODE_POSITION, 
            positions, 
            speed, 
            torque,
            motor_mask=masks[finger_index]
        )
        self._send(packet)
    
    def disconnect(self):
        """關閉連線"""
        if self.sock:
            self.sock.close()
            self.sock = None
        self._connected = False
    
    @property
    def is_connected(self) -> bool:
        """檢查是否已連線"""
        return self._connected
    
    def __del__(self):
        """解構時關閉連線"""
        self.disconnect()
    
    def __enter__(self):
        """Context manager 支援"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager 支援"""
        self.disconnect()
        return False
