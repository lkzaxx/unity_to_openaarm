#!/usr/bin/env python3
"""
USBCANFD-200U CAN ID 掃描工具 (Listen-only 模式)

使用 ZLGCAN 動態庫進行 CAN FD 監聽，掃描總線上的 CAN ID。

安裝步驟:
    1. 將 libcontrolcanfd.so 複製到 /usr/local/lib/
       sudo cp libcontrolcanfd.so /usr/local/lib/
       sudo ldconfig

使用方法:
    sudo python3 usbcanfd_scan.py [--duration SECONDS]

注意: 需要 root 權限存取 USB 設備

刪除此檔案時請同步更新 dexterous_hand_guide.md
"""

import argparse
import os
import sys
import time
from ctypes import (
    POINTER,
    Structure,
    Union,
    byref,
    c_byte,
    c_ubyte,
    c_uint,
    c_uint16,
    c_uint32,
    c_uint64,
    c_ulong,
    c_void_p,
    cdll,
    sizeof,
)
from typing import Optional, Set

# -----------------------------------------------------------------------------
# 常量定義
# -----------------------------------------------------------------------------
DEVICE_TYPE_USBCANFD_200U = 41  # USBCANFD-200U 設備類型
INVALID_DEVICE_HANDLE = 0       # 無效設備句柄

TYPE_CAN = 0
TYPE_CANFD = 1

# CAN 模式
MODE_NORMAL = 0      # 正常模式
MODE_LISTEN_ONLY = 1 # 只聽模式 (被動監聽，不發送 ACK)


# -----------------------------------------------------------------------------
# 結構體定義 (參考 ZLGCAN SDK - controlcan.h)
# -----------------------------------------------------------------------------

class ZCAN_CHANNEL_CAN_INIT_CONFIG(Structure):
    """標準 CAN 配置"""
    _fields_ = [
        ("acc_code", c_uint),
        ("acc_mask", c_uint),
        ("reserved", c_uint),
        ("filter", c_ubyte),
        ("timing0", c_ubyte),
        ("timing1", c_ubyte),
        ("mode", c_ubyte),
    ]


class ZCAN_CHANNEL_CANFD_INIT_CONFIG(Structure):
    """CAN FD 配置"""
    _fields_ = [
        ("acc_code", c_uint),
        ("acc_mask", c_uint),
        ("abit_timing", c_uint),
        ("dbit_timing", c_uint),
        ("brp", c_uint),
        ("filter", c_ubyte),
        ("mode", c_ubyte),
        ("pad", c_uint16),
        ("reserved", c_uint),
    ]


class ZCAN_CHANNEL_INIT_CONFIG_UNION(Union):
    """配置聯合體"""
    _fields_ = [
        ("can", ZCAN_CHANNEL_CAN_INIT_CONFIG),
        ("canfd", ZCAN_CHANNEL_CANFD_INIT_CONFIG),
    ]


class ZCAN_CHANNEL_INIT_CONFIG(Structure):
    """通道初始化配置"""
    _fields_ = [
        ("can_type", c_uint),
        ("config", ZCAN_CHANNEL_INIT_CONFIG_UNION),
    ]


class ZCAN_CHANNEL_ERR_INFO(Structure):
    """錯誤資訊"""
    _fields_ = [
        ("error_code", c_uint),
        ("passive_ErrData", c_ubyte * 3),
        ("arLost_ErrData", c_ubyte),
    ]


class ZCAN_CHANNEL_STATUS(Structure):
    """通道狀態"""
    _fields_ = [
        ("errInterrupt", c_ubyte),
        ("regMode", c_ubyte),
        ("regStatus", c_ubyte),
        ("regALCapture", c_ubyte),
        ("regECCapture", c_ubyte),
        ("regEWLimit", c_ubyte),
        ("regRECounter", c_ubyte),
        ("regTECounter", c_ubyte),
        ("Reserved", c_uint),
    ]


class ZCAN_CAN_FRAME(Structure):
    """標準 CAN 訊框"""
    _fields_ = [
        ("can_id", c_uint),
        ("can_dlc", c_ubyte),
        ("__pad", c_ubyte),
        ("__res0", c_ubyte),
        ("__res1", c_ubyte),
        ("data", c_ubyte * 8),
    ]


class ZCAN_CANFD_FRAME(Structure):
    """CAN FD 訊框"""
    _fields_ = [
        ("can_id", c_uint),
        ("len", c_ubyte),
        ("brs", c_ubyte),
        ("__res0", c_ubyte),
        ("__res1", c_ubyte),
        ("data", c_ubyte * 64),
    ]


class ZCAN_Receive_Data(Structure):  # 修正: frame 在前
    """標準 CAN 接收數據"""
    _fields_ = [
        ("timestamp", c_uint64),
        ("frame", ZCAN_CAN_FRAME),
    ]


class ZCAN_ReceiveFD_Data(Structure):
    """CAN FD 接收數據 (修正: frame 在前, timestamp 在後)"""
    _fields_ = [
        ("frame", ZCAN_CANFD_FRAME),
        ("timestamp", c_uint64),
    ]


class ZCAN_Transmit_Data(Structure):
    """標準 CAN 發送數據"""
    _fields_ = [
        ("frame", ZCAN_CAN_FRAME),
        ("transmit_type", c_uint),
    ]


class ZCAN_TransmitFD_Data(Structure):
    """CAN FD 發送數據"""
    _fields_ = [
        ("frame", ZCAN_CANFD_FRAME),
        ("transmit_type", c_uint),
    ]


# -----------------------------------------------------------------------------
# ZLGCAN API 封裝
# -----------------------------------------------------------------------------

class ZLGCAN:
    """ZLGCAN API 封裝類"""
    
    def __init__(self, lib_path: Optional[str] = None):
        self.dll = self._load_library(lib_path)
        self._setup_functions()
        self.device_handle = None
        self.channel_handle = None
    
    def _load_library(self, lib_path: Optional[str]) -> object:
        """載入動態庫"""
        lib_paths = [
            lib_path,
            "./libcontrolcanfd.so",
            "/usr/local/lib/libcontrolcanfd.so",
            "/usr/lib/libcontrolcanfd.so",
        ]
        
        for path in lib_paths:
            if path is None:
                continue
            try:
                dll = cdll.LoadLibrary(path)
                print(f"[INFO] 已載入動態庫: {path}")
                return dll
            except OSError as e:
                continue
        
        print("[ERROR] 無法載入 libcontrolcanfd.so")
        print("請確認動態庫已安裝:")
        print("  sudo cp libcontrolcanfd.so /usr/local/lib/")
        print("  sudo ldconfig")
        sys.exit(1)
    
    def _setup_functions(self):
        """設置函數返回類型和參數類型"""
        # ZCAN_OpenDevice
        self.dll.ZCAN_OpenDevice.argtypes = [c_uint, c_uint, c_uint]
        self.dll.ZCAN_OpenDevice.restype = c_void_p
        
        # ZCAN_CloseDevice
        self.dll.ZCAN_CloseDevice.argtypes = [c_void_p]
        self.dll.ZCAN_CloseDevice.restype = c_uint
        
        # ZCAN_InitCAN
        self.dll.ZCAN_InitCAN.argtypes = [c_void_p, c_uint, POINTER(ZCAN_CHANNEL_INIT_CONFIG)]
        self.dll.ZCAN_InitCAN.restype = c_void_p
        
        # ZCAN_StartCAN
        self.dll.ZCAN_StartCAN.argtypes = [c_void_p]
        self.dll.ZCAN_StartCAN.restype = c_uint
        
        # ZCAN_GetReceiveNum
        self.dll.ZCAN_GetReceiveNum.argtypes = [c_void_p, c_ubyte]
        self.dll.ZCAN_GetReceiveNum.restype = c_uint
        
        # ZCAN_Receive
        self.dll.ZCAN_Receive.argtypes = [c_void_p, POINTER(ZCAN_Receive_Data), c_uint, c_int]
        self.dll.ZCAN_Receive.restype = c_uint
        
        # ZCAN_ReceiveFD
        self.dll.ZCAN_ReceiveFD.argtypes = [c_void_p, POINTER(ZCAN_ReceiveFD_Data), c_uint, c_int]
        self.dll.ZCAN_ReceiveFD.restype = c_uint
        
        # 波特率設置函數
        try:
            self.dll.ZCAN_SetAbitBaud.argtypes = [c_void_p, c_uint, c_uint]
            self.dll.ZCAN_SetAbitBaud.restype = c_uint
            self.dll.ZCAN_SetDbitBaud.argtypes = [c_void_p, c_uint, c_uint]
            self.dll.ZCAN_SetDbitBaud.restype = c_uint
            self._has_baud_funcs = True
        except AttributeError:
            self._has_baud_funcs = False
            print("[WARN] 波特率設置函數不可用，將使用預設值")
    
    def open_device(self, device_type: int = DEVICE_TYPE_USBCANFD_200U, 
                    device_index: int = 0) -> bool:
        """開啟設備"""
        print(f"[INFO] 開啟設備 (type={device_type}, index={device_index})...")
        
        self.device_handle = self.dll.ZCAN_OpenDevice(device_type, device_index, 0)
        
        # 檢查無效句柄 (NULL 或 -1)
        if self.device_handle is None or self.device_handle == 0:
            print("[ERROR] 無法開啟設備 (handle=NULL)")
            return False
        
        # 將 void* 轉為整數檢查
        handle_int = self.device_handle if isinstance(self.device_handle, int) else \
                     c_void_p(self.device_handle).value or 0
        
        if handle_int == 0 or handle_int == 0xFFFFFFFF or handle_int == -1:
            print(f"[ERROR] 無法開啟設備 (handle=0x{handle_int:X})")
            return False
        
        print(f"[INFO] 設備已開啟 (handle=0x{handle_int:X})")
        return True
    
    def set_baudrate(self, channel: int = 0, 
                     abit_baud: int = 1000000, 
                     dbit_baud: int = 5000000) -> bool:
        """設置波特率"""
        if not self._has_baud_funcs:
            return True
        
        print(f"[INFO] 設定波特率: {abit_baud//1000}kbps (仲裁) / {dbit_baud//1000}kbps (數據)")
        
        ret1 = self.dll.ZCAN_SetAbitBaud(self.device_handle, channel, abit_baud)
        ret2 = self.dll.ZCAN_SetDbitBaud(self.device_handle, channel, dbit_baud)
        
        if ret1 != 1 or ret2 != 1:
            print(f"[WARN] 波特率設置返回: abit={ret1}, dbit={ret2}")
        
        return True
    
    def init_channel(self, channel: int = 0, mode: int = MODE_LISTEN_ONLY) -> bool:
        """初始化通道"""
        mode_str = "Listen-only" if mode == MODE_LISTEN_ONLY else "Normal"
        print(f"[INFO] 初始化通道 {channel} ({mode_str} 模式)...")
        
        config = ZCAN_CHANNEL_INIT_CONFIG()
        config.can_type = 1  # CAN FD
        config.config.canfd.mode = mode
        config.config.canfd.filter = 0
        config.config.canfd.acc_code = 0
        config.config.canfd.acc_mask = 0xFFFFFFFF  # 接收所有 ID
        
        self.channel_handle = self.dll.ZCAN_InitCAN(self.device_handle, channel, byref(config))
        
        if self.channel_handle is None:
            print("[ERROR] 初始化通道失敗 (handle=NULL)")
            return False
        
        handle_int = self.channel_handle if isinstance(self.channel_handle, int) else \
                     c_void_p(self.channel_handle).value or 0
        
        if handle_int == 0 or handle_int == 0xFFFFFFFF:
            print(f"[ERROR] 初始化通道失敗 (handle=0x{handle_int:X})")
            return False
        
        print(f"[INFO] 通道已初始化 (handle=0x{handle_int:X})")
        return True
    
    def start_can(self) -> bool:
        """啟動 CAN"""
        ret = self.dll.ZCAN_StartCAN(self.channel_handle)
        if ret != 1:
            print(f"[WARN] StartCAN 返回: {ret}")
        return ret == 1
    
    def get_receive_num(self, msg_type: int = TYPE_CANFD) -> int:
        """獲取接收緩衝區中的訊息數量"""
        return self.dll.ZCAN_GetReceiveNum(self.channel_handle, msg_type)
    
    def receive(self, count: int = 100, timeout_ms: int = 100) -> list:
        """接收標準 CAN 訊息"""
        msgs = (ZCAN_Receive_Data * count)()
        received = self.dll.ZCAN_Receive(self.channel_handle, msgs, count, timeout_ms)
        return [msgs[i] for i in range(received)]
    
    def receive_fd(self, count: int = 100, timeout_ms: int = 100) -> list:
        """接收 CAN FD 訊息"""
        msgs = (ZCAN_ReceiveFD_Data * count)()
        received = self.dll.ZCAN_ReceiveFD(self.channel_handle, msgs, count, timeout_ms)
        return [msgs[i] for i in range(received)]
    
    def transmit_fd(self, can_id: int, data: bytes, timeout_ms: int = 100) -> bool:
        """發送 CAN FD 訊息"""
        # 設置發送函數類型
        try:
            self.dll.ZCAN_TransmitFD.argtypes = [c_void_p, POINTER(ZCAN_TransmitFD_Data), c_uint]
            self.dll.ZCAN_TransmitFD.restype = c_uint
        except:
            pass
        
        msg = ZCAN_TransmitFD_Data()
        msg.frame.can_id = can_id
        msg.frame.len = len(data)
        msg.frame.brs = 1  # Bit rate switch
        msg.transmit_type = 0  # 正常發送
        
        for i, b in enumerate(data):
            msg.frame.data[i] = b
        
        ret = self.dll.ZCAN_TransmitFD(self.channel_handle, byref(msg), 1)
        return ret == 1
    
    def close(self):
        """關閉設備"""
        if self.device_handle:
            print("[INFO] 關閉設備...")
            self.dll.ZCAN_CloseDevice(self.device_handle)
            self.device_handle = None
            self.channel_handle = None


# 添加缺少的 c_int 導入
from ctypes import c_int

# -----------------------------------------------------------------------------
# 主程式
# -----------------------------------------------------------------------------

def scan_can_ids(duration: float = 10.0) -> None:
    """
    使用 Listen-only 模式掃描 CAN 總線上的 ID
    
    Args:
        duration: 掃描持續時間 (秒)
    """
    # 檢查 root 權限
    if os.geteuid() != 0:
        print("[WARN] 建議使用 sudo 執行以獲得 USB 設備存取權限")
    
    can = ZLGCAN()
    
    try:
        # 開啟設備
        if not can.open_device():
            return
        
        # 設置波特率
        can.set_baudrate(0, 1000000, 5000000)
        
        # 初始化通道 (Listen-only)
        if not can.init_channel(0, MODE_LISTEN_ONLY):
            return
        
        # 啟動
        can.start_can()
        
        print(f"\n[INFO] 開始掃描 CAN ID ({duration} 秒)...")
        print("-" * 60)
        
        # 掃描
        found_ids: Set[int] = set()
        found_fd_ids: Set[int] = set()
        start_time = time.time()
        msg_count = 0
        
        while time.time() - start_time < duration:
            # 標準 CAN
            num_can = can.get_receive_num(TYPE_CAN)
            if num_can > 0:
                msgs = can.receive(min(num_can, 100))
                for msg in msgs:
                    can_id = msg.frame.can_id & 0x1FFFFFFF
                    msg_count += 1
                    if can_id not in found_ids:
                        found_ids.add(can_id)
                        data_hex = " ".join(f"{msg.frame.data[j]:02X}" 
                                           for j in range(msg.frame.can_dlc))
                        print(f"[CAN] ID=0x{can_id:03X} LEN={msg.frame.can_dlc} DATA={data_hex}")
            
            # CAN FD
            num_fd = can.get_receive_num(TYPE_CANFD)
            if num_fd > 0:
                msgs = can.receive_fd(min(num_fd, 100))
                for msg in msgs:
                    can_id = msg.frame.can_id & 0x1FFFFFFF
                    msg_count += 1
                    if can_id not in found_fd_ids:
                        found_fd_ids.add(can_id)
                        data_len = msg.frame.len
                        data_hex = " ".join(f"{msg.frame.data[j]:02X}" 
                                           for j in range(min(data_len, 16)))
                        if data_len > 16:
                            data_hex += " ..."
                        print(f"[CAN FD] ID=0x{can_id:03X} LEN={data_len} DATA={data_hex}")
            
            time.sleep(0.01)
        
        # 統計
        print("-" * 60)
        print(f"[統計] 掃描完成，共接收 {msg_count} 條訊息")
        print(f"  CAN: {len(found_ids)} 個 ID - {', '.join(f'0x{i:03X}' for i in sorted(found_ids)) or 'None'}")
        print(f"  CAN FD: {len(found_fd_ids)} 個 ID - {', '.join(f'0x{i:03X}' for i in sorted(found_fd_ids)) or 'None'}")
        
        expected = {0x11, 0x12}
        found = found_ids | found_fd_ids
        if expected & found:
            print(f"\n[✓] 檢測到靈巧手: {', '.join(f'0x{i:02X}' for i in sorted(expected & found))}")
        elif msg_count == 0:
            print("\n[!] 未接收到任何訊息，請檢查:")
            print("    1. CAN 總線上是否有其他設備在發送")
            print("    2. 波特率是否匹配 (1Mbps/5Mbps)")
            print("    3. CAN_H/CAN_L 接線是否正確")
        else:
            print("\n[!] 未檢測到靈巧手 (0x11=右手, 0x12=左手)")
    
    finally:
        can.close()
        print("[INFO] 完成")


def test_dexterous_hand(can_id: int = 0x12) -> None:
    """
    測試靈巧手通訊 - 發送回零命令並等待回應
    
    Args:
        can_id: 靈巧手 CAN ID (0x11=右手, 0x12=左手)
    """
    if os.geteuid() != 0:
        print("[WARN] 建議使用 sudo 執行")
    
    can = ZLGCAN()
    
    try:
        if not can.open_device():
            return
        
        can.set_baudrate(0, 1000000, 5000000)
        
        # 使用正常模式 (需要發送)
        if not can.init_channel(0, MODE_NORMAL):
            return
        
        can.start_can()
        
        # 構建 32 bytes 回零命令
        # Byte1=0xFD (全選寫入), Byte2=0x04 (回零)
        cmd = bytes([0xFD, 0x04] + [0xFF] * 30)  # 特殊命令用 0xFF
        
        print(f"\n[發送] 靈巧手回零命令 -> ID=0x{can_id:02X}")
        print(f"  DATA: {' '.join(f'{b:02X}' for b in cmd[:16])} ...")
        
        success = can.transmit_fd(can_id, cmd)
        if success:
            print("[OK] 命令已發送")
        else:
            print("[FAIL] 發送失敗")
            return
        
        # 等待回應
        print("\n[接收] 等待回應 (3 秒)...")
        start = time.time()
        received = False
        
        while time.time() - start < 3.0:
            num = can.get_receive_num(TYPE_CANFD)
            if num > 0:
                msgs = can.receive_fd(num)
                for msg in msgs:
                    # 打印原始數據
                    print(f"  <- 收到 CAN FD 數據")
                    received = True
            
            num = can.get_receive_num(TYPE_CAN)
            if num > 0:
                msgs = can.receive(num)
                for msg in msgs:
                    print(f"  <- 收到 CAN 數據")
                    received = True
            
            time.sleep(0.01)
        
        if not received:
            print("\n[!] 未收到回應，請檢查:")
            print("    1. 靈巧手是否已通電 (24V)")
            print("    2. CAN_H/CAN_L 接線是否正確")
            print("    3. CAN ID 是否正確 (右手=0x11, 左手=0x12)")
        else:
            print("\n[✓] 通訊成功!")
    
    finally:
        can.close()
        print("[INFO] 完成")


def demo_gestures(can_id: int = 0x12) -> None:
    """
    靈巧手動作演示 - 開合 + 讚
    
    Args:
        can_id: 靈巧手 CAN ID (0x11=右手, 0x12=左手)
    """
    if os.geteuid() != 0:
        print("[WARN] 建議使用 sudo 執行")
    
    can = ZLGCAN()
    
    # 靈巧手命令定義 (32 bytes)
    # Byte1: 馬達選擇 (0xFD=全選寫入)
    # Byte2: 控制模式 (0x01=位置, 0x02=開, 0x03=合, 0x04=回零)
    # Byte3-32: 6組馬達參數 (各5 bytes: Position, Speed, Torque, Reserved×2)
    
    def build_cmd(mode: int, motor_positions: list = None, speed: int = 200, torque: int = 200) -> bytes:
        """構建 32 bytes 命令"""
        if motor_positions is None:
            # 特殊命令（0x02=張開, 0x03=握拳, 0x04=回零）用 0xFF 填充
            return bytes([0xFD, mode] + [0xFF] * 30)
        else:
            # 位置模式（0x01）：Reserved 填 0x00
            data = [0xFD, 0x01]
            for pos in motor_positions:
                # 每個馬達: Position(1), Speed(1), Torque(1), Reserved(2)
                data.extend([pos, speed, torque, 0x00, 0x00])  # Reserved 填 0x00
            while len(data) < 32:
                data.append(0x00)
            return bytes(data[:32])
    
    # 預定義命令
    CMD_HOME = build_cmd(0x04)       # 回零
    CMD_OPEN = build_cmd(0x02)       # 張開
    CMD_CLOSE = build_cmd(0x03)      # 握緊
    
    # 讚的手勢: 拇指伸出，其他握緊
    # M1=拇指旋轉, M2=拇指伸縮, M3=食指, M4=中指, M5=無名指, M6=尾指
    CMD_THUMBS_UP = build_cmd(0x01, [0, 0, 255, 255, 255, 255], speed=200, torque=200)
    
    try:
        if not can.open_device():
            return
        
        can.set_baudrate(0, 1000000, 5000000)
        
        if not can.init_channel(0, MODE_NORMAL):
            return
        
        can.start_can()
        
        hand_name = "左手" if can_id == 0x12 else "右手"
        print(f"\n{'='*50}")
        print(f"   靈巧手動作演示 ({hand_name} ID=0x{can_id:02X})")
        print(f"{'='*50}\n")
        
        def send_and_wait(name: str, cmd: bytes, wait: float = 1.5):
            """發送命令並等待"""
            print(f"[動作] {name}")
            success = can.transmit_fd(can_id, cmd)
            if not success:
                print("  [!] 發送失敗")
                return False
            print(f"  等待 {wait} 秒...")
            time.sleep(wait)
            return True
        
        # 動作序列（只使用基本命令：回零/張開/握緊，避免位置模式導致卡住）
        actions = [
            ("1. 回零 (初始化)", CMD_HOME, 2.0),
            ("2. 張開", CMD_OPEN, 1.5),
            ("3. 握緊", CMD_CLOSE, 1.5),
            ("4. 張開", CMD_OPEN, 1.5),
            ("5. 握緊", CMD_CLOSE, 1.5),
            ("6. 張開", CMD_OPEN, 1.5),
            ("7. 握緊", CMD_CLOSE, 1.5),
            ("8. 張開 (結束)", CMD_OPEN, 1.0),
        ]
        
        for name, cmd, wait in actions:
            if not send_and_wait(name, cmd, wait):
                break
            print()
        
        print(f"{'='*50}")
        print(f"   演示完成！")
        print(f"{'='*50}")
    
    finally:
        can.close()
        print("[INFO] 完成")


def interactive_menu(can_id: int = 0x12) -> None:
    """
    靈巧手互動式控制選單
    
    Args:
        can_id: 靈巧手 CAN ID (0x11=右手, 0x12=左手)
    """
    if os.geteuid() != 0:
        print("[WARN] 建議使用 sudo 執行")
    
    can = ZLGCAN()
    
    def build_cmd(mode: int, motor_positions: list = None, speed: int = 200, torque: int = 200) -> bytes:
        if motor_positions is None:
            # 特殊命令（0x02=張開, 0x03=握拳, 0x04=回零）用 0xFF 填充
            return bytes([0xFD, mode] + [0xFF] * 30)
        else:
            # 位置模式（0x01）：Reserved 填 0x00
            data = [0xFD, 0x01]
            for pos in motor_positions:
                data.extend([pos, speed, torque, 0x00, 0x00])  # Reserved 填 0x00
            while len(data) < 32:
                data.append(0x00)
            return bytes(data[:32])
    
    # M1=拇指旋轉, M2=拇指伸縮, M3=食指, M4=中指, M5=無名指, M6=尾指
    # 0=伸直, 255=彎曲
    GESTURES = {
        "disable": (build_cmd(0x00), "禁用 ⛔"),  # 釋放電機
        "home": (build_cmd(0x04), "回零"),
        "open": (build_cmd(0x02), "張開 ✋"),
        "close": (build_cmd(0x03), "握緊 ✊"),
        "thumbs_up": (build_cmd(0x01, [0, 0, 255, 255, 255, 255]), "讚 👍"),
        "peace": (build_cmd(0x01, [128, 255, 0, 0, 255, 255]), "比YA ✌️"),
        "ok": (build_cmd(0x01, [100, 200, 200, 0, 0, 0]), "OK 👌"),
        "point": (build_cmd(0x01, [128, 255, 0, 255, 255, 255]), "指 👆"),
        "rock": (build_cmd(0x01, [0, 0, 0, 255, 255, 0]), "搖滾 🤘"),
    }
    
    try:
        if not can.open_device():
            return
        
        can.set_baudrate(0, 1000000, 5000000)
        
        if not can.init_channel(0, MODE_NORMAL):
            return
        
        can.start_can()
        
        hand_name = "左手" if can_id == 0x12 else "右手"
        
        # 連線測試：發送回零命令（跳過讀取檢查，因為 USB CANFD 接收功能可能不正常）
        print(f"\n[初始化] 發送回零命令到 {hand_name} (ID=0x{can_id:02X})...")
        CMD_HOME = bytes([0xFD, 0x04] + [0xFF] * 30)  # 特殊命令用 0xFF
        can.transmit_fd(can_id, CMD_HOME)
        time.sleep(2)
        print(f"[初始化] 完成！如果手沒有回零，請檢查電源和接線。")
        
        def send_gesture(key: str, wait: float = 0.3) -> bool:
            if key not in GESTURES:
                return False, "未知手勢"
            cmd, name = GESTURES[key]
            success = can.transmit_fd(can_id, cmd)
            if not success:
                return False, f"{name} - 發送失敗"
            time.sleep(wait)
            return True, name
        
        def clear_screen():
            """清除螢幕"""
            os.system('clear' if os.name != 'nt' else 'cls')
        
        def show_menu(last_action: str = ""):
            clear_screen()
            print("=" * 55)
            print(f"   USBCANFD 靈巧手控制器 ({hand_name} ID=0x{can_id:02X})")
            print("=" * 55)
            print()
            print("  0. 禁用 ⛔  1. 回零    2. 張開 ✋   3. 握緊 ✊")
            print()
            print("  4. 讚 👍   5. 比YA ✌️   6. OK 👌")
            print("  7. 指 👆   8. 搖滾 🤘")
            print()
            print("  r. 重置(禁用+回零)  f. 單指連續  t. 拇指測試")
            print("  d. 自動演示         m. 選單      q. 離開")
            print("-" * 55)
            if last_action:
                print(f"  上次: {last_action}")
                print("-" * 55)
        
        def run_demo():
            clear_screen()
            print("[自動演示] 開始...\n")
            demo_actions = [
                ("home", 2.0), ("open", 1.2), ("close", 1.2), ("open", 1.0),
                ("thumbs_up", 1.5), ("open", 1.0),
                ("peace", 1.5), ("open", 1.0),
                ("ok", 1.5), ("open", 1.0),
                ("rock", 1.5), ("open", 1.0),
            ]
            for i, (gesture, wait) in enumerate(demo_actions, 1):
                _, name = send_gesture(gesture, wait)
                print(f"  {i}/{len(demo_actions)} {name}")
            print("\n[自動演示] 完成!")
            time.sleep(1)
            return "自動演示完成"
        
        def run_finger_sequence():
            """單指連續動作：從握拳開始，依序伸出每個手指"""
            clear_screen()
            print("[單指連續] 開始...\n")
            print("  M1=拇指旋轉, M2=拇指伸縮, M3=食指, M4=中指, M5=無名指, M6=尾指")
            print("  0=伸直, 255=彎曲\n")
            
            # 單指序列：從握拳開始，依序把每個手指伸出（其他握拳）
            # 大拇指需要 M1(旋轉) 和 M2(伸縮) 都設為 0 才完全伸出
            finger_sequence = [
                ([255, 255, 255, 255, 255, 255], "0. 握拳 ✊"),
                ([0,   0,   255, 255, 255, 255], "1. 大拇指 👍"),
                ([255, 255, 0,   255, 255, 255], "2. 食指 ☝️"),
                ([255, 255, 255, 0,   255, 255], "3. 中指 🖕"),
                ([255, 255, 255, 255, 0,   255], "4. 無名指 💍"),
                ([255, 255, 255, 255, 255, 0  ], "5. 小指 🤙"),
                ([255, 255, 255, 255, 255, 255], "6. 握拳 ✊"),
                ([0,   0,   0,   0,   0,   0  ], "7. 張開 ✋"),
            ]
            
            for i, (positions, name) in enumerate(finger_sequence):
                cmd = build_cmd(0x01, positions, speed=200, torque=200)
                print(f"  {name}")
                print(f"    位置: {positions}")
                success = can.transmit_fd(can_id, cmd)
                if not success:
                    print("    [!] 發送失敗")
                    break
                time.sleep(1.5)  # 等待動作完成
            
            print("\n[單指連續] 完成!")
            time.sleep(1)
            return "單指連續完成"
        
        def run_thumb_test():
            """拇指獨立測試：分別測試 M1(旋轉) 和 M2(伸縮)，其他手指張開"""
            clear_screen()
            print("[拇指測試] 開始...\n")
            print("  M1=拇指旋轉 (0=不旋轉, 255=旋轉)")
            print("  M2=拇指伸縮 (0=伸直, 255=彎曲)")
            print("  其他手指保持張開 (0)\n")
            
            # 拇指測試序列：其他手指都是 0（張開）
            thumb_sequence = [
                # [M1, M2, M3, M4, M5, M6]
                ([0,   0,   0, 0, 0, 0], "0. 全部張開 ✋"),
                
                # M1 旋轉測試
                ([64,  0,   0, 0, 0, 0], "1. M1=64  (旋轉 25%)"),
                ([128, 0,   0, 0, 0, 0], "2. M1=128 (旋轉 50%)"),
                ([192, 0,   0, 0, 0, 0], "3. M1=192 (旋轉 75%)"),
                ([255, 0,   0, 0, 0, 0], "4. M1=255 (旋轉 100%)"),
                ([0,   0,   0, 0, 0, 0], "5. M1=0   (復位)"),
                
                # M2 伸縮測試
                ([0,   64,  0, 0, 0, 0], "6. M2=64  (彎曲 25%)"),
                ([0,   128, 0, 0, 0, 0], "7. M2=128 (彎曲 50%)"),
                ([0,   192, 0, 0, 0, 0], "8. M2=192 (彎曲 75%)"),
                ([0,   255, 0, 0, 0, 0], "9. M2=255 (彎曲 100%)"),
                ([0,   0,   0, 0, 0, 0], "10. M2=0  (復位)"),
                
                # M1 + M2 組合
                ([128, 128, 0, 0, 0, 0], "11. M1=128, M2=128 (旋轉+彎曲 50%)"),
                ([255, 255, 0, 0, 0, 0], "12. M1=255, M2=255 (旋轉+彎曲 100%)"),
                ([0,   0,   0, 0, 0, 0], "13. 全部張開 ✋"),
            ]
            
            for i, (positions, name) in enumerate(thumb_sequence):
                cmd = build_cmd(0x01, positions, speed=200, torque=200)
                print(f"  {name}")
                print(f"    位置: {positions}")
                success = can.transmit_fd(can_id, cmd)
                if not success:
                    print("    [!] 發送失敗")
                    break
                time.sleep(1.5)  # 等待動作完成
            
            print("\n[拇指測試] 完成!")
            time.sleep(1)
            return "拇指測試完成"
        
        last_action = ""
        show_menu()
        
        while True:
            try:
                choice = input("請選擇: ").strip().lower()
                
                if choice in ["q", "quit", "exit"]:
                    clear_screen()
                    print("再見！")
                    break
                elif choice == "0":
                    _, last_action = send_gesture("disable", 0.5)
                    print("  [提示] 電機已禁用，可手動撥開卡住的手指")
                    show_menu(last_action)
                elif choice in ["r", "reset"]:
                    # 重置序列：先禁用再回零
                    print("  [重置] 步驟 1/2: 禁用電機...")
                    send_gesture("disable", 1.0)
                    print("  [重置] 步驟 2/2: 執行回零...")
                    _, last_action = send_gesture("home", 2.0)
                    last_action = "重置完成 (禁用+回零)"
                    show_menu(last_action)
                elif choice == "1":
                    _, last_action = send_gesture("home", 0.5)
                    show_menu(last_action)
                elif choice == "2":
                    _, last_action = send_gesture("open")
                    show_menu(last_action)
                elif choice == "3":
                    _, last_action = send_gesture("close")
                    show_menu(last_action)
                elif choice == "4":
                    _, last_action = send_gesture("thumbs_up", 0.3)
                    show_menu(last_action)
                elif choice == "5":
                    _, last_action = send_gesture("peace", 0.3)
                    show_menu(last_action)
                elif choice == "6":
                    _, last_action = send_gesture("ok", 0.3)
                    show_menu(last_action)
                elif choice == "7":
                    _, last_action = send_gesture("point", 0.3)
                    show_menu(last_action)
                elif choice == "8":
                    _, last_action = send_gesture("rock", 0.3)
                    show_menu(last_action)
                elif choice in ["d", "demo"]:
                    last_action = run_demo()
                    show_menu(last_action)
                elif choice in ["f", "finger"]:
                    last_action = run_finger_sequence()
                    show_menu(last_action)
                elif choice in ["t", "thumb"]:
                    last_action = run_thumb_test()
                    show_menu(last_action)
                elif choice in ["m", "menu"]:
                    show_menu(last_action)
                elif choice == "":
                    continue
                else:
                    print("  [!] 無效選項")
                    
            except KeyboardInterrupt:
                print("\n\n中斷...")
                break
            except EOFError:
                break
    
    finally:
        can.close()
        print("[INFO] 設備已關閉")


def main():
    parser = argparse.ArgumentParser(
        description="USBCANFD-200U 靈巧手測試工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用範例:
  互動式選單 (左手):   sudo python3 usbcanfd_scan.py
  互動式選單 (右手):   sudo python3 usbcanfd_scan.py --right
  掃描 CAN 總線:       sudo python3 usbcanfd_scan.py -t scan
        """
    )
    parser.add_argument(
        "--duration", "-d",
        type=float,
        default=10.0,
        help="掃描持續時間 (秒)"
    )
    parser.add_argument(
        "--test", "-t",
        type=str,
        choices=["menu", "scan", "demo"],
        default="menu",
        help="模式: menu=選單, scan=掃描, demo=演示"
    )
    parser.add_argument(
        "--right", "-r",
        action="store_true",
        help="使用右手 (預設左手)"
    )
    args = parser.parse_args()
    
    can_id = 0x11 if args.right else 0x12
    
    if args.test == "scan":
        scan_can_ids(duration=args.duration)
    elif args.test == "demo":
        demo_gestures(can_id)
    else:
        interactive_menu(can_id)


if __name__ == "__main__":
    main()



