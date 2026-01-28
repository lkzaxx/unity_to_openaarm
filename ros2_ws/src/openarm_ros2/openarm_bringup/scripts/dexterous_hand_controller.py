#!/usr/bin/env python3
"""
靈巧手控制器模組 (Dexterous Hand Controller)

封裝 HITBOT eHand-6 靈巧手的低階 CAN FD 通訊，提供高階控制介面。

功能：
- 位置控制（0~1 → 0~255 轉換）
- 狀態讀取（位置、速度、故障）
- 碰撞偵測（堵轉檢測 + 位置偏差）
- 防塞車機制（時間估算 + 張開超時）

作者: Antigravity AI
日期: 2026-01-28
版本: 1.0.0
"""

import time
from typing import Optional, List, Dict


class DexterousHandController:
    """
    靈巧手控制器
    
    管理單隻靈巧手（左手或右手）的通訊和控制邏輯。
    使用 ZLGCAN SDK 透過 USB CANFD 介面進行 CAN FD 通訊。
    
    Attributes:
        can_id (int): CAN ID (0x11=右手, 0x12=左手)
        speed (int): 預設速度 (0~255)
        torque (int): 預設扭矩 (0~255)
        zlgcan: ZLGCAN SDK 實例（由外部提供）
    """
    
    # 狀態碼對照
    STATE_MAP = {
        0: '初始化', 1: '待機', 2: '校準中', 3: '位置模式',
        4: '保留', 5: '老化', 6: '故障', 7: '等待響應'
    }
    
    FAULT_MAP = {
        0: '無故障', 1: '過流', 2: '過壓', 3: '欠壓',
        4: '過熱', 5: '堵轉', 6: '通訊超時', 7: '硬體故障'
    }
    
    FINGER_NAMES = ['拇指橫向', '拇指縱向', '食指', '中指', '無名指', '小指']
    
    def __init__(self, zlgcan, can_id: int, speed: int = 230, torque: int = 230):
        """
        初始化靈巧手控制器
        
        Args:
            zlgcan: ZLGCAN SDK 實例
            can_id: CAN ID (0x11=右手, 0x12=左手)
            speed: 預設速度 (0~255)
            torque: 預設扭矩 (0~255)
        """
        self.zlgcan = zlgcan
        self.can_id = can_id
        self.speed = speed
        self.torque = torque
        
        # 狀態追蹤（用於防塞車機制）
        self._state = {
            'last_pos': None,
            'target_pos': None,
            'last_send_time': 0,
            'estimated_arrival': 0
        }
        
        # 張開超時檢測狀態
        self._open_state = {
            'open_start_time': None,
            'last_reset_time': 0,
            'had_activity': False
        }
        
        # 碰撞偵測緩存（用於位置偏差檢測）
        self._last_target = [0] * 6  # 最後一次發送的目標位置
        
        # Debug 計數器
        self._send_count = 0
    
    def is_ready(self) -> bool:
        """檢查靈巧手是否就緒"""
        return self.zlgcan is not None
    
    def send_home(self) -> bool:
        """
        發送回零命令
        
        Returns:
            bool: 發送成功返回 True
        """
        if not self.is_ready():
            return False
        
        # 回零命令: 0xFD 0x04 + 30 bytes 填充
        cmd = bytes([0xFD, 0x04] + [0x00] * 30)
        return self.zlgcan.transmit_fd(self.can_id, cmd)
    
    def send_positions(self, positions: List[float]) -> bool:
        """
        發送位置命令（帶時間估算防塞車機制 + 張開超時自動重置）
        
        機制說明：
        1. 根據位置變化量估算移動時間，在預計到達前不發送新命令
        2. 偵測持續張開超過 2 秒 → 發送 Disable + Open 解除卡住
        
        Args:
            positions: 6 個手指位置 (0~1 範圍)
        
        Returns:
            bool: 發送成功返回 True，跳過返回 False
        """
        if not self.is_ready():
            return False
        
        if len(positions) != 6:
            raise ValueError(f"positions 必須是 6 個元素的列表，收到 {len(positions)} 個")
        
        current_time = time.time()
        
        # === 張開超時檢測 ===
        self._check_open_timeout(positions, current_time)
        
        # === 位置轉換 (0~1 → 0~255) ===
        pos_values = []
        for i in range(6):
            pos_value = int(positions[i] * 255)
            pos_value = max(0, min(255, pos_value))
            pos_values.append(pos_value)
        
        # === 防塞車機制：動態等待時間 ===
        MAX_TRAVEL_TIME = 1.5  # 全程移動時間（秒）
        MIN_INTERVAL = 0.1     # 最小發送間隔（秒）
        CHANGE_THRESHOLD = 5   # 變化閾值
        
        if self._state['target_pos'] is not None:
            time_since_last = current_time - self._state.get('last_send_time', 0)
            
            # 計算目標變化
            target_change = max(abs(pos_values[i] - self._state['target_pos'][i]) for i in range(6))
            
            # 如果目標沒變，跳過
            if target_change < CHANGE_THRESHOLD:
                return False
            
            # 計算上次動作的預估時間
            if self._state['last_pos'] is not None:
                last_move = max(abs(self._state['target_pos'][i] - self._state['last_pos'][i]) for i in range(6))
                estimated_time = (last_move / 255.0) * MAX_TRAVEL_TIME + MIN_INTERVAL
            else:
                estimated_time = MIN_INTERVAL
            
            # 等待上次動作完成
            if time_since_last < estimated_time:
                return False
        
        # === 更新狀態 ===
        self._state['last_pos'] = self._state['target_pos']
        self._state['target_pos'] = pos_values[:]
        self._state['last_send_time'] = current_time
        
        # 保存目標（用於碰撞偵測）
        self._last_target = pos_values[:]
        
        # === 建立 CAN FD 封包 (32 bytes) ===
        # [0xFD][0x01][M1-M6: 各5bytes]
        data = [0xFD, 0x01]  # 全選寫入 + 位置模式
        
        for pos_value in pos_values:
            # 每個馬達 5 bytes: Position, Speed, Torque, Reserved, Reserved
            data.extend([pos_value, self.speed, self.torque, 0x00, 0x00])
        
        # === 發送 ===
        result = self.zlgcan.transmit_fd(self.can_id, bytes(data))
        
        # Debug 輸出（每 10 次）
        self._send_count += 1
        if self._send_count % 10 == 0:
            hand_name = "LEFT" if self.can_id == 0x12 else "RIGHT"
            data_hex = ' '.join(f'{b:02X}' for b in data[:16])
            print(f"[{hand_name}_HAND] len={len(data)}, data={data_hex}..., result={result}")
        
        return result
    
    def _check_open_timeout(self, positions: List[float], current_time: float):
        """
        檢查是否持續張開超時（內部方法）
        
        如果手持續張開超過 2 秒，可能是卡住，執行 Disable + Open 重置。
        """
        OPEN_THRESHOLD = 0.3      # 30% 以下視為張開
        CLOSE_THRESHOLD = 0.5     # 50% 以上視為有動作
        OPEN_TIMEOUT = 2.0        # 超時時間
        OPEN_COOLDOWN = 5.0       # 冷卻時間
        
        # 判斷是否為全張開手勢
        is_open_gesture = all(pos < OPEN_THRESHOLD for pos in positions)
        has_activity = any(pos > CLOSE_THRESHOLD for pos in positions)
        
        # 記錄活動
        if has_activity:
            self._open_state['had_activity'] = True
        
        if is_open_gesture:
            # 只有在曾經有過動作後才計時
            if self._open_state['had_activity']:
                if self._open_state['open_start_time'] is None:
                    self._open_state['open_start_time'] = current_time
                
                elif current_time - self._open_state['open_start_time'] > OPEN_TIMEOUT:
                    # 超時，執行重置
                    if current_time - self._open_state['last_reset_time'] > OPEN_COOLDOWN:
                        hand_name = "左手" if self.can_id == 0x12 else "右手"
                        print(f"⚠️ [{hand_name}] 持續張開超過 {OPEN_TIMEOUT} 秒，執行 Disable + Open")
                        
                        # Disable
                        cmd_disable = bytes([0xFD, 0x00] + [0x00] * 30)
                        self.zlgcan.transmit_fd(self.can_id, cmd_disable)
                        time.sleep(0.3)
                        
                        # Open
                        cmd_open = bytes([0xFD, 0x02] + [0x00] * 30)
                        self.zlgcan.transmit_fd(self.can_id, cmd_open)
                        
                        # 重置狀態
                        self._open_state['open_start_time'] = None
                        self._open_state['last_reset_time'] = current_time
                        self._open_state['had_activity'] = False
        else:
            # 不是張開，重置計時
            self._open_state['open_start_time'] = None
    
    def read_status(self) -> Optional[Dict]:
        """
        讀取靈巧手狀態（使用 0xFC 命令）
        
        Returns:
            dict: 狀態字典，包含：
                - 'hand_state': 整機狀態
                - 'hand_fault': 整機故障碼
                - 'fingers': 6 個手指的狀態列表
                    - 'name': 手指名稱
                    - 'state': 狀態
                    - 'fault': 故障碼
                    - 'position': 當前位置 (0-255)
                    - 'position_percent': 位置百分比 (0-100)
                    - 'speed': 當前速度 (0-255)
            None: 無回應或讀取失敗
        """
        if not self.is_ready():
            return None
        
        # 發送讀取命令 (必須是 32 字節)
        cmd_read = bytes([0xFC] + [0x00] * 31)
        self.zlgcan.transmit_fd(self.can_id, cmd_read)
        time.sleep(0.3)  # 等待回應
        
        # 接收回應
        try:
            from usbcanfd_scan import TYPE_CANFD
            num = self.zlgcan.get_receive_num(TYPE_CANFD)
            if num > 0:
                msgs = self.zlgcan.receive_fd(num, 200)
                for msg in msgs:
                    data = bytes([msg.frame.data[i] for i in range(msg.frame.len)])
                    
                    # 檢查是否為讀回應 (Byte1 低2位 = 2)
                    if len(data) >= 32 and (data[0] & 0x03) == 2:
                        return self._parse_status(data)
        except Exception as e:
            print(f"⚠️ 讀取狀態失敗: {e}")
            return None
        
        return None
    
    def _parse_status(self, data: bytes) -> Dict:
        """解析狀態回應封包（內部方法）"""
        # 解析整機狀態
        hand_state = data[1] & 0x0F
        hand_fault = (data[1] >> 4) & 0x0F
        
        result = {
            'raw': data,
            'hand_state': self.STATE_MAP.get(hand_state, f'0x{hand_state:X}'),
            'hand_fault': self.FAULT_MAP.get(hand_fault, f'0x{hand_fault:X}'),
            'fingers': []
        }
        
        # 解析 6 個手指
        for i in range(6):
            offset = 2 + i * 5
            sf = data[offset]
            position = data[offset + 1]
            speed = data[offset + 2]
            
            result['fingers'].append({
                'name': self.FINGER_NAMES[i],
                'state': self.STATE_MAP.get(sf & 0x0F, '?'),
                'fault': self.FAULT_MAP.get((sf >> 4) & 0x0F, '?'),
                'position': position,
                'position_percent': position / 255.0 * 100.0,
                'speed': speed,
            })
        
        return result
    
    def detect_contact(self, status: Optional[Dict] = None) -> Dict[str, Dict]:
        """
        偵測碰撞（混合策略：堵轉 + 位置偏差）
        
        Args:
            status: 狀態字典（如果為 None 則自動讀取）
        
        Returns:
            dict: 碰撞資訊，key 為手指名稱，value 為碰撞詳情
                {
                    '食指': {
                        'detected': True,
                        'method': 'stall_detection',  # 或 'position_deviation'
                        'confidence': 'high',  # high, medium, very_high
                        'deviation': 85  # (僅位置偏差方法)
                    }
                }
        """
        if status is None:
            status = self.read_status()
        
        if status is None:
            return {}
        
        contacts = {}
        
        # 策略 1: 堵轉檢測（高可靠性）
        for finger in status['fingers']:
            if finger['fault'] == '堵轉':
                contacts[finger['name']] = {
                    'detected': True,
                    'method': 'stall_detection',
                    'confidence': 'high'
                }
        
        # 策略 2: 位置偏差檢測（高靈敏度）
        DEVIATION_THRESHOLD = 75  # 約 30%
        
        for i, finger in enumerate(status['fingers']):
            expected = self._last_target[i]
            actual = finger['position']
            deviation = abs(expected - actual)
            
            # 偏差 > 30% 且實際位置 < 期望位置（被阻擋）
            if deviation > DEVIATION_THRESHOLD and actual < expected:
                if finger['name'] in contacts:
                    # 堵轉和偏差都檢測到，提升信心度
                    contacts[finger['name']]['confidence'] = 'very_high'
                    contacts[finger['name']]['method'] = 'hybrid'
                    contacts[finger['name']]['deviation'] = deviation
                else:
                    contacts[finger['name']] = {
                        'detected': True,
                        'method': 'position_deviation',
                        'confidence': 'medium',
                        'deviation': deviation
                    }
        
        return contacts
    
    def __repr__(self):
        """字串表示"""
        hand_name = "左手" if self.can_id == 0x12 else "右手"
        return f"DexterousHandController({hand_name}, CAN_ID=0x{self.can_id:02X})"
