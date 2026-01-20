#!/usr/bin/env python3
"""
Joint Data Logger - 關節數據記錄模組

用於記錄 Unity 原始目標位置與 OpenArm 實際位置，
以便分析兩者之間的追蹤誤差。

使用方式：
    from utils.joint_data_logger import JointDataLogger
    
    # 初始化
    self.joint_logger = JointDataLogger(log_frequency=50)
    
    # 在 control_loop 中記錄
    self.joint_logger.log(
        left_unity_target=self.left_target,
        left_actual=left_actual_positions,
        right_unity_target=self.right_target,
        right_actual=right_actual_positions
    )
    
    # 儲存數據
    self.joint_logger.save()
"""

import time
import pickle
import os
from datetime import datetime
from typing import List, Optional


class JointDataLogger:
    """
    關節數據記錄器
    
    記錄 Unity 原始目標與 OpenArm 實際位置的對應關係
    """
    
    def __init__(
        self,
        log_frequency: int = 50,
        control_frequency: int = 500,
        max_duration: float = 60.0,
        save_dir: Optional[str] = None
    ):
        """
        初始化數據記錄器
        
        Args:
            log_frequency: 記錄頻率 (Hz)，預設 50Hz
            control_frequency: 控制迴圈頻率 (Hz)，預設 500Hz
            max_duration: 最大記錄時長 (秒)，預設 60 秒
            save_dir: 儲存目錄，預設為 utils 目錄
        """
        self.log_frequency = log_frequency
        self.control_frequency = control_frequency
        self.max_duration = max_duration
        
        # 計算降採樣間隔
        self.log_interval = control_frequency // log_frequency
        self.loop_count = 0
        
        # 儲存目錄
        if save_dir is None:
            self.save_dir = os.path.dirname(os.path.abspath(__file__))
        else:
            self.save_dir = save_dir
        
        # 數據結構
        self.data = {
            'timestamp': [],
            'left_unity_target': [[] for _ in range(7)],
            'left_actual': [[] for _ in range(7)],
            'right_unity_target': [[] for _ in range(7)],
            'right_actual': [[] for _ in range(7)],
        }
        
        # 記錄狀態
        self.enabled = True
        self.start_time = time.time()
        self.is_recording = False
        
        print(f"[JointDataLogger] 初始化完成")
        print(f"  - 記錄頻率: {log_frequency} Hz (每 {self.log_interval} 次記錄一次)")
        print(f"  - 最大時長: {max_duration} 秒")
        print(f"  - 儲存目錄: {self.save_dir}")
    
    def start(self):
        """開始記錄"""
        self.is_recording = True
        self.start_time = time.time()
        self.loop_count = 0
        # 清空舊數據
        self.data = {
            'timestamp': [],
            'left_unity_target': [[] for _ in range(7)],
            'left_actual': [[] for _ in range(7)],
            'right_unity_target': [[] for _ in range(7)],
            'right_actual': [[] for _ in range(7)],
        }
        print("[JointDataLogger] 開始記錄")
    
    def stop(self):
        """停止記錄"""
        self.is_recording = False
        duration = time.time() - self.start_time
        print(f"[JointDataLogger] 停止記錄，共 {len(self.data['timestamp'])} 筆數據，時長 {duration:.2f} 秒")
    
    def log(
        self,
        left_unity_target: List[float],
        left_actual: List[float],
        right_unity_target: List[float],
        right_actual: List[float]
    ) -> bool:
        """
        記錄一筆數據
        
        Args:
            left_unity_target: 左臂 Unity 原始目標位置 (7 joints)
            left_actual: 左臂 OpenArm 實際位置 (7 joints)
            right_unity_target: 右臂 Unity 原始目標位置 (7 joints)
            right_actual: 右臂 OpenArm 實際位置 (7 joints)
        
        Returns:
            bool: 是否成功記錄（可能因降採樣或超時而跳過）
        """
        if not self.enabled or not self.is_recording:
            return False
        
        self.loop_count += 1
        
        # 降採樣
        if self.loop_count % self.log_interval != 0:
            return False
        
        # 檢查是否超時
        current_time = time.time()
        elapsed = current_time - self.start_time
        if elapsed > self.max_duration:
            self.stop()
            return False
        
        # 記錄時間戳
        self.data['timestamp'].append(elapsed)
        
        # 記錄左臂數據
        for i in range(7):
            self.data['left_unity_target'][i].append(left_unity_target[i])
            self.data['left_actual'][i].append(left_actual[i])
        
        # 記錄右臂數據
        for i in range(7):
            self.data['right_unity_target'][i].append(right_unity_target[i])
            self.data['right_actual'][i].append(right_actual[i])
        
        return True
    
    def save(self, filename: Optional[str] = None) -> str:
        """
        儲存數據到檔案
        
        Args:
            filename: 檔案名稱，預設為 joint_data_YYYYMMDD_HHMMSS.pkl
        
        Returns:
            str: 儲存的檔案路徑
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"joint_data_{timestamp}.pkl"
        
        filepath = os.path.join(self.save_dir, filename)
        
        with open(filepath, 'wb') as f:
            pickle.dump(self.data, f)
        
        print(f"[JointDataLogger] 數據已儲存至: {filepath}")
        print(f"  - 數據筆數: {len(self.data['timestamp'])}")
        
        return filepath
    
    def get_data(self) -> dict:
        """取得記錄的數據"""
        return self.data
    
    def get_stats(self) -> dict:
        """取得統計資訊"""
        if len(self.data['timestamp']) == 0:
            return {'count': 0, 'duration': 0}
        
        return {
            'count': len(self.data['timestamp']),
            'duration': self.data['timestamp'][-1] if self.data['timestamp'] else 0,
            'sample_rate': len(self.data['timestamp']) / (self.data['timestamp'][-1] + 0.001)
        }
