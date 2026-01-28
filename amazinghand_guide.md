# AmazingHand 開源靈巧手指南

> **狀態**: 🔄 待測試  
> **最後更新**: 2026-01-22

---

## 1. 設備資訊

| 項目 | 規格 |
|------|------|
| 型號 | TsingSens AmazingHand (基於 Pollen Robotics 開源設計) |
| 自由度 | 8 DOF (4 根手指，每指 2 個關節) |
| 舵機型號 | Feetech SCS0009 × 8 |
| 通訊協議 | Feetech Serial Bus (TTL 半雙工) |
| Baud Rate | **1,000,000 bps** (預設) |
| 資料格式 | 8N1 |
| USB 晶片 | Silicon Labs CP210x |
| 裝置路徑 | `/dev/ttyUSB0` |
| 工作電壓 | **5V** (USB 供電) |
| 重量 | 約 400g |

---

## 2. 手指對應 (預設 ID)

| ID | 手指 | 關節 |
|----|------|------|
| 1 | 食指 | 屈伸 |
| 2 | 食指 | 外展 |
| 3 | 中指 | 屈伸 |
| 4 | 中指 | 外展 |
| 5 | 無名指 | 屈伸 |
| 6 | 無名指 | 外展 |
| 7 | 小指 | 屈伸 |
| 8 | 小指 | 外展 |

> ⚠️ **注意**: 實際 ID 可能因組裝而異，請用掃描功能確認

---

## 3. 連線設定

### 權限設定 (首次使用)

```bash
# 方法 1: 加入 dialout 群組 (推薦，需重新登入)
sudo usermod -aG dialout $USER

# 方法 2: 臨時修改權限
sudo chmod 666 /dev/ttyUSB0
```

### 確認連線

```bash
# 檢查裝置
ls -la /dev/ttyUSB0

# 檢查 USB
lsusb | grep -i "silicon\|cp210"
```

---

## 4. Feetech SCS 協議

### 封包格式

```
[Header][Header][ID][Length][Instruction][Params...][Checksum]
  0xFF    0xFF   ID   Len      Inst        Data       ~Sum
```

### 指令碼

| 指令 | 代碼 | 說明 |
|------|------|------|
| Ping | 0x01 | 檢測舵機是否存在 |
| Read | 0x02 | 讀取暫存器 |
| Write | 0x03 | 寫入暫存器 |
| Reg Write | 0x04 | 預寫入（等待 Action 觸發）|
| Action | 0x05 | 觸發所有預寫入 |
| Reset | 0x06 | 重置為出廠設定 |
| Sync Write | 0x83 | 同步寫入多個舵機 |

### 常用暫存器

| 位址 | 名稱 | 長度 | 說明 |
|------|------|------|------|
| 0x03 | ID | 1 | 舵機 ID (1-253) |
| 0x06 | Baud Rate | 1 | 波特率設定 |
| 0x2A | Goal Position | 2 | 目標位置 (0-1023) |
| 0x2C | Running Speed | 2 | 運行速度 |
| 0x38 | Present Position | 2 | 當前位置 |
| 0x3C | Present Speed | 2 | 當前速度 |
| 0x3E | Present Load | 2 | 當前負載 |

---

## 5. 測試腳本

### 位置

```
~/amazinghand_test.py
```

### 使用方式

```bash
# 掃描舵機
sudo python3 ~/amazinghand_test.py scan

# 移動單個舵機
sudo python3 ~/amazinghand_test.py move <ID> <位置>
# 例如: sudo python3 ~/amazinghand_test.py move 1 512

# 全部張開
sudo python3 ~/amazinghand_test.py open

# 全部握緊
sudo python3 ~/amazinghand_test.py close

# 互動模式
sudo python3 ~/amazinghand_test.py
```

---

## 6. Python 程式碼範例

```python
import serial
import time

class AmazingHand:
    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000):
        self.ser = serial.Serial(port, baudrate, timeout=0.5)
        self.servo_ids = list(range(1, 9))  # ID 1-8
    
    def _checksum(self, data):
        return (~sum(data)) & 0xFF
    
    def _send(self, servo_id, instruction, params=[]):
        length = len(params) + 2
        pkt = [0xFF, 0xFF, servo_id, length, instruction] + params
        pkt.append(self._checksum(pkt[2:]))
        self.ser.reset_input_buffer()
        self.ser.write(bytes(pkt))
        self.ser.flush()
        time.sleep(0.01)
        return self.ser.read(self.ser.in_waiting or 10)
    
    def ping(self, servo_id):
        """Ping 舵機"""
        resp = self._send(servo_id, 0x01)
        return len(resp) >= 6
    
    def set_position(self, servo_id, position, speed=500):
        """設定位置 (0-1023)"""
        pos_l = position & 0xFF
        pos_h = (position >> 8) & 0xFF
        spd_l = speed & 0xFF
        spd_h = (speed >> 8) & 0xFF
        # 寫入 Goal Position (0x2A) 和 Speed (0x2C)
        self._send(servo_id, 0x03, [0x2A, pos_l, pos_h, spd_l, spd_h])
    
    def read_position(self, servo_id):
        """讀取當前位置"""
        resp = self._send(servo_id, 0x02, [0x38, 2])
        if len(resp) >= 8:
            return resp[5] | (resp[6] << 8)
        return None
    
    def open_hand(self):
        """張開手掌"""
        for sid in self.servo_ids:
            self.set_position(sid, 200)  # 張開位置
            time.sleep(0.05)
    
    def close_hand(self):
        """握緊手掌"""
        for sid in self.servo_ids:
            self.set_position(sid, 800)  # 握緊位置
            time.sleep(0.05)
    
    def disconnect(self):
        self.ser.close()

# 使用範例
if __name__ == "__main__":
    hand = AmazingHand()
    
    # 掃描
    print("掃描舵機...")
    for sid in range(1, 9):
        if hand.ping(sid):
            print(f"  ID {sid}: ✅")
    
    # 動作
    hand.open_hand()
    time.sleep(1)
    hand.close_hand()
    
    hand.disconnect()
```

---

## 7. 故障排除

| 症狀 | 可能原因 | 解決方案 |
|------|----------|----------|
| Permission denied | 權限不足 | `sudo chmod 666 /dev/ttyUSB0` 或加入 dialout |
| 無回應 | Baud rate 錯誤 | 嘗試 38400, 115200, 500000, 1000000 |
| 無回應 | ID 錯誤 | 用掃描功能 (ID 0-253) |
| 無回應 | 接線問題 | 檢查 USB 連接、供電 |
| 部分舵機不動 | 供電不足 | 使用獨立 5V 電源 |
| 動作卡頓 | 速度過快 | 增加指令間隔 |

---

## 8. 官方資源

- [Pollen Robotics AmazingHand GitHub](https://github.com/pollen-robotics/AmazingHand)
- [Feetech FTServo Python SDK](https://github.com/ftservo/FTServo_Python)
- [Seeed Studio Wiki](https://wiki.seeedstudio.com/cn/hand_amazinghand/)

---

## 9. 與 HITBOT eHand-6 比較

| 項目 | AmazingHand | HITBOT eHand-6 |
|------|-------------|----------------|
| 自由度 | 8 DOF | 6 DOF |
| 通訊 | Serial Bus (TTL) | CAN FD |
| 驅動 | Feetech SCS0009 舵機 | 內建馬達 |
| 供電 | 5V USB | 24V |
| 成本 | ~$200 | 較高 |
| 開源 | ✅ 完全開源 | ❌ |
