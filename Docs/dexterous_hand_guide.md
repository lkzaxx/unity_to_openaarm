# 靈巧手整合指南

> **狀態**: ⚠️ 部分功能正常（左手穩定，右手不穩定）  
> **最後更新**: 2026-01-23

---

## 1. 設備資訊

| 項目 | 規格 |
|------|------|
| 型號 | HITBOT eHand-6 (RBTX-HITBO-0002) |
| 通訊協議 | CAN FD |
| 仲裁波特率 | 1 Mbps |
| 數據波特率 | 5 Mbps |
| 封包大小 | 32 bytes |
| 右手 CAN ID | **0x11** |
| 左手 CAN ID | **0x12** |
| 工作電壓 | **24VDC ±10%** (21.6V~26.4V) |

---

## 2. 接線定義 (4-pin M8)

| 線色 | 功能 |
|------|------|
| 棕 | 24V+ |
| 藍 | GND (0V) |
| 白 | CAN_L |
| 黑 | CAN_H |

---

## 3. CAN 介面設定

```bash
# 設定 CAN FD 模式 (靈巧手專用)
sudo ip link set canX down
sudo ip link set canX type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set canX up

# 確認設定 (MTU 應為 72)
ip link show canX
```

> ⚠️ **注意**: 靈巧手需要 CAN FD 模式，與達妙馬達 (標準 CAN) **不能共用同一介面**

---

## 4. 測試指令

```bash
# 使用測試腳本 (推薦)
python3 openarm_can/setup/test_dexterous_hand.py canX 0x12  # 左手
python3 openarm_can/setup/test_dexterous_hand.py canX 0x11  # 右手

# 手動發送指令（Reserved 填 0x00）
cansend canX 012##1FD04000000000000000000000000000000000000000000000000000000000000  # 回零
cansend canX 012##1FD02000000000000000000000000000000000000000000000000000000000000  # 張開
cansend canX 012##1FD03000000000000000000000000000000000000000000000000000000000000  # 握緊
```

---

## 5. 32 Bytes 封包格式

```
[Byte1][Byte2][Motor1-6: 各5bytes] = 32 bytes
```

| 位置 | 內容 | 說明 |
|------|------|------|
| Byte1 | 馬達選擇 + 讀寫 | 0xFD=全選寫入, 0xFE=全選讀取 |
| Byte2 | 控制模式 | 0x01=位置, 0x02=開, 0x03=合, 0x04=回零 |
| Byte3~32 | 6組馬達參數 | 每組 5 bytes: Position, Speed, Torque, Reserved×2 (**Reserved 填 0x00**) |

### 馬達對應

| 索引 | 馬達 | 功能 |
|------|------|------|
| 0 | M1 | 拇指旋轉 |
| 1 | M2 | 拇指伸縮 |
| 2 | M3 | 食指 |
| 3 | M4 | 中指 |
| 4 | M5 | 無名指 |
| 5 | M6 | 尾指 |

---

## 6. 讀取靈巧手狀態

> **2026-01-23 新增**: 可透過 `0xFC` 命令讀取靈巧手的即時狀態

### 讀取命令格式

| 項目 | 值 | 說明 |
|------|-----|------|
| **發送** | `0xFC` + 31 bytes (0x00) | 32 字節，後續字節無須填充 |
| **回應** | `0xFE` + 31 bytes 狀態數據 | 32 字節狀態反饋幀 |

```
發送: FC 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
      ↑
      0xFC = 11111100 (低2位=00=讀取, 高6位=111111=全選)

回應: FE 60 0F 00 00 00 00 0F 00 00 00 00 0F 00 00 00 00 0F 00 00 00 00 0F 00 00 00 00 0F 00 00 00 00
      ↑  ↑  └─────────────────── 6 組手指狀態 (每組 5 bytes) ───────────────────┘
      │  └── Byte2: 整機狀態 + 故障碼
      └───── Byte1: 0xFE = 讀回應 (低2位=10=2)
```

### 回應解析

#### Byte1 (回應類型)

| 低 2 位 | 含義 |
|---------|------|
| 0 | 幀格式錯誤 |
| 1 | 故障上傳 |
| **2** | **讀命令回應** |
| 3 | 寫命令回應 |

#### Byte2 (整機狀態)

| 位元 | 含義 | 值對照 |
|------|------|--------|
| 低 4 位 | 整機狀態 | 0=初始化, 1=待機, 2=校準中, 3=位置模式, 6=故障 |
| 高 4 位 | 故障碼 | 0=無故障, 1=過流, 2=過壓, 3=欠壓, 4=過熱, **5=堵轉**, **6=通訊超時**, 7=硬體故障 |

#### Bytes 3-32 (6 組手指狀態，每組 5 bytes)

| Offset | 內容 | 說明 |
|--------|------|------|
| +0 | 狀態+故障 | 低4位=狀態, 高4位=故障 |
| +1 | 當前位置 | 0-255 對應 0-100% |
| +2 | 當前速度 | 0-255 對應 0-100% |
| +3 | 保留 | - |
| +4 | 保留 | - |

### Python 讀取範例

```python
import time
from ctypes import *

# ... (ZLGCAN 初始化代碼省略，見 usbcanfd_scan.py) ...

# 狀態碼對照
STATE_MAP = {0: '初始化', 1: '待機', 2: '校準中', 3: '位置模式', 
             4: '保留', 5: '老化', 6: '故障', 7: '等待響應'}
FAULT_MAP = {0: '無故障', 1: '過流', 2: '過壓', 3: '欠壓',
             4: '過熱', 5: '堵轉', 6: '通訊超時', 7: '硬體故障'}
FINGER_NAMES = ['拇指橫向', '拇指縱向', '食指', '中指', '無名指', '小指']

def read_hand_status(can, can_id=0x12):
    """讀取靈巧手狀態
    
    Args:
        can: ZLGCAN 實例
        can_id: 0x12=左手, 0x11=右手
    
    Returns:
        dict: 狀態資訊，或 None（無回應）
    """
    # 發送讀取命令 (必須是 32 字節!)
    cmd_read = bytes([0xFC] + [0x00] * 31)
    can.transmit_fd(can_id, cmd_read)
    time.sleep(0.3)
    
    # 接收回應
    num = can.get_receive_num(TYPE_CANFD)
    if num > 0:
        msgs = can.receive_fd(num, 200)
        for msg in msgs:
            data = bytes([msg.frame.data[i] for i in range(msg.frame.len)])
            
            # 檢查是否為讀回應 (Byte1 低2位 = 2)
            if len(data) >= 32 and (data[0] & 0x03) == 2:
                # 解析整機狀態
                hand_state = data[1] & 0x0F
                hand_fault = (data[1] >> 4) & 0x0F
                
                result = {
                    'raw': data,
                    'state': STATE_MAP.get(hand_state, f'0x{hand_state:X}'),
                    'fault': FAULT_MAP.get(hand_fault, f'0x{hand_fault:X}'),
                    'fingers': []
                }
                
                # 解析 6 個手指
                for i in range(6):
                    offset = 2 + i * 5
                    sf = data[offset]
                    result['fingers'].append({
                        'name': FINGER_NAMES[i],
                        'state': STATE_MAP.get(sf & 0x0F, '?'),
                        'fault': FAULT_MAP.get((sf >> 4) & 0x0F, '?'),
                        'position': data[offset + 1],
                        'speed': data[offset + 2],
                    })
                
                return result
    
    return None

# 使用範例
status = read_hand_status(can, 0x12)
if status:
    print(f"整機: {status['state']} / {status['fault']}")
    for f in status['fingers']:
        print(f"  {f['name']}: 位置={f['position']} ({f['position']/255*100:.1f}%)")
else:
    print("無回應")
```

### 快速測試命令

```bash
# 讀取左手狀態
sudo python3 -c "
import sys, time
from ctypes import *
sys.path.insert(0, '/home/idaka/openarm_can/setup')
from usbcanfd_scan import ZLGCAN, MODE_NORMAL, TYPE_CANFD

can = ZLGCAN()
can.open_device()
can.set_baudrate(0, 1000000, 5000000)
can.init_channel(0, MODE_NORMAL)
can.start_can()

# 發送讀取命令 (32 bytes)
cmd = bytes([0xFC] + [0x00] * 31)
can.transmit_fd(0x12, cmd)  # 左手
time.sleep(0.3)

# 接收回應
num = can.get_receive_num(TYPE_CANFD)
if num > 0:
    msgs = can.receive_fd(num, 200)
    for m in msgs:
        data = bytes([m.frame.data[i] for i in range(m.frame.len)])
        print(f'回應: {\" \".join(f\"{b:02X}\" for b in data)}')
        if (data[0] & 0x03) == 2:
            fault = (data[1] >> 4) & 0x0F
            faults = {0:'無故障', 5:'堵轉', 6:'通訊超時', 7:'硬體故障'}
            print(f'故障碼: {faults.get(fault, fault)}')

can.close()
"
```

### 重要注意事項

| 項目 | 說明 |
|------|------|
| **命令長度** | 必須是 **32 字節**，其他長度無法正確讀取 |
| **等待時間** | 發送後需等待約 200-300ms 才能收到回應 |
| **故障碼 6** | 「通訊超時」表示靈巧手內部 MCU 與馬達通訊異常 |
| **狀態碼 0xF** | 可能是廠商自定義狀態，需確認固件版本 |

### 常見狀態範例

| 原始數據 | 整機狀態 | 說明 |
|----------|----------|------|
| `FE 00 00...` | 初始化/無故障 | ✅ 正常 |
| `FE 60 0F...` | 初始化/**通訊超時** | ⚠️ 內部通訊問題，可能需要斷電重啟 |
| `FE 64 0F...` | 保留4/**通訊超時** | ⚠️ 同上 |

---

## 7. 程式碼使用

```python
from openarm.can.dexterous_hand import DexterousHand

# 初始化
hand = DexterousHand("can0", can_id=0x12)  # 左手

# 控制
hand.home()              # 回零
hand.open()              # 張開
hand.close()             # 握緊
hand.set_grip(0.5)       # 50% 握合
hand.set_positions([0, 128, 128, 128, 128, 128])  # 各指位置

# 關閉
hand.disconnect()
```

---

## 8. 整合到 unity_interface_follower.py

修改 `unity_interface_follower.py` 中的設定：

```python
# 切換末端執行器類型
RIGHT_END_EFFECTOR_TYPE = "dexterous_hand"  # 或 "gripper"

# 靈巧手配置
DEXTEROUS_HAND_CAN_INTERFACE = "can0"  # 獨立 CAN 介面
DEXTEROUS_HAND_CAN_ID = 0x12           # 左手=0x12, 右手=0x11
```

---

## 9. 故障排除

| 症狀 | 可能原因 | 解決方案 |
|------|----------|----------|
| 無回應 | 電壓不足 | 確認 24V ±10% |
| 無回應 | CAN 模式錯誤 | 確認使用 CAN FD (MTU=72) |
| 無回應 | ID 錯誤 | 確認右手=0x11, 左手=0x12 |
| 無回應 | USB 接口問題 | **換一個 USB 接口重新插入** |
| 有回應但不動 | 接線反 | 交換 CAN_H/L |
| 有回應但不動 | 電壓不足 | 確認 24V (非 20V) |
| handle=NULL | USB 設備異常 | 換 USB 接口或執行 `sudo usbreset` |

---

## 10. USBCANFD 設備

> 用於獨立測試 CAN FD 通訊的 USB 轉 CAN FD 適配器 (ZLGCAN USBCANFD-200U)

### 檔案位置

| 項目 | 路徑 |
|------|------|
| 動態庫 | `/usr/local/lib/libcontrolcanfd.so` |
| 源檔案 | `~/Downloads/libcontrolcanfd.so` |
| 互動式控制工具 | `openarm_can/setup/usbcanfd_scan.py` |
| 設置指南 | `openarm_can/setup/USBCANFD_SETUP.md` |
| udev 規則 | `/etc/udev/rules.d/99-usbcanfd.rules` |

### 測試指令

```bash
# 互動式選單 (左手)
sudo python3 ~/openarm_can/setup/usbcanfd_scan.py

# 互動式選單 (右手)
sudo python3 ~/openarm_can/setup/usbcanfd_scan.py --right

# 被動掃描 CAN 總線
sudo python3 ~/openarm_can/setup/usbcanfd_scan.py -t scan -d 10

# 自動演示模式
sudo python3 ~/openarm_can/setup/usbcanfd_scan.py -t demo
```

### 診斷指令

```bash
# 1. 檢查 USB 設備是否識別
lsusb | grep -i can

# 2. 檢查動態庫
ldconfig -p | grep controlcanfd

# 3. 重置 USB 設備 (解決 Segmentation fault)
sudo usbreset "USB CANFD DEBUG"
```

### 選單功能

| 按鍵 | 功能 |
|------|------|
| 1 | 回零 |
| 2 | 張開 ✋ |
| 3 | 握緊 ✊ |
| 4 | 讚 👍 |
| 5 | 比YA ✌️ |
| 6 | OK 👌 |
| 7 | 指 👆 |
| 8 | 搖滾 🤘 |
| d | 自動演示 |
| q | 離開 |

刪除以上檔案即可卸載 USBCANFD 支援。

---

## 11. 調試歷程記錄

### 2026-01-23: 左手「通訊超時」診斷

**症狀**: 左手亮綠燈但卡住不動

**診斷過程**:

1. 使用狀態讀取命令 (`0xFC`) 比較左右手：
   ```
   左手回應: FE 64 0F 00 00 00 00 0F 00 00 00 00 ...
   右手回應: FE 00 00 00 00 00 00 00 00 00 00 00 ...
   ```

2. 解析結果：
   | 項目 | 左手 | 右手 |
   |------|------|------|
   | Byte1 | `0x64` | `0x00` |
   | 整機狀態 | 4 (保留) | 0 (初始化) |
   | **故障碼** | **6 (通訊超時)** | 0 (無故障) |
   | 各手指狀態 | `0x0F` (異常) | `0x00` (正常) |

**結論**: 
- 左手內部 MCU 與馬達驅動之間的**通訊超時**
- 需要硬體檢修或斷電重啟

**解決建議**:
1. 斷電重啟（關閉 24V，等待 10 秒，重新上電）
2. 如果重啟後仍顯示「通訊超時」，建議聯繫廠商

---

### 2026-01-15: 初始調試

### 一開始為什麼不能動？

**初始症狀**：執行 `python3 usbcanfd_scan.py` 後靈巧手完全沒反應

**診斷過程**：

1. **檢查 CAN 介面狀態**
   ```bash
   ip -d link show can0
   # 發現: state BUS-OFF, berr-counter tx 248 rx 0
   # 原因: Jetson 內建 can0 沒有連接到靈巧手
   ```

2. **檢查 USB 設備**
   ```bash
   lsusb | grep -i can
   # 顯示: ID 04d8:0053 Microchip Technology, Inc. USB CANFD DEBUG
   ```

3. **嘗試用 ZLGCAN SDK 開啟設備**
   - 結果: `[ERROR] 無法開啟設備 (handle=NULL)`
   - 原因: USB 接口可能接觸不良

### 如何解決？

**關鍵步驟：換 USB 接口**

```bash
# 1. 拔掉 USB CANFD 設備
# 2. 換插到另一個 USB 接口
# 3. 確認設備識別
lsusb | grep -i can

# 4. 再次執行測試
python3 ~/openarm_can/setup/usbcanfd_scan.py -t demo
```

**成功後的輸出**：
```
[INFO] 設備已開啟 (handle=0xFFFF...)  # 不是 NULL 就是成功
```

---

### 測試結果總結

| 項目 | 狀態 | 說明 |
|------|------|------|
| USB CANFD 發送 | ✅ 正常 | 可以發送命令 |
| USB CANFD 接收 | ✅ 正常 | 2026-03-12 修正結構體順序後正常工作 |
| 左手基本動作 | ✅ 正常 | 回零/張開/握緊 穩定 |
| 右手基本動作 | ⚠️ 不穩定 | 有時正常，有時會卡住 |
| 右手大拇指 | ❌ 故障 | 經常卡住，可能硬體問題 |
| 位置模式 (0x01) | ⚠️ 不穩定 | 容易導致手指卡住 |

---

### 發現的問題

#### 問題 1: USB CANFD 接收功能不工作 (已修正 ✅)

**症狀**: `usbcanfd_scan.py` 顯示「靈巧手無回應」

**原因**: 腳本會發送讀取命令 (0xFE) 並等待回應，但 USB CANFD 的接收功能不正常

**解決**: 已修改腳本，移除讀取檢查，改為直接發送回零命令

#### 問題 2: 右手不穩定

**症狀**: 
- 執行幾次動作後會卡住
- 大拇指經常先卡住，然後整隻手卡住
- 重新上電後可以動幾次，然後又卡住

**可能原因**:
1. 右手大拇指馬達/齒輪問題
2. 右手控制板問題
3. USB CANFD 設備不穩定

**建議**: 聯繫廠商檢查右手硬體

#### 問題 3: 位置模式命令容易導致卡住

**症狀**: 使用位置模式 (Byte2=0x01) 時手指會卡住

**解決**: 只使用基本命令（回零/張開/握緊），避免位置模式

#### 問題 4: 訊息塞車（命令發送過快）

**症狀**: 
- 手指只動了一點點就卡住
- 左右手部分手指動，其他不動
- CAN 發送成功 (result=True) 但硬體沒有完整反應

**原因**: 
- 發送頻率過高（如 50Hz），下一個命令到達時上一個動作還沒完成
- 左右手連續發送沒有間隔，CAN FD 總線負載過高
- 重複發送相同位置導致硬體處理不過來

**解決方案**（已實作在 `unity_interface_follower.py`）:

1. **降低控制頻率**: 從 50Hz 降到 5Hz（每 200ms 發送一組命令）
   ```python
   DEXTEROUS_HAND_CONTROL_FREQ = 5  # 控制頻率 (Hz)
   ```

2. **左右手發送間隔**: 在左手和右手發送之間加入 50ms 延遲
   ```python
   time.sleep(0.05)  # 左手發送後等待 50ms 再發送右手
   ```

3. **位置變化偵測**: 如果位置沒有明顯變化（閾值 5 = 約 2%），則跳過發送
   ```python
   CHANGE_THRESHOLD = 5  # 最大變化 < 5 則不發送
   if max_change < CHANGE_THRESHOLD:
       return False  # 跳過發送
   ```

**最佳實踐**:
- 發送頻率建議 5-10 Hz（100-200ms 間隔）
- 每次發送後建議等待 50-100ms
- 避免重複發送相同的位置命令

#### 問題 5: Reserved 位元組應填 0x00（2026-01-22 修正）

**症狀**: 單指控制時（如比「1」、比「2」），手指動作不完整

**原因**: 封包中的 Reserved 位元組填了 `0xFF`，但手冊規範應填 `0x00`

**手冊規範**:
- 每組馬達參數 5 bytes: `Position, Speed, Torque, Reserved, Reserved`
- 手冊示例中 Reserved 一律填 `0x00`
- 填 `0xFF` 可能被韌體視為非法參數

**代碼修改** (`unity_interface_follower.py`):

```python
# 修改 1: 回零命令 (第 ~420 行)
# 修改前
cmd = bytes([0xFD, 0x04] + [0xFF] * 30)
# 修改後
cmd = bytes([0xFD, 0x04] + [0x00] * 30)

# 修改 2: 位置控制 (第 ~494 行)
# 修改前
data.extend([pos_value, DEXTEROUS_HAND_SPEED, DEXTEROUS_HAND_TORQUE, 0xFF, 0xFF])
# 修改後
data.extend([pos_value, DEXTEROUS_HAND_SPEED, DEXTEROUS_HAND_TORQUE, 0x00, 0x00])

# 修改 3: 張開命令 (第 ~852 行)
# 修改前
cmd_open = bytes([0xFD, 0x02] + [0xFF] * 30)
# 修改後
cmd_open = bytes([0xFD, 0x02] + [0x00] * 30)
```

**封包格式對照**:
```
修改前（錯誤）: FD 01 00 C8 C8 FF FF 00 C8 C8 FF FF ...
修改後（正確）: FD 01 00 C8 C8 00 00 00 C8 C8 00 00 ...
```

#### 問題 6: 單指控制動作不完整的真正原因（2026-01-22 結論）

**症狀**: 
- 比「2」時，食指/中指沒有完全伸直
- 其他手指沒有完全彎曲
- 移動速度很慢

**診斷過程**:

1. 檢查 Unity 發送的原始數據：
   ```
   [ehand] L=[0.78, 0.68, 0.24, 0.17, 0.76, 1.0], R=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
   ```

2. 分析數據：
   | 手指 | 期望動作 | 期望值 | Unity 實際發送 | 轉換後 |
   |------|----------|--------|----------------|--------|
   | 食指 (F3) | 完全伸展 | 0.0 或 1.0 | **0.24** | 61 |
   | 中指 (F4) | 完全伸展 | 0.0 或 1.0 | **0.17** | 43 |
   | 其他 | 收緊 | 0.0 或 1.0 | 0.68~1.0 | 173~255 |

**結論**: 
- **問題不在靈巧手控制程式**
- **問題在 Unity 端的數據來源**（手部追蹤/手勢識別）
- Unity 發送的值是中間值（0.17~0.78），不是極值（0.0 或 1.0）
- 靈巧手程式正確地把 0.24 轉成 61 發送，只是 61 不是「完全伸展」的位置

**解決方向**:
1. **Unity 端調整**：讓手勢識別輸出更接近 0.0 / 1.0 的極值
2. **或在 Linux 端加閾值映射**：
   ```python
   # 範例：小於 0.3 強制變 0.0，大於 0.7 強制變 1.0
   if value < 0.3:
       value = 0.0
   elif value > 0.7:
       value = 1.0
   ```

#### 問題 7: 大拇指與其他手指干涉卡住（2026-01-22）

**症狀**: 
- **握拳時**：大拇指與其他手指同時彎曲，大拇指會卡到食指或中指
- **張開時**：從握拳狀態張開，大拇指因為還沒退開而阻礙其他手指伸展
- **單指伸出**：從握拳要伸出某一指時，大拇指沒有先退開，導致卡住

**原因分析**:

靈巧手的物理結構問題：
- M1=大拇指旋轉，M2=大拇指伸縮
- M3~M6=食指、中指、無名指、小指
- 當所有手指**同時移動**時，大拇指的運動路徑會與其他手指重疊

**解決方案：分階段動作控制**

採用**順序性動作**，避免同時移動導致干涉：

##### 策略 1: 握拳動作改良（張開 → 握緊）
```
階段1: 4 指（M3~M6）先彎曲到 50%          (0.3s)
階段2: 大拇指旋轉到位（M1）                (0.3s)
階段3: 4 指完全握緊（M3~M6 = 255）         (0.5s)
階段4: 大拇指彎曲收緊（M2 = 255）           (0.3s)
```

##### 策略 2: 張開動作改良（握緊 → 張開）
```
階段1: 大拇指先完全伸直（M1=0, M2=0）      (0.3s)
階段2: 其他 4 指伸直（M3~M6=0）             (0.5s)
```

##### 策略 3: 單指伸出改良（從握拳狀態）
```
階段1: 大拇指先退開（M1=0, M2=0）          (0.3s)
階段2: 目標手指伸出，其他保持握拳           (0.5s)
```

**程式碼實作範例**（位於 `usbcanfd_scan.py`）:

```python
def build_gesture_sequence(gesture_type: str) -> list:
    """
    構建分階段手勢序列
    
    Returns:
        list of (positions, delay): 每階段的位置和延遲時間
    """
    if gesture_type == "close":  # 握拳
        return [
            ([255, 255, 128, 128, 128, 128], 0.3),  # 4指先半握
            ([128, 255, 128, 128, 128, 128], 0.3),  # 拇指旋轉
            ([128, 255, 255, 255, 255, 255], 0.5),  # 4指完全握緊
            ([128, 255, 255, 255, 255, 255], 0.3),  # 拇指收緊
        ]
    
    elif gesture_type == "open":  # 張開
        return [
            ([0,   0,   255, 255, 255, 255], 0.3),  # 拇指先伸直
            ([0,   0,   0,   0,   0,   0  ], 0.5),  # 全部伸直
        ]
    
    elif gesture_type == "thumbs_up":  # 讚
        return [
            ([255, 255, 255, 255, 255, 255], 0.3),  # 先握拳
            ([0,   100, 255, 255, 255, 255], 0.3),  # 拇指稍退
            ([0,   0,   255, 255, 255, 255], 0.5),  # 拇指伸直
        ]
```

**測試方法**:

```bash
# 使用改良後的互動式控制
sudo python3 ~/openarm_can/setup/usbcanfd_scan.py

# 選單中選擇:
# 2. 張開
# 3. 握緊
# 觀察大拇指是否會卡到其他手指
```

**預期效果**:
- 握拳時，4 指先收緊，大拇指最後收緊，不會卡住
- 張開時，大拇指先退開，其他手指才伸展，動作流暢
- 單指伸出時，大拇指先退避，目標手指才伸出

**實作狀態**: 🚧 待實作（需修改 `usbcanfd_scan.py` 和 `unity_interface_follower.py`）

---

### 穩定的測試方法

```bash
# 最穩定的測試方式（只用基本命令）
cd /home/idaka/openarm_can/setup && python3 << 'EOF'
import sys
sys.path.insert(0, '.')
from usbcanfd_scan import ZLGCAN, MODE_NORMAL
import time

can = ZLGCAN()
can.open_device()
can.set_baudrate(0, 1000000, 5000000)
can.init_channel(0, MODE_NORMAL)
can.start_can()

HAND_ID = 0x12  # 左手=0x12, 右手=0x11

CMD_HOME = bytes([0xFD, 0x04] + [0x00] * 30)
CMD_OPEN = bytes([0xFD, 0x02] + [0x00] * 30)
CMD_CLOSE = bytes([0xFD, 0x03] + [0x00] * 30)

print("[回零]")
can.transmit_fd(HAND_ID, CMD_HOME)
time.sleep(3)

print("[張開]")
can.transmit_fd(HAND_ID, CMD_OPEN)
time.sleep(2)

print("[握緊]")
can.transmit_fd(HAND_ID, CMD_CLOSE)
time.sleep(2)

print("[張開]")
can.transmit_fd(HAND_ID, CMD_OPEN)

can.close()
EOF
```

---

### 故障排除速查表

| 問題 | 解決方案 |
|------|----------|
| handle=NULL | **換 USB 接口** |
| 手沒反應 | 1. 確認 24V 電源 2. 換 USB 接口 3. 重新上電 |
| 「靈巧手無回應」 | 檢查結構體順序是否正確（frame 在前，timestamp 在後） |
| 手指卡住 | 1. 重新上電 2. 避免使用位置模式 |
| 右手不穩定 | 建議只用左手測試，右手可能有硬體問題 |
| `usb_detach_kernel_driver_np error` | 可忽略，不影響功能 |

---

### 開發建議

1. **優先使用左手**進行開發測試（較穩定）
2. **只用基本命令**：回零 (0x04)、張開 (0x02)、握緊 (0x03)
3. **避免位置模式** (0x01)：容易導致卡住
4. 如果連不上，**先換 USB 接口或重新上電**

---

## 12. 參考資料

- [HITBOT 官網](https://www.hitbot.cc/ehand-6/)
- 驅動程式: `openarm_can/python/openarm/can/dexterous_hand.py`
- 測試腳本: `openarm_can/setup/test_dexterous_hand.py`
- USBCANFD 工具: `openarm_can/setup/usbcanfd_scan.py`

---

### 2026-03-12: 修正 USB CANFD 接收功能

**問題**: `usbcanfd_scan.py` 的 `ZCAN_ReceiveFD_Data` 結構體順序錯誤

**症狀**: 
- `ZCAN_GetReceiveNum()` 返回 > 0，但數據解析錯誤
- CAN ID 顯示異常值（如 `0xFD3104FE`）
- 訊息長度顯示為 0

**根本原因**:
```python
# 錯誤順序 (原本)
class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [
        ("timestamp", c_uint64),
        ("frame", ZCAN_CANFD_FRAME),  # 順序錯誤！
    ]

# 正確順序 (官方範例)
class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [
        ("frame", ZCAN_CANFD_FRAME),  # frame 在前
        ("timestamp", c_uint64),       # timestamp 在後
    ]
```

**修正檔案**: `~/openarm_can/setup/usbcanfd_scan.py`

**驗證**:
```bash
# 讀取右手狀態
sudo python3 -c "
import sys, time
sys.path.insert(0, /home/idaka/openarm_can/setup)
from usbcanfd_scan import ZLGCAN, MODE_NORMAL, TYPE_CANFD

can = ZLGCAN()
can.open_device()
can.set_baudrate(0, 1000000, 5000000)
can.init_channel(0, MODE_NORMAL)
can.start_can()

can.transmit_fd(0x11, bytes([0xFC] + [0x00] * 31))
time.sleep(0.3)

num = can.get_receive_num(TYPE_CANFD)
if num > 0:
    msgs = can.receive_fd(num, 100)
    data = bytes([msgs[0].frame.data[i] for i in range(32)])
    print(原始:,  .join({:02X}.format(b) for b in data[:16]))
    if (data[0] & 0x03) == 2:
        print(✅ 接收功能正常!)
        for i, name in enumerate([拇指旋轉,拇指伸縮,食指,中指,無名指,小指]):
            print(f {name}: {data[2+i*5+1]}/255)
can.close()
"
```

**參考**: `~/CANFD_Docs/二次开发样例源码/python(x64)_example_python3.8.8_v2.0/cxcanfd_x64_v2.0.py`
