# 靈巧手整合指南

> **狀態**: ⚠️ 部分功能正常（左手穩定，右手不穩定）  
> **最後更新**: 2026-01-15

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

# 手動發送指令
cansend canX 012##1FD04FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF  # 回零
cansend canX 012##1FD02FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF  # 張開
cansend canX 012##1FD03FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF  # 握緊
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
| Byte3~32 | 6組馬達參數 | 每組 5 bytes: Position, Speed, Torque, Reserved×2 |

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

## 6. 程式碼使用

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

## 7. 整合到 unity_interface_follower.py

修改 `unity_interface_follower.py` 中的設定：

```python
# 切換末端執行器類型
RIGHT_END_EFFECTOR_TYPE = "dexterous_hand"  # 或 "gripper"

# 靈巧手配置
DEXTEROUS_HAND_CAN_INTERFACE = "can0"  # 獨立 CAN 介面
DEXTEROUS_HAND_CAN_ID = 0x12           # 左手=0x12, 右手=0x11
```

---

## 8. 故障排除

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

## 9. USBCANFD 設備

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

## 10. 調試歷程記錄 (2026-01-15)

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
| USB CANFD 接收 | ❌ 不工作 | 無法接收靈巧手回應（設備可能損壞） |
| 左手基本動作 | ✅ 正常 | 回零/張開/握緊 穩定 |
| 右手基本動作 | ⚠️ 不穩定 | 有時正常，有時會卡住 |
| 右手大拇指 | ❌ 故障 | 經常卡住，可能硬體問題 |
| 位置模式 (0x01) | ⚠️ 不穩定 | 容易導致手指卡住 |

---

### 發現的問題

#### 問題 1: USB CANFD 接收功能不工作

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

CMD_HOME = bytes([0xFD, 0x04] + [0xFF] * 30)
CMD_OPEN = bytes([0xFD, 0x02] + [0xFF] * 30)
CMD_CLOSE = bytes([0xFD, 0x03] + [0xFF] * 30)

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
| 「靈巧手無回應」 | 可忽略（接收功能不工作），直接操作 |
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

## 11. 參考資料

- [HITBOT 官網](https://www.hitbot.cc/ehand-6/)
- 驅動程式: `openarm_can/python/openarm/can/dexterous_hand.py`
- 測試腳本: `openarm_can/setup/test_dexterous_hand.py`
- USBCANFD 工具: `openarm_can/setup/usbcanfd_scan.py`
