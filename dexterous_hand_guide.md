# 靈巧手整合指南

> **狀態**: 🔧 硬體驗證中  
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
| 有回應但不動 | 接線反 | 交換 CAN_H/L |
| 有回應但不動 | 電壓不足 | 確認 24V (非 20V) |

---

## 9. 參考資料

- [HITBOT 官網](https://www.hitbot.cc/ehand-6/)
- 驅動程式: `openarm_can/python/openarm/can/dexterous_hand.py`
- 測試腳本: `openarm_can/setup/test_dexterous_hand.py`
