# 靈巧手整合技術規格書

> **狀態**: ✅ 程式碼完成，待硬體測試  
> **最後更新**: 2026-01-13  
> **請在此文件中修正不正確的內容**

---

## 決策記錄

| 項目 | 決策 | 說明 |
|------|------|------|
| C++ 實作 | ❌ 暫不需要 | 先用 Python 快速驗證，測試通過後再視需求加 C++ |
| 保留舊夾爪 | ✅ 需保留 | 手還沒裝上，程式碼要能切換 Gripper / DexterousHand |
| 測試範圍 | 🔵 右手優先 | 先測試右手，成功後再擴展到左手 |

---

## 架構設計原則

### ✅ 確認：以新增檔案為主

```
┌────────────────────────────────────────────────────────────────────────────┐
│  架構設計                                                                   │
│                                                                            │
│  1. 靈巧手核心邏輯 → 【新增獨立檔案】                                       │
│     └─ dexterous_hand.py (完全獨立，不依賴現有 openarm_can)                │
│                                                                            │
│  2. 切換與測試 → 【修改 unity_interface_follower.py】                       │
│     └─ 加入開關變數，import 新模組，最小化修改                              │
│                                                                            │
│  3. 舊夾爪邏輯 → 【完全保留不動】                                           │
│     └─ gripper_component.hpp/cpp 零修改                                    │
│                                                                            │
└────────────────────────────────────────────────────────────────────────────┘
```

### 檔案職責分工

| 檔案 | 職責 | 修改程度 |
|------|------|----------|
| `dexterous_hand.py` 🆕 | 靈巧手通訊協議、封包組裝、CAN FD 發送 | **新增**（核心邏輯） |
| `test_dexterous_hand.py` 🆕 | 獨立測試靈巧手（不需 ROS2） | **新增** |
| `unity_interface_follower.py` 📝 | 切換開關 + 呼叫靈巧手 | **小幅修改**（約 30 行） |
| `gripper_component.*` ✅ | 舊夾爪控制 | **不動** |

---

## 一、硬體資訊

### 1.1 介面與線序（4 芯 M8）

| 顏色 | 功能 |
|------|------|
| 棕 | 24V+ |
| 藍 | 0V / GND |
| 白 | CAN_L |
| 黑 | CAN_H |

### 1.2 通訊參數

| 參數 | 值 | 備註 |
|------|-----|------|
| 通訊類型 | CAN FD | 非 classical CAN |
| 仲裁波特率 | 1,000,000 bps | 1 Mbps |
| 數據波特率 | 5,000,000 bps | 5 Mbps |
| 終端電阻 | 120Ω | 不穩時補上 |

### 1.3 CAN ID 設定

<!-- TODO: 請確認靈巧手的 CAN ID -->

| 項目 | CAN ID | 備註 |
|------|--------|------|
| 靈巧手發送 ID | `???` | 待確認 |
| 靈巧手接收 ID | `???` | 待確認 |

---

## 二、32 Bytes 封包格式

### 2.1 整體結構

```
[Byte1][Byte2][Motor1: 5bytes][Motor2: 5bytes][Motor3: 5bytes][Motor4: 5bytes][Motor5: 5bytes][Motor6: 5bytes]
  1B  +  1B  +      5B      +      5B      +      5B      +      5B      +      5B      +      5B      = 32 bytes
```

### 2.2 Byte1：馬達選擇 + 讀寫命令

```
高 6 位: 選擇馬達 (Motor1~6)
低 2 位: 操作命令
         01 = 寫入
         10 = 讀出
```

**常用值（寫入）**:

| 值 | 意義 |
|----|------|
| `0xFD` | 全選（6顆）寫入 |
| `0x05` | 拇指旋轉 (M1) 寫入 |
| `0x09` | 拇指伸縮 (M2) 寫入 |
| `0x11` | 食指 (M3) 寫入 |
| `0x21` | 中指 (M4) 寫入 |
| `0x41` | 無名指 (M5) 寫入 |
| `0x81` | 尾指 (M6) 寫入 |
| `0x19` | 拇指伸縮+食指 寫入 |

**讀出版本**: 同 mask，低 2 位改為 `10`
- 全選讀: `0xFE`（或 `0xFC`，待確認）

### 2.3 Byte2：控制字

```
高 4 位: 點位編號 (0~15)，先填 0
低 4 位: 控制模式
         0x01 = 位置模式
         0x02 = 伸展（打開）
         0x03 = 收緊（握緊）
         0x04 = 初始化回零
```

### 2.4 Byte3~32：馬達參數（每顆 5 bytes × 6）

| Offset | 內容 | 範圍 |
|--------|------|------|
| +0 | Position | 0~255 |
| +1 | Speed | 0~255 |
| +2 | Torque | 0~255 |
| +3 | 保留 | 0x00 |
| +4 | 保留 | 0x00 |

### 2.5 馬達對應

| 索引 | Motor | 功能 |
|------|-------|------|
| 0 | Motor1 | 拇指旋轉 |
| 1 | Motor2 | 拇指伸縮 |
| 2 | Motor3 | 食指 |
| 3 | Motor4 | 中指 |
| 4 | Motor5 | 無名指 |
| 5 | Motor6 | 尾指 |

---

## 三、常用封包範例

### 3.1 初始化回零

```
Byte1 = 0xFD (全選寫入)
Byte2 = 0x04 (回零)
Motor1~6 = FF FF FF 00 00 (每組)

完整 32 bytes (hex):
FD 04 FF FF FF 00 00 FF FF FF 00 00 FF FF FF 00 00 FF FF FF 00 00 FF FF FF 00 00 FF FF FF 00 00
```

### 3.2 伸展（全開）

```
Byte1 = 0xFD
Byte2 = 0x02

完整 32 bytes (hex):
FD 02 FF FF FF 00 00 FF FF FF 00 00 FF FF FF 00 00 FF FF FF 00 00 FF FF FF 00 00 FF FF FF 00 00
```

### 3.3 收緊（全握）

```
Byte1 = 0xFD
Byte2 = 0x03

完整 32 bytes (hex):
FD 03 FF FF FF 00 00 FF FF FF 00 00 FF FF FF 00 00 FF FF FF 00 00 FF FF FF 00 00 FF FF FF 00 00
```

### 3.4 位置模式範例

```
Byte1 = 0xFD (全選)
Byte2 = 0x01 (位置模式)
各馬達: Position=128, Speed=128, Torque=128

完整 32 bytes (hex):
FD 01 80 80 80 00 00 80 80 80 00 00 80 80 80 00 00 80 80 80 00 00 80 80 80 00 00 80 80 80 00 00
```

---

## 四、接線方案

### 4.1 選項 A：獨立 CAN 介面（推薦）

```
Jetson CAN 介面分配:
- can1: 右手臂 (已使用)
- can2: 左手臂 (已使用)
- can0 或 USB-CAN: 靈巧手 (新增)
```

<!-- TODO: 確認要用哪個 CAN 介面 -->

**選擇的 CAN 介面**: `______`

### 4.2 選項 B：共用 CAN 介面

將靈巧手接在左/右手臂同一條 CAN bus 上，需注意：
- CAN ID 不能衝突
- 帶寬分配

---

## 五、專案結構與檔案變更

### 5.1 現有專案結構（相關部分）

```
/home/idaka/
├── openarm_can/                          # CAN 控制函式庫
│   ├── include/openarm/
│   │   ├── can/socket/
│   │   │   ├── arm_component.hpp         # 手臂組件
│   │   │   ├── gripper_component.hpp     # 舊夾爪組件
│   │   │   └── openarm.hpp               # 主控制類別
│   │   ├── canbus/
│   │   │   ├── can_socket.hpp            # CAN Socket 封裝
│   │   │   └── can_device.hpp
│   │   └── damiao_motor/                 # 達妙馬達驅動
│   │       └── ...
│   ├── src/openarm/
│   │   ├── can/socket/
│   │   │   ├── arm_component.cpp
│   │   │   ├── gripper_component.cpp
│   │   │   └── openarm.cpp
│   │   └── canbus/
│   │       └── can_socket.cpp            # 已支援 CAN FD
│   ├── python/
│   │   ├── openarm/can/
│   │   │   ├── __init__.py
│   │   │   └── core.py
│   │   └── src/
│   │       └── openarm_can.cpp           # Python bindings
│   └── setup/
│       ├── configure_socketcan.sh        # CAN 設定腳本
│       └── configure_socketcan_4_arms.sh
│
└── ros2_ws/src/openarm_ros2/
    └── openarm_bringup/scripts/
        ├── unity_interface_follower.py   # 主要控制腳本 ⭐
        └── ...
```

### 5.2 需要新增的檔案

| 檔案路徑 | 說明 | 優先級 | 狀態 |
|----------|------|--------|------|
| `openarm_can/python/openarm/can/dexterous_hand.py` | 靈巧手 Python 模組 | 🔴 高 | ✅ 完成 |
| `openarm_can/setup/test_dexterous_hand.py` | 靈巧手測試腳本 | 🔴 高 | ✅ 完成 |
| `openarm_can/include/openarm/can/socket/dexterous_hand.hpp` | 靈巧手 C++ 標頭檔 | ⚪ 暫緩 | Python 驗證後再做 |
| `openarm_can/src/openarm/can/socket/dexterous_hand.cpp` | 靈巧手 C++ 實作 | ⚪ 暫緩 | Python 驗證後再做 |

### 5.3 需要修改的檔案

| 檔案路徑 | 修改內容 | 優先級 | 狀態 |
|----------|----------|--------|------|
| `ros2_ws/.../unity_interface_follower.py` | 新增切換開關 + 靈巧手控制（右手） | 🔴 高 | ✅ 完成 |
| `openarm_can/python/openarm/can/__init__.py` | 匯出靈巧手模組 | 🟡 中 | ✅ 完成 |
| `openarm_can/python/src/openarm_can.cpp` | 新增 Python bindings | ⚪ 暫緩 | 視需求 |
| `openarm_can/include/openarm/can/socket/openarm.hpp` | 新增 `init_dexterous_hand()` | ⚪ 暫緩 | 視需求 |

**優先級圖例**: 🔴 高 / 🟡 中 / ⚪ 暫緩

### 5.4 unity_interface_follower.py 修改範圍（精確行號）

```python
# ============================================================================
# 現有檔案結構 (549 行) - 修改點標記
# ============================================================================

# 【修改點 1】第 48 行後 - 新增配置區塊
# 現有: GRIPPER_RECV_ID = 0x18
# 新增: RIGHT_END_EFFECTOR_TYPE, DEXTEROUS_HAND_* 等配置變數

# 【修改點 2】第 163 行後 - __init__ 中
# 現有: self.right_arm.init_gripper_motor(...)
# 新增: 靈巧手初始化邏輯 (條件式)

# 【修改點 3】第 402~412 行 - control_loop 中
# 現有:
#   left_grip_cmds = [oa.MITParam(GRIPPER_KP, GRIPPER_KD, left_grip, 0.0, 0.0)]
#   right_grip_cmds = [oa.MITParam(GRIPPER_KP, GRIPPER_KD, right_grip, 0.0, 0.0)]
#   ...
#   self.right_arm.get_gripper().mit_control_all(right_grip_cmds)
# 修改: 右手改為條件式，判斷使用靈巧手或舊夾爪

# 【修改點 4】第 528 行後 - shutdown 中
# 現有: self.right_arm.disable_all()
# 新增: 靈巧手關閉邏輯

# ============================================================================
# 預估修改量
# ============================================================================
# 新增配置: ~15 行
# 初始化: ~10 行
# 控制邏輯: ~15 行
# 關閉: ~5 行
# 總計: ~45 行新增/修改（原檔案 549 行）
```

### 5.5 整合後的結構圖

```
/home/idaka/
├── openarm_can/
│   ├── include/openarm/can/socket/
│   │   ├── arm_component.hpp
│   │   ├── gripper_component.hpp         # ✅ 保留（切換用）
│   │   ├── dexterous_hand.hpp            # ⚪ 暫緩（C++ 版本）
│   │   └── openarm.hpp
│   ├── src/openarm/can/socket/
│   │   ├── arm_component.cpp
│   │   ├── gripper_component.cpp         # ✅ 保留
│   │   ├── dexterous_hand.cpp            # ⚪ 暫緩（C++ 版本）
│   │   └── openarm.cpp
│   ├── python/openarm/can/
│   │   ├── __init__.py                   # 📝 修改
│   │   ├── core.py
│   │   └── dexterous_hand.py             # 🆕 新增（Python 版本）
│   └── setup/
│       ├── configure_socketcan.sh
│       └── test_dexterous_hand.py        # 🆕 新增
│
└── ros2_ws/src/openarm_ros2/
    └── openarm_bringup/scripts/
        └── unity_interface_follower.py   # 📝 修改（加切換開關）
```

**圖例**:
- 🆕 = 新增檔案（優先）
- 📝 = 需要修改
- ✅ = 保留不動
- ⚪ = 暫緩（測試成功後再做）

---

## 六、軟體架構

### 6.1 C++ vs Python 說明

| 方案 | 優點 | 缺點 | 適用場景 |
|------|------|------|----------|
| **Python** | 開發快、調試方便、無需編譯 | 效能略低、GIL 限制 | 快速驗證、控制頻率 <200Hz |
| **C++** | 高效能、無 GIL、與現有架構一致 | 開發慢、需編譯 | 高頻控制、正式產品 |

**當前決策**: 先用 **Python** 快速驗證，靈巧手控制頻率 50~100Hz 足夠。測試通過後視需求再加 C++。

### 6.2 末端執行器切換機制

```
┌─────────────────────────────────────────────────────────────┐
│  unity_interface_follower.py                                │
│                                                             │
│  # 配置開關                                                 │
│  END_EFFECTOR_TYPE = "dexterous_hand"  # 或 "gripper"       │
│                                                             │
│  if END_EFFECTOR_TYPE == "dexterous_hand":                  │
│      使用 DexterousHand 類別 (32B 封包)                     │
│  else:                                                      │
│      使用原本的 GripperComponent (達妙 MIT)                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**切換方式**: 修改 `END_EFFECTOR_TYPE` 變數即可，無需改動其他程式碼。

### 6.3 控制頻率

| 組件 | 頻率 | 備註 |
|------|------|------|
| 手臂 MIT 控制 | 500 Hz | 現有 |
| 靈巧手控制 | 50~100 Hz | 建議值（CAN FD 帶寬足夠） |
| 舊夾爪控制 | 500 Hz | 與手臂同頻 |
| Unity 指令 | ~60 Hz | 現有 |

### 6.2 Unity 介面設計

**方案 A：簡單映射（推薦起步）**

Unity 只傳一個 `grip_value` (0~1)，驅動程式內部映射到 6 指：

```
Unity: grip_value = 0.0 (全開) ~ 1.0 (全握)
       ↓
Driver: 6 指都設為 position = grip_value * 255
```

**方案 B：完整控制**

Unity 傳 6 個獨立手指位置 + 手勢模式：

```
topic: /unity/hand_commands
  - positions: [thumb_rot, thumb_ext, index, middle, ring, pinky]  # 0~255
  - gesture: "open" | "close" | "pinch" | "position"
  - speed: 0~255
  - torque: 0~255
```

<!-- TODO: 選擇哪個方案？ -->

**選擇**: `______`

### 6.4 控制流程圖（右手測試配置）

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Unity (Windows)                                 │
│                                                                             │
│   [VR Controller] ──► R_EE grip_value (0~1) ──► ROS2 Publisher             │
└─────────────────────────────────────────────┬───────────────────────────────┘
                                              │
                                              │ /unity/joint_commands
                                              │ (JointState msg)
                                              ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Jetson (Linux + ROS2)                               │
│                                                                             │
│   unity_interface_follower.py                                               │
│   ┌───────────────────────────────────────────────────────────────────┐    │
│   │                                                                   │    │
│   │   END_EFFECTOR_TYPE = "dexterous_hand"  # 切換開關               │    │
│   │                                                                   │    │
│   │   unity_callback()                                                │    │
│   │        │                                                          │    │
│   │        ▼                                                          │    │
│   │   ┌─────────────┐     ┌─────────────┐     ┌─────────────────┐    │    │
│   │   │ Left Arm    │     │ Right Arm   │     │ Right Hand      │    │    │
│   │   │ Target      │     │ Target      │     │ Target (R_EE)   │    │    │
│   │   │ (7 joints)  │     │ (7 joints)  │     │ grip: 0~1       │    │    │
│   │   └──────┬──────┘     └──────┬──────┘     └────────┬────────┘    │    │
│   │          │                   │                     │ 🆕          │    │
│   │          ▼                   ▼                     ▼             │    │
│   │   ┌──────────────────────────────────────────────────────────┐   │    │
│   │   │                  control_loop (500Hz)                     │   │    │
│   │   │                                                          │   │    │
│   │   │   MIT Control ──────────────────► CAN1 (Right Arm)       │   │    │
│   │   │   MIT Control ──────────────────► CAN2 (Left Arm)        │   │    │
│   │   │   舊夾爪 MIT ───────────────────► CAN1 (Right Gripper)   │   │    │
│   │   │        或                                                │   │    │
│   │   │   Hand 32B ─────────────────────► CAN? (Dexterous Hand)  │   │    │
│   │   │                                   ↑ 依 END_EFFECTOR_TYPE │   │    │
│   │   └──────────────────────────────────────────────────────────┘   │    │
│   └───────────────────────────────────────────────────────────────────┘    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                              │
                    ┌─────────────────────────┼─────────────────────────┐
                    │                         │                         │
                    ▼                         ▼                         ▼
            ┌───────────────┐         ┌───────────────┐         ┌───────────────┐
            │   CAN1        │         │   CAN2        │         │   CAN?        │
            │   Right Arm   │         │   Left Arm    │         │   Right Hand  │
            │   (達妙 MIT)  │         │   (達妙 MIT)  │         │   (32B 封包)  │
            └───────────────┘         └───────────────┘         └───────────────┘
                    │                         │                         │
                    ▼                         ▼                         ▼
            ┌───────────────┐         ┌───────────────┐         ┌───────────────┐
            │  7x DM Motors │         │  7x DM Motors │         │  6x 手指馬達  │
            │  + Gripper    │         │  + Gripper    │         │  (靈巧手)     │
            │  (可切換)     │         │  (保留原樣)  │         │  🆕 測試中    │
            └───────────────┘         └───────────────┘         └───────────────┘

圖例:
🆕 = 新增功能
右手: 可切換 Gripper ↔ DexterousHand
左手: 暫時保持原夾爪，測試成功後再擴展
```

### 6.4 封包組裝流程

```
Unity grip_value (0.0~1.0)
        │
        ▼
┌───────────────────────────────────────┐
│  grip_to_hand_positions()             │
│                                       │
│  positions = [                        │
│    thumb_rot,   # Motor1 (0 或固定)   │
│    thumb_ext,   # Motor2              │
│    index,       # Motor3              │
│    middle,      # Motor4              │
│    ring,        # Motor5              │
│    pinky        # Motor6              │
│  ]                                    │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│  build_hand_command()                 │
│                                       │
│  packet[0] = 0xFD (全選寫入)          │
│  packet[1] = 0x01 (位置模式)          │
│  packet[2:7]   = Motor1 params        │
│  packet[7:12]  = Motor2 params        │
│  packet[12:17] = Motor3 params        │
│  packet[17:22] = Motor4 params        │
│  packet[22:27] = Motor5 params        │
│  packet[27:32] = Motor6 params        │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│  CAN FD Frame                         │
│                                       │
│  can_id = HAND_CAN_ID                 │
│  data   = 32 bytes packet             │
│  flags  = CANFD_BRS                   │
└───────────────────┬───────────────────┘
                    │
                    ▼
              [SocketCAN]
                    │
                    ▼
              [靈巧手硬體]
```

---

## 七、待確認事項清單

請在確認後打勾 ✅：

### 硬體相關
- [ ] 靈巧手的 CAN ID（發送/接收）
- [ ] 全選讀取的 Byte1 值是 `0xFE` 還是 `0xFC`？
- [ ] 使用哪個 CAN 介面（can0 / USB-CAN / 共用 can1）
- [ ] 是否需要 120Ω 終端電阻（已有還是要補？）
- [ ] Position 值 0 和 255 分別對應開還是合？
- [ ] 上電後是否必須先執行回零 (0x04)？

### 軟體相關（已決策）
- [x] ~~是否需要 C++ 實作？~~ → **暫不需要**，先用 Python 快速驗證
- [x] ~~是否保留舊的 GripperComponent？~~ → **需保留**，加入切換開關
- [x] ~~左右手都要接靈巧手嗎？~~ → **先測試右手**，成功後再擴展

### 軟體相關（待確認）
- [ ] 建議的控制頻率（目前預設 50Hz）
- [ ] Unity 介面選擇（方案 A 或 B）

---

## 八、測試步驟

### Step 1：設定 CAN FD

```bash
# 替換 canX 為實際使用的介面
sudo ip link set canX down
sudo ip link set canX type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set canX up
```

或使用現有腳本：

```bash
cd ~/openarm_can/setup
./configure_socketcan.sh canX -fd
```

### Step 2：監聽 CAN

```bash
candump canX
```

### Step 3：發送回零測試

```bash
# 注意：CAN ID 需替換為實際值
cansend canX CAN_ID##0FD04FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000
```

### Step 4：發送開合測試

```bash
# 伸展（開）
cansend canX CAN_ID##0FD02FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000

# 收緊（握）
cansend canX CAN_ID##0FD03FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000
```

---

## 九、程式碼草稿

### 9.1 dexterous_hand.py（新增）

```python
"""
靈巧手 CAN FD 控制模組
"""

import socket
import struct
from typing import List, Optional

class DexterousHand:
    """慧靈科技靈巧手控制類別"""
    
    # 控制模式
    MODE_POSITION = 0x01
    MODE_OPEN = 0x02
    MODE_CLOSE = 0x03
    MODE_HOME = 0x04
    
    # 馬達索引
    MOTOR_THUMB_ROT = 0   # 拇指旋轉
    MOTOR_THUMB_EXT = 1   # 拇指伸縮
    MOTOR_INDEX = 2       # 食指
    MOTOR_MIDDLE = 3      # 中指
    MOTOR_RING = 4        # 無名指
    MOTOR_PINKY = 5       # 尾指
    
    def __init__(self, can_interface: str, can_id: int = 0x01):
        """
        初始化靈巧手
        
        Args:
            can_interface: CAN 介面名稱 (e.g., "can0")
            can_id: 靈巧手 CAN ID (待確認)
        """
        self.can_interface = can_interface
        self.can_id = can_id
        self.sock = None
        self._connect()
    
    def _connect(self):
        """建立 CAN FD 連線"""
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        self.sock.bind((self.can_interface,))
    
    def _build_packet(self, mode: int, positions: List[int], 
                      speed: int = 128, torque: int = 128) -> bytes:
        """
        組裝 32 bytes 封包
        
        Args:
            mode: 控制模式 (0x01~0x04)
            positions: 6 個馬達位置 [0~255]
            speed: 速度 (0~255)
            torque: 力矩 (0~255)
        """
        packet = bytearray(32)
        packet[0] = 0xFD  # 全選寫入
        packet[1] = mode
        
        for i in range(6):
            offset = 2 + i * 5
            packet[offset + 0] = positions[i] if i < len(positions) else 0
            packet[offset + 1] = speed
            packet[offset + 2] = torque
            packet[offset + 3] = 0x00
            packet[offset + 4] = 0x00
        
        return bytes(packet)
    
    def _send(self, data: bytes):
        """發送 CAN FD 封包"""
        # CAN FD frame 格式: can_id (4B) + len (1B) + flags (1B) + padding (2B) + data (64B)
        frame = struct.pack("=IBB2x", self.can_id, len(data), 0x01)  # 0x01 = CANFD_BRS
        frame += data.ljust(64, b'\x00')
        self.sock.send(frame)
    
    def home(self):
        """初始化回零"""
        packet = self._build_packet(self.MODE_HOME, [0xFF]*6, 0xFF, 0xFF)
        self._send(packet)
    
    def open(self):
        """伸展（全開）"""
        packet = self._build_packet(self.MODE_OPEN, [0xFF]*6, 0xFF, 0xFF)
        self._send(packet)
    
    def close(self):
        """收緊（全握）"""
        packet = self._build_packet(self.MODE_CLOSE, [0xFF]*6, 0xFF, 0xFF)
        self._send(packet)
    
    def set_positions(self, positions: List[int], 
                      speed: int = 128, torque: int = 128):
        """
        設定各手指位置（位置模式）
        
        Args:
            positions: [thumb_rot, thumb_ext, index, middle, ring, pinky]
            speed: 速度 (0~255)
            torque: 力矩 (0~255)
        """
        packet = self._build_packet(self.MODE_POSITION, positions, speed, torque)
        self._send(packet)
    
    def set_grip(self, grip_value: float, speed: int = 128, torque: int = 128):
        """
        簡易握合控制（Unity 用）
        
        Args:
            grip_value: 0.0 (全開) ~ 1.0 (全握)
        """
        pos = int(grip_value * 255)
        positions = [0, pos, pos, pos, pos, pos]  # 拇指旋轉先不動
        self.set_positions(positions, speed, torque)
    
    def disconnect(self):
        """關閉連線"""
        if self.sock:
            self.sock.close()
            self.sock = None
```

### 9.2 unity_interface_follower.py 修改部分

```python
# ============================================================================
# 末端執行器配置（新增）
# ============================================================================

# 切換開關: "gripper" = 舊夾爪 (達妙 MIT), "dexterous_hand" = 靈巧手 (32B 封包)
RIGHT_END_EFFECTOR_TYPE = "dexterous_hand"  # 右手: 測試靈巧手
LEFT_END_EFFECTOR_TYPE = "gripper"           # 左手: 保持舊夾爪

# 靈巧手配置（僅 RIGHT_END_EFFECTOR_TYPE == "dexterous_hand" 時使用）
DEXTEROUS_HAND_CAN_INTERFACE = "can0"  # TODO: 確認實際介面
DEXTEROUS_HAND_CAN_ID = 0x01           # TODO: 確認實際 CAN ID

# 靈巧手控制參數
HAND_SPEED = 128      # 0~255
HAND_TORQUE = 128     # 0~255
HAND_CONTROL_FREQ = 50  # Hz (比手臂低)

# ============================================================================
# 在 __init__ 中新增
# ============================================================================

# 右手末端執行器初始化
self.right_hand = None
if RIGHT_END_EFFECTOR_TYPE == "dexterous_hand":
    from openarm.can.dexterous_hand import DexterousHand
    self.get_logger().info(f"Initializing Right Dexterous Hand on {DEXTEROUS_HAND_CAN_INTERFACE}...")
    self.right_hand = DexterousHand(DEXTEROUS_HAND_CAN_INTERFACE, DEXTEROUS_HAND_CAN_ID)
    self.right_hand.home()
    time.sleep(1.0)
    self.get_logger().info("✅ Right Dexterous Hand initialized!")
else:
    # 使用原本的 gripper（已在 self.right_arm 中初始化）
    self.get_logger().info("Using original gripper for right arm")

# 左手保持原樣（舊夾爪）
self.get_logger().info("Using original gripper for left arm")

# ============================================================================
# 在 control_loop 中修改夾爪/靈巧手控制部分
# ============================================================================

# 右手末端執行器控制
if RIGHT_END_EFFECTOR_TYPE == "dexterous_hand" and self.right_hand:
    # 靈巧手控制（較低頻率: 50Hz）
    if loop_count % (CONTROL_FREQUENCY // HAND_CONTROL_FREQ) == 0:
        self.right_hand.set_grip(right_grip, HAND_SPEED, HAND_TORQUE)
else:
    # 原本的夾爪 MIT 控制（500Hz，保持不變）
    right_grip_cmds = [oa.MITParam(GRIPPER_KP, GRIPPER_KD, right_grip, 0.0, 0.0)]
    self.right_arm.get_gripper().mit_control_all(right_grip_cmds)

# 左手末端執行器控制（保持原樣）
left_grip_cmds = [oa.MITParam(GRIPPER_KP, GRIPPER_KD, left_grip, 0.0, 0.0)]
self.left_arm.get_gripper().mit_control_all(left_grip_cmds)

# ============================================================================
# 在 shutdown() 中新增
# ============================================================================

if self.right_hand:
    self.right_hand.open()  # 關機前打開靈巧手
    self.right_hand.disconnect()
```

### 9.3 test_dexterous_hand.py（測試腳本）

```python
#!/usr/bin/env python3
"""
靈巧手測試腳本
用法: python3 test_dexterous_hand.py [can_interface]
"""

import sys
import time

# TODO: 根據實際路徑調整
sys.path.insert(0, '/home/idaka/openarm_can/python')
from openarm.can.dexterous_hand import DexterousHand

def main():
    can_if = sys.argv[1] if len(sys.argv) > 1 else "can0"
    
    print(f"[INFO] 連接靈巧手於 {can_if}")
    hand = DexterousHand(can_if, can_id=0x01)  # TODO: 確認 CAN ID
    
    try:
        print("[TEST] 1. 回零...")
        hand.home()
        time.sleep(2.0)
        
        print("[TEST] 2. 伸展（開）...")
        hand.open()
        time.sleep(2.0)
        
        print("[TEST] 3. 收緊（握）...")
        hand.close()
        time.sleep(2.0)
        
        print("[TEST] 4. 位置模式測試...")
        for grip in [0.0, 0.25, 0.5, 0.75, 1.0, 0.5, 0.0]:
            print(f"       grip = {grip}")
            hand.set_grip(grip)
            time.sleep(1.0)
        
        print("[DONE] 測試完成！")
        
    except KeyboardInterrupt:
        print("\n[INFO] 中斷...")
    finally:
        hand.home()
        hand.disconnect()

if __name__ == "__main__":
    main()
```

---

## 十、實作階段

### Phase 1: 基礎驗證（✅ 程式碼完成）

| 步驟 | 任務 | 檔案 | 狀態 |
|------|------|------|------|
| 1.1 | 建立 `dexterous_hand.py` | `openarm_can/python/openarm/can/` | ✅ 完成 |
| 1.2 | 建立測試腳本 | `openarm_can/setup/test_dexterous_hand.py` | ✅ 完成 |
| 1.3 | 硬體連線測試（回零/開/合） | - | ⬜ 待測試 |

### Phase 2: 整合 Unity 控制（✅ 程式碼完成）

| 步驟 | 任務 | 檔案 | 狀態 |
|------|------|------|------|
| 2.1 | 修改 `__init__.py` 匯出模組 | `openarm_can/python/openarm/can/` | ✅ 完成 |
| 2.2 | 修改 `unity_interface_follower.py` | `ros2_ws/.../scripts/` | ✅ 完成 |
| 2.3 | Unity 端對端測試（右手） | - | ⬜ 待測試 |

### Phase 3: 擴展（測試成功後）

| 步驟 | 任務 | 備註 |
|------|------|------|
| 3.1 | 左手靈巧手整合 | 複製右手配置 |
| 3.2 | C++ 版本（視需求） | 高頻控制需求時 |
| 3.3 | 多手勢支援 | 方案 B |

---

## 十一、備註

<!-- 在這裡添加任何額外資訊 -->

```
```

---

**文件結束**
