# OpenArm ROS2 中文開發與整合指南

本文件為 OpenArm 雙臂機器人 (基於達妙馬達) 以及關聯靈巧手設備的開發與除錯大全。不僅記錄了基礎的系統架構、啟動方式與通訊協定，也匯總了開發過程中遇到的硬體、通訊、軟體整合問題的解決方案。適合在未來的開發或重新部署時作為核心參考。

## 目錄
1. [專案簡介與系統架構](#1-專案簡介與系統架構)
2. [專案啟動與運行方式](#2-專案啟動與運行方式)
3. [通訊協定與發送封包格式](#3-通訊協定與發送封包格式)
4. [硬體通訊設定與掉包排解 (CAN Bus)](#4-硬體通訊設定與掉包排解-can-bus)
5. [控制系統與震盪問題排解](#5-控制系統與震盪問題排解)
6. [HITBOT eHand-6 靈巧手整合](#6-hitbot-ehand-6-靈巧手整合)
7. [TsingSens AmazingHand 開源靈巧手整合](#7-tsingsens-amazinghand-開源靈巧手整合)
8. [系統資源爭奪與即時控制穩定性](#8-系統資源爭奪與即時控制穩定性)
9. [獨立 Python 腳本控制導致的抖動問題](#9-獨立-python-腳本控制導致的抖動問題)
10. [IMX219-83 立體相機使用說明](#10-imx219-83-立體相機使用說明)
11. [監測手臂位置](#11-監測手臂位置)
12. [夾爪控制](#12-夾爪控制)
13. [關節正值方向參考](#13-關節正值方向參考)
14. [特殊指令：回零 (Home) 與重新啟用 (Enable)](#14-特殊指令回零-home-與重新啟用-enable)

---

## 1. 專案簡介與系統架構

### 1.1 專案目標
本專案的主要目標是實現 **OpenArm 雙臂機器人的高頻 (500Hz) 遠端遙操作 (Teleoperation)**。藉由讀取來自外部環境（如 Unity VR 虛擬實境或 AI 模型生成的最佳軌跡），將目標關節角度即時轉換為雙臂 14 軸馬達與末端夾爪/靈巧手的控制命令。

### 1.2 系統架構流程
1. **指令輸入端 (Unity/AI)**: 透過 ROS TCP Endpoint 向 ROS2 網路發佈 `/unity/joint_commands` (針對手臂關節) 以及 `/unity/ehand_commands` (針對靈巧手) 主題。
2. **核心控制節點 (`unity_interface_follower.py`)**: 
   - 接收目標位置，計算軌跡平滑 (Ruckig 或 Rate Limiting)。
   - 即時計算手臂的**重力補償 (Gravity Compensation)**。
   - 將目標角度、速度前饋與力矩轉換為符合達妙馬達的 **MIT 控制參數**。
3. **硬體通訊 (CAN Bus)**:
   - 透過 Linux 系統的 `can1` 及 `can2` 介面，以 1Mbps 波特率向達妙馬達發送指令。
### 1.3 手臂關節配置與運動方式 (Kinematics)
OpenArm 單臂為 **7 自由度 (7-DOF)** 架構，全機共有 14 個達妙伺服馬達 (不含末端執行器)。從基座 (Base) 到手腕 (Wrist) 的關節定義如下：

| Joint 索引 | 名稱 / 部位 | 物理意義 | 運動範圍參考* (左臂) | 運動範圍參考* (右臂) |
| :---: | :--- | :--- | :--- | :--- |
| **1 (J1)** | `Shoulder Pitch` (肩部俯仰) | 控制上手臂向前/向後抬起或放下 | -200° ~ +80° | -80° ~ +200° |
| **2 (J2)** | `Shoulder Roll` (肩部側翻) | 控制上手臂向側邊平舉或內收 | -190° ~ +10° | -10° ~ +190° |
| **3 (J3)** | `Shoulder Yaw` (肩部水平旋轉) | 控制整隻手臂的水平自轉扭轉 | -90° ~ +90° | -90° ~ +90° |
| **4 (J4)** | `Elbow Pitch` (手肘俯仰) | 控制下手臂的屈曲與伸展 | 0° ~ +140° | 0° ~ +140° |
| **5 (J5)** | `Wrist Roll` (手腕側翻) | 控制手腕的第一階扭轉 / 翻轉 | -90° ~ +90° | -90° ~ +90° |
| **6 (J6)** | `Wrist Yaw` (手腕偏航) | 控制手腕向左 / 向右偏擺。**右手正值往內（靠近身體），左手相反** | -45° ~ +45° | -45° ~ +45° |
| **7 (J7)** | `Wrist Pitch` (手腕俯仰) | 控制末端執行器 (手掌/夾爪) 向上翹起或下壓 | -90° ~ +90° | -90° ~ +90° |
*(備註：實際硬體極限可能會因排線組裝干涉而有所縮減，軟體中預設會對極端角度做安全限制)*

---

## 2. 專案啟動與運行方式

### 2.1 環境與硬體準備
在執行程式前，請確保：
1. OpenArm 雙臂的 24V 電源已確實開啟。
2. Jetson 宿主機與開發網路 (如 Unity 所在的 PC) 已連線。
3. 執行 CAN 介面啟動腳本，初始化通訊：
   ```bash
   cd ~/ros2_ws/scripts
   ./cansetup.sh            # 一般初始化
   ./cansetup.sh --reset    # 斷電後需 USB 重設時加 --reset（或 -r）
   ```

### 2.2 啟動 Follower 控制迴圈
啟動 `openarm_bringup` 提供的 bash 腳本，該腳本會一併啟動 ROS TCP 橋接器與 Python 控制節點：
```bash
# 啟動 Unity Follower 主程式
cd ~/ros2_ws/src/openarm_ros2/openarm_bringup/scripts
./start_follower.sh
```
啟動後，終端機將顯示 "Enabling all motors..."，馬達啟動後即會維持當下姿態，等待 `/unity/joint_commands` 傳入。

> ⚠️ **注意**: 請**不要**將此腳本與 C++ 版的 `ros2 launch openarm_bringup openarm.bimanual.launch.py` 同時執行，由於 CAN 介面會被兩者同時競爭，會導致 CAN Bus 流量崩潰。

---

## 3. 通訊協定與發送封包格式

### 3.1 達妙手臂馬達 (MIT 控制模式)
我們使用達妙馬達提供的 MIT 模式進行 500Hz 高頻控制，每一個馬達的控制封包要求以下 5 個參數：
1. **目標位置 (Position)**：弧度 (rad)。
2. **目標速度 (Velocity)**：通常作為速度前饋，預設不啟用時可為 0。
3. **前饋力矩 (Torque/Effort)**：極重要！用來實作**重力補償 (Gravity Compensation)**，根據馬達當下角度產生的抵抗地心引力的力矩。
4. **位置剛度 (Kp)**：彈簧係數 (建議維持溫和的參數 `[30.0, 30.0, 20.0, 20.0, 5.0, 5.0, 5.0]`)。
5. **速度阻尼 (Kd)**：阻尼係數 (與上對應為 `[2.75, 2.5, 0.7, 0.4, 0.7, 0.6, 0.5]`)。

#### 3.1.1 MIT 控制封包 Byte 格式 (8 Bytes)

**發送（主機 → 馬達）**：CAN ID = 馬達 ID（例如 `001`）

5 個浮點參數透過 `float_to_uint` 編碼後壓入 8 bytes：

| 參數 | 物理範圍 | 位元數 | 編碼公式 |
| :--- | :--- | :---: | :--- |
| Position | -π ~ +π rad | 16-bit | `(val - P_MIN) / (P_MAX - P_MIN) × 0xFFFF` |
| Velocity | -30 ~ +30 rad/s | 12-bit | `(val - V_MIN) / (V_MAX - V_MIN) × 0xFFF` |
| Kp | 0 ~ 500 | 12-bit | `val / KP_MAX × 0xFFF` |
| Kd | 0 ~ 5 | 12-bit | `val / KD_MAX × 0xFFF` |
| Torque | -18 ~ +18 Nm | 12-bit | `(val - T_MIN) / (T_MAX - T_MIN) × 0xFFF` |

**Bit 排列**：
```
Byte:  [  0  ][  1  ][  2  ][  3  ][  4  ][  5  ][  6  ][  7  ]
       pppppppp pppppppp vvvvvvvv vvvvkkkk kkkkkkkk dddddddddddd tttttttttttt
       |── Position ──| |─ Vel ─| Kp | |── Kp ──| |── Kd ──| T | |── Torque─|
       (16-bit)         (12-bit) (12-bit) (12-bit)   (12-bit)
```

**回傳（馬達 → 主機）**：CAN ID = 馬達 ID + `0x10`（例如 `001` → `011`），包含實際 Position、Velocity、Torque。

#### 3.1.2 特殊指令封包

特殊指令以 7 bytes `0xFF` 為固定前綴，最後 1 byte 區分功能：

| 封包 | 指令 | 說明 |
| :--- | :--- | :--- |
| `FF FF FF FF FF FF FF FC` | **Enable** | 進入 MIT 控制模式，馬達回傳狀態 |
| `FF FF FF FF FF FF FF FD` | **Disable** | 退出控制模式，馬達釋放力矩 |
| `FF FF FF FF FF FF FF FE` | **Set Zero** | 將當前位置設為零點 |
| `FF FF FF FF FF FF FF FB` | **Clear Error** | 清除故障碼 |

### 3.2 HITBOT 靈巧手 (CAN FD 特殊協議)
靈巧手的架構有別於單顆馬達，它是透過一包 **32 Bytes 的 CAN FD 封包** 同時控制 6 個手指微型伺服器。

#### 3.2.1 封包整體結構
```
[Byte1][Byte2][Motor1: 5B][Motor2: 5B][Motor3: 5B][Motor4: 5B][Motor5: 5B][Motor6: 5B]
  1B  +  1B  +    5B    +    5B    +    5B    +    5B    +    5B    +    5B   = 32 bytes
```

#### 3.2.2 Byte1：馬達選擇 + 讀寫命令

高 6 位選擇馬達（Motor1~6），低 2 位為操作命令（`01`=寫入, `10`=讀出）。

| 值 | 意義 | 值 | 意義 |
|----|------|----|------|
| `0xFD` | **全選寫入** | `0xFE` / `0xFC` | 全選讀出 |
| `0x05` | 拇指旋轉 (M1) 寫入 | `0x09` | 拇指伸縮 (M2) 寫入 |
| `0x11` | 食指 (M3) 寫入 | `0x21` | 中指 (M4) 寫入 |
| `0x41` | 無名指 (M5) 寫入 | `0x81` | 尾指 (M6) 寫入 |
| `0x19` | 拇指伸縮+食指 寫入 | | |

#### 3.2.3 Byte2：控制模式

高 4 位為點位編號（0~15，通常填 0），低 4 位為控制模式：

| 值 | 模式 |
|----|------|
| `0x01` | 位置模式（正常控制）|
| `0x02` | 伸展（全開）|
| `0x03` | 收緊（全握）|
| `0x04` | 初始化回零 |

#### 3.2.4 Byte3~32：馬達參數（每顆 5 bytes × 6）

| Offset | 內容 | 範圍 |
|--------|------|------|
| +0 | Position | 0~255 |
| +1 | Speed | 0~255 |
| +2 | Torque | 0~255 |
| +3 | 保留 | **必須 0x00** |
| +4 | 保留 | **必須 0x00** |

#### 3.2.5 馬達對應表

| 索引 | Motor | 功能 |
|------|-------|------|
| 0 | Motor1 | 拇指旋轉 |
| 1 | Motor2 | 拇指伸縮 |
| 2 | Motor3 | 食指 |
| 3 | Motor4 | 中指 |
| 4 | Motor5 | 無名指 |
| 5 | Motor6 | 尾指 |

#### 3.2.6 CAN ID 與通訊參數

| 參數 | 值 |
|------|-----|
| 通訊類型 | CAN FD（非 classical CAN）|
| 仲裁波特率 | 1 Mbps |
| 數據波特率 | 5 Mbps |
| 終端電阻 | 120Ω |
| 右手 CAN ID | `0x11` |
| 左手 CAN ID | `0x12` |

**發送範例**：
```bash
# 回零（0xFD 全選寫入，0x04 回零，後方 30 Bytes 參數全為 0x00）
cansend can1 012##1FD04000000000000000000000000000000000000000000000000000000000000

# 全開
cansend can1 012##1FD02FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000

# 全握
cansend can1 012##1FD03FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000FFFFFF0000
```

> [!CAUTION]
> **Reserved 必須補 0x00**：先前測試中，每顆馬達結尾的 2 bytes `Reserved` 若填入 `0xFF` 會導致靈巧手某些手指動作不完全，**務必老實填入 `0x00`。**

---

## 4. 硬體通訊設定與掉包排解 (CAN Bus)

### 4.0 PCAN-USB FD 序號綁定與 CAN 介面對應

本系統使用兩個 **PEAK System PCAN-USB FD** 轉接器，透過 USB 連接 Jetson Orin Nano。由於 pcan 驅動以 USB 偵測順序分配 `can1` / `can2` 名稱（且無法透過 udev 或 `ip link set name` 覆寫），更換 USB 接口後 `can1` / `can2` 可能對調。

`cansetup.sh` 透過 sysfs 讀取 PCAN 序號與 USB path 排序，自動產生 **arm mapping 檔** (`/tmp/can_arm_map`)，所有腳本讀取此檔即可正確對應左右臂。

**PCAN-USB FD 序號對應表：**

| PCAN 序號 | 手臂 | 備註 |
| :--- | :---: | :--- |
| `206F307B4153` (4153) | **右臂** | 7 軸達妙馬達 (ID 001~007) |
| `2048335F5052` (5052) | **左臂** | 7 軸達妙馬達 (ID 001~007) |

> [!IMPORTANT]
> `can1` / `can2` 的分配**取決於 USB 插入的 port**，不是固定的。正確的左右臂對應請以 `/tmp/can_arm_map` 為準。

**使用方式：**
```bash
# 一般初始化（快速，自動提升 sudo）
./cansetup.sh

# USB 重設 + 初始化（斷電後或 PCAN 偵測不到時使用）
./cansetup.sh --reset    # 或 -r

# 馬達連線測試（互動式選單，自動讀取 arm mapping）
./can_ping.sh
```

**Mapping 檔格式** (`/tmp/can_arm_map`)：
```bash
RIGHT_CAN=can1
LEFT_CAN=can2
```
其他腳本或程式可 `source /tmp/can_arm_map` 取得 `$RIGHT_CAN` / `$LEFT_CAN` 變數。

**查看序號：**
```bash
lsusb | grep -i peak

for d in /sys/bus/usb/devices/*/serial; do
  [ -f "$d" ] && s=$(cat "$d") && echo "$d : $s"
done | grep -v 0000000
```

> [!NOTE]
> **為何不直接改 CAN 介面名稱？** pcan 驅動在每次 `ip link set up` 時都會重新註冊介面，覆蓋 `ip link set name` 或 udev 規則的改名。因此採用 mapping 檔方案，不改名但透過變數正確對應。

---

### 4.1 CAN 介面預設佇列過短問題 (txqueuelen)
**現象與症狀：**
執行高頻率的 MIT 模式時，手臂出現**無法預測的劇烈抽搐、震盪**，且無法順利抵達目標位置。此現象在系統有其他背景程式 (如 OpenClaw, Docker, Nginx) 運行時特別容易發生。

**根本原因：**
Linux 系統啟動 CAN 介面時，預設分配的傳送佇列長度 (`qlen` 或 `txqueuelen`) **只有 10**。當系統進程切換 (Context Switch) 稍微延遲時，瞬間塞入的 7 個馬達指令會很容易超過這個長度，導致作業系統**主動丟棄 (Drop)** 多出來的封包。馬達漏接指令後，累積的誤差會讓 Kp 剛度產生巨大回拉力矩，形成無限震盪的惡性循環。

**解決方案：**
必須強制放大 CAN 介面的發送佇列。確保您的 `~/ros2_ws/scripts/cansetup.sh` 中已包含 `txqueuelen` 設定：
```bash
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 txqueuelen 1000
sudo ip link set can1 up
```
> [!IMPORTANT]
> **每次 Jetson 重新開機後，txqueuelen 會被重置回 10。務必重新執行腳本確保控制暢通。**

---

## 5. 控制系統與震盪問題排解

### 5.1 Kp / Kd 參數與動態放大
**現象：**
手臂在接近目標點時頻繁小幅抖動，或者移動瞬間力道過猛。

**原因與解決方式：**
* 過高的基礎 Kp (例如 `130.0`) 會導致極端的不穩定。
* `unity_interface_follower.py` 中的 `USE_DYNAMIC_KP_KD` 如果開啟，會在誤差變大時動態乘上放大倍率 (如 1.5 倍)，這在沒有平滑軌跡限制加速度的狀況下是非常危險的。
* **推薦穩定設定**：將 `USE_DYNAMIC_KP_KD` 關閉，並維持溫和的參數 `KP = [30.0, 30.0, 20.0...]` 即可達到足夠的跟隨效果，**請將對抗重力的責任交給重力補償**，而不是 Kp。

### 5.2 重力補償 (Gravity Compensation) 必須常駐
**現象：**
手臂移動時往下掉，或是無法維持在某些懸空姿態，並伴隨低頻的上下抖動 (俗稱「山峰現象」)。

**原因與解決方式：**
重力補償透過 `self._calculate_gravity_compensation()` 函式計算出額外的力矩並加到 MIT 的 Torque 參數中。如果不給予這個前饋力量，馬達就只能依賴「位置誤差產生向下拉力」然後「被 Kp 強制拉回」，導致永遠無法收斂於定點。**請確保此代碼區段未被註解關閉**。

---

## 6. HITBOT eHand-6 靈巧手整合

本專案將末端執行器 (End Effector) 除了達妙夾爪外，另可替換為 HITBOT eHand-6 靈巧手。

### 6.1 基本資訊與設定切換
* **通訊協議**: CAN FD (**不可與達妙標準 CAN 共用，必須獨立 CAN 介面或使用 USB-CAN 轉接模組**)
* **CAN ID**: 右手 `0x11` / 左手 `0x12`
在 `unity_interface_follower.py` 中，可自由切換末端：
```python
RIGHT_END_EFFECTOR_TYPE = "dexterous_hand"  # 使用 HITBOT 靈巧手
LEFT_END_EFFECTOR_TYPE = "gripper"          # 使用舊有達妙夾爪

DEXTEROUS_HAND_CAN_INTERFACE = "can0"       # 獨立的 CAN FD 介面
DEXTEROUS_HAND_CAN_ID = 0x11                # 右手對應的 ID
```

### 6.2 靈巧手開發踩雷紀錄
1. **通訊發送過快塞車**：
   若以 500Hz 控制會導致硬體處理不及。**解法**：在 Python 端將靈巧手控制降頻至 5Hz~10Hz，並於左右手連續發送時加入 50ms 微小延遲。
2. **大拇指物理干涉 (路徑重疊)**：
   同時對 6 顆馬達下達「握緊」指令，大拇指的旋轉 (M1) 會與其他四指產生干涉卡死。**解法**：建議實作分階段的順序性動作，例如：先彎四指50% -> 拇指旋轉 -> 四指握緊 -> 拇指最終收緊。
3. **通訊超時 (故障碼 6)**：
   若手部亮綠燈無反應，且透過 `0xFC` 狀態查詢指令得知第 2 Byte 高 4 位為 `6`，代表手掌內部 MCU 與伺服馬達失去連線。這屬於硬體錯亂，**解法：將 24V 電源斷電 10 秒後重新上電**。
4. **USB-CAN 設備無法開啟 (`handle=NULL`)**:
   使用 ZLGCAN-200U 測試模組時若發現啟動失敗，**不用懷疑，直接換一個 USB 接口重新插入即解**。

---

## 7. TsingSens AmazingHand 開源靈巧手整合

本系統額外提供另一款低成本「開源靈巧手」的方案 (基於 Pollen Robotics 設計)。
* **通訊協議**: Feetech Serial Bus (TTL 半雙工)
* **硬體連接**: 透過 USB 轉接模組 (CP210x)，掛載為 `/dev/ttyUSB0`
* **鮑率 (Baud Rate)**: `1,000,000 bps` (1 Mbps, 8N1)
* **電源需求**: 5V USB 供電 (若電量不足可能導致伺服馬達不轉，建議具備專用 5V 變壓器)

**排解 Linux USB 權限 (`Permission denied`)**：
為確保 Python script 可直接讀寫 `/dev/ttyUSB0`，最一勞永逸的方式是將當前帳號加入通訊群組：
```bash
sudo usermod -aG dialout $USER
# 執行完成後請重新登入系統或重新啟動方可套用
```
相較於 eHand-6，AmazingHand 只需使用普通的 Serial 通訊 (`pyserial`) 就能控制 8 顆 Feetech 舵機，整合門檻較低。但也因此控制精準度與回饋速度與工業封閉式產品有所區別。

---

## 8. 系統資源爭奪與即時控制穩定性

Jetson 宿主機僅有 **7.6 GB** 的物理記憶體。當同時運行多個服務時，系統資源會被嚴重瓜分，導致 500Hz 的即時控制迴圈無法準時執行，最終引發手臂震盪。

### 8.1 各進程記憶體佔用實測 (2026-02-24)
以下為開啟圖形介面與相機功能後，各背景服務實際佔用的記憶體排行榜 (Top 20)：

| 進程名稱 | RSS 記憶體 | 說明 |
|----------|-----------|------|
| **language_server (AI 助手)** | **~436 MB** | Antigravity 程式助手後端 |
| **node (VS Code/Cursor)** | **~405 MB** | 遠端開發擴展主控 |
| **openclaw-gateway** | **~344 MB** | LLM 遙操作閘道 |
| **nvargus-daemon** | **~234 MB** | NVIDIA 攝影機硬體加速守護程式 |
| **python3** | **~211 MB** | `unity_interface_follower.py` (高頻控制) |
| **python3** | **~202 MB** | `camera_publisher.py` (節點發布) |
| **gnome-shell** | **~200 MB** | 桌面圖形化介面主程 |
| **node** | ~174 MB | VS Code 擴展附屬進程 (下同) |
| **gnome-software** | ~172 MB | 軟體中心背景服務 |
| **node** | ~118 MB | VS Code 擴展附屬進程 |
| **node** | ~107 MB | VS Code 擴展附屬進程 |
| **node** | ~107 MB | VS Code 擴展附屬進程 |
| **dockerd** | ~91 MB | 容器服務守護行程 |
| **node** | ~68 MB | VS Code 擴展附屬進程 |
| **evolution-alarm** | ~67 MB | GNOME 附屬服務 |
| **packagekitd** | ~65 MB | 套件管理後端服務 |
| **node** | ~64 MB | VS Code 擴展附屬進程 |
| **tailscaled** | ~62 MB | 網路代理服務 |
| **containerd** | ~56 MB | 容器執行階段 |
| **update-notifier** | ~45 MB | Ubuntu 更新通知服務 |
| **合計** | **~3.2 GB+** | (本次測量時可用記憶體狀態較為健康，尚餘 4.3GB 空閒) |

> [!CAUTION]
> 雖然上述大頭加起來只佔約 2GB，但 Jetson 系統內另外還跑了超過 300 支背景程式，通常會默默吃掉另外 2GB~3GB 的記憶體。當可用記憶體低於約 200 MB 時，Linux 核心會頻繁進行 Swap 交換操作，造成突發性的進程凍結 (stall)，直接破壞 500Hz 控制迴圈的即時性。

### 8.2 CAN 封包掉落的兩個方向
即使 `txqueuelen` 已設為 1000，封包仍可能在**接收方向 (RX)** 掉落：
```bash
# 檢查 CAN 封包統計
ip -s link show can1
```
| 方向 | 含義 | 正常值 | 異常值 |
|------|------|--------|--------|
| **TX dropped** | 發送給馬達的指令被丟棄 | 0 | >0 → 加大 `txqueuelen` |
| **RX dropped** | 馬達回傳的狀態被丟棄 | 0 | >0 → **系統 CPU/記憶體不足**，控制程式來不及讀取 |

當 RX dropped 持續增加時，代表 `unity_interface_follower.py` 無法及時從 CAN 介面讀回馬達狀態，導致控制迴圈計算使用過舊的位置數據，進而使得 Kp 產生錯誤的校正力矩引發震盪。

### 8.3 治標：操控手臂期間釋放資源
在不需要同時使用 AI 擴展時，可暫停不必要的服務以釋放記憶體：

```bash
# 1. 關閉 VS Code / Cursor 的遠端擴展 (節省 ~1 GB 以上)
#    → 直接在 PC 端關閉 VS Code Remote SSH 視窗即可

# 2. 停止 OpenClaw Gateway (節省 ~350 MB)
sudo docker stop openclaw-gateway  # 如以 Docker 運行
# 或
pkill -f openclaw-gateway           # 如以原生進程運行

# 3. 關閉桌面環境 (純 SSH 操作時可省 600~800 MB)
sudo systemctl stop gdm3
# 若想讓系統每次開機都不啟動圖形介面：
sudo systemctl set-default multi-user.target
```

> [!WARNING]
> **關閉桌面環境 (gdm3) 的副作用：相機失效**
> 若您有關閉桌面環境，`camera_publisher.py` 會無法讀取 IMX219 相機畫面 (報錯 `Failed to read camera frame`)。這是因為 NVIDIA 的 `nvarguscamerasrc` GStreamer 插件高度依賴 X11/Wayland 或 EGL 顯示伺服器來分配 NVMM 緩衝區。**如果您需要將實體相機畫面傳回 Unity，請保持桌面環境開啟，並改用 [8.4 節](#84-治本提升控制程式優先級) 的作法來確保穩定性。**

### 8.4 治本：提升控制程式優先級
若必須同時運行 AI 服務與手臂控制，可將控制程式提升為 Linux 即時排程優先級，讓它搶到 CPU 時間：
```bash
# 以 FIFO 即時排程策略啟動 follower (優先級 50)
sudo chrt -f 50 python3 unity_interface_follower.py
```
或者對已經在運行的程式動態調整：
```bash
# 查詢 follower 的 PID
pgrep -f unity_interface_follower
# 動態提升優先級
sudo chrt -f -p 50 <PID>
```

### 8.5 OpenClaw 相關設定提醒
1. **底層 CAN 佇列**: 即為 [4.1 節](#41-can-介面預設佇列過短問題-txqueuelen) 所述情形。**絕對要在每次開機啟動 `cansetup.sh` 時確認有加入 `txqueuelen 1000`。**
2. **記憶體 Swap 依賴不可關閉**: 系統已建立 16GB 的 `/swapfile` 交換空間以及 `vm.swappiness = 30` 來支援大型 AI 容器模型，請勿隨便撤銷。
3. **Nginx 反向代理與目錄設定**: 為繞過跨域同源政策並支援 WebSockets 即時終端，系統已設置 Nginx 反向代理與自簽 SSL。

> 關於 OpenClaw 的詳細環境依賴與 Nginx 設定細節，請參閱：[`/home/idaka/openclaw/OPENCLAW_EXTERNAL_CONFIGS.md`](../../../../openclaw/OPENCLAW_EXTERNAL_CONFIGS.md)。

---

## 9. 獨立 Python 腳本控制導致的抖動問題

### 9.1 問題現象
嘗試繞過 unity_interface_follower.py，直接使用獨立 Python 腳本調用 openarm_can 庫控制馬達時，會發生嚴重的手臂抖動，即使：
- 記憶體充足（可用 > 3GB）
- CAN txqueuelen 已設為 1000+
- 無封包丟失（TX/RX dropped = 0）
- 使用了與 unity_interface_follower.py 相同的 Kp/Kd 參數

### 9.2 根本原因
經測試確認，獨立腳本缺少 unity_interface_follower.py 的完整控制架構：

| 項目 | unity_interface_follower (穩定) | 獨立 Python 腳本 (抖動) |
|------|--------------------------------------|--------------------------|
| 控制架構 | ROS2 節點 + 獨立線程 | 單一主線程 |
| 控制循環 | 持續運行 500Hz | 移動完就結束 |
| 雙臂控制 | 同時控制 can1 + can2 | 通常只控制單臂 |
| 命令發送 | 交替發送左右臂命令 | 只發單臂命令 |
| 結束處理 | 持續保持位置 | 調用 disable_all() |

關鍵差異：
1. 持續控制循環：即使沒有新目標，unity_interface_follower.py 仍持續以 500Hz 發送命令保持位置穩定
2. 雙臂同步控制：CAN 總線上需要同時有雙臂的通訊才能穩定運作
3. 獨立線程：控制循環在獨立線程中運行，不受其他操作的延遲影響

### 9.3 正確的控制方式
務必通過 unity_interface_follower.py + ROS2 topic 來控制手臂：

1. 設置 CAN
   cd ~/ros2_ws/scripts
   ./cansetup.sh            # 或 ./cansetup.sh --reset（斷電後）

2. 啟動 Follower Interface
   ./start_follower.sh

3. 在另一個終端發送控制命令
   ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState "{name: ['R_J1','R_J2','R_J3','R_J4','R_J5','R_J6','R_J7'], position: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once

> [!CAUTION]
> 不要嘗試用獨立腳本直接控制馬達
> 即使程式碼邏輯看似正確（有 rate limiting、正確的 Kp/Kd），缺少完整的 ROS2 控制架構仍會導致不穩定的抖動現象。若需要程式化控制，請透過發布 ROS2 topic 的方式與 unity_interface_follower.py 互動。


---

## 10. IMX219-83 立體相機使用說明

### 10.1 硬體規格

#### 基本資訊
| 項目 | 規格 |
|------|------|
| **型號** | IMX219-83 Stereo Camera |
| **感測器** | Sony IMX219 × 2（雙目） |
| **單眼解析度** | 3280 × 2464（8MP） |
| **CMOS 尺寸** | 1/4 英吋 |
| **焦距** | 2.6mm |
| **畸變** | < 1% |
| **基線距離** | 60mm |
| **尺寸** | 24mm × 85mm |
| **連接介面** | CSI（連接至 Jetson） |

#### 視角（FOV）
| 方向 | 角度 |
|------|------|
| **對角線** | 83° |
| **水平** | 73° |
| **垂直** | 50° |

#### 內建 IMU（ICM20948）
| 感測器 | 解析度 | 量測範圍 |
|--------|--------|----------|
| **加速度計** | 16-bit | ±2, ±4, ±8, ±16 g |
| **陀螺儀** | 16-bit | ±250, ±500, ±1000, ±2000 °/sec |
| **磁力計** | 16-bit | ±4900 μT |

#### 目前使用設定
| 項目 | 設定值 |
|------|--------|
| **發佈解析度** | 640 × 480（可調） |
| **發佈幀率** | 15 FPS（可調） |
| **JPEG 品質** | 50（可調） |

> [!NOTE]
> **硬體同步限制**：IMX219-83 不具備硬體同步功能，雙目同步較困難。適用於 AI 視覺應用如深度視覺、立體視覺等。

> **規格來源**：[Waveshare Wiki](https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera)

### 10.2 啟動方式

相機發佈節點已整合在 `start_follower.sh` 中，執行該腳本時會自動啟動：

```bash
cd ~/ros2_ws/scripts
./start_follower.sh
```

啟動順序：
1. `cansetup.sh` - 設置 CAN 介面
2. `ros_tcp_endpoint` - ROS-Unity 橋接（背景）
3. `camera_publisher.py` - 相機發佈（背景）
4. `unity_interface_follower.py` - 手臂控制（前景）

### 10.3 單獨啟動相機

若只需要相機功能，不需要手臂控制：

```bash
# 1. Source ROS2 環境
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 2. 設定 DISPLAY（nvarguscamerasrc 需要）
export DISPLAY=:0

# 3. 啟動相機發佈節點
python3 ~/ros2_ws/src/openarm_ros2/openarm_bringup/scripts/camera_publisher.py
```

### 10.4 ROS2 Topics

| Topic | 類型 | 說明 |
|-------|------|------|
| `/camera/left/compressed` | `sensor_msgs/CompressedImage` | 左眼壓縮影像 |
| `/camera/right/compressed` | `sensor_msgs/CompressedImage` | 右眼壓縮影像 |

### 10.5 可調參數

啟動時可透過 `--ros-args` 調整參數：

```bash
python3 camera_publisher.py --ros-args \
    -p width:=640 \
    -p height:=480 \
    -p fps:=15 \
    -p jpeg_quality:=50 \
    -p enable_left:=true \
    -p enable_right:=true \
    -p use_test_pattern:=false
```

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `width` | 640 | 影像寬度 |
| `height` | 480 | 影像高度 |
| `fps` | 15 | 幀率 |
| `jpeg_quality` | 90 | JPEG 壓縮品質（1-100），`start_follower.sh` 中設為 50 以節省頻寬 |
| `enable_left` | true | 啟用左眼相機 |
| `enable_right` | true | 啟用右眼相機 |
| `use_test_pattern` | false | 使用測試圖案（無實體相機時用於測試） |

### 10.6 檢視影像

在另一個終端檢視相機是否正常發佈：

```bash
# 檢查 topic 是否存在
ros2 topic list | grep camera

# 檢查發佈頻率
ros2 topic hz /camera/left/compressed

# 檢視影像資訊
ros2 topic echo /camera/left/compressed --no-arr
```

### 10.7 常見問題

#### 問題：Failed to read camera frame

**原因**：桌面環境未啟動。`nvarguscamerasrc` GStreamer 插件需要 X11/Wayland 顯示伺服器。

**解決方案**：
```bash
# 確保桌面環境正在運行
sudo systemctl start gdm3

# 設定 DISPLAY 環境變數
export DISPLAY=:0
```

> [!WARNING]
> 如需使用相機功能，請勿關閉桌面環境 (gdm3)。詳見 [8.3 節](#83-治標操控手臂期間釋放資源) 的說明。

#### 問題：相機畫面延遲或卡頓

**可能原因**：
1. 網路頻寬不足
2. JPEG 品質設定過高

**解決方案**：
```bash
# 降低 JPEG 品質以減少頻寬使用
python3 camera_publisher.py --ros-args -p jpeg_quality:=30

# 或降低解析度
python3 camera_publisher.py --ros-args -p width:=320 -p height:=240
```

### 10.8 記憶體使用

`camera_publisher.py` 約佔用 **200 MB** 記憶體。若系統記憶體緊張，可考慮：
- 只啟用單眼相機（`-p enable_right:=false`）
- 降低解析度和幀率


---

## 11. 監測手臂位置

### 11.1 ROS2 Topic

`unity_interface_follower.py` 會以 50 Hz 頻率發佈手臂關節狀態：

| Topic | 類型 | 頻率 | 說明 |
|-------|------|------|------|
| `/openarm/joint_states` | `sensor_msgs/JointState` | 50 Hz | 雙臂 14 個關節的位置、速度、力矩 |

### 11.2 即時監測指令

```bash
# 檢視即時關節狀態（位置、速度、力矩）
ros2 topic echo /openarm/joint_states

# 只看位置數值（簡化輸出）
ros2 topic echo /openarm/joint_states --field position

# 檢視發佈頻率
ros2 topic hz /openarm/joint_states

# 單次檢視（不持續更新）
ros2 topic echo /openarm/joint_states --once
```

### 11.3 JointState 訊息格式

```yaml
header:
  stamp: {sec: ..., nanosec: ...}
  frame_id: ''
name: ['R_J1', 'R_J2', 'R_J3', 'R_J4', 'R_J5', 'R_J6', 'R_J7',
       'L_J1', 'L_J2', 'L_J3', 'L_J4', 'L_J5', 'L_J6', 'L_J7']
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # 右臂 R_J1~R_J7 (rad)
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 左臂 L_J1~L_J7 (rad)
velocity: [...]   # 各關節速度 (rad/s)
effort: [...]     # 各關節力矩 (Nm)
```

### 11.4 關節名稱對照

| 索引 | 名稱 | 說明 |
|------|------|------|
| 0 | R_J1 | 右臂關節 1（肩部旋轉） |
| 1 | R_J2 | 右臂關節 2（肩部抬升） |
| 2 | R_J3 | 右臂關節 3（上臂旋轉） |
| 3 | R_J4 | 右臂關節 4（肘部） |
| 4 | R_J5 | 右臂關節 5（前臂旋轉） |
| 5 | R_J6 | 右臂關節 6（腕部俯仰） |
| 6 | R_J7 | 右臂關節 7（腕部旋轉） |
| 7 | L_J1 | 左臂關節 1（肩部旋轉） |
| 8 | L_J2 | 左臂關節 2（肩部抬升） |
| 9 | L_J3 | 左臂關節 3（上臂旋轉） |
| 10 | L_J4 | 左臂關節 4（肘部） |
| 11 | L_J5 | 左臂關節 5（前臂旋轉） |
| 12 | L_J6 | 左臂關節 6（腕部俯仰） |
| 13 | L_J7 | 左臂關節 7（腕部旋轉） |

### 11.5 Python 程式監測範例

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointMonitor(Node):
    def __init__(self):
        super().__init__('joint_monitor')
        self.subscription = self.create_subscription(
            JointState,
            '/openarm/joint_states',
            self.callback,
            10
        )

    def callback(self, msg):
        # 取得右臂 J1 位置（弧度）
        r_j1_pos = msg.position[0]
        # 轉換為角度
        r_j1_deg = r_j1_pos * 180.0 / 3.14159
        self.get_logger().info(f'R_J1: {r_j1_deg:.2f} deg')

def main():
    rclpy.init()
    node = JointMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 11.6 Unity 端接收

Unity 透過 `ros_tcp_endpoint` 訂閱 `/openarm/joint_states` 即可取得手臂位置，用於視覺化或同步虛擬手臂。


---

## 12. 夾爪控制

### 12.1 控制方式

夾爪透過 `/unity/joint_commands` topic 控制，與手臂關節使用相同的介面。

| 項目 | 說明 |
|------|------|
| **Topic** | `/unity/joint_commands` |
| **類型** | `sensor_msgs/JointState` |
| **左夾爪 Joint 名稱** | `L_EE` |
| **右夾爪 Joint 名稱** | `R_EE` |
| **Position 範圍** | `0` ~ `0.0425`（米） |
| **0** | 完全關閉 |
| **0.0425** | 完全打開 |

### 12.2 控制指令範例

#### 打開右夾爪
```bash
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState "{name: ['R_EE'], position: [0.0425]}" --once
```

#### 關閉右夾爪
```bash
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState "{name: ['R_EE'], position: [0.0]}" --once
```

#### 打開左夾爪
```bash
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState "{name: ['L_EE'], position: [0.0425]}" --once
```

#### 關閉左夾爪
```bash
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState "{name: ['L_EE'], position: [0.0]}" --once
```

#### 同時控制雙夾爪
```bash
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState "{name: ['L_EE', 'R_EE'], position: [0.0425, 0.0425]}" --once
```

### 12.3 夾爪與手臂同時控制

可以在同一個指令中同時控制手臂關節和夾爪：

```bash
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState "{name: ['R_J1', 'R_J2', 'R_J3', 'R_J4', 'R_J5', 'R_J6', 'R_J7', 'R_EE'], position: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0425]}" --once
```

### 12.4 注意事項

> [!WARNING]
> **夾爪控制需要 Follower 運行中**
> 夾爪控制指令透過 `unity_interface_follower.py` 處理。請確保已執行 `start_follower.sh` 啟動控制節點。

> [!TIP]
> **位置單位是米（m）**
> Position 值代表夾爪開合距離，單位是米。0.0425m = 4.25cm 為最大開度。


---

## 13. 關節正值方向參考

> **視角說明**：從機器人**後方**看（面對機器人背部），機器人前方為正方向。

### 13.1 左臂正值方向（+1.5 rad）

| 關節 | 名稱 | 正值方向 | 旋轉方向 |
|------|------|----------|----------|
| J1 | 基座旋轉 | 往後（遠離前方） | - |
| J2 | 肩部上下 | 整隻手往身體靠（內收） | 逆時針 |
| J3 | 上臂旋轉 | 以垂直軸往身體旋轉 | 順時針 |
| J4 | 肘部 | 肘往上（前） | - |
| J5 | 手腕旋轉1 | 以垂直軸往身體旋轉 | 順時針 |
| J6 | 手腕旋轉2 | 手往身體外轉 | 順時針 |
| J7 | 手腕旋轉3 | 手往後 | - |

### 13.2 右臂正值方向（+1.5 rad）

| 關節 | 名稱 | 正值方向 | 旋轉方向 |
|------|------|----------|----------|
| J1 | 基座旋轉 | 往前（朝向前方） | - |
| J2 | 肩部上下 | 整隻手往身體外靠（外展） | 逆時針 |
| J3 | 上臂旋轉 | 以垂直軸往身體外旋轉 | 順時針 |
| J4 | 肘部 | 肘往上（前） | - |
| J5 | 手腕旋轉1 | 以垂直軸往身體外旋轉 | 順時針 |
| J6 | 手腕旋轉2 | 手往身體轉（內轉） | 順時針 |
| J7 | 手腕旋轉3 | 手往前 | - |

### 13.3 左右臂對稱性

| 關節 | 左臂正值 | 右臂正值 | 關係 |
|------|----------|----------|------|
| J1 | 往後 | 往前 | **相反** |
| J2 | 往身體靠 | 往身體外 | **相反** |
| J3 | 往身體轉 | 往身體外轉 | **相反** |
| J4 | 肘往上 | 肘往上 | 相同 |
| J5 | 往身體轉 | 往身體外轉 | **相反** |
| J6 | 往身體外轉 | 往身體轉 | **相反** |
| J7 | 往後 | 往前 | **相反** |

> [!TIP]
> **記憶技巧**：除了 J4（肘部）相同外，左右臂的正值方向大致呈**鏡像對稱**。

---

## 10. IMX219-83 立體相機使用說明

### 10.1 硬體資訊
- **型號**：IMX219-83 Stereo Camera
- **解析度**：640×480（可調）
- **幀率**：15 FPS（可調）
- **連接**：CSI 介面連接至 Jetson

### 10.2 啟動方式

相機發佈節點已整合在  中，執行該腳本時會自動啟動：



啟動順序：
1.  - 設置 CAN 介面
2.  - ROS-Unity 橋接（背景）
3.  - 相機發佈（背景）
4.  - 手臂控制（前景）

### 10.3 單獨啟動相機

若只需要相機功能，不需要手臂控制：



### 10.4 ROS2 Topics

| Topic | 類型 | 說明 |
|-------|------|------|
|  |  | 左眼壓縮影像 |
|  |  | 右眼壓縮影像 |

### 10.5 可調參數

啟動時可透過  調整參數：



| 參數 | 預設值 | 說明 |
|------|--------|------|
|  | 640 | 影像寬度 |
|  | 480 | 影像高度 |
|  | 15 | 幀率 |
|  | 90 | JPEG 壓縮品質（1-100）， 中設為 50 以節省頻寬 |
|  | true | 啟用左眼相機 |
|  | true | 啟用右眼相機 |
|  | false | 使用測試圖案（無實體相機時用於測試） |

### 10.6 檢視影像

在另一個終端檢視相機是否正常發佈：



### 10.7 常見問題

#### 問題：Failed to read camera frame

**原因**：桌面環境未啟動。 GStreamer 插件需要 X11/Wayland 顯示伺服器。

**解決方案**：


> [!WARNING]
> 如需使用相機功能，請勿關閉桌面環境 (gdm3)。詳見 [8.3 節](#83-治標操控手臂期間釋放資源) 的說明。

#### 問題：相機畫面延遲或卡頓

**可能原因**：
1. 網路頻寬不足
2. JPEG 品質設定過高

**解決方案**：


### 10.8 記憶體使用

 約佔用 **200 MB** 記憶體。若系統記憶體緊張，可考慮：
- 只啟用單眼相機（）
- 降低解析度和幀率

---

## 9. 獨立 Python 腳本控制導致的抖動問題

### 9.1 問題現象
嘗試繞過 ，直接使用獨立 Python 腳本調用  庫控制馬達時，會發生**嚴重的手臂抖動**，即使：
- 記憶體充足（可用 > 3GB）
- CAN  已設為 1000+
- 無封包丟失（TX/RX dropped = 0）
- 使用了與  相同的 Kp/Kd 參數

### 9.2 根本原因
經測試確認，獨立腳本缺少  的**完整控制架構**：

| 項目 | unity_interface_follower (✅ 穩定) | 獨立 Python 腳本 (❌ 抖動) |
|------|--------------------------------------|--------------------------|
| **控制架構** | ROS2 節點 + 獨立線程 | 單一主線程 |
| **控制循環** | **持續運行 500Hz** | 移動完就結束 |
| **雙臂控制** | **同時控制 can1 + can2** | 通常只控制單臂 |
| **命令發送** | 交替發送左右臂命令 | 只發單臂命令 |
| **結束處理** | 持續保持位置 | 調用  |

關鍵差異：
1. **持續控制循環**：即使沒有新目標， 仍持續以 500Hz 發送命令保持位置穩定
2. **雙臂同步控制**：CAN 總線上需要同時有雙臂的通訊才能穩定運作
3. **獨立線程**：控制循環在獨立線程中運行，不受其他操作的延遲影響

### 9.3 正確的控制方式
**務必通過  + ROS2 topic 來控制手臂：**



> [!CAUTION]
> **不要嘗試用獨立腳本直接控制馬達**
> 即使程式碼邏輯看似正確（有 rate limiting、正確的 Kp/Kd），缺少完整的 ROS2 控制架構仍會導致不穩定的抖動現象。若需要程式化控制，請透過發布 ROS2 topic 的方式與  互動。
已添加到 README_zh.md cat
已添加到 README_zh.md cat
已添加到 README_zh.md cat
已添加到 README_zh.md cat
已添加到 README_zh.md cat
已添加到 README_zh.md cat
已添加到 README_zh.md cat
已添加到 README_zh.md cat
已添加到 README_zh.md cat
已添加到 README_zh.md cat
已添加到 README_zh.md cat
已添加到 README_zh.md cat
已添加到 README_zh.md cat

---

## 14. 特殊指令：回零 (Home) 與重新啟用 (Enable)

### 14.1 功能說明

透過 `/unity/joint_commands` topic 發送特殊指令名稱，可以讓手臂**平滑回到零位後自動 disable（斷電）**，或重新啟用已 disable 的手臂。

### 14.2 可用指令

| 指令名稱 | 說明 |
|-----------|------|
| `R_HOME` | 右手臂回零 → 到位後 disable |
| `L_HOME` | 左手臂回零 → 到位後 disable |
| `HOME` | 雙臂同時回零 → 到位後 disable |
| `R_ENABLE` | 重新啟用已 disable 的右手臂 |
| `L_ENABLE` | 重新啟用已 disable 的左手臂 |
| `ENABLE` | 重新啟用雙臂 |

### 14.3 使用方式

```bash
# 右手回零並 disable
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: ['R_HOME'], position: [0.0]}" --once

# 雙臂同時回零
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: ['HOME'], position: [0.0]}" --once

# 重新啟用右手
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: ['R_ENABLE'], position: [0.0]}" --once

# 重新啟用雙臂
ros2 topic pub /unity/joint_commands sensor_msgs/msg/JointState \
  "{name: ['ENABLE'], position: [0.0]}" --once
```

### 14.4 運作流程

1. **HOME 指令**：收到後將目標位置設為全零，控制迴圈會以 rate limiting 平滑移動
2. **到位判定**：所有關節位置 < 0.05 rad 時視為到位
3. **自動 Disable**：到位後自動呼叫 `disable_all()`，馬達斷電
4. **ENABLE 指令**：重新呼叫 `enable_all()` 啟用馬達，恢復控制

> [!NOTE]
> Disable 狀態下，控制迴圈會跳過該手臂的 MIT 指令發送與接收，不會產生多餘的 CAN 流量。

### 14.5 靈巧手特殊指令

透過 `/unity/ehand_commands` topic 發送特殊指令，控制靈巧手回零、張開、握緊、禁用或重新啟用。


> [!CAUTION]
> **Topic 類型是 `sensor_msgs/msg/JointState`，不是 `std_msgs/msg/String`！**
> 特殊指令放在 `name[0]` 欄位，`position` 留空陣列。若誤用 `std_msgs/msg/String` 發送，
> `ros2 topic pub --once` 會因型別不匹配而**無限等待訂閱者**，造成整個控制程式卡死。
#### 可用指令

| 指令名稱 | 說明 |
|-----------|------|
| `R_HAND_HOME` | 右靈巧手回零 + 停止控制迴圈 |
| `L_HAND_HOME` | 左靈巧手回零 + 停止控制迴圈 |
| `HAND_HOME` | 雙手回零 + 停止控制迴圈 |
| `R_HAND_OPEN` | 右靈巧手張開 |
| `L_HAND_OPEN` | 左靈巧手張開 |
| `HAND_OPEN` | 雙手張開 |
| `R_HAND_CLOSE` | 右靈巧手握緊 |
| `L_HAND_CLOSE` | 左靈巧手握緊 |
| `HAND_CLOSE` | 雙手握緊 |
| `R_HAND_DISABLE` | 右靈巧手禁用（釋放電機） |
| `L_HAND_DISABLE` | 左靈巧手禁用（釋放電機） |
| `HAND_DISABLE` | 雙手禁用 |
| `R_HAND_ENABLE` | 重新啟用右靈巧手（發送回零恢復） |
| `L_HAND_ENABLE` | 重新啟用左靈巧手 |
| `HAND_ENABLE` | 重新啟用雙手 |

#### 使用方式

```bash
# 右靈巧手回零 + 停止控制
ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
  "{name: ['R_HAND_HOME'], position: [0.0]}" --once

# 雙手張開
ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
  "{name: ['HAND_OPEN'], position: [0.0]}" --once

# 右靈巧手禁用（釋放電機）
ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
  "{name: ['R_HAND_DISABLE'], position: [0.0]}" --once

# 重新啟用雙手
ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState \
  "{name: ['HAND_ENABLE'], position: [0.0]}" --once
```

#### 靈巧手 CAN FD 命令對照

| 動作 | Byte1 | Byte2 | 說明 |
|------|-------|-------|------|
| 禁用 | 0xFD | 0x00 | 釋放電機 |
| 位置控制 | 0xFD | 0x01 | 正常控制模式 |
| 張開 | 0xFD | 0x02 | 全部手指張開 |
| 握緊 | 0xFD | 0x03 | 全部手指握緊 |
| 回零 | 0xFD | 0x04 | 回到零位 |

> [!NOTE]
> HOME 和 DISABLE 指令會停止控制迴圈對該手的位置命令發送。使用 ENABLE 指令可恢復控制。
