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
8. [OpenClaw 系統整合注意事項](#8-openclaw-系統整合注意事項)

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
| **6 (J6)** | `Wrist Yaw` (手腕偏航) | 控制手腕向左 / 向右偏擺 | -45° ~ +45° | -45° ~ +45° |
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
   ./cansetup.sh
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

### 3.2 HITBOT 靈巧手 (CAN FD 特殊協議)
靈巧手的架構有別於單顆馬達，它是透過一包 **32 Bytes 的 CAN FD 封包** 同時控制 6 個手指微型伺服器。
* **封包總長**: 32 Bytes
* **格式範例**: `[Byte 1] [Byte 2] [Motor 1 (5 Bytes)] [Motor 2 (5 Bytes)] ... [Motor 6 (5 Bytes)]`
* **欄位解析**:
  - `Byte 1`: 控制旗標（`0xFD` 代表全選寫入控制指令，`0xFC` 代表向靈巧手請求讀取狀態）。
  - `Byte 2`: 控制模式（`0x01`=位置模式, `0x02`=全開, `0x03`=全握緊, `0x04`=初始化回零）。
  - `Motor 1~6 (各 5 Bytes)`: 分別對應 [Position (0~255), Speed (0~255), Torque (0~255), Reserved, Reserved]。

**發送範例（以回零為例）**：
```bash
# 0xFD 寫入，0x04 回零，後方 30 Bytes 參數全為 0x00
cansend can1 012##1FD04000000000000000000000000000000000000000000000000000000000000
```
> **👉 [踩雷注意] Reserved 必須補 0x00**: 先前測試中，每顆馬達結尾的 2 bytes `Reserved` 若填入 `0xFF` 會導致靈巧手某些手指動作不完全，**務必老實填入 `0x00`。**

---

## 4. 硬體通訊設定與掉包排解 (CAN Bus)

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
以下為手臂控制期間，各背景服務佔用的記憶體與 CPU 實測數據：

| 進程名稱 | RSS 記憶體 | CPU | 說明 |
|----------|-----------|-----|------|
| **Antigravity 語言伺服器** | **~1.5 GB** | 100% | AI 程式助手後端，吃資源最兇 |
| **Cursor Extension Host** (×2) | **~770 MB** | ~10% | VS Code 遠端開發擴展 |
| **OpenClaw Gateway** | **~378 MB** | <1% | LLM 遙操作閘道 |
| **unity_interface_follower.py** | ~213 MB | ~24% | 手臂控制核心 |
| **camera_publisher.py** | ~151 MB | ~6% | ROS2 攝影機發佈 |
| **GNOME Desktop** | ~190 MB | <1% | 圖形桌面環境 |
| **nvargus-daemon** | ~252 MB | <1% | NVIDIA 攝影機守護程式 |
| **其他 (Nginx, Docker, 等)** | ~200 MB | - | 背景服務 |
| **合計** | **~3.7 GB+** | - | 剩餘可用記憶體僅約 134 MB |

> [!CAUTION]
> 當可用記憶體低於約 200 MB 時，Linux 核心會頻繁進行記憶體回收與 Swap 交換操作，這會造成突發性的進程凍結 (stall)，直接破壞 500Hz 控制迴圈的即時性。

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
在需要操控手臂進行精密控制時，建議暫時關閉以下不必要的服務以釋放記憶體和 CPU：

```bash
# 1. 關閉 VS Code / Cursor 的遠端擴展 (節省 ~2.3 GB)
#    → 直接在 PC 端關閉 VS Code Remote SSH 視窗即可

# 2. 停止 OpenClaw Gateway (節省 ~378 MB)
sudo docker stop openclaw-gateway  # 如以 Docker 運行
# 或
pkill -f openclaw-gateway           # 如以原生進程運行

# 3. 關閉桌面環境 (含相關附屬服務可節省高達 600~800 MB，強烈建議在純 SSH / VSCode Remote 操作時執行)
sudo systemctl stop gdm3

# 若想讓系統每次開機都不啟動圖形介面 (永久生效以確保資源充足)：
sudo systemctl set-default multi-user.target
```

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
