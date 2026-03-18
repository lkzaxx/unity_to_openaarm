# SlimeVR 追蹤器設定

## 硬體資訊

| 項目 | 值 |
|------|-----|
| 裝置 | DIY SlimeVR Tracker |
| 開發板 | SlimeVR Dev (board ID: 2) |
| 微控制器 | ESP8266 |
| IMU 感測器 | ICM45686 |
| 通訊協議版本 | 21 |
| MAC 位址 | 94:B9:7E:12:A0:6B |
| 韌體 | git-NOT_GIT (自訂編譯) |
| USB 晶片 | CH340 (COM7, 115200 baud) |
| 電池 | Li-Po (USB-C 充電) |
| 來源 | 3D 列印外殼, 開發者: k-lilito (GitHub) |

## 網路設定

- WiFi SSID: `iDakaServer` (僅支援 2.4GHz, ESP8266 限制)
- 追蹤器 IP: `192.168.0.54` (DHCP 分配)
- 通訊方式: 區域網路 UDP 廣播
- SlimeVR Server 必須在同一個區域網路上運行

## 串口指令 (COM7, 115200 baud)

```bash
# 查看追蹤器資訊
GET INFO

# 設定 WiFi 帳密
SET WIFI "SSID" "PASSWORD"

# 查看 WiFi 狀態
GET WIFI
```

## WiFi 狀態碼

| 狀態碼 | 意義 |
|--------|------|
| 4 | 未連線 / 無帳密 |
| 5 | 已連線 |
| 6 | 連線中 / 連線失敗 |

## 設定步驟

### 1. 安裝 SlimeVR Server

**Windows:**
下載: https://github.com/SlimeVR/SlimeVR-Installer/releases/latest/download/slimevr_web_installer.exe

**Jetson (aarch64 Linux):**
```bash
# 從 GitHub Releases 下載 aarch64 deb 套件
# https://github.com/SlimeVR/SlimeVR-Server/releases
wget https://github.com/SlimeVR/SlimeVR-Server/releases/latest/download/SlimeVR-aarch64.deb
sudo dpkg -i SlimeVR-aarch64.deb
```

### 2. 設定 WiFi (如需要)

用 USB-C 連接追蹤器，執行以下 Python 腳本:
```python
import serial, time
ser = serial.Serial('COM7', 115200, timeout=1)  # Linux 上改為 /dev/ttyUSB0
time.sleep(0.5)
ser.write(b'SET WIFI "iDakaServer" "12345678"\r\n')
time.sleep(3)
print(ser.read(4096).decode('utf-8', errors='replace'))
ser.close()
```

### 3. 啟動 SlimeVR Server

- 追蹤器會自動透過 UDP 廣播找到 Server
- 在 SlimeVR Server GUI 中指定追蹤器對應的身體部位
- 校準: 按照螢幕指示操作 (重置、安裝位置校準)

### 4. 與 SteamVR / Unity 整合

- SlimeVR Server 在 SteamVR 中模擬虛擬 Vive Tracker
- Unity 可透過 SteamVR/OpenXR 或直接 UDP 讀取追蹤器資料

## 追蹤器能力與限制

### 能提供的資料 (3DOF 旋轉)

| 封包類型 | ID | 說明 |
|----------|----|------|
| ROTATION_DATA | 17 | 四元數旋轉資料 (x, y, z, w)，主要追蹤資料，高頻率 |
| ACCEL | 4 | 原始加速度資料 |
| TEMPERATURE | 20 | IMU 溫度 (補償用) |
| BATTERY | 12 | 電池電壓/電量 |
| SENSOR_INFO | 15 | 感測器狀態 |
| SIGNAL_STRENGTH | 19 | WiFi 訊號強度 (RSSI) |

### 四元數資料格式

```
四元數 q = (x, y, z, w)  表示追蹤器在 3D 空間中的旋轉姿態
範例: q = (+0.0962, +0.4673, -0.0064, +0.8788)
轉換歐拉角: roll=+16.7°, pitch=+55.3°, yaw=+8.0°

- Roll  = 左右傾斜
- Pitch = 前後傾斜
- Yaw   = 水平旋轉方向
```

### 不能提供的資料 (無位置追蹤)

SlimeVR 是純 IMU 裝置，**只能測量旋轉方向，無法測量位置座標 (x, y, z)**。

| 能提供 | 不能提供 |
|--------|----------|
| 旋轉方向 (四元數/歐拉角) | 絕對位置 (x, y, z) |
| 角速度 | 相對位移 |
| 加速度 (原始值) | 穩定的座標追蹤 |

> 理論上可以對加速度做兩次積分得到位移，但 IMU 誤差會快速累積漂移，幾秒內就不可用。

### 與其他追蹤方案的比較

| 方案 | 位置 | 旋轉 | 自由度 |
|------|------|------|--------|
| **SlimeVR (本追蹤器)** | 無 | 有 | 3DOF |
| Quest 3 控制器 | 有 | 有 | 6DOF |
| Vive Tracker + 基站 | 有 | 有 | 6DOF |
| Quest 3 IOBT (Inside-Out Body Tracking) | 有 (估算) | 有 | 6DOF |

### 對 OpenArm 的實際用途

- 綁在手臂上可以追蹤手臂的**傾斜角度、旋轉方向**
- 搭配 Quest 3 控制器的位置資料，補充**手肘方向**資訊
- 幫助 IK 解算器更準確地推算手肘位置 (解決 7-DOF 冗餘自由度)
- **不能**單獨提供手臂在空間中的 xyz 座標，定位仍需靠 Quest 3

## 與 OpenArm 專案整合

SlimeVR 追蹤器可用於 OpenArm VR 遙操作管線中的身體追蹤:

```
SlimeVR Tracker (WiFi/UDP)
         |
    SlimeVR Server (PC 或 Jetson)
         |
    ┌────┴────┐
    |         |
 SteamVR   直接 UDP 讀取
 (虛擬 Vive Tracker)  (適用 Jetson/ROS2)
    |         |
 Unity      ROS2 Node
 (OpenXR)     |
    |      OpenArm Robot
 OpenArmRetarget / IK Solver
    |
 ROSTCPManager -> ROS2 -> OpenArm Robot
```

## Jetson Orin Nano 使用方式

Jetson (192.168.0.15) 與追蹤器在同一個區域網路 (iDakaServer),
可以直接在 Jetson 上運行 SlimeVR Server 或自訂 UDP 接收器。

```bash
# SSH 到 Jetson
ssh idaka@192.168.0.15

# SlimeVR Server 已安裝 (v18.2.0)
# 啟動方式 (headless):
java -jar /usr/share/slimevr/slimevr.jar run --no-gui

# 或使用自訂 UDP 接收腳本 (已部署到 ~/slimevr_tracker/)
# 注意: 路由器阻擋 UDP 廣播，需使用 --tracker-ip 主動連線
python3 ~/slimevr_tracker/udp_receiver.py --tracker-ip 192.168.0.54 --euler
```

### 路由器 UDP 廣播問題

iDakaServer 路由器會阻擋 WiFi 裝置間的 UDP 廣播 (AP Isolation)。
追蹤器的廣播發現機制無法正常運作，但 unicast (ping) 可以通。

**解決方法**: 使用 `--tracker-ip` 參數，讓接收器主動向追蹤器發送握手 (unicast)。

```bash
# PC 上 (廣播正常，不需指定 IP)
python udp_receiver.py --euler

# Jetson 上 (需指定追蹤器 IP 繞過廣播限制)
python3 udp_receiver.py --tracker-ip 192.168.0.54 --euler
```

## 相關連結

- SlimeVR 官方文件: https://docs.slimevr.dev/
- SlimeVR Server: https://github.com/SlimeVR/SlimeVR-Server
- SlimeVR Tracker 韌體 (ESP): https://github.com/SlimeVR/SlimeVR-Tracker-ESP
- 快速設定指南: https://docs.slimevr.dev/quick-setup.html
- Linux 安裝指南: https://docs.slimevr.dev/tools/linux-installation.html
