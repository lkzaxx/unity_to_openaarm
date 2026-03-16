# RealSense D435i on Jetson Orin Nano Super

Intel RealSense D435i 深度相機在 NVIDIA Jetson Orin Nano Super 上的安裝與使用說明。

## 硬體環境

| 項目 | 規格 |
|------|------|
| 開發板 | NVIDIA Jetson Orin Nano Super Developer Kit |
| 相機 | Intel RealSense D435I |
| 序號 | 346522072982 |
| 韌體版本 | 5.15.1.55 (建議更新至 5.17.0.10) |
| USB 連接 | USB 3.2 |
| 相機功能 | RGB + Depth + IMU (加速度計 + 陀螺儀) |

## 軟體環境

| 項目 | 版本 |
|------|------|
| OS | Ubuntu 22.04.5 LTS (Jammy) |
| JetPack | R36.4.4 |
| Python | 3.10.12 |
| librealsense2 | 2.56.5 |
| pyrealsense2 | 2.56.5.9235 (pip) |
| OpenCV | 4.5.4 |
| NumPy | 1.21.5 |

## 安裝步驟紀錄

### 1. librealsense2 (APT 安裝)

已新增 Intel 官方 APT 來源：

```bash
# APT source (已設定)
# /etc/apt/sources.list.d/archive_uri-https_librealsense_intel_com_debian_apt-repo-jammy.list
deb https://librealsense.intel.com/Debian/apt-repo jammy main
```

已安裝的套件：

```
librealsense2           2.56.5  - runtime
librealsense2-dev       2.56.5  - 開發檔案
librealsense2-gl        2.56.5  - GLSL 擴充
librealsense2-udev-rules 2.56.5 - udev 規則
librealsense2-utils     2.56.5  - 工具與範例 (rs-enumerate-devices 等)
```

### 2. pyrealsense2 (pip 安裝)

```bash
pip3 install pyrealsense2
# 安裝位置: ~/.local/lib/python3.10/site-packages/pyrealsense2/
```

### 3. IMU 支援 - hid-sensor-hub 核心模組 (手動編譯)

Jetson Orin Nano 的預設核心不包含 D435i IMU 所需的 HID sensor 模組，需手動編譯。

編譯產物位於 `hid-sensor-hub-build/` 目錄，包含以下 `.ko` 模組：

| 模組 | 用途 |
|------|------|
| `hid-sensor-hub.ko` | HID sensor hub 核心驅動 |
| `hid-sensor-iio-common.ko` | IIO 共用功能 |
| `hid-sensor-trigger.ko` | IIO 觸發器 |
| `hid-sensor-accel-3d.ko` | 3D 加速度計驅動 |
| `hid-sensor-gyro-3d.ko` | 3D 陀螺儀驅動 |

安裝模組：

```bash
cd ~/realsense/hid-sensor-hub-build/
sudo insmod hid-sensor-hub.ko
sudo insmod hid-sensor-iio-common.ko
sudo insmod hid-sensor-trigger.ko
sudo insmod hid-sensor-accel-3d.ko
sudo insmod hid-sensor-gyro-3d.ko
```

#### 開機自動載入 (已設定)

檔案 `/etc/modules-load.d/hid-sensor.conf`：

```
# RealSense D435i IMU support
hid-sensor-hub
hid-sensor-iio-common
hid-sensor-trigger
hid-sensor-accel-3d
hid-sensor-gyro-3d
```

驗證模組是否載入：

```bash
lsmod | grep hid_sensor
```

## 測試腳本

所有腳本位於 Jetson 上的 `~/realsense/`，與本資料夾 git 同步。

### test_realsense.py - RGB + Depth 基本測試

測試相機連線、Depth + Color 串流 (640x480@30fps)。

```bash
python3 ~/realsense/test_realsense.py
```

輸出內容：裝置資訊、深度比例、影像解析度、深度範圍、中心點深度。

### capture_frame.py - 擷取影像

擷取一幀對齊後的 RGB + Depth 影像，儲存為 PNG 和 NPY 格式。同時輸出相機內參 (fx, fy, cx, cy)。

```bash
python3 ~/realsense/capture_frame.py
```

輸出檔案：
- `color.png` - RGB 彩色影像
- `depth_colormap.png` - 深度偽彩圖 (JET colormap)
- `color_raw.npy` - RGB 原始資料
- `depth_raw.npy` - 深度原始資料

### test_imu.py - IMU 測試

測試加速度計與陀螺儀 (各 200Hz)，讀取 5 秒資料。

```bash
python3 ~/realsense/test_imu.py
```

**前提：** hid-sensor-hub 等核心模組必須已載入。

### realsense_web_viewer.py - 網頁即時檢視器

在 port 8081 啟動 HTTP 串流伺服器，可從瀏覽器即時查看影像。

```bash
python3 ~/realsense/realsense_web_viewer.py
```

瀏覽器開啟：http://192.168.0.15:8081/

功能：
- `/stream?mode=color` - RGB 即時串流
- `/stream?mode=depth` - 深度圖串流
- `/stream?mode=overlay` - RGB + Depth 疊合
- `/stream?mode=sbs` - 左右並排
- `/snapshot?mode=color` - 高畫質截圖
- `/imu` - IMU JSON 資料 (加速度計、陀螺儀、中心深度)

## 相機串流規格

| 串流 | 解析度 | 格式 | FPS |
|------|--------|------|-----|
| Color (RGB) | 640x480 | BGR8 | 30 |
| Depth | 640x480 | Z16 | 30 |
| Infrared | 最高 1280x800 | Y8/Y16 | 最高 300 |
| Accel (IMU) | - | motion_xyz32f | 200 |
| Gyro (IMU) | - | motion_xyz32f | 200 |

## 疑難排解

### 相機未偵測到

```bash
# 確認 USB 連接
lsusb | grep Intel
# 應顯示: Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i

# 列舉裝置
rs-enumerate-devices | head -20
```

### IMU 無法使用

```bash
# 確認核心模組已載入
lsmod | grep hid_sensor

# 若未載入，手動載入
sudo modprobe hid-sensor-hub
# 或使用編譯好的模組
cd ~/realsense/hid-sensor-hub-build/
sudo insmod hid-sensor-hub.ko
```

### 韌體更新

目前韌體 5.15.1.55，建議版本為 5.17.0.10：

```bash
rs-fw-update --list  # 查看可用韌體
```
