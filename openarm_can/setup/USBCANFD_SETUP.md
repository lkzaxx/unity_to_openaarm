# USBCANFD-200U 設置指南

## 安裝步驟

### 1. 複製動態庫
```bash
sudo cp ~/Downloads/libcontrolcanfd.so /usr/local/lib/
sudo ldconfig
```

### 2. 創建 udev 規則 (允許普通用戶存取)
```bash
sudo nano /etc/udev/rules.d/99-usbcanfd.rules
```

內容:
```
# ZLGCAN USBCANFD-200U
SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", MODE="0666"
```

重新載入規則:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3. 驗證安裝
```bash
# 檢查動態庫
ldconfig -p | grep controlcanfd

# 執行掃描工具
cd ~/openarm_can/setup
python3 usbcanfd_scan.py --duration 10
```

## 相關檔案位置

| 檔案 | 路徑 |
|------|------|
| 動態庫 | `/usr/local/lib/libcontrolcanfd.so` |
| 掃描工具 | `~/openarm_can/setup/usbcanfd_scan.py` |
| udev 規則 | `/etc/udev/rules.d/99-usbcanfd.rules` |

## 刪除方式
```bash
sudo rm /usr/local/lib/libcontrolcanfd.so
sudo rm /etc/udev/rules.d/99-usbcanfd.rules
sudo ldconfig
rm ~/openarm_can/setup/usbcanfd_scan.py
```
