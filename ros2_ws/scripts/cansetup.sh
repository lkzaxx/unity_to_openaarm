#!/bin/bash
# CAN bus setup with fixed serial-to-arm mapping
# Usage: ./cansetup.sh          # init only
#        ./cansetup.sh --reset   # USB reset + init
#
# Serial mapping (based on physical cable connections):
#   206F307B4153 (4153) = Right Arm
#   2048335F5052 (5052) = Left Arm
#
# [2026-04-08] 修正：不再假設 USB path 排序 = can 編號順序
# 改用 pcan sysfs → device symlink → USB parent → serial 直接查詢

# Auto-elevate to root if not already
if [ "$(id -u)" -ne 0 ]; then
    exec sudo "$0" "$@"
fi

RIGHT_SERIAL="206F307B4153"
LEFT_SERIAL="2048335F5052"

# --reset: USB reset PCAN devices first
if [ "$1" = "--reset" ] || [ "$1" = "-r" ]; then
    echo "=== Stopping CAN interfaces ==="
    for iface in $(ip -br link show type can 2>/dev/null | awk '{print $1}'); do
        ip link set "$iface" down 2>/dev/null
        echo "  $iface down"
    done

    echo ""
    echo "=== Unloading pcan driver ==="
    modprobe -r pcan 2>/dev/null && echo "  pcan unloaded" || echo "  pcan not loaded"
    sleep 1

    echo "=== Reloading pcan driver ==="
    modprobe pcan 2>/dev/null && echo "  pcan loaded" || echo "  pcan load failed"

    # [2026-04-08] 等待兩個 PCAN USB 設備和 CAN interface 都就緒（最多 30 秒）
    echo "  Waiting for 2 PCAN USB devices + CAN interfaces..."
    for i in $(seq 1 60); do
        pcan_count=$(ls -d /sys/class/pcan/pcanusbfd* 2>/dev/null | wc -l)
        # 計算非 can0 的 CAN interface 數量
        can_count=$(ip -br link show type can 2>/dev/null | awk '{print $1}' | grep -v can0 | wc -l)
        if [ "$pcan_count" -ge 2 ] && [ "$can_count" -ge 2 ]; then
            echo "  Ready: $pcan_count PCAN devices, $can_count CAN interfaces ($((i/2)).$(( (i%2)*5 ))s)"
            break
        fi
        sleep 0.5
    done
    pcan_count=$(ls -d /sys/class/pcan/pcanusbfd* 2>/dev/null | wc -l)
    can_count=$(ip -br link show type can 2>/dev/null | awk '{print $1}' | grep -v can0 | wc -l)
    if [ "$pcan_count" -lt 2 ] || [ "$can_count" -lt 2 ]; then
        echo "  WARNING: pcan=$pcan_count, can_ifaces=$can_count after 30s!"
    fi
    echo ""
fi

# Step 1: Init all CAN interfaces
echo "=== Initializing CAN ==="
for iface in $(ip -br link show type can 2>/dev/null | awk '{print $1}' | grep -v can0); do
    ip link set "$iface" down 2>/dev/null
    ip link set "$iface" type can bitrate 1000000 2>/dev/null
    ip link set "$iface" txqueuelen 1000 2>/dev/null
    ip link set "$iface" up 2>/dev/null
done

# Step 2: 透過 pcan sysfs 直接查詢 CAN interface → USB serial 對應
# 方法：pcan device → device symlink → 往上找 USB parent → 讀 serial
# 這個方法不受 USB 列舉順序影響，100% 可靠
declare -A SERIAL_TO_CAN
echo "=== Detecting PCAN serial mapping ==="
for pcan_dev in /sys/class/pcan/pcanusbfd*; do
    [ -d "$pcan_dev" ] || continue
    ndev=$(cat "$pcan_dev/ndev" 2>/dev/null)
    [ -z "$ndev" ] && continue

    # 從 pcan device 的 device symlink 往上追溯到 USB device
    devpath=$(readlink -f "$pcan_dev/device" 2>/dev/null)
    serial=""
    while [ "$devpath" != "/" ] && [ -n "$devpath" ]; do
        if [ -f "$devpath/serial" ]; then
            serial=$(cat "$devpath/serial")
            break
        fi
        devpath=$(dirname "$devpath")
    done

    if [ -n "$serial" ]; then
        SERIAL_TO_CAN["$serial"]="$ndev"
        echo "  $ndev → USB serial=$serial"
    else
        echo "  $ndev → serial not found (WARNING)"
    fi
done

RIGHT_CAN="${SERIAL_TO_CAN[$RIGHT_SERIAL]}"
LEFT_CAN="${SERIAL_TO_CAN[$LEFT_SERIAL]}"

# Fallback if detection fails
if [ -z "$RIGHT_CAN" ]; then
    echo "WARNING: Right arm serial $RIGHT_SERIAL not found, falling back to can1"
    RIGHT_CAN="can1"
fi
if [ -z "$LEFT_CAN" ]; then
    echo "WARNING: Left arm serial $LEFT_SERIAL not found, falling back to can2"
    LEFT_CAN="can2"
fi

# Sanity check: left and right should not be the same
if [ "$RIGHT_CAN" = "$LEFT_CAN" ]; then
    echo "ERROR: Right and Left mapped to same interface ($RIGHT_CAN)!"
    echo "Falling back: Right=can1, Left=can2"
    RIGHT_CAN="can1"
    LEFT_CAN="can2"
fi

# Save mapping
cat > /tmp/can_arm_map << EOF
RIGHT_CAN=$RIGHT_CAN
LEFT_CAN=$LEFT_CAN
EOF
chmod 644 /tmp/can_arm_map

echo ""
echo "Mapping: Right Arm = $RIGHT_CAN, Left Arm = $LEFT_CAN"
echo "$RIGHT_CAN (Right Arm) UP"
echo "$LEFT_CAN (Left Arm) UP"
echo "Saved to /tmp/can_arm_map"
