#!/bin/bash
# CAN bus setup with fixed serial-to-arm mapping
# Usage: ./cansetup.sh          # init only
#        ./cansetup.sh --reset   # USB reset + init
#
# Serial mapping (based on physical cable connections):
#   206F307B4153 (4153) = Right Arm
#   2048335F5052 (5052) = Left Arm
#
# pcan driver names CAN interfaces in USB enumeration order:
#   lowest USB path → can1, next → can2
# So we find each PCAN's USB path, sort, and map accordingly.

# Auto-elevate to root if not already
if [ "$(id -u)" -ne 0 ]; then
    exec sudo "$0" "$@"
fi

RIGHT_SERIAL="2048335F5052"
LEFT_SERIAL="206F307B4153"

# --reset: Full USB bus rescan to re-enumerate all USB devices
# This deauthorizes ALL USB buses (usb1+usb2), waits, then reauthorizes.
# All USB devices (Bluetooth, ZLGCAN, RealSense, etc.) will briefly disconnect.
if [ "$1" = "--reset" ] || [ "$1" = "-r" ]; then
    echo "=== Stopping CAN interfaces ==="
    for iface in $(ip -br link show type can 2>/dev/null | awk '{print $1}'); do
        ip link set "$iface" down 2>/dev/null
        echo "  $iface down"
    done

    echo ""
    echo "=== Full USB bus rescan ==="
    # Deauthorize all USB buses
    for bus in /sys/bus/usb/devices/usb*/authorized; do
        echo 0 > "$bus" 2>/dev/null
    done
    echo "  All USB buses deauthorized, waiting 2s..."
    sleep 2

    # Reauthorize all USB buses
    for bus in /sys/bus/usb/devices/usb*/authorized; do
        echo 1 > "$bus" 2>/dev/null
    done
    echo "  All USB buses reauthorized, waiting 5s for enumeration..."
    sleep 5

    # Verify PCAN devices came back
    PCAN_COUNT=$(lsusb -d 0c72:0012 2>/dev/null | wc -l)
    echo "  PCAN devices found: $PCAN_COUNT"
    if [ "$PCAN_COUNT" -lt 2 ]; then
        echo "  ⚠️ Expected 2 PCAN devices, found $PCAN_COUNT"
    else
        echo "  ✅ USB bus rescan done"
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

# Step 2: Find PCAN USB devices, sort by path → map to can1/can2
declare -A SERIAL_TO_CAN
idx=1
for dev in $(for d in /sys/bus/usb/devices/*/idVendor; do
    dir=$(dirname "$d")
    vendor=$(cat "$dir/idVendor" 2>/dev/null)
    product=$(cat "$dir/idProduct" 2>/dev/null)
    if [ "$vendor" = "0c72" ] && [ "$product" = "0012" ]; then
        echo "$(basename "$dir")"
    fi
done | sort); do
    serial=$(cat "/sys/bus/usb/devices/$dev/serial" 2>/dev/null)
    SERIAL_TO_CAN["$serial"]="can${idx}"
    ((idx++))
done

RIGHT_CAN="${SERIAL_TO_CAN[$RIGHT_SERIAL]}"
LEFT_CAN="${SERIAL_TO_CAN[$LEFT_SERIAL]}"

# Fallback if detection fails
[ -z "$RIGHT_CAN" ] && RIGHT_CAN="can1"
[ -z "$LEFT_CAN" ] && LEFT_CAN="can2"

# Save mapping
cat > /tmp/can_arm_map << EOF
RIGHT_CAN=$RIGHT_CAN
LEFT_CAN=$LEFT_CAN
EOF
chmod 644 /tmp/can_arm_map

echo "Mapping: Right Arm = $RIGHT_CAN, Left Arm = $LEFT_CAN"
echo "$RIGHT_CAN (Right Arm) UP"
echo "$LEFT_CAN (Left Arm) UP"
echo "Saved to /tmp/can_arm_map"
