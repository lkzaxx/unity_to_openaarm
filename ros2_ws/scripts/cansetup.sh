#!/bin/bash
# CAN bus setup with hardcoded mapping (PCAN-USB Pro FD, single USB cable)
# Usage: ./cansetup.sh          # init only
#        ./cansetup.sh --reset   # USB reset + init
#
# Mapping (verified 2026-04-29 with PCAN-USB Pro FD):
#   can1 = Right Arm (8 motors incl. dexterous hand)
#   can2 = Left Arm  (7 motors)
#
# Note: Pro FD presents both channels under the same Tegra USB controller
# path, so the previous serial-based detection (designed for 2x discrete
# PCAN-USB FD adapters) returns ambiguous "3610000.usb". Hardcoded here
# because the rig has a single Pro FD with stable port assignment.

# Auto-elevate to root if not already
if [ "$(id -u)" -ne 0 ]; then
    exec sudo "$0" "$@"
fi

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

    echo "  Waiting for 2 PCAN USB devices + CAN interfaces..."
    for i in $(seq 1 60); do
        pcan_count=$(ls -d /sys/class/pcan/pcanusbfd* 2>/dev/null | wc -l)
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

# Step 1: Init all CAN interfaces (CAN 2.0 @ 1Mbps)
echo "=== Initializing CAN ==="
for iface in $(ip -br link show type can 2>/dev/null | awk '{print $1}' | grep -v can0); do
    ip link set "$iface" down 2>/dev/null
    ip link set "$iface" type can bitrate 1000000 2>/dev/null
    ip link set "$iface" txqueuelen 1000 2>/dev/null
    ip link set "$iface" up 2>/dev/null
done

# Step 2: Hardcoded mapping (PCAN-USB Pro FD, verified 2026-04-29)
RIGHT_CAN="can1"
LEFT_CAN="can2"

# Sanity check: both interfaces must exist
for iface in "$RIGHT_CAN" "$LEFT_CAN"; do
    if ! ip link show "$iface" &>/dev/null; then
        echo "ERROR: $iface not found! Is PCAN-USB Pro FD connected?"
        exit 1
    fi
done

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
