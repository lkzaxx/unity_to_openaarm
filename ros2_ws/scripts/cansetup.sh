#!/bin/bash
# CAN bus setup with fixed serial-to-name mapping
# can1 = Right Arm (serial 4153)
# can2 = Left Arm  (serial 5052)

RIGHT_SERIAL="206F307B4153"
LEFT_SERIAL="2048335F5052"

# Find which interface has which serial
get_iface_for_serial() {
    local target_serial=$1
    for dev in /sys/bus/usb/devices/*/serial; do
        [ -f "$dev" ] || continue
        serial=$(cat "$dev")
        if [ "$serial" = "$target_serial" ]; then
            devdir=$(dirname "$dev")
            # find the CAN net interface under this USB device
            for net in "$devdir"/*/net/can*; do
                [ -d "$net" ] && basename "$net" && return
            done
        fi
    done
}

# Bring all CAN interfaces down
for iface in $(ip -br link show type can | awk '{print $1}'); do
    ip link set "$iface" down 2>/dev/null
done

right_iface=$(get_iface_for_serial "$RIGHT_SERIAL")
left_iface=$(get_iface_for_serial "$LEFT_SERIAL")

echo "Detected: Right($RIGHT_SERIAL) = $right_iface, Left($LEFT_SERIAL) = $left_iface"

# Swap names if needed
if [ -n "$right_iface" ] && [ "$right_iface" != "can1" ]; then
    echo "Swapping: $right_iface -> can1, $left_iface -> can2"
    ip link set "$right_iface" name can_tmp 2>/dev/null
    ip link set can1 name can2 2>/dev/null
    ip link set can_tmp name can1 2>/dev/null
fi

# Init can1 (Right Arm)
ip link set can1 down 2>/dev/null
ip link set can1 type can bitrate 1000000
ip link set can1 txqueuelen 1000
ip link set can1 up
echo "can1 (Right Arm) UP"

# Init can2 (Left Arm)
ip link set can2 down 2>/dev/null
ip link set can2 type can bitrate 1000000
ip link set can2 txqueuelen 1000
ip link set can2 up
echo "can2 (Left Arm) UP"
