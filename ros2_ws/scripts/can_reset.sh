#!/bin/bash
# Reset all PCAN-USB devices and re-init CAN buses
# Usage: sudo ./can_reset.sh

echo "=== Stopping CAN interfaces ==="
for iface in $(ip -br link show type can | awk '{print $1}'); do
    ip link set "$iface" down
    echo "  $iface down"
done

echo ""
echo "=== Resetting USB PCAN devices ==="
for dev in /sys/bus/usb/devices/*/idVendor; do
    dir=$(dirname "$dev")
    vendor=$(cat "$dir/idVendor" 2>/dev/null)
    product=$(cat "$dir/idProduct" 2>/dev/null)
    if [ "$vendor" = "0c72" ] && [ "$product" = "0012" ]; then
        serial=$(cat "$dir/serial" 2>/dev/null)
        echo "  Resetting PCAN $serial at $dir"
        echo 0 > "$dir/authorized"
        sleep 0.5
        echo 1 > "$dir/authorized"
    fi
done

sleep 2
echo ""
echo "=== Re-init CAN buses ==="
bash ~/ros2_ws/scripts/cansetup.sh

echo ""
echo "=== Result ==="
ip link show type can
