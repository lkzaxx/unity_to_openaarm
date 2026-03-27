#!/bin/bash
# Interactive CAN motor ping tool
# Reads /tmp/can_arm_map for arm-to-interface mapping
# Usage: ./can_ping.sh

PING_DATA="FFFFFFFFFFFFFFFC"
INTERVAL=0.5

# Load mapping from cansetup.sh
if [ -f /tmp/can_arm_map ]; then
    source /tmp/can_arm_map
else
    RIGHT_CAN=can1
    LEFT_CAN=can2
fi

show_menu() {
    clear
    echo "╔══════════════════════════════════════════════╗"
    echo "║          CAN Motor Ping Tool                 ║"
    echo "╠══════════════════════════════════════════════╣"
    echo "║  Right Arm ($RIGHT_CAN)     Left Arm ($LEFT_CAN)      ║"
    echo "║  ──────────────      ──────────────       ║"
    echo "║  1) Right #001       8)  Left #001        ║"
    echo "║  2) Right #002       9)  Left #002        ║"
    echo "║  3) Right #003       10) Left #003        ║"
    echo "║  4) Right #004       11) Left #004        ║"
    echo "║  5) Right #005       12) Left #005        ║"
    echo "║  6) Right #006       13) Left #006        ║"
    echo "║  7) Right #007       14) Left #007        ║"
    echo "║                                            ║"
    echo "║  15) Right ALL       16) Left ALL          ║"
    echo "║  17) ALL (Right+Left)                      ║"
    echo "║                                            ║"
    echo "║  0) Exit                                   ║"
    echo "╚══════════════════════════════════════════════╝"
    echo ""
}

cleanup() {
    echo ""
    echo "--- Stopped ---"
    kill $DUMP_PID 2>/dev/null
    wait $DUMP_PID 2>/dev/null
}

ping_single() {
    local bus=$1
    local id=$2

    echo "=== Pinging $bus #$id  (Ctrl+C to stop, then return to menu) ==="
    echo "    Sending: cansend $bus ${id}#${PING_DATA}"
    echo "    Interval: ${INTERVAL}s"
    echo "────────────────────────────────────────────"

    trap 'cleanup; return' INT

    candump "$bus" -t a &
    DUMP_PID=$!

    while kill -0 $DUMP_PID 2>/dev/null; do
        cansend "$bus" "${id}#${PING_DATA}" 2>/dev/null
        if [ $? -ne 0 ]; then
            echo "[ERROR] cansend failed - bus down?"
            break
        fi
        sleep "$INTERVAL"
    done

    kill $DUMP_PID 2>/dev/null
    wait $DUMP_PID 2>/dev/null
    trap - INT
}

ping_multi() {
    local bus=$1
    shift
    local ids=("$@")

    local label=""
    for id in "${ids[@]}"; do label="$label #$id"; done
    echo "=== Pinging $bus $label  (Ctrl+C to stop) ==="
    echo "    Interval: ${INTERVAL}s"
    echo "────────────────────────────────────────────"

    trap 'cleanup; return' INT

    candump "$bus" -t a &
    DUMP_PID=$!

    while kill -0 $DUMP_PID 2>/dev/null; do
        for id in "${ids[@]}"; do
            cansend "$bus" "${id}#${PING_DATA}" 2>/dev/null
        done
        sleep "$INTERVAL"
    done

    kill $DUMP_PID 2>/dev/null
    wait $DUMP_PID 2>/dev/null
    trap - INT
}

ping_all_buses() {
    echo "=== Pinging ALL (Right=$RIGHT_CAN + Left=$LEFT_CAN) #001~#007  (Ctrl+C to stop) ==="
    echo "    Interval: ${INTERVAL}s"
    echo "────────────────────────────────────────────"

    trap 'cleanup; return' INT

    candump any -t a &
    DUMP_PID=$!

    while kill -0 $DUMP_PID 2>/dev/null; do
        for id in 001 002 003 004 005 006 007; do
            cansend "$RIGHT_CAN" "${id}#${PING_DATA}" 2>/dev/null
            cansend "$LEFT_CAN" "${id}#${PING_DATA}" 2>/dev/null
        done
        sleep "$INTERVAL"
    done

    kill $DUMP_PID 2>/dev/null
    wait $DUMP_PID 2>/dev/null
    trap - INT
}

# --- Main loop ---
while true; do
    show_menu
    read -rp "Select> " choice

    case $choice in
        1)  ping_single "$RIGHT_CAN" 001 ;;
        2)  ping_single "$RIGHT_CAN" 002 ;;
        3)  ping_single "$RIGHT_CAN" 003 ;;
        4)  ping_single "$RIGHT_CAN" 004 ;;
        5)  ping_single "$RIGHT_CAN" 005 ;;
        6)  ping_single "$RIGHT_CAN" 006 ;;
        7)  ping_single "$RIGHT_CAN" 007 ;;
        8)  ping_single "$LEFT_CAN" 001 ;;
        9)  ping_single "$LEFT_CAN" 002 ;;
        10) ping_single "$LEFT_CAN" 003 ;;
        11) ping_single "$LEFT_CAN" 004 ;;
        12) ping_single "$LEFT_CAN" 005 ;;
        13) ping_single "$LEFT_CAN" 006 ;;
        14) ping_single "$LEFT_CAN" 007 ;;
        15) ping_multi "$RIGHT_CAN" 001 002 003 004 005 006 007 ;;
        16) ping_multi "$LEFT_CAN" 001 002 003 004 005 006 007 ;;
        17) ping_all_buses ;;
        0)  echo "Bye!"; exit 0 ;;
        *)  echo "Invalid option"; sleep 1 ;;
    esac

    echo ""
    read -rp "Press Enter to return to menu..."
done
