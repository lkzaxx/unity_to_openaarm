#!/bin/bash
# Interactive CAN motor ping tool
# Usage: ./can_ping.sh

PING_DATA="FFFFFFFFFFFFFFFC"
INTERVAL=0.5

show_menu() {
    clear
    echo "╔══════════════════════════════════════════╗"
    echo "║        CAN Motor Ping Tool               ║"
    echo "╠══════════════════════════════════════════╣"
    echo "║  CAN1 (Right Arm)   CAN2 (Left Arm)     ║"
    echo "║  ───────────────    ───────────────      ║"
    echo "║  1) can1 #001       8)  can2 #001        ║"
    echo "║  2) can1 #002       9)  can2 #002        ║"
    echo "║  3) can1 #003       10) can2 #003        ║"
    echo "║  4) can1 #004       11) can2 #004        ║"
    echo "║  5) can1 #005       12) can2 #005        ║"
    echo "║  6) can1 #006       13) can2 #006        ║"
    echo "║  7) can1 #007       14) can2 #007        ║"
    echo "║                                          ║"
    echo "║  15) can1 ALL       16) can2 ALL         ║"
    echo "║  17) ALL (can1+can2)                     ║"
    echo "║                                          ║"
    echo "║  0) Exit                                 ║"
    echo "╚══════════════════════════════════════════╝"
    echo ""
}

cleanup() {
    echo ""
    echo "--- Stopped ---"
    # kill background candump if any
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

    # start background listener
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
    echo "=== Pinging ALL (can1 + can2) #001~#007  (Ctrl+C to stop) ==="
    echo "    Interval: ${INTERVAL}s"
    echo "────────────────────────────────────────────"

    trap 'cleanup; return' INT

    candump any -t a &
    DUMP_PID=$!

    while kill -0 $DUMP_PID 2>/dev/null; do
        for id in 001 002 003 004 005 006 007; do
            cansend can1 "${id}#${PING_DATA}" 2>/dev/null
            cansend can2 "${id}#${PING_DATA}" 2>/dev/null
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
        1)  ping_single can1 001 ;;
        2)  ping_single can1 002 ;;
        3)  ping_single can1 003 ;;
        4)  ping_single can1 004 ;;
        5)  ping_single can1 005 ;;
        6)  ping_single can1 006 ;;
        7)  ping_single can1 007 ;;
        8)  ping_single can2 001 ;;
        9)  ping_single can2 002 ;;
        10) ping_single can2 003 ;;
        11) ping_single can2 004 ;;
        12) ping_single can2 005 ;;
        13) ping_single can2 006 ;;
        14) ping_single can2 007 ;;
        15) ping_multi can1 001 002 003 004 005 006 007 ;;
        16) ping_multi can2 001 002 003 004 005 006 007 ;;
        17) ping_all_buses ;;
        0)  echo "Bye!"; exit 0 ;;
        *)  echo "Invalid option"; sleep 1 ;;
    esac

    echo ""
    read -rp "Press Enter to return to menu..."
done
