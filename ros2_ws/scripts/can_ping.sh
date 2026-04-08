#!/bin/bash
# Interactive CAN motor ping tool
# Reads /tmp/can_arm_map for arm-to-interface mapping
#
# Usage:
#   ./can_ping.sh           # 互動選單
#   ./can_ping.sh --right   # 快速檢查右臂，只顯示不通的
#   ./can_ping.sh --left    # 快速檢查左臂，只顯示不通的
#   ./can_ping.sh --both    # 快速檢查兩臂，只顯示不通的
#   ./can_ping.sh -r        # 同 --right
#   ./can_ping.sh -l        # 同 --left
#   ./can_ping.sh -b        # 同 --both

PING_DATA="FFFFFFFFFFFFFFFC"
INTERVAL=0.5
JOINT_NAMES=("J1(肩)" "J2(肩)" "J3(肘)" "J4(肘)" "J5(腕)" "J6(腕)" "J7(腕)")

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

# --- Quick check: ping one arm, only show failures ---
quick_check_arm() {
    local arm_name=$1
    local bus=$2
    local ids=(001 002 003 004 005 006 007)
    local recv_ids=(011 012 013 014 015 016 017)
    local all_ok=true

    echo "=== $arm_name ($bus) ==="
    for i in "${!ids[@]}"; do
        local sid=${ids[$i]}
        local rid=${recv_ids[$i]}
        # 送 enable，用 candump 等 0.3 秒收回應
        local resp
        resp=$(timeout 0.3 bash -c "candump $bus,$rid:7FF -n 1 2>/dev/null & sleep 0.05; cansend $bus ${sid}#${PING_DATA} 2>/dev/null; wait" 2>&1)
        if echo "$resp" | grep -q "$rid"; then
            : # OK, 不印
        else
            echo "  ❌ ${JOINT_NAMES[$i]} (#$sid) — no response"
            all_ok=false
        fi
    done
    if $all_ok; then
        echo "  ✅ All 7 motors OK"
    fi
}

quick_check() {
    local mode=$1
    if [ "$mode" = "right" ] || [ "$mode" = "both" ]; then
        quick_check_arm "Right Arm" "$RIGHT_CAN"
    fi
    if [ "$mode" = "left" ] || [ "$mode" = "both" ]; then
        quick_check_arm "Left Arm" "$LEFT_CAN"
    fi
}

# --- Command line arguments ---
case "${1:-}" in
    --right|-r) quick_check "right"; exit 0 ;;
    --left|-l)  quick_check "left";  exit 0 ;;
    --both|-b)  quick_check "both";  exit 0 ;;
    --help|-h)
        echo "Usage: $0 [--right|-r] [--left|-l] [--both|-b]"
        echo "  No args: interactive menu"
        echo "  --right/-r: quick check right arm"
        echo "  --left/-l:  quick check left arm"
        echo "  --both/-b:  quick check both arms"
        exit 0
        ;;
esac

# --- Main loop (interactive) ---
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
