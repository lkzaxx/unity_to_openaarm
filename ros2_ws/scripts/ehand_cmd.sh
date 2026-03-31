#!/bin/bash
# eHand 快速指令工具
# 用法：
#   ./ehand_cmd.sh home [R|L|B]      # 歸零（預設 R=右手）
#   ./ehand_cmd.sh open [R|L|B]      # 張開
#   ./ehand_cmd.sh close [R|L|B]     # 全握
#   ./ehand_cmd.sh disable [R|L|B]   # 禁用
#   ./ehand_cmd.sh enable [R|L|B]    # 啟用
#   ./ehand_cmd.sh grip R 0.5        # 右手半握 (0~1)
#   ./ehand_cmd.sh status [R|L|B]    # 讀狀態
#
# B = 雙手, R = 右手, L = 左手

CMD="${1:-home}"
SIDE="${2:-R}"
VALUE="${3:-1.0}"
TOPIC="/unity/ehand_commands"
DURATION=1  # 發送秒數

case "$SIDE" in
    R|r) PREFIX="R" ;;
    L|l) PREFIX="L" ;;
    B|b) PREFIX="BOTH" ;;
    *)   PREFIX="R" ;;
esac

case "$CMD" in
    home|HOME)
        if [ "$PREFIX" = "BOTH" ]; then NAME="HAND_HOME"
        else NAME="${PREFIX}_HAND_HOME"; fi
        MSG="{name: ['$NAME'], position: [0]}"
        ;;
    open|OPEN)
        if [ "$PREFIX" = "BOTH" ]; then NAME="HAND_OPEN"
        else NAME="${PREFIX}_HAND_OPEN"; fi
        MSG="{name: ['$NAME'], position: [0]}"
        ;;
    close|CLOSE)
        if [ "$PREFIX" = "BOTH" ]; then NAME="HAND_CLOSE"
        else NAME="${PREFIX}_HAND_CLOSE"; fi
        MSG="{name: ['$NAME'], position: [0]}"
        ;;
    disable|DISABLE)
        if [ "$PREFIX" = "BOTH" ]; then NAME="HAND_DISABLE"
        else NAME="${PREFIX}_HAND_DISABLE"; fi
        MSG="{name: ['$NAME'], position: [0]}"
        ;;
    enable|ENABLE)
        if [ "$PREFIX" = "BOTH" ]; then NAME="HAND_ENABLE"
        else NAME="${PREFIX}_HAND_ENABLE"; fi
        MSG="{name: ['$NAME'], position: [0]}"
        ;;
    status|STATUS)
        if [ "$PREFIX" = "BOTH" ]; then NAME="HAND_STATUS"
        else NAME="${PREFIX}_HAND_STATUS"; fi
        MSG="{name: ['$NAME'], position: [0]}"
        ;;
    grip|GRIP)
        if [ "$PREFIX" = "BOTH" ]; then
            MSG="{name: ['L_F1','L_F2','L_F3','L_F4','L_F5','L_F6','R_F1','R_F2','R_F3','R_F4','R_F5','R_F6'], position: [$VALUE,$VALUE,$VALUE,$VALUE,$VALUE,$VALUE,$VALUE,$VALUE,$VALUE,$VALUE,$VALUE,$VALUE]}"
        else
            MSG="{name: ['${PREFIX}_F1','${PREFIX}_F2','${PREFIX}_F3','${PREFIX}_F4','${PREFIX}_F5','${PREFIX}_F6'], position: [$VALUE,$VALUE,$VALUE,$VALUE,$VALUE,$VALUE]}"
        fi
        ;;
    *)
        echo "Unknown command: $CMD"
        echo "Usage: $0 {home|open|close|disable|enable|status|grip} [R|L|B] [value]"
        exit 1
        ;;
esac

echo ">> $CMD $SIDE → $MSG"

# 發送：10Hz 持續 DURATION 秒後自動停止
ros2 topic pub -r 10 "$TOPIC" sensor_msgs/msg/JointState "$MSG" &
PUB_PID=$!
sleep "$DURATION"
kill $PUB_PID 2>/dev/null
wait $PUB_PID 2>/dev/null
echo "<< done"
