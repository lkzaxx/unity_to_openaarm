#!/bin/bash
# ============================================================
# 方式 A：Raw CAN (cansend + candump) 讀取馬達角度
#
# 原理：發送 refresh 指令到 CAN ID 0x7FF，馬達回傳狀態封包
# 回傳 data[1:2]=position, data[3:4]=velocity, data[4:5]=torque
# 需手動解碼 uint16 → float（達妙 MIT 協議）
#
# 用法：./test_read_angles_A.sh [can1|can2]
# ============================================================

# 讀取 CAN mapping
if [ -f /tmp/can_arm_map ]; then
    source /tmp/can_arm_map
else
    RIGHT_CAN=can1
    LEFT_CAN=can2
fi

BUS=${1:-$RIGHT_CAN}
echo "=== 方式 A: Raw CAN 讀取角度 ==="
echo "CAN bus: $BUS"
echo ""

# 達妙馬達 Motor Type 的 pMax（所有類型都是 12.5 rad）
P_MAX=12.5

# 馬達配置：Motor ID → (send_id, recv_id, motor_type_name)
# V10: J1=DM8009, J2=DM8009, J3=DM4340, J4=DM4340, J5=DM4310, J6=DM4310, J7=DM4310
SEND_IDS=(001 002 003 004 005 006 007)
RECV_IDS=(011 012 013 014 015 016 017)
MOTOR_NAMES=("J1-DM8009" "J2-DM8009" "J3-DM4340" "J4-DM4340" "J5-DM4310" "J6-DM4310" "J7-DM4310")

# 開始 candump 背景監聽，存到暫存檔
TMPFILE=$(mktemp /tmp/can_dump_XXXXXX.txt)
candump "$BUS" -t a -n 7 > "$TMPFILE" 2>&1 &
DUMP_PID=$!

sleep 0.1

# 對每顆馬達發送 refresh 指令
# refresh 封包格式：CAN ID=0x7FF, data=[send_id, 0x00, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00]
echo "發送 refresh 指令..."
for sid in "${SEND_IDS[@]}"; do
    cansend "$BUS" "7FF#${sid}00CC0000000000"
    sleep 0.005  # 5ms 間隔避免 CAN bus 擁塞
done

# 等待回應
sleep 0.5
kill $DUMP_PID 2>/dev/null
wait $DUMP_PID 2>/dev/null

echo ""
echo "--- Raw CAN 回應 ---"
cat "$TMPFILE"
echo ""

# 解碼回應
echo "--- 解碼角度 ---"
while IFS= read -r line; do
    # candump 格式: (timestamp) interface CAN_ID#DATA
    # 例: (1711900000.000000) can1 011#AABBCCDDEEFF0102
    hex_id=$(echo "$line" | grep -oP '[0-9A-Fa-f]{3}(?=#)')
    hex_data=$(echo "$line" | grep -oP '(?<=#)[0-9A-Fa-f]+')

    if [ -z "$hex_id" ] || [ -z "$hex_data" ]; then
        continue
    fi

    # 找到對應的馬達
    motor_idx=-1
    for i in "${!RECV_IDS[@]}"; do
        if [ "$hex_id" = "${RECV_IDS[$i]}" ]; then
            motor_idx=$i
            break
        fi
    done

    if [ $motor_idx -lt 0 ]; then
        echo "  未知 CAN ID: 0x$hex_id"
        continue
    fi

    # 解碼 position: data[1] << 8 | data[2]  (16-bit uint)
    # data bytes: 索引 0=byte0, 2chars each
    b1=${hex_data:2:2}   # data[1]
    b2=${hex_data:4:2}   # data[2]
    q_uint=$((16#${b1}${b2}))

    # uint16 → double: value = q_uint / 65535 * 2*pMax - pMax
    # 用 awk 做浮點運算
    angle_rad=$(awk "BEGIN {printf \"%.4f\", $q_uint / 65535.0 * 2.0 * $P_MAX - $P_MAX}")
    angle_deg=$(awk "BEGIN {printf \"%.2f\", $angle_rad * 180.0 / 3.14159265}")

    echo "  ${MOTOR_NAMES[$motor_idx]}: raw=0x${b1}${b2} (${q_uint}) → ${angle_rad} rad (${angle_deg}°)"

done < "$TMPFILE"

rm -f "$TMPFILE"
echo ""
echo "=== 方式 A 完成 ==="
