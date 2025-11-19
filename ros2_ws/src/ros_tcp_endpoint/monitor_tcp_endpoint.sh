#!/bin/bash
# 監控 ROS2 TCP Endpoint 狀態（持續監聽，每秒刷新）

# 設定刷新間隔（秒）
REFRESH_INTERVAL=3

# 捕獲 Ctrl+C 信號，優雅退出
trap 'echo ""; echo "監控已停止"; exit 0' INT TERM

# 持續監控循環
while true; do
    # 清屏
    clear
    
    # 顯示標題和時間
    echo "=== ROS2 TCP Endpoint 監控 ==="
    echo "更新時間: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "刷新間隔: ${REFRESH_INTERVAL} 秒 | 按 Ctrl+C 退出"
    echo ""
    
    # 檢查進程
    echo "1. 進程狀態:"
    if ps aux | grep -q "[d]efault_server_endpoint"; then
        echo "   ✓ TCP Endpoint 正在運行"
        ps aux | grep "[d]efault_server_endpoint" | grep -v grep | awk '{print "   PID: " $2 " | CPU: " $3 "% | MEM: " $4 "%"}'
    else
        echo "   ✗ TCP Endpoint 未運行"
    fi
    echo ""
    
    # 檢查監聽狀態
    echo "2. 監聽狀態:"
    if netstat -tlnp 2>/dev/null | grep -q ":10000" || ss -tlnp 2>/dev/null | grep -q ":10000"; then
        echo "   ✓ Port 10000 正在監聽"
        LISTEN_INFO=$(netstat -tlnp 2>/dev/null | grep ":10000" || ss -tlnp 2>/dev/null | grep ":10000")
        echo "   $LISTEN_INFO"
    else
        echo "   ✗ Port 10000 未監聽"
    fi
    echo ""
    
    # 檢查 ROS2 節點
    echo "3. ROS2 節點:"
    # 嘗試 source ROS2 環境（如果尚未 source）
    if [ -f /home/idaka/ros2_ws/install/setup.bash ]; then
        source /home/idaka/ros2_ws/install/setup.bash >/dev/null 2>&1
    fi
    
    if ros2 node list 2>/dev/null | grep -q "UnityEndpoint"; then
        echo "   ✓ UnityEndpoint 節點存在"
        ros2 node list 2>/dev/null | grep "UnityEndpoint" | sed 's/^/   /'
    else
        echo "   ✗ UnityEndpoint 節點不存在"
    fi
    echo ""
    
    # 顯示 IP 地址
    echo "4. 網路資訊:"
    IP=$(ip addr show | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}' | cut -d/ -f1 | head -1)
    if [ -n "$IP" ]; then
        echo "   Jetson IP: $IP"
        echo "   Unity 應連接到: $IP:10000"
    else
        echo "   無法取得 IP 地址"
    fi
    echo ""
    
    # 檢查連接數
    echo "5. 當前連接:"
    ESTABLISHED=$(netstat -an 2>/dev/null | grep ":10000" | grep ESTABLISHED | wc -l)
    LISTENING=$(netstat -an 2>/dev/null | grep ":10000" | grep LISTEN | wc -l)
    echo "   已建立連接數: $ESTABLISHED"
    echo "   監聽狀態數: $LISTENING"
    echo ""
    
    # 顯示連接詳情（如果有連接）
    if [ "$ESTABLISHED" -gt 0 ]; then
        echo "6. 連接詳情:"
        netstat -an 2>/dev/null | grep ":10000" | grep ESTABLISHED | while read line; do
            echo "   $line"
        done
        echo ""
    fi
    
    # 監聽 Unity 接收的 Topics
    echo "7. Unity 接收的 Topics (Unity 端會發布訊息到這些 topics):"
    echo "   說明: 這些 topics 已預先註冊，但需要 Unity 連接並發送訊息後才會出現在列表中"
    echo "   狀態: TCP endpoint 已預先註冊這些 publishers，等待 Unity 連接..."
    echo ""
    
    # 定義要監聽的 topics
    declare -a UNITY_TOPICS=(
        "/openarm/joint_states:關節狀態回饋"
        "/openarm/end_effector_pose:末端執行器位置"
        "/openarm/status:OpenArm 系統狀態"
        "/unity/heartbeat_echo:心跳回音"
    )
    
    for topic_info in "${UNITY_TOPICS[@]}"; do
        IFS=':' read -r topic_name topic_desc <<< "$topic_info"
        
        # 檢查 topic 是否存在
        if ros2 topic list 2>/dev/null | grep -q "^${topic_name}$"; then
            echo "   ✓ ${topic_name} (${topic_desc})"
            
            # 獲取發布者資訊
            PUBLISHER_INFO=$(ros2 topic info ${topic_name} 2>/dev/null | grep "Publisher count:" | head -1)
            if [ -n "$PUBLISHER_INFO" ]; then
                PUB_COUNT=$(echo "$PUBLISHER_INFO" | awk '{print $3}')
                if [ "$PUB_COUNT" -gt 0 ]; then
                    # 獲取發布者節點名稱
                    PUBLISHER_NODES=$(ros2 topic info ${topic_name} 2>/dev/null | grep -A 10 "Publisher count:" | grep -E "^  /" | head -3)
                    if [ -n "$PUBLISHER_NODES" ]; then
                        echo "      發布者節點:"
                        echo "$PUBLISHER_NODES" | sed 's/^/         /'
                    fi
                    
                    # 獲取訊息類型
                    MSG_TYPE=$(ros2 topic type ${topic_name} 2>/dev/null)
                    if [ -n "$MSG_TYPE" ]; then
                        echo "      訊息類型: $MSG_TYPE"
                    fi
                    
                    # 獲取發布頻率（快速檢查，不阻塞）
                    HZ_OUTPUT=$(timeout 0.3 ros2 topic hz ${topic_name} --window 2 2>/dev/null | head -3)
                    if echo "$HZ_OUTPUT" | grep -q "average rate"; then
                        HZ_RATE=$(echo "$HZ_OUTPUT" | grep "average rate" | sed 's/average rate: //' | awk '{print $1}')
                        echo "      發布頻率: ~${HZ_RATE} Hz"
                    fi
                    
                    # 獲取最新訊息（快速獲取，限制時間和行數）
                    # 使用背景進程和超時來避免阻塞
                    TEMP_MSG=$(mktemp)
                    (timeout 0.25 ros2 topic echo ${topic_name} --once 2>/dev/null | head -12 > "$TEMP_MSG" 2>&1) &
                    MSG_PID=$!
                    sleep 0.2
                    if kill -0 $MSG_PID 2>/dev/null; then
                        kill $MSG_PID 2>/dev/null
                    fi
                    wait $MSG_PID 2>/dev/null
                    
                    if [ -s "$TEMP_MSG" ]; then
                        # 過濾並格式化訊息
                        MSG_CONTENT=$(cat "$TEMP_MSG" | grep -v "^data:" | grep -v "^---" | grep -v "^$" | head -10)
                        if [ -n "$MSG_CONTENT" ]; then
                            MSG_COUNT=$(echo "$MSG_CONTENT" | wc -l)
                            echo "      最新訊息 (來自發布者節點):"
                            echo "$MSG_CONTENT" | sed 's/^/         /' | head -8
                            if [ "$MSG_COUNT" -gt 8 ]; then
                                echo "         ... (顯示前8行)"
                            fi
                        fi
                    fi
                    rm -f "$TEMP_MSG" 2>/dev/null
                else
                    echo "      ⚠ 無發布者"
                fi
            fi
        else
            echo "   ✗ ${topic_name} (${topic_desc})"
            echo "      ⚠ Topic 不存在 - 需要節點發布此 topic 才會出現"
            # 檢查是否有類似的 topic（例如 /joint_states vs /openarm/joint_states）
            SIMILAR_TOPIC=$(ros2 topic list 2>/dev/null | grep -i "$(basename ${topic_name})" | head -1)
            if [ -n "$SIMILAR_TOPIC" ]; then
                echo "      提示: 發現類似 topic: ${SIMILAR_TOPIC}"
            fi
        fi
        echo ""
    done
    
    # 顯示實際存在的相關 topics
    echo "8. 實際存在的相關 Topics:"
    EXISTING_TOPICS=$(ros2 topic list 2>/dev/null | grep -E "joint|openarm|unity|status|pose" | head -10)
    if [ -n "$EXISTING_TOPICS" ]; then
        echo "$EXISTING_TOPICS" | sed 's/^/   - /'
    else
        echo "   無相關 topics"
    fi
    echo ""
    
    # 等待指定時間後刷新
    sleep $REFRESH_INTERVAL
done

