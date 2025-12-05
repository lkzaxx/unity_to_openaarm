jetson nano 系統
使用者密碼   idaka/idaka987
進入~/ros2_ws/scripts

模擬器執行 openfakearm.sh

實機按照如下步驟
給手臂供電，粗紅色24v，粗黑地
細紅/黃 canH  細黑綠 canL
1. 先設定can
./cansetup.sh

2. 啟動openarm
./openarm.sh
執行後can模組閃燈，所有手臂的電機亮綠燈則正常

3.一定要先小角度測試，看左右手對不對，比如執行左手的腳本看是
不是左手動作，如果不是，交換can的接線
./leftrighttest/leftarm.sh  小角度左手
./leftrighttest/rightarm.sh   小角度右手
./leftrighttest/lefthand.sh  左手夾子
./leftrighttest/righthand.sh  右手夾子

4.上述都確認完成後可以執行腳本
dance.sh 電舞
heart.sh 比心
shake.sh 招手
o.sh 比圈
x.sh 比X

速度保護測試腳本（Unity Interface）：

【Docker 環境】
1）先從 Windows 宿主機複製測試腳本到 Docker 容器（在 Windows CMD 執行）：
   docker cp C:\code\vr_robot\openarm\unity_to_openaarm\ros2_ws\scripts\test_velocity_protection.py <容器ID或名稱>:/root/ros2_ws/scripts/
   範例：
   docker cp C:\code\vr_robot\openarm\unity_to_openaarm\ros2_ws\scripts\test_velocity_protection.py openarm:/root/ros2_ws/scripts/

2）在 Docker 容器終端執行測試：
   cd ~/ros2_ws/scripts
   python3 test_velocity_protection.py

【Jetson Nano 實機】
1）先從開發機複製測試腳本到 Jetson Nano（在 Windows CMD 執行）：
   scp C:\code\vr_robot\openarm\unity_to_openaarm\ros2_ws\scripts\test_velocity_protection.py idaka@<jetson_ip>:~/ros2_ws/scripts/

2）在 Jetson Nano 終端執行測試：
   cd ~/ros2_ws/scripts
   python3 test_velocity_protection.py

注意：執行前確保 ROS2 環境已正確初始化

關節限制測試腳本（測試正負極限值）：

【Docker 環境】
1）複製測試腳本到容器（在 Windows CMD 執行）：
   docker cp C:\code\vr_robot\openarm\unity_to_openaarm\ros2_ws\scripts\test_joint_limits.py openarm:/root/ros2_ws/scripts/

2）在容器內執行測試：
   cd ~/ros2_ws/scripts
   python3 test_joint_limits.py
   
   測試模式：
   - 模式 1: 左手完整測試（所有關節正負極限）
   - 模式 2: 右手完整測試（所有關節正負極限）
   - 模式 3: 對比模式（驗證左右手方向是否需要鏡像）
   - 模式 4: 單一關節測試
   
   用途：驗證 JOINT_LIMITS_FIX_PROPOSAL.md 中的關節方向設定
