jetson nano 系统
用户密码   idaka/idaka987
进入~/ros2_ws/scripts

模拟器运行 openfakearm.sh

真机按照如下步骤
给手臂供电，粗红色24v，粗黑地
细红/黄 canH  细黑绿 canL
1. 先设置can
./cansetup.sh

2. 启动openarm
./openarm.sh
执行后can模块闪灯，所有手臂的电机亮绿灯则正常

3.一定要先小角度测试，看左右手对不对，比如执行左手的脚本看是
不是左手动作，如果不是，交换can的接线
./leftrighttest/leftarm.sh  小角度左手
./leftrighttest/rightarm.sh   小角度右手
./leftrighttest/lefthand.sh  左手夹子
./leftrighttest/righthand.sh  右手夹子

4.上述都确认完成后可以执行脚本
dance.sh 电舞
heart.sh 比心
shake.sh 招手
o.sh 比圈
x.sh 比X