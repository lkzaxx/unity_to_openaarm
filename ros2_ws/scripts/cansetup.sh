sudo -A ip link set can1 down
sudo -A ip link set can1 type can bitrate 1000000
sudo -A ip link set can1 txqueuelen 1000
sudo -A ip link set can1 up
sudo -A ip link set can2 down
sudo -A ip link set can2 type can bitrate 1000000
sudo -A ip link set can2 txqueuelen 1000
sudo -A ip link set can2 up
