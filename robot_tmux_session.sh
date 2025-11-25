#!/bin/bash

# This script runs ON THE ROBOT

# 1. Create the menu script
cat <<'EOF' > ~/ros2_ws/scripts/run_menu.sh
#!/bin/bash
while true; do
    clear
    echo '=== OpenArm Control Menu ==='
    echo '1. Shake (shake.sh)'
    echo '2. Continuous Shake (shake_continuous.sh)'
    echo '3. X Pose (x.sh)'
    echo '4. O Pose (o.sh)'
    echo '5. Dance (dance.sh)'
    echo '6. Heart (heart.sh)'
    echo '7. Stop (stop.sh)'
    echo 'q. Quit Menu (Detach)'
    echo '=========================='
    read -p 'Select option: ' opt
    case $opt in
        1) ./shake.sh ;;
        2) ./shake_continuous.sh ;;
        3) ./x.sh ;;
        4) ./o.sh ;;
        5) ./dance.sh ;;
        6) ./heart.sh ;;
        7) ./stop.sh ;;
        q) exit ;;
        *) echo 'Invalid option'; sleep 1 ;;
    esac
    echo 'Press Enter to continue...'
    read
done
EOF
chmod +x ~/ros2_ws/scripts/run_menu.sh

# 2. Check/Create Tmux Session
# Force kill existing session to ensure clean state (optional, but safer for debugging)
tmux kill-session -t openarm_session 2>/dev/null

echo 'Creating new tmux session...'

# Window 0: Setup & Launch
tmux new-session -d -s openarm_session -n 'Setup'
tmux send-keys -t openarm_session:0 'cd ~/ros2_ws/scripts' C-m

# Cleanup previous processes
tmux send-keys -t openarm_session:0 'echo "Cleaning up previous processes..."' C-m
tmux send-keys -t openarm_session:0 'pkill -f ros2; pkill -f openarm_bringup; pkill -f robot_state_publisher; sleep 2' C-m

# Setup CAN with delay
tmux send-keys -t openarm_session:0 'echo "Setting up CAN..."' C-m
tmux send-keys -t openarm_session:0 'echo "idaka987" | sudo -S ./cansetup.sh' C-m
tmux send-keys -t openarm_session:0 'echo "Waiting for CAN to stabilize..."' C-m
tmux send-keys -t openarm_session:0 'sleep 3' C-m

# Launch ROS2
tmux send-keys -t openarm_session:0 'source ~/ros2_ws/install/setup.bash' C-m
tmux send-keys -t openarm_session:0 'ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=false' C-m

# Window 1: Diagnostics
tmux new-window -t openarm_session -n 'Diagnostics'
tmux send-keys -t openarm_session:1 'source ~/ros2_ws/install/setup.bash' C-m
tmux send-keys -t openarm_session:1 'watch -n 1 "ros2 topic list && ros2 control list_controllers"' C-m

# Split Window 1
tmux split-window -v -t openarm_session:1
tmux send-keys -t openarm_session:1.1 'source ~/ros2_ws/install/setup.bash' C-m
tmux send-keys -t openarm_session:1.1 'ros2 topic echo /joint_states' C-m

# Window 2: Menu
tmux new-window -t openarm_session -n 'Menu'
tmux send-keys -t openarm_session:2 'cd ~/ros2_ws/scripts' C-m
tmux send-keys -t openarm_session:2 'source ~/ros2_ws/install/setup.bash' C-m
tmux send-keys -t openarm_session:2 './run_menu.sh' C-m

tmux select-window -t openarm_session:2

# 3. Attach
# Use exec to replace the current shell with tmux, ensuring it takes over the terminal
exec tmux attach-session -t openarm_session
