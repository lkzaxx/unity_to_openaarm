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
