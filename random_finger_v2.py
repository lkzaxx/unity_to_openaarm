#!/usr/bin/env python3
"""Random finger movement v2 - keeps arm still, randomizes fingers via ROS2 topics
Sends R_J1-R_J7 to enable right_tracking_unity, then R_F1-R_F6 random positions."""
import subprocess, random, time, signal, sys

DURATION = 120
FINGER_INTERVAL = 2.0  # seconds between random finger moves
ROS_SETUP = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"

stop = False
def handler(s, f): 
    global stop; stop = True
signal.signal(signal.SIGINT, handler)

def ros_pub(topic, names, positions):
    n = ",".join(names)
    p = ",".join(str(round(x, 4)) for x in positions)
    cmd = f'{ROS_SETUP} && ros2 topic pub {topic} sensor_msgs/msg/JointState "{{name: [{n}], position: [{p}]}}" --once'
    subprocess.run(["bash", "-c", cmd], capture_output=True, timeout=8)

# Step 1: Read current arm position
print("[init] Reading current arm position...", flush=True)
result = subprocess.run(["bash", "-c", 
    f'{ROS_SETUP} && ros2 topic echo /openarm/joint_states --once'],
    capture_output=True, text=True, timeout=10)

# Parse right arm positions from output
lines = result.stdout.split('\n')
positions = []
in_position = False
for line in lines:
    if 'position:' in line:
        in_position = True
        continue
    if in_position:
        line = line.strip()
        if line.startswith('- '):
            positions.append(float(line[2:]))
        else:
            break

# Right arm is indices 7-13
if len(positions) >= 14:
    arm_pos = positions[7:14]
else:
    print("[ERROR] Could not read arm position!", flush=True)
    sys.exit(1)

arm_names = ['R_J1','R_J2','R_J3','R_J4','R_J5','R_J6','R_J7']
print(f"[init] Right arm: {[round(x,3) for x in arm_pos]}", flush=True)

# Step 2: Send arm position to enable right_tracking_unity
print("[init] Enabling right_tracking_unity...", flush=True)
ros_pub('/unity/joint_commands', arm_names, arm_pos)
time.sleep(1)

# Step 3: Home hand first
print("[init] Hand HOME...", flush=True)
subprocess.run(["bash", "-c",
    f"""{ROS_SETUP} && ros2 topic pub /unity/ehand_commands sensor_msgs/msg/JointState "{{name: ['R_HAND_HOME'], position: []}}" --once"""],
    capture_output=True, timeout=8)
time.sleep(2)

# Step 4: Random finger loop + periodic arm keepalive
print(f"[start] Random finger movement for {DURATION}s", flush=True)
finger_names = ['R_F1','R_F2','R_F3','R_F4','R_F5','R_F6']
start_time = time.time()
count = 0

while not stop and (time.time() - start_time) < DURATION:
    # Random finger positions
    fingers = [round(random.uniform(0.0, 1.0), 2) for _ in range(6)]
    ros_pub('/unity/ehand_commands', finger_names, fingers)
    count += 1
    elapsed = time.time() - start_time
    print(f"[{elapsed:.0f}s] #{count} fingers={fingers}", flush=True)
    
    # Keepalive: re-send arm position every 5th iteration to prevent unity timeout
    if count % 3 == 0:
        ros_pub('/unity/joint_commands', arm_names, arm_pos)
    
    time.sleep(FINGER_INTERVAL)

print(f"[done] Sent {count} random finger commands in {time.time()-start_time:.0f}s", flush=True)
