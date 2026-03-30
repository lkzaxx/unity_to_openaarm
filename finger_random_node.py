
import rclpy, random, time, signal, threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState

ARM_POS = [-0.005912870984969842, 0.005531395437552433, -0.013542381933318026, 1.555848020141909, -0.007438773174639479, 0.007820248722056888, 0.0]
DURATION = 20
FINGER_INTERVAL = 2.0

class FingerRandomizer(Node):
    def __init__(self):
        super().__init__("finger_randomizer")
        vol_qos = QoSProfile(depth=1)
        vol_qos.durability = DurabilityPolicy.VOLATILE
        self.arm_pub = self.create_publisher(JointState, "/unity/joint_commands", vol_qos)
        self.hand_pub = self.create_publisher(JointState, "/unity/ehand_commands", vol_qos)
        self.start_time = time.time()
        self.count = 0

        # Arm keepalive at 2 Hz
        self.arm_timer = self.create_timer(0.5, self.send_arm)
        # Finger random at FINGER_INTERVAL
        self.finger_timer = self.create_timer(FINGER_INTERVAL, self.send_finger)
        # Home first
        self.home_sent = False
        self.create_timer(1.0, self.send_home_once)

    def send_arm(self):
        if time.time() - self.start_time > DURATION:
            raise SystemExit
        msg = JointState()
        msg.name = ["R_J1","R_J2","R_J3","R_J4","R_J5","R_J6","R_J7"]
        msg.position = ARM_POS[:]
        self.arm_pub.publish(msg)

    def send_home_once(self):
        if not self.home_sent:
            self.home_sent = True
            msg = JointState()
            msg.name = ["R_HAND_HOME"]
            msg.position = []
            self.hand_pub.publish(msg)
            self.get_logger().info("Hand HOME sent")

    def send_finger(self):
        if time.time() - self.start_time > DURATION:
            raise SystemExit
        if time.time() - self.start_time < 3.0:
            return  # wait for home to finish
        fingers = [round(random.uniform(0.0, 1.0), 2) for _ in range(6)]
        msg = JointState()
        msg.name = ["R_F1","R_F2","R_F3","R_F4","R_F5","R_F6"]
        msg.position = fingers
        self.hand_pub.publish(msg)
        self.count += 1
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"[{elapsed:.0f}s] #{self.count} fingers={fingers}")

rclpy.init()
node = FingerRandomizer()
try:
    rclpy.spin(node)
except (SystemExit, KeyboardInterrupt):
    pass
node.destroy_node()
rclpy.shutdown()
print(f"[done] Finished", flush=True)
