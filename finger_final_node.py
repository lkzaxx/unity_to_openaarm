
import rclpy, random, time, signal
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

DURATION = 999999
FINGER_INTERVAL = 2.0

class FingerRandomizer(Node):
    def __init__(self):
        super().__init__("finger_randomizer_final")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.arm_pub = self.create_publisher(JointState, "/unity/joint_commands", qos)
        self.hand_pub = self.create_publisher(JointState, "/unity/ehand_commands", qos)
        self.start_time = None
        self.count = 0
        self.ready = False

        # Wait for discovery, then setup
        self.setup_timer = self.create_timer(0.1, self.wait_for_subs)

    def wait_for_subs(self):
        if self.arm_pub.get_subscription_count() > 0 and self.hand_pub.get_subscription_count() > 0:
            self.setup_timer.cancel()
            self.get_logger().info("Subscribers found, initializing...")
            self.init_hand()

    def init_hand(self):
        # Read current arm position
        self.arm_pos = [-0.02, 0.006, -0.014, 1.526, -0.007, 0.008, 0.0]

        # Arm keepalive at 5Hz
        self.arm_timer = self.create_timer(0.2, self.send_arm)

        # ENABLE hand first (undo HOME disabled)
        msg = JointState()
        msg.name = ["R_HAND_ENABLE"]
        msg.position = []
        self.hand_pub.publish(msg)
        self.get_logger().info("R_HAND_ENABLE sent")

        # Start fingers after 2s
        self.create_timer(2.0, self.start_fingers_once)

    def start_fingers_once(self):
        if self.ready: return
        self.ready = True
        self.start_time = time.time()
        self.finger_timer = self.create_timer(FINGER_INTERVAL, self.send_finger)
        self.get_logger().info(f"Starting random fingers for {DURATION}s")

    def send_arm(self):
        if self.start_time and time.time() - self.start_time > DURATION + 5:
            raise SystemExit
        msg = JointState()
        msg.name = ["R_J1","R_J2","R_J3","R_J4","R_J5","R_J6","R_J7"]
        msg.position = self.arm_pos[:]
        self.arm_pub.publish(msg)

    def send_finger(self):
        if not self.start_time: return
        if time.time() - self.start_time > DURATION:
            raise SystemExit
        fingers = [round(random.uniform(0.0, 1.0), 2) for _ in range(6)]
        msg = JointState()
        msg.name = ["R_F1","R_F2","R_F3","R_F4","R_F5","R_F6"]
        msg.position = fingers
        self.hand_pub.publish(msg)
        self.count += 1
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"[{elapsed:.0f}s] #{self.count} {fingers}")

rclpy.init()
node = FingerRandomizer()
try:
    rclpy.spin(node)
except (SystemExit, KeyboardInterrupt):
    pass
node.destroy_node()
rclpy.shutdown()
