
import rclpy, random, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

class F(Node):
    def __init__(self):
        super().__init__("finger_inf")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.ap = self.create_publisher(JointState, "/unity/joint_commands", qos)
        self.hp = self.create_publisher(JointState, "/unity/ehand_commands", qos)
        self.ready = False
        self.create_timer(0.1, self.wait_subs)

    def wait_subs(self):
        if self.ap.get_subscription_count() > 0 and self.hp.get_subscription_count() > 0:
            if not self.ready:
                self.ready = True
                self.arm_pos = [-0.02, 0.006, -0.014, 1.526, -0.007, 0.008, 0.0]
                self.create_timer(0.2, self.ka)
                m = JointState(); m.name = ["R_HAND_ENABLE"]; m.position = []
                self.hp.publish(m)
                self.get_logger().info("ENABLE + start")
                self.create_timer(2.0, self.sf_once)

    def sf_once(self):
        if hasattr(self, '_started'): return
        self._started = True
        self.create_timer(INTERVAL, self.sf)

    def ka(self):
        m = JointState()
        m.name = ["R_J1","R_J2","R_J3","R_J4","R_J5","R_J6","R_J7"]
        m.position = self.arm_pos[:]
        self.ap.publish(m)

    def sf(self):
        f = [round(random.uniform(0.0, 1.0), 2) for _ in range(6)]
        m = JointState()
        m.name = ["R_F1","R_F2","R_F3","R_F4","R_F5","R_F6"]
        m.position = f
        self.hp.publish(m)

INTERVAL = 2.0
rclpy.init()
n = F()
try:
    rclpy.spin(n)
except (SystemExit, KeyboardInterrupt):
    pass
n.destroy_node(); rclpy.shutdown()
