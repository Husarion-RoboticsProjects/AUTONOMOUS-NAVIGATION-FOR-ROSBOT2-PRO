from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.fl = 0.0
        self.fr = 0.0
        self.rl = 0.0
        self.rr = 0.0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Range, '/range/fl', self.cb_fl, qos)
        self.create_subscription(Range, '/range/fr', self.cb_fr, qos)
        self.create_subscription(Range, '/range/rl', self.cb_rl, qos)
        self.create_subscription(Range, '/range/rr', self.cb_rr, qos)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Nodo iniciado OK...")

    # Callbacks
    def cb_fl(self, msg):
        self.get_logger().info(f"[CALLBACK FL] recibido: {msg.range}")
        self.fl = msg.range

    def cb_fr(self, msg):
        self.get_logger().info(f"[CALLBACK FR] recibido: {msg.range}")
        self.fr = msg.range

    def cb_rl(self, msg):
        self.get_logger().info(f"[CALLBACK RL] recibido: {msg.range}")
        self.rl = msg.range

    def cb_rr(self, msg):
        self.get_logger().info(f"[CALLBACK RR] recibido: {msg.range}")
        self.rr = msg.range

    def control_loop(self):
        twist = Twist()
        TH = 0.6

        if self.fl < TH or self.fr < TH:
            twist.linear.x = 0.0
            twist.angular.z = 0.8 if self.fl > self.fr else -0.8
        else:
            twist.linear.x = 0.4
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f"[CONTROL LOOP] fl={self.fl:.3f}, fr={self.fr:.3f}, "
            f"rl={self.rl:.3f}, rr={self.rr:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

