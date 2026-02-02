#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Variables para guardar lecturas reales de los sensores
        self.fl = 1.0
        self.fr = 1.0
        self.rl = 1.0
        self.rr = 1.0

        # Suscripción a tus tópicos reales de sensores
        self.create_subscription(Range, '/range/fl', self.cb_fl, 10)
        self.create_subscription(Range, '/range/fr', self.cb_fr, 10)
        self.create_subscription(Range, '/range/rl', self.cb_rl, 10)
        self.create_subscription(Range, '/range/rr', self.cb_rr, 10)

        # Publicador de velocidad para mover el robot
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer de control a 10 Hz aprox (0.1s)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Obstacle avoidance node started OK — waiting for real sensor data...")

    # Callbacks que guardan la distancia que llega en metros
    def cb_fl(self, msg):
        self.fl = msg.range

    def cb_fr(self, msg):
        self.fr = msg.range

    def cb_rl(self, msg):
        self.rl = msg.range

    def cb_rr(self, msg):
        self.rr = msg.range

    # Loop de control principal
    def control_loop(self):
        cmd = Twist()
        threshold = 0.30  # 30 cm en metros

        # Imprimir lecturas crudas REALES en log
        self.get_logger().info(
            f"SENSORS → fl={self.fl:.3f}m, fr={self.fr:.3f}m, rl={self.rl:.3f}m, rr={self.rr:.3f}m"
        )

        # Si hay obstáculo al frente
        if min(self.fl, self.fr) < threshold:
            cmd.linear.x = 0.0  # detener avance
            # Girar al lado donde haya más espacio
            if self.fl > self.fr:
                cmd.angular.z = 0.6  # girar izquierda
                self.get_logger().warn("Obstacle front — turning LEFT")
            else:
                cmd.angular.z = -0.6  # girar derecha
                self.get_logger().warn("Obstacle front — turning RIGHT")
        else:
            cmd.linear.x = 0.24  # avanzar normal equilibrado
            cmd.angular.z = 0.0

        self.vel_pub.publish(cmd)

def main():
    rclpy.init()
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
