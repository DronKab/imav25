#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class PlatformLandingNode(Node):
    def __init__(self):
        super().__init__('platform_landing')
        self.get_logger().info('platform_landing node started')

        # Inicializar variables de error
        self.x_error = 0
        self.y_error = 0

        # Suscriptor al tópico de errores
        self.error_sub = self.create_subscription(
            Int32MultiArray,
            '/platform_error',
            self.error_callback,
            10
        )

    def error_callback(self, msg: Int32MultiArray):
        # Guardar errores en variables
        if len(msg.data) >= 2:
            self.x_error = msg.data[0]
            self.y_error = msg.data[1]

        # Imprimir errores en la terminal
        self.get_logger().info(f"x_error: {self.x_error}, y_error: {self.y_error}")

def main(args=None):
    rclpy.init(args=args)
    node = PlatformLandingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
