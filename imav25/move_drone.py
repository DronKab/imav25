#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class MoveDroneNode(Node):
    def __init__(self):
        super().__init__('move_drone')
        self.get_logger().info('move_drone node started')

        self.cmd_pub = self.create_publisher(Twist, '/px4_driver/cmd_vel', 10)
        self.do_height_control_pub = self.create_publisher(Bool, "/px4_driver/do_height_control", 10)

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.declare_parameter("do_height_control", False)
        self.do_height_control = self.get_parameter("do_height_control").get_parameter_value().bool_value

        # Parámetros de escalado
        self.declare_parameter('scale_linear', 2.0)
        self.declare_parameter('scale_angular', 2.0)
        self.declare_parameter('enable_button', 4)  

    def joy_callback(self, msg: Joy):
        enable_button = self.get_parameter('enable_button').value
        twist = Twist()
        do_height_control_msg = Bool()
        do_height_control_msg.data = self.do_height_control
        self.do_height_control_pub.publish(do_height_control_msg)

        if enable_button < len(msg.buttons) and msg.buttons[enable_button] == 1:
            roll  = msg.axes[4]  # roll
            pitch  = msg.axes[3]  # pitch
            yaw = msg.axes[0]  # yaw
            z = msg.axes[1]  # z
            
            twist.linear.x = roll * self.get_parameter('scale_linear').value
            twist.linear.y = pitch * self.get_parameter('scale_linear').value
            twist.linear.z = z * self.get_parameter('scale_linear').value
            twist.angular.z = -yaw * self.get_parameter('scale_angular').value

            self.cmd_pub.publish(twist)

        else:
            twist = Twist()
            self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveDroneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
