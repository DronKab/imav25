#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from smach import State 

class ExitOk(Exception):
    pass

class NodeState(State):
    def __init__(self):
        super().__init__(outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        try:
            node = FlyDroneNode()
            rclpy.spin(node)
        except ExitOk:
            # detener nodo y salir bien
            node.destroy_node()
            # rclpy.shutdown()
            return 'succeeded'
        except Exception as e:
            print("Error en estado Fly drone:", e)
            return 'aborted'

class FlyDroneNode(Node):
    def __init__(self):
        super().__init__('fly_drone')
        self.get_logger().info('fly_drone node started')

        self.cmd_pub = self.create_publisher(Twist, '/px4_driver/cmd_vel', 10)
        self.do_height_control_pub = self.create_publisher(Bool, "/px4_driver/do_height_control", 10)


        self.declare_parameter("do_height_control", False)
        self.do_height_control = self.get_parameter("do_height_control").get_parameter_value().bool_value

        self.ts = 0.1
        self.count = 0
        self.count_goal = 100

        self.x_vel = 0.0
        self.y_vel = 0.0
        self.z_vel = 0.0
        self.yaw_vel = 0.0

        self.timer = self.create_timer(self.ts, self.move_loop)

    def move_loop(self):
        twist = Twist()
        
        if self.count < self.count_goal:
            twist.linear.y = self.x_vel
            twist.linear.z = self.y_vel
            twist.linear.x = self.z_vel
            twist.angular.z = self.yaw_vel
        
        else: 
            # salida 
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            raise ExitOk

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FlyDroneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
