#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool
from geometry_msgs.msg import Twist
import time

class CrossTunnelNode(Node):
    def __init__(self):
        super().__init__('cross_tunnel')
        self.get_logger().info('cross_tunnel node started')

        # Inicializar errores
        self.x_error = 0
        self.y_error = 0

        # PID gains
        self.kp_x = 0.002
        self.ki_x = 0.0
        self.kd_x = 0.05

        self.kp_y = 0.002
        self.ki_y = 0.0
        self.kd_y = 0.05

        # PID states
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        self.ts = 0.02

        self.max_vel = 0.5

        # Publisher de velocidades
        self.cmd_pub = self.create_publisher(Twist, '/px4_driver/cmd_vel', 10)
        self.do_height_control_pub = self.create_publisher(Bool, "/px4_driver/do_height_control", 10)

        # Subscriber de errores
        self.error_sub = self.create_subscription(Int32MultiArray, '/tunnel_error', self.error_callback, 10)

        # Timer para control cada 20 ms (50 Hz)
        self.timer = self.create_timer(self.ts, self.control_loop)
        self.last_time = time.time()

    def error_callback(self, msg: Int32MultiArray):
        if len(msg.data) >= 2:
            self.x_error = msg.data[0]
            self.y_error = msg.data[1]

    def control_loop(self):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.0:
            dt = self.ts  
        self.last_time = current_time
        self.prev_error_x = self.x_error
        self.prev_error_y = self.y_error
        do_height_control_msg = Bool()
        do_height_control_msg.data = False
        self.do_height_control_pub.publish(do_height_control_msg)
        
        if(abs(self.x_error) >=30 or (abs(self.y_error) >=30)):
            z_vel = 0.0
            if(abs(self.x_error) >=45):
                # --- PID en X ---
                self.integral_x += self.x_error * dt
                derivative_x = (self.x_error - self.prev_error_x) / dt
                x_vel = (
                    self.kp_x * self.x_error +
                    self.ki_x * self.integral_x +
                    self.kd_x * derivative_x
                )
            else:
                x_vel = 0.0
            

            if(abs(self.y_error) >=45):
                # --- PID en Y ---
                self.integral_y += self.y_error * dt
                derivative_y = (self.y_error - self.prev_error_y) / dt
                y_vel = (
                    self.kp_y * self.y_error +
                    self.ki_y * self.integral_y +
                    self.kd_y * derivative_y
                )
            else:
                y_vel = 0.0
        else:
            x_vel = 0.0
            y_vel = 0.0
            z_vel = 0.3

        if(abs(x_vel) > self.max_vel):
            if(x_vel < 0):
                x_vel = -self.max_vel
            else:
                x_vel = self.max_vel
        
        if(abs(y_vel) > self.max_vel):
            if(y_vel < 0):
                y_vel = -self.max_vel
            else:
                y_vel = self.max_vel
        

        # Crear mensaje Twist
        twist = Twist()
        twist.linear.y = -float(x_vel)
        twist.linear.z = -float(y_vel)
        twist.linear.x = z_vel
        twist.angular.z = 0.0

        # Publicar velocidades
        self.cmd_pub.publish(twist)

        # Logs
        self.get_logger().info(
            f"Errors -> x: {self.x_error}, y: {self.y_error} | "
            f"Vel -> x: {x_vel:.3f}, y: {y_vel:.3f}, z: {z_vel:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CrossTunnelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
