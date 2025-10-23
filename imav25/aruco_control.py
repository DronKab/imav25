#!/usr/bin/env python3
import rclpy
import numpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Bool
from aruco_opencv_msgs.msg import ArucoDetection
import math

class ArucoControlNode(Node):
    def __init__(self):
        super().__init__('aruco_control')
        self.get_logger().info('aruco_control node started')

        self.vel_pub = self.create_publisher(Twist, '/px4_driver/cmd_vel', 10)
        self.do_height_control_pub = self.create_publisher(Bool, "/px4_driver/do_height_control", 10)

        self.aruco_sub = self.create_subscription(ArucoDetection, "/aruco_detections", self.aruco_callback, 10)

        self.declare_parameter("do_height_control", False)
        self.do_height_control = self.get_parameter("do_height_control").get_parameter_value().bool_value

        self.aruco_goal = 100 
        self.aruco_id = 100
        self.aruco_visible = False

        self.x_distance = 0.0
        self.y_distance = 0.0
        self.z_distance = 0.5

        self.px_gain = 1.0
        self.dx_gain = 0.3
        self.nx_filter = 0.3

        self.py_gain = 1.0
        self.dy_gain = 0.3
        self.ny_filter = 0.3

        self.pz_gain = 1.0
        self.dz_gain = 0.3
        self.nz_filter = 0.3

        self.p_pitch = 0.01
        self.d_pitch = 0.0
        self.n_pitch = 0.0

        self.x_error = 0.0
        self.x_error_1 = 0.0
        self.x_output = 0.0
        self.x_output_1 = 0.0

        self.y_error = 0.0
        self.y_error_1 = 0.0
        self.y_output = 0.0
        self.y_output_1 = 0.0

        self.z_error = 0.0
        self.z_error_1 = 0.0
        self.z_output = 0.0
        self.z_output_1 = 0.0

        self.pitch_error = 0.0
        self.pitch_error_1 = 0.0
        self.pitch_output = 0.0
        self.pitch_output_1 = 0.0

        self.q1 = Quaternion()
        self.yaw_error= 0.0
        self.roll_error = 0.0

        self.max_vel = 0.5
        self.max_vel_z = -0.1
        self.max_vel_yaw = 3.0

        self.last_known_x = 0.0
        self.last_known_y = 0.0
        self.last_known_z = 0.0
        self.last_known_pitch = 0.0
        
        self.ts = 0.05
        self.heartbeat_timer = self.create_timer(self.ts, self.control)


    def aruco_callback(self, msg):
        aruco_index = 0
        if len(msg.markers) > 0: 
            for i in range(0, len(msg.markers)):
                if msg.markers[i].marker_id == self.aruco_goal:
                    aruco_index = i
                    self.get_logger().info(f"Aruco ID: {msg.markers[i].marker_id}")
                    break

            if msg.markers[aruco_index].marker_id == self.aruco_goal:
                self.aruco_id = msg.markers[aruco_index].marker_id
                self.q1 = msg.markers[aruco_index].pose.orientation
                roll, pitch, yaw = self.euler_from_quaternion(self.q1)
                self.roll_error = math.degrees(roll)
                self.pitch_error = math.degrees(pitch)
                self.yaw_error= math.degrees(yaw)

                if math.isnan(msg.markers[aruco_index].pose.position.x):
                    self.x_error = self.last_known_x
                else:
                    self.x_error = -msg.markers[aruco_index].pose.position.x + self.x_distance
                
                if math.isnan(msg.markers[aruco_index].pose.position.y):
                    self.y_error = self.last_known_y
                else:
                    self.y_error = -msg.markers[aruco_index].pose.position.y + self.y_distance
                
                if math.isnan(msg.markers[aruco_index].pose.position.z):
                    self.z_error = self.last_known_z
                else:
                    self.z_error = -msg.markers[aruco_index].pose.position.z + self.z_distance

                self.last_known_x = self.x_error
                self.last_known_y = self.y_error
                self.last_known_z = self.z_error
                self.last_known_pitch = self.pitch_error 
                self.aruco_visible = True 

            else:
                self.aruco_visible = False

        else:
            self.aruco_visible = False

    def control(self):
        msg = Twist()

        if not self.aruco_visible:
            self.x_error = self.last_known_x * 0.02
            self.y_error = self.last_known_y * 0.02
            self.z_error = self.last_known_z * 0.02
            self.pitch_error = self.last_known_pitch * 0.02
        
        # self.get_logger().info(f"Errores: x={self.x_error}, y={self.y_error}, z={self.z_error}, pitch={self.pitch_error}")
        if (abs(self.x_error) > 0.1 or abs(self.y_error) > 0.1 or abs(self.z_error) > 0.1 or abs(self.pitch_error) > 5):
            if (abs(self.x_error) > 0.1 or abs(self.y_error) > 0.1 or abs(self.y_error) > 0.1):
                if (abs(self.x_error) > 0.1):
                    px_action = self.x_error * self.px_gain
                    # ix_action = self.x_output_1 + self.x_error * self.ix_gain * self.ts
                    dx_action = self.x_output_1 * (self.nx_filter * self.dx_gain * (self.x_error - self.x_error_1)) / (1 + self.nx_filter * self.ts)
                    # self.x_output = float(px_action + ix_action + dx_action)
                    self.x_output = float(px_action + dx_action)
                    self.x_error_1 = self.x_error
                    self.x_output_1 = self.x_output
                else: 
                    self.x_output = 0.0
                    self.x_error = 0.0
                    self.last_known_x = 0.0
                    self.x_error_1 = self.x_error
                    self.x_output_1 = self.x_output

                if (abs(self.y_error) > 0.1):
                    py_action = self.y_error * self.py_gain
                    # iy_action = self.y_output_1 + self.y_error * self.iy_gain * self.ts
                    dy_action = self.y_output_1 * (self.ny_filter * self.dy_gain * (self.y_error - self.y_error_1)) / (1 + self.ny_filter * self.ts)
                    # self.y_output = float(py_action + iy_action + dy_action)
                    self.y_output = float(py_action + dy_action)
                    self.y_error_1 = self.y_error
                    self.y_output_1 = self.y_output
                else:
                    self.y_output = 0.0
                    self.y_error = 0.0
                    self.last_known_y = 0.0
                    self.y_error_1 = self.y_error
                    self.y_output_1 = self.y_output

                if (abs(self.z_error) > 0.05):
                    pz_action = self.z_error * self.pz_gain
                    # iz_action = self.z_output_1 + self.z_error * self.iz_gain * self.ts
                    dz_action = self.z_output_1 * (self.nz_filter * self.dz_gain * (self.z_error - self.z_error_1)) / (1 + self.nz_filter * self.ts)
                    # self.z_output = float(pz_action + iz_action + dz_action)
                    self.z_output = float(pz_action + dz_action)
                    self.z_error_1 = self.z_error
                    self.z_output_1 = self.z_output
                else:
                    self.z_output = 0.0
                    self.z_error = 0.0
                    self.last_known_z = 0.0
                    self.z_error_1 = self.z_error
                    self.z_output_1 = self.z_output
            else: 
                self.x_output = 0.0
                self.x_error = 0.0
                self.last_known_x = 0.0
                self.x_error_1 = self.x_error
                self.x_output_1 = self.x_output

                self.y_output = 0.0
                self.y_error = 0.0
                self.last_known_y = 0.0
                self.y_error_1 = self.y_error
                self.y_output_1 = self.y_output

                self.z_output = 0.0
                self.z_error = 0.0
                self.last_known_z = 0.0
                self.z_error_1 = self.z_error
                self.z_output_1 = self.z_output

                if (abs(self.pitch_error) > 5):
                    p_pitch_action = self.pitch_error * self.p_pitch
                    # i_pitch_action = self.pitch_output_1 + self.pitch_error * self.i_pitch * self.ts
                    d_pitch_action = self.pitch_output_1 * (self.n_pitch * self.d_pitch * (self.pitch_error - self.pitch_error_1)) / (1 + self.n_pitch * self.ts)
                    # self.pitch_output = float(p_pitch_action + i_pitch_action + d_pitch_action)
                    self.pitch_output = float(p_pitch_action + d_pitch_action)
                    self.pitch_error_1 = self.pitch_error
                    self.pitch_output_1 = self.pitch_output
                else:
                    self.pitch_output = 0.0
                    self.pitch_error = 0.0
                    self.last_known_pitch = 0.0
                    self.pitch_error_1 = self.pitch_error
                    self.pitch_output_1 = self.pitch_output
                
            self.get_logger().info(f"Errores: x={self.x_error}, y={self.y_error}, z={self.z_error}, pitch={self.pitch_error}")
            self.get_logger().info(f"Mensajes: x={msg.linear.x}, y={msg.linear.y}, z={msg.linear.z}, yaw={msg.angular.z}")

        else:
            self.x_output = 0.0
            self.x_error = 0.0
            self.last_known_x = 0.0
            self.x_error_1 = self.x_error
            self.x_output_1 = self.x_output

            self.y_output = 0.0
            self.y_error = 0.0
            self.last_known_y = 0.0
            self.y_error_1 = self.y_error
            self.y_output_1 = self.y_output

            self.z_output = 0.0
            self.z_error = 0.0
            self.last_known_z = 0.0
            self.z_error_1 = self.z_error
            self.z_output_1 = self.z_output

            self.pitch_output = 0.0
            self.pitch_error = 0.0
            self.last_known_pitch = 0.0
            self.pitch_error_1 = self.pitch_error
            self.pitch_output_1 = self.pitch_output

            self.get_logger().info('Ready to draw')
                

        if abs(self.x_output) > self.max_vel:
            if self.x_output > 0:
                self.x_output = self.max_vel
            else:
                self.x_output = -self.max_vel

        if abs(self.y_output) > self.max_vel:
            if self.y_output > 0:
                self.y_output = self.max_vel
            else:
                self.y_output = -self.max_vel

        if abs(self.z_output) > self.max_vel:
            if self.z_output > 0:
                self.z_output = self.max_vel
            else:
                self.z_output = -self.max_vel
        
        if abs(self.pitch_output) > self.max_vel_yaw:
            if self.pitch_output > 0:
                self.pitch_output = self.max_vel_yaw
            else:
                self.pitch_output = -self.max_vel_yaw

        msg.linear.x = -self.z_output
        msg.linear.y = self.x_output
        msg.linear.z = self.y_output
        msg.angular.z = self.pitch_output

        # self.get_logger().info(f"Angulos: roll={self.roll_error}, pitch={self.pitch_error}, yaw={self.yaw}") 
        # self.get_logger().info(f"Salidas: x={self.x_output}, y={self.y_output}, z={self.z_output}")
        
        """
        self.get_logger().info(f"Errores: pitch={self.pitch_error}")
        self.get_logger().info(f"Mensajes: yaw={msg.angular.z}")

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = 0.0
        """
        
        self.vel_pub.publish(msg)
       
    
    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = ArucoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
