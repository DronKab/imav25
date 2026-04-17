#!/usr/bin/env python3
import rclpy
import numpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Bool
from aruco_opencv_msgs.msg import ArucoDetection
from smach import State
import math

class ExitOk(Exception):
    pass

class NodeState(State):
    def __init__(self, x_distance=0.0, y_distance=0.0, z_distance=1.0):
        super().__init__(outcomes=['succeeded', 'aborted'])
        self.x_distance = x_distance
        self.y_distance = y_distance
        self.z_distance = z_distance

    def execute(self, userdata):
        try:
            node = ArucoControlNode(self.x_distance, self.y_distance, self.z_distance)
            rclpy.spin(node)
        except ExitOk:
            node.destroy_node()
            return 'succeeded'  # ← Eliminado rclpy.shutdown() para no matar ROS2
        except Exception as e:
            print("Error en estado ArucoControl:", e)
            return 'aborted'

class ArucoControlNode(Node):
    def __init__(self, x_distance=0.0, y_distance=0.0, z_distance=1.0):
        super().__init__('aruco_control')
        self.get_logger().info('aruco_control node started')

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.aruco_sub = self.create_subscription(ArucoDetection, "/aruco_detections", self.aruco_callback, 10)

        self.aruco_goal = 100
        self.aruco_visible = False

        self.x_distance = x_distance
        self.y_distance = y_distance
        self.z_distance = z_distance

        # PD gains
        self.px_gain = 2.0
        self.dx_gain = 0.3

        self.py_gain = 2.0
        self.dy_gain = 0.3

        self.pz_gain = 2.0
        self.dz_gain = 0.0

        self.p_pitch = 0.5
        self.d_pitch = 0.1

        self.x_error = 0.0
        self.x_error_1 = 0.0
        self.x_output = 0.0

        self.y_error = 0.0
        self.y_error_1 = 0.0
        self.y_output = 0.0

        self.z_error = 0.0
        self.z_error_1 = 0.0
        self.z_output = 0.0

        self.pitch_error = 0.0
        self.pitch_error_1 = 0.0
        self.pitch_output = 0.0

        self.q1 = Quaternion()

        self.max_vel = 0.5
        self.max_vel_z = 0.3
        self.max_vel_yaw = 3.0

        self.last_known_x = 0.0
        self.last_known_y = 0.0
        self.last_known_z = 0.0

        self.linear_limit = 0.05
        self.angular_limit = 10

        self.ts = 0.05
        self.timer = self.create_timer(self.ts, self.control)

    def aruco_callback(self, msg):
        if len(msg.markers) == 0:
            self.aruco_visible = False
            return

        marker = msg.markers[0]

        if marker.marker_id != self.aruco_goal:
            self.aruco_visible = False
            return

        self.get_logger().info(f"Aruco ID: {marker.marker_id}")

        self.q1 = marker.pose.orientation
        roll, pitch, yaw = self.euler_from_quaternion(self.q1)

        self.pitch_error = math.degrees(pitch)

        if math.isnan(marker.pose.position.x):
            self.x_error = self.last_known_x
        else:
            self.x_error = -marker.pose.position.x + self.x_distance

        if math.isnan(marker.pose.position.y):
            self.y_error = self.last_known_y
        else:
            self.y_error = -marker.pose.position.y + self.y_distance

        if math.isnan(marker.pose.position.z):
            self.z_error = self.last_known_z
        else:
            self.z_error = -marker.pose.position.z + self.z_distance

        self.last_known_x = self.x_error
        self.last_known_y = self.y_error
        self.last_known_z = self.z_error

        self.aruco_visible = True

    def control(self):
        msg = Twist()

        if not self.aruco_visible:
            # Forzar error en Z del ArUco para que el dron avance en X
            # hasta volver a encontrar el marcador
            self.x_error = 0.0
            self.y_error = 0.0
            self.z_error = self.z_distance + self.linear_limit + 0.1
            self.pitch_error = 0.0

            # Resetear error anteriores para evitar spike derivativo al recuperar señal
            self.x_error_1 = 0.0
            self.y_error_1 = 0.0
            self.z_error_1 = self.z_error
            self.pitch_error_1 = 0.0

        self.get_logger().info(
            f"Errores: x={self.x_error:.3f}, y={self.y_error:.3f}, z={self.z_error:.3f}, pitch={self.pitch_error:.3f}"
        )

        flag_error = (
            abs(self.x_error) >= self.linear_limit or
            abs(self.y_error) >= self.linear_limit or
            abs(self.z_error) >= self.linear_limit or
            abs(self.pitch_error) >= self.angular_limit
        )

        if flag_error:

            if (
                abs(self.x_error) > self.linear_limit or
                abs(self.y_error) > self.linear_limit or
                abs(self.z_error) > self.linear_limit
            ):

                if abs(self.x_error) > self.linear_limit:
                    self.x_output = (
                        self.px_gain * self.x_error +
                        self.dx_gain * (self.x_error - self.x_error_1) / (self.ts + 1e-6)
                    )
                    self.x_error_1 = self.x_error
                else:
                    self.x_output = 0.0
                    self.x_error_1 = 0.0  # ← Reset para evitar spike derivativo

                if abs(self.y_error) > self.linear_limit:
                    self.y_output = (
                        self.py_gain * self.y_error +
                        self.dy_gain * (self.y_error - self.y_error_1) / (self.ts + 1e-6)
                    )
                    self.y_error_1 = self.y_error
                else:
                    self.y_output = 0.0
                    self.y_error_1 = 0.0  # ← Reset para evitar spike derivativo

                if abs(self.z_error) > self.linear_limit:
                    self.z_output = (
                        self.pz_gain * self.z_error +
                        self.dz_gain * (self.z_error - self.z_error_1) / (self.ts + 1e-6)
                    )
                    self.z_error_1 = self.z_error
                else:
                    self.z_output = 0.0
                    self.z_error_1 = 0.0  # ← Reset para evitar spike derivativo

                self.pitch_output = 0.0

            else:
                if abs(self.pitch_error) > self.angular_limit:
                    self.pitch_output = (
                        self.p_pitch * self.pitch_error +
                        self.d_pitch * (self.pitch_error - self.pitch_error_1) / (self.ts + 1e-6)
                    )
                    self.pitch_error_1 = self.pitch_error
                else:
                    self.pitch_output = 0.0
                    self.pitch_error_1 = 0.0  # ← Reset para evitar spike derivativo

        else:
            if self.aruco_visible:
                self.get_logger().info('    Ready to draw ! ! ! !')
                raise ExitOk

        self.x_output = max(-self.max_vel, min(self.max_vel, self.x_output))
        self.y_output = max(-self.max_vel, min(self.max_vel, self.y_output))
        self.z_output = max(-self.max_vel_z, min(self.max_vel_z, self.z_output))
        self.pitch_output = max(-self.max_vel_yaw, min(self.max_vel_yaw, self.pitch_output))

        msg.linear.x = self.z_output
        msg.linear.y = -self.x_output
        msg.linear.z = self.y_output
        msg.angular.z = -self.pitch_output

        self.get_logger().info(
            f"Mensajes: x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, z={msg.linear.z:.3f}, yaw={msg.angular.z:.3f}"
        )

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
        sinp = max(-1.0, min(1.0, sinp))
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