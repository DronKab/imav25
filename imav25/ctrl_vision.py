#!/usr/bin/env python3
import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool
from geometry_msgs.msg import Twist
from smach import State
import time

class ExitOk(Exception):
    """Excepción personalizada para salir del spin de ROS2 de forma controlada."""
    pass

class CtrlVisNodeState(State):
    def __init__(self, target_class="Azul", action_flag=True, pos_flag=True):
        # action_flag: True = Centrarse, False = Evitar
        # pos_flag: True = Camara al frente, False = Camara hacia abajo
        State.__init__(self, outcomes=["succeeded", "aborted"])
        self.target_class = target_class
        self.action_flag = action_flag
        self.pos_flag = pos_flag

    def execute(self, userdata):
        self.node = VisualDroneControlNode(
            target_class=self.target_class, 
            action_flag=self.action_flag,
            pos_flag=self.pos_flag
        )
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self.node)
        try:
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.05)
        except ExitOk:
            self.node.get_logger().info("Estado completado con éxito.")
            return "succeeded"
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            return "aborted"
        except Exception as e:
            self.node.get_logger().error(f"Error en el estado: {e}")
            return "aborted"
        finally:
            executor.remove_node(self.node)
            self.node.destroy_node()


class VisualDroneControlNode(Node):
    def __init__(self, target_class, action_flag, pos_flag):
        super().__init__('ctrl_vision')

        self.target_class = target_class
        self.action_flag = action_flag
        self.pos_flag = pos_flag
        self.exit_counter = 0

        # Suscribirse al tópico de error ya calculado por classes_publishers
        topic_map = {
            "Azul":   "/tunnelA_error",
            "Postes": "/obstacle_error",
            "Plataforma": "/plat_error",
            "Verde": "/tunnelV_error"
        }
        sub_topic = topic_map.get(self.target_class, "/tunnel_error")

        self.error_sub   = self.create_subscription(
            Int32MultiArray, sub_topic, self.error_callback, 10)
        self.cmd_pub         = self.create_publisher(Twist, '/cmd_vel', 10)
        self.height_ctrl_pub = self.create_publisher(Bool, "/px4_driver/do_height_control", 10)

        # Variables de control
        self.current_x_error = 0
        self.current_y_error = 0
        self.object_detected = False  # True mientras lleguen mensajes de error

        # PID
        self.kp_x, self.kd_x = 0.002, 0.05
        self.kp_y, self.kd_y = 0.002, 0.05
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.max_vel = 0.5

        # Timer de control (50 Hz)
        self.ts = 0.02
        self.timer = self.create_timer(self.ts, self.control_loop)
        self.last_time = time.time()

        # Timeout para detectar pérdida de objeto
        self.last_detection_time = time.time()
        self.detection_timeout = 0.5  # segundos sin mensaje → objeto perdido

    def error_callback(self, msg):
        """
        Recibe [x_error, y_error] ya calculados por classes_publishers.
        Solo aplica la lógica de acción (centrar vs evitar).
        """
        raw_x_error = msg.data[0]
        raw_y_error = msg.data[1]

        if self.action_flag:
            # Modo CENTRAR: usar el error directo
            self.current_x_error = raw_x_error
            self.current_y_error = raw_y_error

        else:
            # Modo EVITAR: invertir el error para alejarse
            self.current_x_error = -raw_x_error if abs(raw_x_error) < 150 else 0
            self.current_y_error = -raw_y_error if abs(raw_y_error) < 150 else 0

        self.object_detected = True
        self.last_detection_time = time.time()

    def control_loop(self):
        dt = time.time() - self.last_time
        self.last_time = time.time()

        # Si no llegan mensajes por más del timeout, objeto perdido
        if time.time() - self.last_detection_time > self.detection_timeout:
            self.object_detected = False
            self.current_x_error = self.prev_error_x * 0.5
            self.current_y_error = self.prev_error_y * 0.5

        # Siempre delegamos altura al nodo externo
        h_msg = Bool()
        h_msg.data = True
        self.height_ctrl_pub.publish(h_msg)

        x_vel, y_vel = 0.0, 0.0
        threshold = 30 if self.action_flag else 100

        if abs(self.current_x_error) > threshold or abs(self.current_y_error) > threshold:
            deriv_x = (self.current_x_error - self.prev_error_x) / dt
            x_vel = (self.kp_x * self.current_x_error) + (self.kd_x * deriv_x)

            deriv_y = (self.current_y_error - self.prev_error_y) / dt
            y_vel = (self.kp_y * self.current_y_error) + (self.kd_y * deriv_y)
    
        else:
            # Centrado conseguido → avanzar si modo centrar y objeto visible
            if self.action_flag and self.object_detected:
                self.exit_counter += 1

        x_vel = max(min(x_vel, self.max_vel), -self.max_vel)
        y_vel = max(min(y_vel, self.max_vel), -self.max_vel)

        self.get_logger().info(
            f"Errores: X={self.current_x_error} Y={self.current_y_error} | "
            f"Vels: x={x_vel:.3f} y={y_vel:.3f} | "
            f"Objeto: {self.object_detected}"
        )

        twist = Twist()
        if self.pos_flag == True:
            twist.linear.y = -float(x_vel)
            if self.action_flag == True:
                twist.linear.x = 0.0
            else:
                twist.linear.x = 0.2
        else:
            twist.linear.y = -float(x_vel)
            twist.linear.x = float(y_vel)
        
        twist.linear.z = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        self.prev_error_x = self.current_x_error
        self.prev_error_y = self.current_y_error

        # Condición de salida para SMACH (ejemplo: 50 iteraciones centrado)
        if self.exit_counter > 75:
            raise ExitOk


def main(args=None):
    rclpy.init(args=args)
    node = VisualDroneControlNode(target_class="tunnel", action_flag=True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()