#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from smach import State
import time

class ExitOk(Exception):
    """Excepción personalizada para salir del spin de ROS2 de forma controlada."""
    pass

class CtrlVisNodeState(State):
    def __init__(self, target_class="class2", action_flag=True, pos_flag=True):
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
        try:
            rclpy.spin(self.node)
        except ExitOk:
            self.node.get_logger().info("Estado completado con éxito.")
            return "succeeded"
        except Exception as e:
            self.node.get_logger().error(f"Error en el estado: {e}")
            return "aborted"
        finally:
            self.node.destroy_node()

class VisualDroneControlNode(Node):
    def __init__(self, target_class, action_flag, pos_flag):
        super().__init__('ctrl_vision')
        
        # Parámetros de misión
        self.target_class = target_class
        self.action_flag = action_flag
        self.pos_flag = pos_flag
        self.exit_counter = 0
        
        # Configuración de tópicos según target_class
        topic_map = {
            "tunnel": "/tunnel_error",
            "obstacle": "/obstacle_error",
            "wb": "/wb_error",
            "far_plat": "/far_plat_error",
            "top_plat": "/top_plat_error"
        }
        # Seleccionamos el tópico, por defecto usamos tunnel si no coincide
        pub_topic = topic_map.get(self.target_class, "/tunnel_error")
        
        # Publicadores y Suscriptores
        self.error_pub = self.create_publisher(Int32MultiArray, pub_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, '/px4_driver/cmd_vel', 10)
        self.height_ctrl_pub = self.create_publisher(Bool, "/px4_driver/do_height_control", 10)
        
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/oak/nn/detections', self.detection_callback, 10)

        # Variables de Control
        self.center_x = 320.0
        self.center_y = 320.0
        self.current_x_error = 0
        self.current_y_error = 0
        self.object_detected = False
        
        # PID Constants
        self.kp_x, self.kd_x = 0.002, 0.05
        self.kp_y, self.kd_y = 0.002, 0.05
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.max_vel = 0.5
        
        # Timer de Control (50Hz)
        self.ts = 0.02
        self.timer = self.create_timer(self.ts, self.control_loop)
        self.last_time = time.time()

    def detection_callback(self, msg):
        found = False
        for det in msg.detections:
            for result in det.results:
                if result.hypothesis.class_id == self.target_class and result.hypothesis.score > 0.45:
                    # Calcular error en pixeles
                    raw_x_error = int(det.bbox.center.position.x - self.center_x)
                    raw_y_error = int(det.bbox.center.position.y - self.center_y)
                    
                    # LÓGICA DE ACCIÓN
                    if self.action_flag:
                        # Modo CENTRAR: Error directo
                        self.current_x_error = raw_x_error
                        self.current_y_error = raw_y_error
                    else:
                        # Modo EVITAR: Invertimos el error para que el PID empuje al lado contrario
                        # Si el objeto está a la derecha (+), generamos error positivo para que el drone
                        # (que invierte en el Twist) se mueva a la izquierda.
                        self.current_x_error = -raw_x_error if abs(raw_x_error) < 150 else 0
                        self.current_y_error = -raw_y_error if abs(raw_y_error) < 150 else 0
                    
                    # Publicar el error en el tópico seleccionado
                    err_msg = Int32MultiArray()
                    err_msg.data = [self.current_x_error, self.current_y_error]
                    self.error_pub.publish(err_msg)
                    
                    found = True
                    break
        
        self.object_detected = found
        if not found:
            self.current_x_error = 0
            self.current_y_error = 0

    def control_loop(self):
        dt = time.time() - self.last_time
        self.last_time = time.time()
        
        # Desactivar control de altura interno de PX4 driver si es necesario
        h_msg = Bool()
        h_msg.data = True
        self.height_ctrl_pub.publish(h_msg)

        x_vel, y_vel = 0.0, 0.0
        
        # Umbral de zona muerta (Deadband)
        threshold = 30 if self.action_flag else 100 # Más margen para evitar

        if abs(self.current_x_error) > threshold or abs(self.current_y_error) > threshold:
            # PID X (Lateral en el frame de la cámara, suele ser Y en el drone)
            deriv_x = (self.current_x_error - self.prev_error_x) / dt
            x_vel = (self.kp_x * self.current_x_error) + (self.kd_x * deriv_x)
            
            # PID Y (Vertical en cámara, suele ser Z o X en drone)
            deriv_y = (self.current_y_error - self.prev_error_y) / dt
            y_vel = (self.kp_y * self.current_y_error) + (self.kd_y * deriv_y)
    
        else:
            # Si está centrado (o lejos del objeto a evitar) y es modo centrar, avanzamos
            if self.action_flag and self.object_detected:
                self.exit_counter += 1
            
        # Saturación
        x_vel = max(min(x_vel, self.max_vel), -self.max_vel)
        y_vel = max(min(y_vel, self.max_vel), -self.max_vel)

        # Mapeo a Twist (PX4 standard: x forward, y left, z up)
        # Nota: Ajustado según tu código original
        twist = Twist()
        if self.pos_flag == True:
            twist.linear.y = -float(x_vel)
            twist.linear.x = 0.0
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
    node = VisualDroneControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
