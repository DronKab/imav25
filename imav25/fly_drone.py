#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from smach import State
import math


class ExitOk(Exception):
    pass


class NodeState(State):
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        super().__init__(outcomes=['succeeded', 'aborted'])
        self.x = x
        self.y = y
        self.yaw = yaw

    def execute(self, userdata):
        try:
            node = FlyDroneNode(self.x, self.y, self.yaw)
            rclpy.spin(node)
        except ExitOk:
            node.destroy_node()
            return 'succeeded'
        except Exception as e:
            print("Error en estado FlyDrone:", e)
            return 'aborted'


class FlyDroneNode(Node):
    def __init__(self, x_tar=0.0, y_tar=0.0, yaw_tar=0.0):
        super().__init__('fly_drone')

        # frame del vehiculo
        self.x_tar = x_tar
        self.y_tar = y_tar
        self.yaw_tar = math.radians(yaw_tar)

        self.get_logger().info(f"Target: x={x_tar}, y={y_tar}, yaw={yaw_tar}")

        self.cmd_pub = self.create_publisher(Twist, '/px4_driver/cmd_vel', 10)

        self.ts = 0.02  # 50 Hz

        self.v = 0.30 # magnitud de velocidad lineal constante (m/s)

        self.w = 0.35 # velocidad angular constante (rad/s)

        # WARMUP 
        # Tiempo inicial en el qeu see envian comandos pero NO se cuenta
        # en el calculo del desplazamiento
        # Idea fea para compensar retardos del sistema (comunicación, control, etc.)
        self.warmup_time = 1.5   # segundos
        self.warmup_elapsed = 0.0

        self.time_elapsed = 0.0 # tiempo acumulado "activo"
        self.phase = "move" # move/rotate

        dx = self.x_tar
        dy = self.y_tar

        # distancia total a recorrer
        self.dist_total = math.sqrt(dx**2 + dy**2)

        if self.dist_total < 0.001:
            self.phase = "rotate"
        else:
            # dirección normalizada y escalada por la velocidad deseada
            self.vx = self.v * dx / self.dist_total
            self.vy = self.v * dy / self.dist_total

            # tiempo necesario para recorrer la distancia:
            # factor 1.05 (5% extra) para compensar retrasos en la respuesta del dron 
            # se aumenta si el dron no llega al target y disminuye si se pasa 
            self.time_total_move = (self.dist_total / self.v) * 1.05

        self.yaw_total = self.yaw_tar

        if self.yaw_total >= 0:
            self.w_sign = 1.0 
        else:
            self.w_sign = -1.0 

        # tiempo necesario para alcanzar el aangulo:
        self.time_total_yaw = abs(self.yaw_total) / self.w

        # factor de correccion (lo mismo que en el de traslacion)
        self.time_total_yaw *= 1.1

        self.get_logger().info(f"Move time: {getattr(self, 'time_total_move', 0):.2f}")
        self.get_logger().info(f"Yaw time: {self.time_total_yaw:.2f}")

        self.timer = self.create_timer(self.ts, self.move_loop)

    def move_loop(self):
        twist = Twist()

        # traslacionn
        if self.phase == "move":

            # Periodo de warmup (no se cuenta en el tiempo efectivo)
            if self.warmup_elapsed < self.warmup_time:
                self.warmup_elapsed += self.ts

                twist.linear.x = self.vx
                twist.linear.y = self.vy
                self.cmd_pub.publish(twist)
                return

            # Tiempo efectivo de movimiento
            self.time_elapsed += self.ts

            if self.time_elapsed >= self.time_total_move:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                self.cmd_pub.publish(twist)

                self.phase = "rotate"
                self.time_elapsed = 0.0
                self.warmup_elapsed = 0.0  # reiniciar warmup para rotate

                self.get_logger().info("Position reached, starting yaw rotation")
                return

            twist.linear.x = self.vx
            twist.linear.y = self.vy
            self.cmd_pub.publish(twist)

        elif self.phase == "rotate":
            if self.warmup_elapsed < self.warmup_time:
                self.warmup_elapsed += self.ts

                twist.angular.z = self.w * self.w_sign
                self.cmd_pub.publish(twist)
                return

            self.time_elapsed += self.ts

            if self.time_elapsed >= self.time_total_yaw:
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)

                self.get_logger().info("Yaw reached")

                raise ExitOk

            twist.angular.z = self.w * self.w_sign
            self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = FlyDroneNode(2.0, 2.0, 90.0)

    try:
        rclpy.spin(node)
    except ExitOk:
        pass
    except Exception as e:
        print(e)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()