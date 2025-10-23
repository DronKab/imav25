#!/usr/bin/env python3
import rclpy
import time
import csv
import os
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLocalPosition


class SavePositionNode(Node):
    def __init__(self):
        super().__init__('save_pos')
        self.get_logger().info('Nodo save_pos iniciado')

        # QoS profile configuration
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Archivo CSV donde se guardarán los datos
        self.csv_filename = 'drone_positions.csv'

        # Si el archivo no existe, crear encabezado
        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Tiempo', 'x', 'y', 'z'])

        # Intervalo de guardado (segundos)
        self.last_saved_time = 0.0
        self.save_interval = 0.5
        self.start_time = time.time()

        # Suscriptor al tópico de posición local del dron
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_saved_time >= self.save_interval:
            elapsed_time = current_time
            pose_x = msg.x
            pose_y = msg.y
            pose_z = msg.z

            # Guardar los datos en el archivo CSV
            with open(self.csv_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([round(elapsed_time, 2), pose_x, pose_y, pose_z])

            self.get_logger().info(f"Guardado: t={elapsed_time:.2f}s, x={pose_x:.2f}, y={pose_y:.2f}, z={pose_z:.2f}")
            self.last_saved_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = SavePositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido por el usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
