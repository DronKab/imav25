#!/usr/bin/env python3
import rclpy
import time
import cv2
import csv
import os
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleLocalPosition
from cv_bridge import CvBridge


class SavePosImageNode(Node):
    def __init__(self):
        super().__init__('save_pos_image')
        self.get_logger().info('Nodo combinado iniciado (posición + imagen)')

        # Configuración del QoS profile para la posición
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Configuración general
        self.bridge = CvBridge()
        self.last_saved_time = 0.0
        self.save_interval = 0.005  # segundos
        self.csv_filename = 'drone_data.csv'

        # Crear carpeta de imágenes si no existe
        self.image_folder = 'images'
        os.makedirs(self.image_folder, exist_ok=True)

        # Crear el CSV con encabezado si no existe
        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Tiempo', 'x', 'y', 'z', 'imagen'])

        # Variables para almacenar el último mensaje recibido
        self.latest_image = None
        self.latest_pose = None

        # Suscripciones
        self.position_sub = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.position_callback, qos_profile)

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def position_callback(self, msg):
        """Guardar la última posición recibida."""
        self.latest_pose = msg

    def image_callback(self, msg):
        """Recibir la imagen y guardar datos sincronizados cada intervalo."""
        current_time = time.time()
        if self.latest_pose is None:
            return  # aún no hay datos de posición
        if current_time - self.last_saved_time < self.save_interval:
            return  # esperar al siguiente intervalo

        elapsed_time = int(current_time)
        timestamp_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(current_time))
        pose_x = self.latest_pose.x
        pose_y = self.latest_pose.y
        pose_z = self.latest_pose.z

        # Guardar imagen
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        image_filename = f"image_{elapsed_time}.png"
        image_path = os.path.join(self.image_folder, image_filename)
        cv2.imwrite(image_path, frame)

        # Guardar datos en CSV
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp_str, pose_x, pose_y, pose_z, image_filename])

        self.last_saved_time = current_time
        self.get_logger().info(f"Guardado: {image_filename} | x={pose_x:.2f}, y={pose_y:.2f}, z={pose_z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SavePosImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido manualmente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
