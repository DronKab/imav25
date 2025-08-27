#!/usr/bin/env python3
import rclpy
import time
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class TakePhotosNode(Node):
    def __init__(self):
        super().__init__('Photos')
        self.get_logger().info('take_photos node started')

        self.bridge = CvBridge()
        self.last_saved_time = 0.0   # último instante en que guardaste
        self.save_interval = 0.5     # segundos entre capturas

        # Suscripción a la cámara
        # self.subscription = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.listener_callback, 10) #fisico webcam
        # self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10) # simulado webcam 

        # self.image_sub = self.create_subscription(CompressedImage, '/pi_camera/image_raw/compressed', self.listener_callback, 10) # fisico picamera 
        self.image_sub = self.create_subscription(Image, '/pi_camera/image_raw', self.listener_callback, 10) # simulado picamera


    def listener_callback(self, data):
        current_time = time.time()
        if current_time - self.last_saved_time >= self.save_interval:
            # current_frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8") fisico 
            current_frame = self.bridge.imgmsg_to_cv2(data, "bgr8") # simulado
            timestamp = int(current_time)
            filename = f"image_{timestamp}.png"
            cv2.imwrite(filename, current_frame)
            self.get_logger().info(f"Imagen guardada como {filename}")
            self.last_saved_time = current_time  # actualizar el último guardado

def main(args=None):
    rclpy.init(args=args)
    node = TakePhotosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
