#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai

class OakCameraNode(Node):
    def __init__(self):
        super().__init__('oak_camera_node')
        self.img_pub = self.create_publisher(Image, '/oak/rgb/image_raw', 10)
        self.bridge = CvBridge()

        # Configuración del Pipeline nativo de Luxonis
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(1280, 720) # Resolución óptima para ArUco
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(30)

        # Usamos la API estable para el stream de video
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("video")
        cam_rgb.preview.link(xout_rgb.input)

        try:
            self.device = dai.Device(pipeline)
            self.q_rgb = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)
            self.get_logger().info("OAK-D Conectada exitosamente.")
        except Exception as e:
            self.get_logger().error(f"Error al conectar OAK-D: {e}")
            raise e

        self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        in_rgb = self.q_rgb.tryGet()
        if in_rgb is not None:
            frame = in_rgb.getCvFrame()
            # Convertir a mensaje de ROS2
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "oak_rgb_camera_optical_frame"
            self.img_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OakCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.device.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()