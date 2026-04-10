#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from vision_msgs.msg import Detection2DArray  # ✅ matches vision_yolo_node output

# ← Replace with your actual class names in the same order as LABEL_MAP
LABEL_MAP = ["class0", "class1", "class2", "class3"]

class TunnelDetectNode(Node):
    def __init__(self):
        super().__init__('tunnel_detect')
        self.get_logger().info('Iniciando detección de túnel...')

        # ✅ Default QoS (Reliable) — matches Detection2DArray publisher
        self.camera_sub = self.create_subscription(
            Detection2DArray,
            '/oak/nn/detections',
            self.detection_callback,
            10
        )
        self.error_pub = self.create_publisher(Int32MultiArray, '/tunnel_error', 10)

        # ✅ 640×640 to match IMG_W/H in vision_yolo_node
        self.center_x = 320.0  # IMG_W / 2
        self.center_y = 320.0  # IMG_H / 2

        # Target class — set to whichever class name you want to track
        self.target_class = "class2"  # ← replace with your real class name

    def detection_callback(self, msg):
        if not msg.detections:
            return

        for detection in msg.detections:
            for result in detection.results:
                class_id   = result.hypothesis.class_id
                confidence = result.hypothesis.score

                self.get_logger().info(
                    f"Detected: {class_id} ({confidence:.2f}) "
                    f"at ({detection.bbox.center.position.x:.1f}, "
                    f"{detection.bbox.center.position.y:.1f})"
                )

                if class_id == self.target_class and confidence > 0.45:
                    # bbox center is already in pixels (640×640 space)
                    bbox_x = detection.bbox.center.position.x
                    bbox_y = detection.bbox.center.position.y

                    x_error = int(bbox_x - self.center_x)
                    y_error = int(bbox_y - self.center_y)

                    error_msg = Int32MultiArray()
                    error_msg.data = [x_error, y_error]
                    self.error_pub.publish(error_msg)

                    self.get_logger().info(f"Error publicado: {error_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = TunnelDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()