#!/usr/bin/env python3
"""
Node name: classes_publishers

Subscriber(s): /oak/nn/detections (Message Type: Detection2DArray)

Publisher(s): /tunnel_error (Message Type: Int32MultiArray)
              /obstacle_error (Message Type: Int32MultiArray)
              /wb_error (Message Type: Int32MultiArray)
              /far_plat_error (Message Type: Int32MultiArray)
              /top_plat_error (Message Type: Int32MultiArray)

Constant(s): -center_x, center_y (Float): Value in pixels of the half of height and width, it is used to 
                calculate the error to the center of the image.
             -LABEL_MAP (String Array): Array with the classes names in training order, it doesn't matter
                if it matches or not the original names (in the json file), but it is highly recommended.

Description: This node must be launched in the beginning. This node takes the information into the detections
             in the topic /oak/nn/detections, and publish in different topics (error_classes) by the class
             detected.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from vision_msgs.msg import Detection2DArray  

# ← Replace with your actual class names in the same order as LABEL_MAP
LABEL_MAP = ["Azul", "Cajas", "Plataforma", "Postes", "Verde"] 

class ClassesPublishersNode(Node):
    def __init__(self):
        super().__init__('classes_publishers')
        self.get_logger().info('Iniciando publicacion de clases en topicos...')

        
        self.camera_sub = self.create_subscription(
            Detection2DArray,
            '/oak/nn/detections',
            self.detection_callback,
            10
        )
        self.error_tunnelA = self.create_publisher(Int32MultiArray, '/tunnelA_error', 10)
        self.error_tunnelV = self.create_publisher(Int32MultiArray, '/tunnelV_error', 10)
        self.error_obstacle = self.create_publisher(Int32MultiArray, '/obstacle_error', 10)
        self.error_platform = self.create_publisher(Int32MultiArray, '/plat_error', 10)

        self.center_x = 320.0  # IMG_W / 2
        self.center_y = 320.0  # IMG_H / 2

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

                #if class_id !=  and confidence > 0.45:
                if confidence > 0.45:
                    # bbox center is already in pixels (640×640 space)
                    bbox_x = detection.bbox.center.position.x
                    bbox_y = detection.bbox.center.position.y

                    x_error = int(bbox_x - self.center_x)
                    y_error = int(bbox_y - self.center_y)

                    error_msg = Int32MultiArray()
                    error_msg.data = [x_error, y_error]

                    if class_id == "Azul":
                        self.error_tunnelA.publish(error_msg)
                    elif class_id == "Postes":
                        self.error_obstacle.publish(error_msg)
                    elif class_id == "Plataforma":
                        self.error_platform.publish(error_msg)
                    elif class_id == "Verde":
                        self.error_tunnelV.publish(error_msg)

                    self.get_logger().info(f"Error publicado: {error_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ClassesPublishersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()