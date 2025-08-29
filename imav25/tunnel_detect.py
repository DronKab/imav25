#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import onnxruntime as ort

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge


class TunnelDetectNode(Node):
    def __init__(self):
        super().__init__('tunnel_detect')
        self.get_logger().info('tunnel_detect node started')
        self.bridge = CvBridge()

        # Subscriber
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Publishers
        self.tunnel_pub = self.create_publisher(Image, '/tunnel_debug', 10)
        self.error_pub = self.create_publisher(Int32MultiArray, '/tunnel_error', 10)

        # Cargar modelo ONNX
        self.model_path = 'best_tunnel_export.onnx'
        self.session = ort.InferenceSession(self.model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(f'Model loaded. Input name: {self.input_name}')

        # Clases
        self.classes = ["wide_tunnel", "medium_tunnel", "small_tunnel"]

        # Definir túnel objetivo
        self.goal_tunnel = "wide_tunnel"   # Cambia según necesidad

    def preprocess(self, frame):
        # Redimensionar a 640x640 (como espera el modelo)
        img_resized = cv2.resize(frame, (640, 640))
        img_chw = img_resized[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        input_tensor = np.expand_dims(img_chw, axis=0)
        return input_tensor

    def draw_detections_and_compute_error(self, frame, output, conf_threshold=0.5):
        preds = output[0]  # [1, N, 6]
        preds = preds.reshape(-1, preds.shape[-1])
        best_detections = {}

        h, w = frame.shape[:2]
        scale_x = w / 640.0
        scale_y = h / 640.0

        for det in preds:
            det = det[:6].flatten()
            x1, y1, x2, y2, conf, class_id = det
            if conf < conf_threshold:
                continue
            class_id = int(class_id.item())
            if class_id not in best_detections or conf > best_detections[class_id][4]:
                best_detections[class_id] = det

        # Inicializar error en 0
        x_error, y_error = 0, 0

        for class_id, det in best_detections.items():
            x1, y1, x2, y2, conf, _ = det
            # Reescalar a tamaño original
            x1 = int(x1 * scale_x)
            x2 = int(x2 * scale_x)
            y1 = int(y1 * scale_y)
            y2 = int(y2 * scale_y)

            # Dibujar cajas
            color = (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"{self.classes[class_id]} {conf:.2f}"
            cv2.putText(frame, label, (x1, max(y1 - 5, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Si es el túnel objetivo, calcular error
            if self.classes[class_id] == self.goal_tunnel:
                bbox_center_x = (x1 + x2) // 2
                bbox_center_y = (y1 + y2) // 2
                cam_center_x = w // 2
                cam_center_y = h // 2
                x_error = bbox_center_x - cam_center_x
                y_error = bbox_center_y - cam_center_y
                self.get_logger().info(
                    f"Goal tunnel detected. x_error={x_error}, y_error={y_error}"
                )

        return frame, x_error, y_error

    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Preprocesar
        input_tensor = self.preprocess(frame)

        # Inferencia
        outputs = self.session.run(None, {self.input_name: input_tensor})

        # Dibujar y calcular error
        frame_debug, x_error, y_error = self.draw_detections_and_compute_error(
            frame, outputs, conf_threshold=0.5
        )

        # Publicar error en /tunnel_error
        error_msg = Int32MultiArray()
        error_msg.data = [x_error, y_error]
        self.error_pub.publish(error_msg)

        # Publicar imagen con detecciones
        debug_msg = self.bridge.cv2_to_imgmsg(frame_debug, "bgr8")
        self.tunnel_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TunnelDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
