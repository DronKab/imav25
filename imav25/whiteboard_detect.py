#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import onnxruntime as ort

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge

class WhiteboardDetectNode(Node):
    def __init__(self):
        super().__init__('whiteboard_detect')
        self.get_logger().info('whiteboard_detect node started')
        self.bridge = CvBridge()

        # Subscriber
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Publisher de imagen con detecciones
        self.whiteboard_pub = self.create_publisher(Image, '/whiteboard_debug', 10)

        # Publisher de errores X y Y
        self.error_pub = self.create_publisher(Int32MultiArray, '/whiteboard_error', 10)

        # Cargar modelo ONNX
        self.model_path = 'best_wb_sim.onnx'
        self.session = ort.InferenceSession(self.model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(f'Model loaded. Input name: {self.input_name}')

        # Clase
        self.classes = ["whiteboard"]

    def preprocess(self, frame):
        # Redimensionar a 640x640
        img_resized = cv2.resize(frame, (640, 640))
        # Convertir BGR a RGB y HWC a CHW
        img_chw = img_resized[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        # Añadir batch dimension
        input_tensor = np.expand_dims(img_chw, axis=0)
        return input_tensor

    def draw_detections(self, frame, output, conf_threshold=0.5):
        preds = output[0].reshape(-1, output[0].shape[-1])
        best_detection = None
        h_orig, w_orig = frame.shape[:2]
        scale_x = w_orig / 640
        scale_y = h_orig / 640

        for det in preds:
            x1, y1, x2, y2, conf, class_id = det[:6].flatten()
            if conf < conf_threshold:
                continue
            if best_detection is None or conf > best_detection[4]:
                best_detection = det

        error_x = 0
        error_y = 0

        if best_detection is not None:
            x1, y1, x2, y2, conf, _ = best_detection[:6]

            # Escalar a tamaño original
            x1 = int(x1 * scale_x)
            x2 = int(x2 * scale_x)
            y1 = int(y1 * scale_y)
            y2 = int(y2 * scale_y)

            # Dibujar bounding box
            color = (0, 0, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"{self.classes[0]} {conf:.2f}"
            cv2.putText(frame, label, (x1, max(y1-5, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Centro y errores
            bbox_cx = (x1 + x2) // 2
            bbox_cy = (y1 + y2) // 2
            img_cx = w_orig // 2
            img_cy = h_orig // 2
            error_x = int(bbox_cx - img_cx)
            error_y = int(bbox_cy - img_cy)

            print(f"[INFO] Error X: {error_x}, Error Y: {error_y}")

        # Publicar errores
        error_msg = Int32MultiArray()
        error_msg.data = [error_x, error_y]
        self.error_pub.publish(error_msg)

        return frame

    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        input_tensor = self.preprocess(frame)
        outputs = self.session.run(None, {self.input_name: input_tensor})
        frame_debug = self.draw_detections(frame, outputs, conf_threshold=0.5)
        debug_msg = self.bridge.cv2_to_imgmsg(frame_debug, "bgr8")
        self.whiteboard_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WhiteboardDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
