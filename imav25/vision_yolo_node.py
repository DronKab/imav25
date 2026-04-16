#!/usr/bin/env python3
"""
Node name: vision_yolo_node

Publisher(s): /oak/rgb (Message Type: Image)
              /oak/camera_info (Message Type: CameraInfo)
              /oak/nn/detections (Message Type: Detection2DArray)

Constant(s): -IMG_W, IMG_H (Integer): Dimensions of the image (input to the ANN), height and width
                in pixels. Usually they have the same value, they are used to initialize the pipeline
                in the OAK-D camera and to get the coordinates of the bounding boxes in pixels.
             -CONF_THRESH (Float): Value between 0 and 1, it's the normalized value of the percentage
                of the confidence threshold, the minimum value to consider the hypothesis correct.
             -IOU_THRESH (Float):  Value between 0 and 1, it's the normalized value of the percentage
                of the Intersection over Union threshold, this value helps to erase multiple detections
                of the same object (superposition).
             -NUM_CLASSES (Integer): Number of classes in the ANN, it has to match with the values
                in the json file in the resources/tmr_model folder.
             -LABEL_MAP (String Array): Array with the classes names in training order.
             -HEADS (Struct: String, Integer): layer_name, grid_size. Used to access output tensors.
             -CAM_FX, CAM_FY, CAM_CX, CAM_CY (Float): Camera intrinsics scaled from 1920x1080
                to 640x640 using SCALE_X and SCALE_Y.
             -DIST_COEFFS (Float Array): Distortion coefficients from OAK-D EEPROM,
                rational_polynomial model (k1 k2 p1 p2 k3 k4 k5 k6).

Description: This node initializes the OAK-D camera pipeline, runs YOLO inference,
             and publishes the raw image, camera calibration info, and detections.
             The camera_info is published with the exact same timestamp as the image
             so that aruco_opencv can synchronize them correctly.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import depthai as dai
import numpy as np
import sys
import os
from ament_index_python.packages import get_package_share_directory

IMG_W, IMG_H = 640, 640
CONF_THRESH  = 0.45
IOU_THRESH   = 0.6
NUM_CLASSES  = 4

LABEL_MAP = ["class0", "class1", "class2", "class3"]

HEADS = [
    ("output1_yolov6r2", 80),
    ("output2_yolov6r2", 40),
    ("output3_yolov6r2", 20),
]

# Parámetros de calibración escalados de 1920x1080 → 640x640
SCALE_X = 640 / 1920
SCALE_Y = 640 / 1080
CAM_FX  = 1028.1087646484375 * SCALE_X
CAM_FY  = 1027.48291015625   * SCALE_Y
CAM_CX  = 640.4381103515625  * SCALE_X
CAM_CY  = 363.9605407714844  * SCALE_Y
DIST_COEFFS = [
    1.5502548217773438, -42.73958206176758,
    5.5246982810785994e-05, -0.00035731212119571865,
    199.03936767578125,
    1.3921493291854858, -41.60566711425781, 194.88555908203125
]


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -500, 500)))


def nms(detections, iou_thresh):
    detections.sort(key=lambda d: -d["confidence"])
    kept = []
    for det in detections:
        suppress = False
        for k in kept:
            if k["label"] != det["label"]:
                continue
            ix1 = max(det["xmin"], k["xmin"]); iy1 = max(det["ymin"], k["ymin"])
            ix2 = min(det["xmax"], k["xmax"]); iy2 = min(det["ymax"], k["ymax"])
            inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
            union = ((det["xmax"] - det["xmin"]) * (det["ymax"] - det["ymin"]) +
                     (k["xmax"]  - k["xmin"])  * (k["ymax"]  - k["ymin"]) - inter)
            if union > 0 and inter / union > iou_thresh:
                suppress = True
                break
        if not suppress:
            kept.append(det)
    return kept


class VisionHostNode(dai.node.HostNode):
    """
    HostNode de DepthAI v3.
    Recibe frames RGB y tensores de la red neuronal directamente
    desde el pipeline y los publica como mensajes ROS2.
    """

    def build(self, rgb_output, nn_output, ros_node):
        self.ros_node      = ros_node
        self._range_logged = False
        self.link_args(rgb_output, nn_output)
        return self

    def process(self, rgb_msg, nn_msg):
        # Timestamp único compartido por imagen, camera_info y detecciones
        stamp    = self.ros_node.get_clock().now().to_msg()
        frame_id = "oak_rgb_frame"

        # — Imagen —
        frame   = rgb_msg.getCvFrame()
        img_msg = self.ros_node.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp    = stamp
        img_msg.header.frame_id = frame_id

        # — Camera info con timestamp idéntico al de la imagen —
        self.ros_node._cam_info.header.stamp    = stamp
        self.ros_node._cam_info.header.frame_id = frame_id

        # — Publica imagen y camera_info juntos —
        self.ros_node.img_pub.publish(img_msg)
        self.ros_node.info_pub.publish(self.ros_node._cam_info)

        # — Detecciones YOLO —
        results = self._decode_yolo(nn_msg)
        det_msg = Detection2DArray()
        det_msg.header.stamp    = stamp
        det_msg.header.frame_id = frame_id

        for det in results:
            d   = Detection2D()
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = (
                LABEL_MAP[det["label"]] if det["label"] < len(LABEL_MAP)
                else str(det["label"])
            )
            hyp.hypothesis.score = det["confidence"]
            d.results.append(hyp)
            d.bbox.center.position.x = ((det["xmin"] + det["xmax"]) / 2) * IMG_W
            d.bbox.center.position.y = ((det["ymin"] + det["ymax"]) / 2) * IMG_H
            d.bbox.size_x = (det["xmax"] - det["xmin"]) * IMG_W
            d.bbox.size_y = (det["ymax"] - det["ymin"]) * IMG_H
            det_msg.detections.append(d)

        if results:
            self.ros_node.get_logger().info(
                f"Detected {len(results)} object(s): "
                + ", ".join(
                    f"{LABEL_MAP[d['label']]}({d['confidence']:.2f})"
                    for d in results
                )
            )

        self.ros_node.det_pub.publish(det_msg)

    def _decode_yolo(self, nn_data):
        detections = []

        for layer_name, grid_size in HEADS:
            try:
                tensor = nn_data.getTensor(layer_name, dequantize=True)
                raw    = np.array(tensor, dtype=np.float32)
            except Exception as e:
                self.ros_node.get_logger().warn(
                    f"Could not read {layer_name}: {e}",
                    throttle_duration_sec=5.0
                )
                continue

            raw = raw.reshape(9, grid_size, grid_size)
            raw = np.transpose(raw, (1, 2, 0))

            if not self._range_logged:
                self.ros_node.get_logger().info(
                    f"{layer_name} | obj: [{raw[:,:,4].min():.2f}, {raw[:,:,4].max():.2f}] "
                    f"| xy: [{raw[:,:,:2].min():.2f}, {raw[:,:,:2].max():.2f}]"
                )

            for gy in range(grid_size):
                for gx in range(grid_size):
                    cell     = raw[gy, gx]
                    obj_conf = sigmoid(cell[4])
                    if obj_conf < CONF_THRESH:
                        continue
                    class_probs = sigmoid(cell[5:5 + NUM_CLASSES])
                    label       = int(np.argmax(class_probs))
                    score       = float(obj_conf * class_probs[label])
                    if score < CONF_THRESH:
                        continue
                    cx = (sigmoid(cell[0]) + gx) / grid_size
                    cy = (sigmoid(cell[1]) + gy) / grid_size
                    bw = np.exp(np.clip(cell[2], -10, 10)) / IMG_W
                    bh = np.exp(np.clip(cell[3], -10, 10)) / IMG_H
                    detections.append({
                        "label":      label,
                        "confidence": score,
                        "xmin": float(np.clip(cx - bw / 2, 0, 1)),
                        "ymin": float(np.clip(cy - bh / 2, 0, 1)),
                        "xmax": float(np.clip(cx + bw / 2, 0, 1)),
                        "ymax": float(np.clip(cy + bh / 2, 0, 1)),
                    })

        if not self._range_logged:
            self._range_logged = True

        return nms(detections, IOU_THRESH)


class DronkabVisionNode(Node):
    def __init__(self):
        super().__init__('dronkab_vision_node')

        # Publishers — topics que aruco_opencv espera con cam_base_topic="oak"
        self.img_pub  = self.create_publisher(Image,            '/oak',           10)
        self.info_pub = self.create_publisher(CameraInfo,       '/camera_info',   10)
        self.det_pub  = self.create_publisher(Detection2DArray, '/oak/nn/detections', 10)
        self.bridge   = CvBridge()

        # Mensaje base de camera_info — el header se actualiza en cada frame
        self._cam_info                  = CameraInfo()
        self._cam_info.width            = IMG_W
        self._cam_info.height           = IMG_H
        self._cam_info.distortion_model = 'rational_polynomial'
        self._cam_info.d                = DIST_COEFFS
        self._cam_info.k                = [CAM_FX, 0.0,    CAM_CX,
                                           0.0,    CAM_FY, CAM_CY,
                                           0.0,    0.0,    1.0]
        self._cam_info.r                = [1.0, 0.0, 0.0,
                                           0.0, 1.0, 0.0,
                                           0.0, 0.0, 1.0]
        self._cam_info.p                = [CAM_FX, 0.0,    CAM_CX, 0.0,
                                           0.0,    CAM_FY, CAM_CY, 0.0,
                                           0.0,    0.0,    1.0,    0.0]

        pkg_share = get_package_share_directory('imav25')
        blob_path = os.path.join(
            pkg_share, 'resources', 'tmr_model', 'best_openvino_2022.1_6shave.blob'
        )

        self.pipeline = dai.Pipeline()

        camRgb = self.pipeline.create(dai.node.ColorCamera)
        camRgb.setPreviewSize(IMG_W, IMG_H)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(30)

        nn = self.pipeline.create(dai.node.NeuralNetwork)
        nn.setBlobPath(blob_path)
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)

        camRgb.preview.link(nn.input)

        # HostNode v3: recibe rgb + nn y publica en ROS2
        self.pipeline.create(VisionHostNode).build(
            camRgb.preview,
            nn.out,
            self
        )

        try:
            self.pipeline.start()
            self.get_logger().info("OAK-D pipeline started (DepthAI v3.5.0)")
        except Exception as e:
            self.get_logger().error(f"Pipeline start failed: {e}")
            sys.exit(1)

        self.get_logger().info("Dronkab Vision Node RUNNING")

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DronkabVisionNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()