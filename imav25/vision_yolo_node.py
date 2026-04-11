#!/usr/bin/env python3
"""
Node name: vision_yolo_node

Publisher(s): /oak/rgb/image_raw (Message Type: Image)
              /oak/nn/detections (Message Type: Detection2DArray)

Constant(s): -IMG_W, IMG_H (Integer): Dimensions of the image (input to the ANN), height and width
                in pixels. Usually they have the same value, they are used to initialize the pipeline
                in the OAK-D camera and to get the coordinates of the bounding boxes in pixels.
             -CONF_THRESH (Float): Value between 0 and 1, it's the normalized value of the percentage
                of the confidence threshold, the minimum value to consider the hypothesis correct.
             -IOU_THRESH (Float):  Value between 0 and 1, it's the normalized value of the percentage
                of the Intersection over Union threshold, this value helps to erase multiple detections
                of the same object (superposition).
             -NUM_CLASSES (Integer): Number of classes in the ANN (Artificial Neural Network), it has to
                match with the values in the json file in the resources/tmr_model folder. 
             -LABEL_MAP (String Array): Array with the classes names in training order, it doesn't matter
                if it matches or not the original names (in the json file), but it is highly recommended.
             -HEADS (Struct: String, Integer): The data in the struct is layer_name, grid_size. These data
                is used to access the output data of the tensors from the OAK-D pipeline (the value set shall
                not be changed).

Description: This node must be launched in the beginning. This node initialize the OAK-D camera pipeline by
             setting up the model and the values needed to run that model, it also get the tensors from the 
             OAK-D camera and publish the detections into a topic (/oak/nn/detections) and the raw image
             from the camera into another (/oak/rgb/image_raw).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import depthai as dai
import numpy as np
import sys
import os
from ament_index_python.packages import get_package_share_directory

IMG_W, IMG_H = 640, 640
CONF_THRESH  = 0.45
IOU_THRESH   = 0.45
NUM_CLASSES  = 4

# ← Replace with your actual class names in training order
LABEL_MAP = ["class0", "class1", "class2", "class3"]

HEADS = [
    ("output1_yolov6r2", 80),
    ("output2_yolov6r2", 40),
    ("output3_yolov6r2", 20),
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


class DronkabVisionNode(Node):
    def __init__(self):
        super().__init__('dronkab_vision_node')

        self.img_pub = self.create_publisher(Image, '/oak/rgb/image_raw', 10)
        self.det_pub = self.create_publisher(Detection2DArray, '/oak/nn/detections', 10)
        self.bridge  = CvBridge()

        pkg_share = get_package_share_directory('oak_ros')
        blob_path = os.path.join(
            pkg_share, 'resources', 'tmr_model', 'best_openvino_2022.1_6shave.blob'
        )

        pipeline = dai.Pipeline()

        camRgb = pipeline.create(dai.node.ColorCamera)
        camRgb.setPreviewSize(IMG_W, IMG_H)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(30)

        nn = pipeline.create(dai.node.NeuralNetwork)
        nn.setBlobPath(blob_path)
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)

        camRgb.preview.link(nn.input)

        self.rgbQueue = camRgb.preview.createOutputQueue(maxSize=4, blocking=False)
        self.nnQueue  = nn.out.createOutputQueue(maxSize=4, blocking=False)

        try:
            pipeline.start()
            self.get_logger().info("OAK-D pipeline started (DepthAI v3).")
        except Exception as e:
            self.get_logger().error(f"Pipeline start failed: {e}")
            sys.exit(1)

        self.pipeline = pipeline
        self._range_logged = False  # log raw ranges once for verification
        self.create_timer(0.033, self.update)
        self.get_logger().info("Dronkab Vision Node RUNNING")

    def _decode_yolo(self, nn_data):
        detections = []

        for layer_name, grid_size in HEADS:
            try:
                # ✅ DepthAI v3 API — replaces getLayerFp16
                tensor = nn_data.getTensor(layer_name, dequantize=True)
                raw = np.array(tensor, dtype=np.float32)
            except Exception as e:
                self.get_logger().warn(
                    f"Could not read {layer_name}: {e}",
                    throttle_duration_sec=5.0
                )
                continue

            # [1, 9, grid, grid] → [grid, grid, 9]
            raw = raw.reshape(9, grid_size, grid_size)
            raw = np.transpose(raw, (1, 2, 0))

            # Log value ranges once
            if not self._range_logged:
                self.get_logger().info(
                    f"{layer_name} | obj: [{raw[:,:,4].min():.2f}, {raw[:,:,4].max():.2f}] "
                    f"| xy:  [{raw[:,:,:2].min():.2f}, {raw[:,:,:2].max():.2f}]"
                )

            for gy in range(grid_size):
                for gx in range(grid_size):
                    cell = raw[gy, gx]

                    obj_conf = sigmoid(cell[4])
                    if obj_conf < CONF_THRESH:
                        continue

                    class_probs = sigmoid(cell[5:5 + NUM_CLASSES])
                    label = int(np.argmax(class_probs))
                    score = float(obj_conf * class_probs[label])
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

    def update(self):
        inRgb = self.rgbQueue.tryGet()
        inDet = self.nnQueue.tryGet()

        if inRgb is not None:
            frame    = inRgb.getCvFrame()
            img_msg  = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp    = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "oak_rgb_frame"
            self.img_pub.publish(img_msg)

        if inDet is not None:
            results  = self._decode_yolo(inDet)
            det_msg  = Detection2DArray()
            det_msg.header.stamp    = self.get_clock().now().to_msg()
            det_msg.header.frame_id = "oak_rgb_frame"

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
                self.get_logger().info(
                    f"Detected {len(results)} object(s): "
                    + ", ".join(f"{LABEL_MAP[d['label']]}({d['confidence']:.2f})" for d in results)
                )

            self.det_pub.publish(det_msg)


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