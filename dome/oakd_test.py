#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

LABELS = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle",
    "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
    "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

class DetectionPrinter(Node):
    def __init__(self):
        super().__init__('detection_printer')
        self.create_subscription(
            Detection3DArray,
            '/oak/nn/spatial_detections',
            self.callback,
            10)

    def callback(self, msg):
        for det in msg.detections:
            for result in det.results:
                class_id = int(result.hypothesis.class_id)
                score = result.hypothesis.score
                label = LABELS[class_id] if class_id < len(LABELS) else f"unknown({class_id})"
                pos = det.bbox.center.position
                print(f"{label:15s} {score:.0%}  x:{pos.x:.2f} y:{pos.y:.2f} z:{pos.z:.2f}")

def main():
    rclpy.init()
    node = DetectionPrinter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()