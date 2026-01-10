#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node

class YOLOv8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')
        self.model = YOLO('yolov8s.pt')
        # Real-time detection pipeline
        pass
