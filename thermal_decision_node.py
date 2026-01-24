import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleGlobalPosition
import json
import math
import re

class ThermalDecisionNode(Node):
    def __init__(self):
        super().__init__('thermal_decision_node')
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('target_class', 'weed')
        self.declare_parameter('fov', 60.0) # degrees
        self.declare_parameter('image_width', 640)
        
        # QoS for PX4 (Must be Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.gps_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.gps_callback,
            qos_profile
        )
        
        self.detection_sub = self.create_subscription(
            String,
            '/thermal/detections',
            self.detection_callback,
            10
        )
        
        # Publisher (The Spray Command)
        self.spray_pub = self.create_publisher(String, '/command/spray', 10)
        
        self.latest_gps = None
        self.get_logger().info("🧠 Thermal Decision Node Active. Waiting for GPS & Detections...")

    def gps_callback(self, msg):
        self.latest_gps = msg

    def detection_callback(self, msg):
        if self.latest_gps is None:
            return # Cannot georeference without GPS

        # Parse the detection string from Level 3
        # Format: "Found: {label} ({score:.1%}) | {latency:.1f}ms"
        data = msg.data
        
        try:
            # Regex to extract label and score
            match = re.search(r"Found: (\w+) \(([\d.]+)%\)", data)
            if not match:
                return
                
            label = match.group(1)
            score = float(match.group(2)) / 100.0
            
            target_class = self.get_parameter('target_class').value
            threshold = self.get_parameter('confidence_threshold').value
            
            if label == target_class and score >= threshold:
                self.trigger_spray(label, score)
                
        except Exception as e:
            self.get_logger().error(f"Parsing error: {e}")

    def trigger_spray(self, label, score):
        # Sensor Fusion: Project Center Pixel to Lat/Lon
        # Simplified Nadir Projection (assuming camera points straight down)
        
        alt = self.latest_gps.alt
        lat = self.latest_gps.lat
        lon = self.latest_gps.lon
        
        # In a real scenario, we would use the bounding box center.
        # Here we assume the detection is roughly centered or we target the drone's current position.
        
        # Create Spray Command
        command = {
            "action": "SPRAY",
            "target": label,
            "confidence": score,
            "location": {
                "lat": lat,
                "lon": lon,
                "alt": alt
            },
            "timestamp": self.get_clock().now().nanoseconds
        }
        
        json_cmd = json.dumps(command)
        self.spray_pub.publish(String(data=json_cmd))
        
        self.get_logger().info(f"💦 SPRAY COMMAND ISSUED! [{label} @ {lat:.6f}, {lon:.6f}]")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ThermalDecisionNode())
    rclpy.shutdown()