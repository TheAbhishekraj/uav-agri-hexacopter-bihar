import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleGlobalPosition
from cv_bridge import CvBridge
import cv2
import time

# Graceful fallback if Ultralytics is not installed (prevents crash during build/test)
try:
    from ultralytics import YOLO
    HAS_YOLO = True
except ImportError:
    HAS_YOLO = False

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        # Requirement: Coordinate Sync (GPS Subscription)
        # QoS must match PX4's RMW settings (Best Effort) for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.gps_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.gps_callback,
            qos_profile
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw', # Ensure this matches your Gazebo camera topic
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.latest_gps = None
        
        # Requirement: Inference Rate > 15 FPS
        # We track FPS to verify this requirement
        self.frame_count = 0
        self.start_time = time.time()

        if HAS_YOLO:
            # Load YOLOv8 Nano model for speed (>15 FPS on Jetson/Modern CPU)
            # 'yolov8n.pt' will download automatically if not present
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info("YOLOv8 Nano model loaded for high-speed inference.")
        else:
            self.get_logger().warn("Ultralytics not installed. Inference will be skipped.")

    def gps_callback(self, msg):
        self.latest_gps = msg

    def image_callback(self, msg):
        # 1. Convert ROS Image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge conversion failed: {e}")
            return

        # 2. Run Inference
        detections = []
        if HAS_YOLO:
            # verbose=False speeds up console output
            results = self.model(cv_image, verbose=False)
            
            # Extract detections (simplified)
            for r in results:
                for box in r.boxes:
                    # In a real agri-app, check for specific classes (e.g., 'weed')
                    detections.append(box.xyxy[0].tolist())

        # 3. Coordinate Sync
        # Tag detections with the latest GPS coordinate
        if self.latest_gps:
            current_lat = self.latest_gps.lat
            current_lon = self.latest_gps.lon
            
            if len(detections) > 0:
                self.get_logger().info(
                    f"WEED DETECTED at Lat: {current_lat}, Lon: {current_lon} | Count: {len(detections)}"
                )
                # TODO: Publish to a custom topic like /agri/detections
        else:
            if len(detections) > 0:
                self.get_logger().warn("Weed detected but waiting for GPS fix...")

        # 4. FPS Monitoring
        self.frame_count += 1
        if time.time() - self.start_time >= 1.0:
            fps = self.frame_count / (time.time() - self.start_time)
            self.get_logger().info(f"Inference Rate: {fps:.2f} FPS")
            self.frame_count = 0
            self.start_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()