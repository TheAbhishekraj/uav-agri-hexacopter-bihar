import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import json
import hashlib

# Try importing lightweight runtime first, fall back to full tensorflow
try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    try:
        import tensorflow.lite.python.interpreter as tflite
    except ImportError:
        tflite = None

class ThermalAINode(Node):
    def __init__(self):
        super().__init__('thermal_ai_node')
        
        if tflite is None:
            self.get_logger().error("❌ TFLite not installed. Run ./setup_tflite.sh")
            return

        # Load Model Assets
        # In a real deploy, use ament_index to find 'resource' path
        home = os.path.expanduser('~')
        base_path = os.path.join(home, 'uav_agricultural_drone_project/src/yolov8_detection/resource')
        model_path = os.path.join(base_path, 'mobilenet_v2_1.0_224_quant.tflite')
        label_path = os.path.join(base_path, 'labels_mobilenet_quant_v1_224.txt')
        info_path = os.path.join(base_path, 'model_info.json')

        self.get_logger().info(f"🧠 Loading Model: {os.path.basename(model_path)}")
        
        # 1. Checksum & Version Verification
        if os.path.exists(info_path):
            try:
                with open(info_path, 'r') as f:
                    meta = json.load(f)
                    expected_sha = meta.get('sha256')
                    version = meta.get('version', 'unknown')
                    self.get_logger().info(f"   Model Version: {version}")
                    
                    if expected_sha:
                        with open(model_path, 'rb') as mf:
                            file_sha = hashlib.sha256(mf.read()).hexdigest()
                            if file_sha != expected_sha:
                                self.get_logger().warn(f"⚠️ Checksum Mismatch! Expected {expected_sha[:8]}..., got {file_sha[:8]}...")
                            else:
                                self.get_logger().info("✅ Checksum Verified.")
            except Exception as e:
                self.get_logger().warn(f"Metadata read failed: {e}")
        
        try:
            # 2. Optimization: Enable Multithreading (4 threads for RPi/Jetson)
            self.interpreter = tflite.Interpreter(model_path=model_path, num_threads=4)
            self.interpreter.allocate_tensors()
            
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            
            with open(label_path, 'r') as f:
                self.labels = [line.strip() for line in f.readlines()]
                
            self.get_logger().info("✅ AI Ready.")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load model: {e}")
            self.interpreter = None

        self.bridge = CvBridge()
        
        # Subscribe to Camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publish Detections
        self.detection_pub = self.create_publisher(String, '/thermal/detections', 10)

    def image_callback(self, msg):
        if self.interpreter is None: return
        
        start_t = time.time()
        
        # 1. Convert ROS Image -> OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Bridge Error: {e}")
            return

        # 2. Preprocess (Resize to 224x224)
        input_shape = self.input_details[0]['shape'] # [1, 224, 224, 3]
        input_data = cv2.resize(cv_image, (input_shape[1], input_shape[2]))
        input_data = np.expand_dims(input_data, axis=0)
        
        # 3. Inference
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        
        # 4. Postprocess
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        results = np.squeeze(output_data)
        top_idx = results.argsort()[-1] # Top 1 prediction
        
        # Handle Quantized (0-255) vs Float outputs
        if self.input_details[0]['dtype'] == np.uint8:
            score = float(results[top_idx]) / 255.0
        else:
            score = float(results[top_idx])
            
        label = self.labels[top_idx]
        latency = (time.time() - start_t) * 1000

        # 5. Publish
        res_str = f"Found: {label} ({score:.1%}) | {latency:.1f}ms"
        self.detection_pub.publish(String(data=res_str))
        self.get_logger().info(res_str)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ThermalAINode())
    rclpy.shutdown()