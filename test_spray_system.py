import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import VehicleGlobalPosition, ActuatorServos
import time
import json

class SprayTester(Node):
    def __init__(self):
        super().__init__('spray_tester')
        
        # Publishers (Inputs to Controller)
        self.gps_pub = self.create_publisher(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', 10)
        self.cmd_pub = self.create_publisher(String, '/command/spray', 10)
        
        # Subscriber (Output from Controller)
        self.actuator_sub = self.create_subscription(ActuatorServos, '/fmu/in/actuator_servos', self.actuator_callback, 10)
        
        self.timer = self.create_timer(1.0, self.inject_trigger)
        self.actuation_detected = False

    def inject_trigger(self):
        if self.actuation_detected: return

        # 1. Fake GPS (Inside Geofence)
        gps_msg = VehicleGlobalPosition()
        gps_msg.lat = 25.3750 # Safe Lat
        gps_msg.lon = 86.4740 # Safe Lon
        gps_msg.timestamp = int(time.time() * 1e6)
        self.gps_pub.publish(gps_msg)
        
        # 2. Fake Spray Command
        cmd = {
            "action": "SPRAY",
            "target": "weed",
            "confidence": 0.95
        }
        self.cmd_pub.publish(String(data=json.dumps(cmd)))
        
        print("💉 Injected: Safe GPS + Spray Command")

    def actuator_callback(self, msg):
        # Check Servo Channel 1 (Index 0)
        servo_val = msg.control[0]
        
        if servo_val > 0.5:
            print(f"\n✅ SUCCESS! Actuator Servo Moved: {servo_val}")
            print("   -> The Water Gun fired!")
            self.actuation_detected = True
            raise SystemExit
        elif servo_val < -0.5:
            # Valve closed, ignore
            pass

def main(args=None):
    rclpy.init(args=args)
    tester = SprayTester()
    try:
        rclpy.spin(tester)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    print("🧪 Starting Spray System Test...")
    main()