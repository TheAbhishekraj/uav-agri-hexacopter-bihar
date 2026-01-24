import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleGlobalPosition, ActuatorServos
import json

class SprayControllerNode(Node):
    def __init__(self):
        super().__init__('spray_controller_node')

        # Parameters
        self.declare_parameter('flow_rate', 50.0) # mL/s
        # Geofence: Munger, Bihar (Approximate Farm Bounds)
        self.declare_parameter('safe_lat_min', 25.3740)
        self.declare_parameter('safe_lat_max', 25.3760)
        self.declare_parameter('safe_lon_min', 86.4730)
        self.declare_parameter('safe_lon_max', 86.4750)
        
        # QoS for PX4 (Best Effort is required for PX4 topics)
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
        
        self.command_sub = self.create_subscription(
            String,
            '/command/spray',
            self.command_callback,
            10
        )

        # Publisher (Actuator Control -> PX4)
        # We map the Sprayer Valve to Servo Channel 1
        self.actuator_pub = self.create_publisher(
            ActuatorServos,
            '/fmu/in/actuator_servos',
            qos_profile
        )

        self.current_pos = None
        self.is_spraying = False
        self.total_volume = 0.0
        self.stop_timer = None
        
        self.get_logger().info("💦 Spray Controller Ready. Waiting for commands...")

    def gps_callback(self, msg):
        self.current_pos = msg

    def command_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('action') != 'SPRAY': return
            
            # Safety Check: Geofence
            if not self.check_geofence():
                self.get_logger().warn("⛔ Spray blocked: Outside Geofence!")
                return

            # Execute Spray Pulse
            duration = 1.0 # Default 1 second burst
            self.activate_sprayer(duration)
            
        except Exception as e:
            self.get_logger().error(f"Command Error: {e}")

    def check_geofence(self):
        if not self.current_pos: return False
        
        lat = self.current_pos.lat
        lon = self.current_pos.lon
        
        min_lat = self.get_parameter('safe_lat_min').value
        max_lat = self.get_parameter('safe_lat_max').value
        min_lon = self.get_parameter('safe_lon_min').value
        max_lon = self.get_parameter('safe_lon_max').value
        
        return (min_lat <= lat <= max_lat) and (min_lon <= lon <= max_lon)

    def activate_sprayer(self, duration):
        if self.is_spraying: return
        
        self.is_spraying = True
        flow = self.get_parameter('flow_rate').value
        volume = flow * duration
        self.total_volume += volume
        
        self.get_logger().info(f"🚿 OPENING VALVE: {duration}s | Vol: {volume}mL | Total: {self.total_volume}mL")
        
        # Send Open Command (Servo 1 -> +1.0)
        self.publish_actuator(1.0)
        
        # Schedule Close
        self.stop_timer = self.create_timer(duration, self.stop_sprayer)

    def stop_sprayer(self):
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None
            
        self.publish_actuator(-1.0) # Close Valve
        self.is_spraying = False
        self.get_logger().info("🚫 VALVE CLOSED")

    def publish_actuator(self, value):
        msg = ActuatorServos()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp_sample = msg.timestamp
        
        # PX4 ActuatorServos expects array of 8 floats (-1 to 1)
        # We map Sprayer to Channel 1 (Index 0)
        controls = [float('nan')] * 8
        controls[0] = value 
        msg.control = controls
        
        self.actuator_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SprayControllerNode())
    rclpy.shutdown()