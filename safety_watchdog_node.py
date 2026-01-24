import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleGlobalPosition, BatteryStatus, VehicleCommand
import time

class SafetyWatchdogNode(Node):
    def __init__(self):
        super().__init__('safety_watchdog_node')

        # Parameters
        self.declare_parameter('min_battery', 0.20) # 20%
        self.declare_parameter('gps_timeout', 2.0)  # Seconds
        
        # QoS for PX4
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
        
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_callback,
            qos_profile
        )

        # Publisher (Command Override)
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )

        self.last_gps_time = time.time()
        self.failsafe_triggered = False
        
        # Check loop (10Hz)
        self.timer = self.create_timer(0.1, self.check_safety)
        self.get_logger().info("🛡️ Safety Watchdog Active.")

    def gps_callback(self, msg):
        self.last_gps_time = time.time()

    def battery_callback(self, msg):
        min_bat = self.get_parameter('min_battery').value
        if msg.remaining < min_bat and not self.failsafe_triggered:
            self.get_logger().error(f"🔋 LOW BATTERY ({msg.remaining:.1%})! Triggering RTL.")
            self.trigger_failsafe(20) # VEHICLE_CMD_NAV_RETURN_TO_LAUNCH

    def check_safety(self):
        if self.failsafe_triggered: return

        # GPS Timeout Check
        timeout = self.get_parameter('gps_timeout').value
        if time.time() - self.last_gps_time > timeout:
            self.get_logger().error("📡 GPS LOST! Triggering Emergency Land.")
            self.trigger_failsafe(21) # VEHICLE_CMD_NAV_LAND

    def trigger_failsafe(self, command_id):
        self.failsafe_triggered = True
        
        msg = VehicleCommand()
        msg.command = command_id
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Send command multiple times to ensure receipt
        for _ in range(3):
            self.cmd_pub.publish(msg)
            time.sleep(0.01)
            
        self.get_logger().warn(f"⚠️ FAILSAFE COMMAND {command_id} SENT!")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SafetyWatchdogNode())
    rclpy.shutdown()