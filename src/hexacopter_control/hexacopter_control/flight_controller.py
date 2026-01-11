import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
from std_msgs.msg import Bool

class FlightController(Node):
    """
    Standard Flight Controller
    Default Home: Munger, Bihar (configured via PX4 env vars)
    """
    def __init__(self):
        super().__init__('flight_controller')

        # Configure QoS profile for PX4 communication (Best Effort is required)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Declare parameters
        self.declare_parameter('takeoff_altitude', 5.0)

        # Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_odometry_sub_ = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.listener_callback, qos_profile)
        self.vehicle_status_sub_ = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # AI Subscriber
        self.weed_detection_sub_ = self.create_subscription(
            Bool, '/ai/weed_detected', self.weed_detection_callback, 10)

        # Timer (20Hz)
        self.timer_period = 0.05  # seconds
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)

        self.offboard_setpoint_counter_ = 0

    def vehicle_status_callback(self, msg):
        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info("Flight Mode: OFFBOARD", throttle_duration_sec=5.0)
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info("Vehicle: ARMED", throttle_duration_sec=5.0)

    def listener_callback(self, msg):
        # self.get_logger().info(f'Received odometry: {msg.position}')
        pass

    def weed_detection_callback(self, msg):
        if msg.data:
            # In a real drone, this would toggle a GPIO pin for the pump
            self.get_logger().info("💦 SPRAYER ACTIVATED: Weed Detected!", throttle_duration_sec=2.0)

    def timer_callback(self):
        if self.offboard_control_mode_publisher_.get_subscription_count() == 0:
            self.get_logger().warn("No subscribers! Is the Micro-XRCE-DDS Agent running?", throttle_duration_sec=5.0)

        if self.offboard_setpoint_counter_ == 10:
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()

        # Publish Offboard Control Mode (Heartbeat)
        self.publish_offboard_control_mode()
        # Publish Trajectory Setpoint
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter_ < 11:
            self.offboard_setpoint_counter_ += 1

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        altitude = self.get_parameter('takeoff_altitude').value
        msg.position = [0.0, 0.0, -altitude] # NED coordinates: x, y, z (negative is up)
        msg.yaw = -3.14 # Face North (approx)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
    
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FlightController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()