#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
import math

class MissionSprayer(Node):
    """
    Z-Pattern Spraying Mission for Agricultural Drone
    Location: Munger, Bihar (25.3748 N, 86.4735 E)
    Path: Takeoff (5m) -> North (10m) -> East (5m) -> South (10m) -> Land
    """
    def __init__(self):
        super().__init__('mission_sprayer')

        # Configure QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

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

        # State variables
        self.vehicle_local_position = [0.0, 0.0, 0.0]
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter_ = 0
        self.mission_state = 0  # 0: Init, 1: Takeoff, 2: North, 3: East, 4: South, 5: Land

        # Timer (10Hz)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Mission Sprayer Node Started. Waiting for PX4 connection...")

    def listener_callback(self, msg):
        self.vehicle_local_position = msg.position

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.get_logger().info("Switching to Offboard Mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Landing command sent")

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

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = -3.14  # Face North
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def distance_to_target(self, target_x, target_y, target_z):
        dx = target_x - self.vehicle_local_position[0]
        dy = target_y - self.vehicle_local_position[1]
        dz = target_z - self.vehicle_local_position[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def timer_callback(self):
        self.publish_offboard_control_mode()

        # Initialize Offboard mode after 10 setpoints (1 second)
        if self.offboard_setpoint_counter_ == 10:
            self.engage_offboard_mode()
            self.arm()
            self.mission_state = 1  # Start Takeoff

        # Mission Logic State Machine
        target_pos = [0.0, 0.0, 0.0]
        
        if self.mission_state == 0:
            target_pos = [0.0, 0.0, 0.0]

        elif self.mission_state == 1:  # Takeoff to 5m
            target_pos = [0.0, 0.0, -5.0]
            if self.distance_to_target(*target_pos) < 0.5:
                self.get_logger().info("Reached Altitude (5m). Executing Z-Pattern: Moving North.")
                self.mission_state = 2

        elif self.mission_state == 2:  # North 10m
            target_pos = [10.0, 0.0, -5.0]
            if self.distance_to_target(*target_pos) < 0.5:
                self.get_logger().info("Reached North Waypoint. Moving East.")
                self.mission_state = 3

        elif self.mission_state == 3:  # East 5m
            target_pos = [10.0, 5.0, -5.0]
            if self.distance_to_target(*target_pos) < 0.5:
                self.get_logger().info("Reached East Waypoint. Moving South.")
                self.mission_state = 4

        elif self.mission_state == 4:  # South 10m (Back to X=0, Y=5)
            target_pos = [0.0, 5.0, -5.0]
            if self.distance_to_target(*target_pos) < 0.5:
                self.get_logger().info("Z-Pattern Complete. Landing.")
                self.mission_state = 5
        
        elif self.mission_state == 5:  # Land
            self.land()
            return  # Stop sending setpoints to let Land mode take over

        # Publish target
        self.publish_trajectory_setpoint(float(target_pos[0]), float(target_pos[1]), float(target_pos[2]))
        
        self.offboard_setpoint_counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = MissionSprayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()