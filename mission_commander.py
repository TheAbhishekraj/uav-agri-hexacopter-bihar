import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus
from nav_msgs.msg import Path
import time
import math

class MissionCommander(Node):
    def __init__(self):
        super().__init__('mission_commander')

        # QoS for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)

        # Subscribers
        self.vehicle_status_subscriber_ = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)
        self.path_subscriber_ = self.create_subscription(Path, "/mission/waypoints", self.path_callback, 10)

        # State Variables
        self.vehicle_status = None
        self.mission_path = []
        self.current_waypoint_index = 0
        self.mission_state = "INIT" # INIT, TAKEOFF, MISSION, RETURN, DONE
        self.offboard_setpoint_counter = 0
        
        # Timer (10Hz for Offboard Control)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("👨‍✈️ Mission Commander Waiting for Path...")

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def path_callback(self, msg):
        if self.mission_state == "INIT":
            self.get_logger().info(f"📜 Received Mission Path with {len(msg.poses)} waypoints.")
            # Store path
            self.mission_path = msg.poses
            self.mission_state = "TAKEOFF"

    def timer_callback(self):
        if self.offboard_setpoint_counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.arm()

        # Send Heartbeat
        self.publish_offboard_control_mode()

        # State Machine
        if self.mission_state == "TAKEOFF":
            # Takeoff to 5m
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0, 0.0)
            if self.offboard_setpoint_counter > 100: # Wait ~10s for takeoff
                self.get_logger().info("🚀 Takeoff Complete. Starting Mission.")
                self.mission_state = "MISSION"

        elif self.mission_state == "MISSION":
            if self.current_waypoint_index < len(self.mission_path):
                target = self.mission_path[self.current_waypoint_index]
                
                # Coordinate Conversion: ROS ENU (x,y,z) -> PX4 NED (North, East, Down)
                # ROS X (East) -> PX4 Y
                # ROS Y (North) -> PX4 X
                # ROS Z (Up)   -> PX4 -Z
                
                ros_x = target.pose.position.x
                ros_y = target.pose.position.y
                ros_z = target.pose.position.z
                
                px4_x = ros_y
                px4_y = ros_x
                px4_z = -ros_z
                
                self.publish_trajectory_setpoint(px4_x, px4_y, px4_z, 0.0)
                
                # Simple time-based waypoint switching (in real app, check distance)
                # We assume 5 seconds per waypoint for this demo
                if self.offboard_setpoint_counter % 50 == 0: 
                    self.current_waypoint_index += 1
                    self.get_logger().info(f"📍 Waypoint {self.current_waypoint_index}/{len(self.mission_path)} reached.")
            else:
                self.get_logger().info("✅ Mission Complete. Returning Home.")
                self.mission_state = "RETURN"

        elif self.mission_state == "RETURN":
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            self.mission_state = "DONE"

        self.offboard_setpoint_counter += 1

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("⚠️ Arming command sent")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
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
    rclpy.spin(MissionCommander())
    rclpy.shutdown()