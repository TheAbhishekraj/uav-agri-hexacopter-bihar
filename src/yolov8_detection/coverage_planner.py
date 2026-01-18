import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')
        self.publisher_ = self.create_publisher(Path, '/mission/waypoints', 10)
        
        # Parameters (Default: 0.25ha approx 50x50m)
        self.declare_parameter('altitude', 10.0) # meters
        self.declare_parameter('fov', 60.0)      # degrees (Horizontal)
        self.declare_parameter('overlap', 0.2)   # 20% overlap
        self.declare_parameter('width', 50.0)    # meters (X axis length)
        self.declare_parameter('height', 50.0)   # meters (Y axis length)
        
        # Timer to publish path periodically (for visualization)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.plan_path)
        self.get_logger().info("🚜 Coverage Planner Started. Calculating path...")

    def plan_path(self):
        # 1. Get Parameters
        alt = self.get_parameter('altitude').value
        fov = self.get_parameter('fov').value
        overlap = self.get_parameter('overlap').value
        w_field = self.get_parameter('width').value
        h_field = self.get_parameter('height').value

        # 2. Calculate Footprint (How much ground the camera sees)
        # Width of ground visible = 2 * h * tan(fov/2)
        fov_rad = math.radians(fov)
        ground_width = 2 * alt * math.tan(fov_rad / 2)
        
        # 3. Effective Lane Spacing (Move less than full width to overlap)
        spacing = ground_width * (1.0 - overlap)
        
        if spacing <= 0:
            self.get_logger().error("Invalid spacing! Check parameters.")
            return

        rows = int(math.ceil(h_field / spacing))
        
        # Log the math for verification
        self.get_logger().info(
            f"PLAN: Alt={alt}m | CamWidth={ground_width:.2f}m | Spacing={spacing:.2f}m | Rows={rows}"
        )

        # 4. Generate Waypoints (Lawnmower Zig-Zag)
        path_msg = Path()
        path_msg.header.frame_id = "map" 
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(rows):
            y = i * spacing
            
            # Zig-Zag logic: Even rows go Right, Odd rows go Left
            if i % 2 == 0:
                x_start, x_end = 0.0, w_field
            else:
                x_start, x_end = w_field, 0.0
            
            # Add Start of Row
            p1 = PoseStamped()
            p1.pose.position.x = float(x_start)
            p1.pose.position.y = float(y)
            p1.pose.position.z = float(alt)
            path_msg.poses.append(p1)

            # Add End of Row
            p2 = PoseStamped()
            p2.pose.position.x = float(x_end)
            p2.pose.position.y = float(y)
            p2.pose.position.z = float(alt)
            path_msg.poses.append(p2)

        self.publisher_.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CoveragePlanner())
    rclpy.shutdown()