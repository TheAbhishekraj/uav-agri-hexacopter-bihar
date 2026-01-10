from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='hexacopter_camera', executable='camera_node'),
        Node(package='yolov8_detection', executable='detection_node'),
        Node(package='hexacopter_control', executable='flight_controller'),
    ])
