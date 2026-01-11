from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Flight Controller Node with parameters
        Node(
            package='hexacopter_control',
            executable='flight_controller',
            name='flight_controller',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'takeoff_altitude': 5.0}
            ]
        ),
        # YOLOv8 Object Detection Node
        Node(
            package='yolov8_detection',
            executable='detection_node',
            name='yolov8_detection',
            output='screen',
            emulate_tty=True
        ),
        # ROS-Gazebo Bridge for Camera Stream
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='image_bridge',
            arguments=['/camera/image_raw'],
            output='screen'
        )
    ])