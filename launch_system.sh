#!/bin/bash

# Ensure all background processes are killed when script exits
trap "kill 0" EXIT

# 0. Check for build
if [ ! -f "$HOME/uav_agricultural_drone_project/install/setup.bash" ]; then
    echo "❌ Error: Workspace not built. Run './clean_build.sh' first."
    exit 1
fi

# 1. Source ROS 2 and YOUR specific Workspace
# We use 'source' to make sure the packages 'yolov8_detection' and 'hexacopter_control' are visible.
source /opt/ros/jazzy/setup.bash
source ~/uav_agricultural_drone_project/install/setup.bash

echo "Starting Micro-XRCE-DDS Agent..."
# Start Agent in background
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
sleep 2

echo "Starting ROS-Gazebo Camera Bridge..."
# Bridge Gazebo topic /camera/image_raw to ROS 2
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image &

echo "Launching custom nodes..."
# Use 'ros2 run' only after confirming the packages are sourced
# Note: We use 'ros2 run' here. Ensure PX4 SITL is running separately!
# (e.g., cd ~/PX4-Autopilot && make px4_sitl gz_x500)
ros2 run yolov8_detection detection_node &
ros2 run hexacopter_control flight_controller &

echo "✅ All systems active. Press Ctrl+C to shutdown."
wait
