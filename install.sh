#!/bin/bash
echo "🚀 Installing Agricultural Drone System..."

# Update system
sudo apt update

# Install ROS 2 Humble
sudo apt install -y ros-humble-desktop

# Install PX4 tools
sudo apt install -y px4-tools

# Python dependencies
pip3 install mavsdk ultralytics opencv-python rclpy

# Build workspace
cd $(dirname $0)
colcon build

echo "✅ Installation complete!"
echo "Run: ./launch_system.sh"
