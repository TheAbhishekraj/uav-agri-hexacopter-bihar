#!/bin/bash
echo "🚀 Installing Agricultural Drone System..."

# Safety Check: Ensure not in virtual environment
if [[ "$VIRTUAL_ENV" != "" ]]; then
    echo "⚠️  Virtual Environment detected ($VIRTUAL_ENV). Deactivating for this script..."
    PATH=${PATH/$VIRTUAL_ENV\/bin:/}
    unset VIRTUAL_ENV
fi

# Update system
sudo apt update

# Install ROS 2 Jazzy
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz libfuse2

# Install PX4 tools
sudo apt install -y px4-tools

# Install System Python Dependencies (Best for ROS 2 compatibility)
sudo apt install -y python3-opencv python3-numpy

# Install remaining Python dependencies
# Note: --break-system-packages is required on Ubuntu 24.04 to install outside a venv
pip3 install "numpy<2" mavsdk ultralytics --break-system-packages

# Download QGroundControl
if [ -f "QGroundControl.AppImage" ]; then
    echo "Removing old QGroundControl to force update..."
    rm QGroundControl.AppImage
fi

echo "Downloading QGroundControl..."
wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.2/QGroundControl.AppImage
chmod +x QGroundControl.AppImage

# Build workspace
cd $(dirname $0)
colcon build --symlink-install

echo "✅ Installation complete!"
echo "Run: ./launch_split.sh"
