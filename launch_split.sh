#!/bin/bash

# Project Paths
WORKSPACE_DIR=~/uav_agricultural_drone_project
PX4_DIR=~/PX4-Autopilot

# Detect ROS Distro (Humble vs Jazzy)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    ROS_DISTRO="jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    ROS_DISTRO="humble"
else
    echo "❌ No supported ROS 2 distro found (checked jazzy, humble)."
    exit 1
fi

# Safety Check: Ensure not in virtual environment
if [[ "$VIRTUAL_ENV" != "" ]]; then
    echo "⚠️  Virtual Environment detected ($VIRTUAL_ENV). Deactivating for this script..."
    PATH=${PATH/$VIRTUAL_ENV\/bin:/}
    unset VIRTUAL_ENV
fi

# --- ENVIRONMENT SYNC (Added per Audit) ---
# Add local models and worlds to Gazebo's search path
export GAZEBO_MODEL_PATH="$WORKSPACE_DIR/src/hexacopter_control/models:$WORKSPACE_DIR/src/hexacopter_control/worlds:$GAZEBO_MODEL_PATH"
export GAZEBO_RESOURCE_PATH="$WORKSPACE_DIR/src/hexacopter_control/models:$WORKSPACE_DIR/src/hexacopter_control/worlds:$GAZEBO_RESOURCE_PATH"

# Ensure PX4 finds the model if using custom SDFs
export PX4_SIM_MODEL=hexacopter_agricultural

echo "🌍 Environment Synced: GAZEBO_MODEL_PATH updated."

# 0. Cleanup previous processes
echo "🛑 Killing previous sessions..."
# Using killall -9 as requested for a clean slate
killall -9 gz_sim px4 MicroXRCEAgent ruby QGroundControl.AppImage flight_controller detection_node parameter_bridge 2>/dev/null
sleep 2

if [ ! -d "$PX4_DIR" ]; then
    echo "❌ Error: PX4 Directory not found at $PX4_DIR"
    exit 1
fi

# 1. Micro-XRCE-DDS Agent (New Window)
gnome-terminal --window --title="Micro-XRCE-DDS Agent" --geometry=100x24+0+0 -- bash -c "
echo 'Starting Agent...';
MicroXRCEAgent udp4 -p 8888; 
exec bash"

# 2. PX4 SITL (New Tab)
gnome-terminal --tab --title="PX4 SITL" -- bash -c "
echo 'Starting PX4 SITL...';
cd $PX4_DIR;
export HEADLESS=0;

# Set Location to Munger, Bihar
export PX4_HOME_LAT=25.3748
export PX4_HOME_LON=86.4735
export PX4_HOME_ALT=45.0

make px4_sitl gz_x500_depth; 
exec bash"

# 3. QGroundControl (New Tab)
if [ ! -f "$WORKSPACE_DIR/QGroundControl.AppImage" ]; then
    echo "⚠️ QGroundControl not found. Downloading automatically..."
    wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.2/QGroundControl.AppImage -O "$WORKSPACE_DIR/QGroundControl.AppImage"
    
    if [ $? -eq 0 ]; then
        chmod +x "$WORKSPACE_DIR/QGroundControl.AppImage"
    else
        echo "❌ Download failed. QGroundControl will not start."
    fi
fi

gnome-terminal --tab --title="QGroundControl" -- bash -c "
if [ -f \"$WORKSPACE_DIR/QGroundControl.AppImage\" ]; then
    echo 'Starting QGC...';
    $WORKSPACE_DIR/QGroundControl.AppImage; 
else
    echo '❌ QGC file missing. Check internet connection.';
fi
exec bash"

# 4. ROS 2 Nodes (New Tab)
gnome-terminal --tab --title="ROS 2 System" -- bash -c "
source /opt/ros/$ROS_DISTRO/setup.bash;
source $WORKSPACE_DIR/install/setup.bash;
cd $WORKSPACE_DIR;

echo '--- Launching ROS 2 Nodes ---';
echo 'Launching Full System (Flight Controller + Vision + Bridge)...';
ros2 launch agri_hexacopter full_system.launch.py
exec bash"

# 5. RViz2 (New Tab)
gnome-terminal --tab --title="RViz2" -- bash -c "
source /opt/ros/$ROS_DISTRO/setup.bash;
echo 'Starting RViz2...';
ros2 run rviz2 rviz2;
exec bash"

echo "✅ System launched in separate terminals."
echo "🎮 For MANUAL FLIGHT: Use QGroundControl Virtual Joysticks after PX4 starts."