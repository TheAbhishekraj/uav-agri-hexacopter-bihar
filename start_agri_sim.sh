#!/bin/bash

# =================================================================
# Master Launch Script: uav-agri-hexacopter-bihar
# Purpose: Foolproof launch sequence for Agricultural Simulation
# =================================================================

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$PROJECT_ROOT"

# 1. Cleanup existing processes to prevent port conflicts
echo "🧹 Cleaning up existing ROS, Gazebo, and MAVLink processes..."
killall -9 gz_sim px4 MicroXRCEAgent ruby QGroundControl flight_controller detection_node mission_commander 2>/dev/null
sleep 2

# 2. Source Environments
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "❌ Error: ROS 2 Jazzy not found at /opt/ros/jazzy/setup.bash"
    exit 1
fi

if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
else
    echo "⚠️  Warning: Workspace not built. Running colcon build..."
    colcon build --symlink-install --packages-select hexacopter_control
    source "$PROJECT_ROOT/install/setup.bash"
fi

# 3. Export Project-Specific Paths
source "$PROJECT_ROOT/setup_project_env.sh"
export GZ_SIM_RESOURCE_PATH="$PROJECT_ROOT/src/hexacopter_control/models:$PROJECT_ROOT/src/hexacopter_control/worlds:$GZ_SIM_RESOURCE_PATH"

LAUNCH_CMD="env -u LD_LIBRARY_PATH -u LD_PRELOAD -u GTK_PATH -u GIO_EXTRA_MODULES gnome-terminal --window"

# 4. Launch Simulation Sequence
echo "🚀 Launching Hexacopter Simulation (Bihar World)..."

# Start Micro-XRCE-DDS Agent in background
$LAUNCH_CMD --title="1. DDS Agent" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"

# Start PX4 SITL and Gazebo
$LAUNCH_CMD --title="2. PX4 SITL" -- bash -c "echo '⏳ Waiting for DDS Agent...'; sleep 5; cd $PX4_DIR; export PX4_GZ_WORLD=bihar_farm; export PX4_GZ_MODEL=hexacopter_agricultural; PX4_SYS_AUTOSTART=4001 make px4_sitl gz_x500 -j2; exec bash"

# 5. Heartbeat Check
echo "⏳ Waiting 5 seconds for MAVLink initialization..."
sleep 5

echo "🔍 Checking MAVLink Heartbeat on port 14540..."
if ss -ulnp | grep -q ":14540"; then
    echo "✅ MAVLink Heartbeat detected on port 14540."
else
    echo "❌ Error: No MAVLink activity on port 14540. Check PX4 logs."
fi

# 6. Launch QGroundControl
if [ ! -f "$PROJECT_ROOT/QGroundControl.AppImage" ]; then
    echo "⚠️ QGroundControl not found. Downloading from GitHub..."
    wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.2/QGroundControl.AppImage -O "$PROJECT_ROOT/QGroundControl.AppImage"
    chmod +x "$PROJECT_ROOT/QGroundControl.AppImage"
fi
$LAUNCH_CMD --title="3. QGroundControl" -- bash -c "unset LD_LIBRARY_PATH; unset LD_PRELOAD; unset GTK_PATH; unset GIO_EXTRA_MODULES; echo 'Waiting for PX4 to boot...'; sleep 10; ./QGroundControl.AppImage; exec bash"

echo "✅ Master Launch Sequence Complete."