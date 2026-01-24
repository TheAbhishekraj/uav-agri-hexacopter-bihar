#!/bin/bash

# Project Paths
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PX4_DIR=~/DronePhD_Lab/PX4/PX4-Autopilot

# Safety Check: Ensure not in virtual environment
if [[ "$VIRTUAL_ENV" != "" ]]; then
    echo "⚠️  Virtual Environment detected ($VIRTUAL_ENV). Deactivating for this script..."
    PATH=${PATH/$VIRTUAL_ENV\/bin:/}
    unset VIRTUAL_ENV
fi

# 0. Cleanup previous processes
echo "🛑 Killing previous sessions..."
killall -9 gz_sim px4 MicroXRCEAgent ruby QGroundControl flight_controller detection_node mission_commander 2>/dev/null

if [ ! -d "$PX4_DIR" ]; then
    echo "❌ Error: PX4 Directory not found at $PX4_DIR"
    exit 1
fi

# Check for outdated world file (missing crop rows or old sensors config)
if [ -f "$WORKSPACE_DIR/src/hexacopter_control/worlds/bihar_farm.sdf" ] && ! grep -q "gz-sim-sensors-system" "$WORKSPACE_DIR/src/hexacopter_control/worlds/bihar_farm.sdf"; then
    echo "⚠️  World file outdated (missing sensor plugins). Deleting to regenerate..."
    rm "$WORKSPACE_DIR/src/hexacopter_control/worlds/bihar_farm.sdf"
fi

# Ensure world file exists before symlinking
if [ ! -f "$WORKSPACE_DIR/src/hexacopter_control/worlds/bihar_farm.sdf" ]; then
    echo "⚠️  World file missing. Running setup_assets.sh..."
    $WORKSPACE_DIR/setup_assets.sh
fi

# Ensure hexacopter model exists
if [ ! -f "$WORKSPACE_DIR/src/hexacopter_control/models/hexacopter_agricultural/model.sdf" ]; then
    echo "⚠️  Hexacopter model missing! Run setup_assets.sh first."
    exit 1
fi

# --- FIX: Symlink World ---
rm "$PX4_DIR/Tools/simulation/gz/worlds/bihar_farm.sdf" 2>/dev/null
rm "$PX4_DIR/Tools/simulation/gz/worlds/bihar_farm.sdf.sdf" 2>/dev/null
ln -sf "$WORKSPACE_DIR/src/hexacopter_control/worlds/bihar_farm.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/bihar_farm.sdf"

LAUNCH_CMD="env -u LD_LIBRARY_PATH -u LD_PRELOAD -u GTK_PATH -u GIO_EXTRA_MODULES gnome-terminal --window"

# 1. Micro-XRCE-DDS Agent (New Window)
$LAUNCH_CMD --title="Micro-XRCE-DDS Agent" --geometry=100x24+0+0 -- bash -c "
echo 'Starting Agent...';
MicroXRCEAgent udp4 -p 8888; 
exec bash"

# 2. PX4 SITL (New Window)
$LAUNCH_CMD --title="PX4 SITL" --geometry=80x24+0+350 -- bash -c "
echo 'Starting PX4 SITL...';
source /opt/ros/jazzy/setup.bash;
cd $PX4_DIR;
export HEADLESS=0;

# Tell Gazebo where to find the custom Hexacopter & Bihar Farm
export GZ_SIM_RESOURCE_PATH=$WORKSPACE_DIR/src/hexacopter_control/models:$WORKSPACE_DIR/src/hexacopter_control/worlds:\$GZ_SIM_RESOURCE_PATH
export PX4_GZ_WORLD=bihar_farm
export PX4_GZ_MODEL=hexacopter_agricultural

# Set Location to Munger, Bihar
export PX4_HOME_LAT=25.3748
export PX4_HOME_LON=86.4735
export PX4_HOME_ALT=45.0

unset LD_LIBRARY_PATH; sleep 5
PX4_SYS_AUTOSTART=4001 make px4_sitl gz_x500 -j2;
exec bash"

# 3. QGroundControl (New Window)
if [ ! -f "$WORKSPACE_DIR/QGroundControl.AppImage" ]; then
    echo "⚠️ QGroundControl not found. Downloading automatically..."
    wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.2/QGroundControl.AppImage -O "$WORKSPACE_DIR/QGroundControl.AppImage"
    
    if [ $? -eq 0 ]; then
        chmod +x "$WORKSPACE_DIR/QGroundControl.AppImage"
    else
        echo "❌ Download failed. QGroundControl will not start."
    fi
fi

$LAUNCH_CMD --title="QGroundControl" --geometry=80x24+600+0 -- bash -c "
unset LD_LIBRARY_PATH; unset LD_PRELOAD; unset GTK_PATH; unset GIO_EXTRA_MODULES;
if [ -f \"$WORKSPACE_DIR/QGroundControl.AppImage\" ]; then
    echo 'Waiting for PX4 to initialize MAVLink...'; sleep 10;
    echo 'Starting QGC...';
    $WORKSPACE_DIR/QGroundControl.AppImage; 
else
    echo '❌ QGC file missing. Check internet connection.';
fi
exec bash"

# 4. ROS 2 Mission (New Window)
$LAUNCH_CMD --title="ROS 2 Mission" --geometry=80x24+600+350 -- bash -c "
source /opt/ros/jazzy/setup.bash;
source $WORKSPACE_DIR/install/setup.bash;
cd $WORKSPACE_DIR;

echo '--- Launching Autonomous Mission (Z-Pattern) ---';
for i in {15..1}; do echo \"⏳ Mission Start in \$i...\"; sleep 1; done;
python3 src/hexacopter_control/hexacopter_control/mission_sprayer.py;
exec bash"

echo "✅ System launched! The drone will take off in ~15 seconds."