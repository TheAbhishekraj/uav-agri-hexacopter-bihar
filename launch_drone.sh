#!/bin/bash

# Project Paths
WORKSPACE_DIR=~/uav_agricultural_drone_project
PX4_DIR=~/DronePhD_Lab/PX4/PX4-Autopilot

# --- FIX: Sanitize Environment ---
unset LD_LIBRARY_PATH
unset LD_PRELOAD
unset GTK_PATH
unset GIO_EXTRA_MODULES

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

# Ensure world file exists before symlinking
if [ ! -f "$WORKSPACE_DIR/src/hexacopter_control/worlds/bihar_farm.sdf" ]; then
    echo "⚠️  World file missing. Running setup_assets.sh..."
    $WORKSPACE_DIR/setup_assets.sh
fi

# --- FIX: Symlink World ---
rm "$PX4_DIR/Tools/simulation/gz/worlds/bihar_farm.sdf" 2>/dev/null
cp "$WORKSPACE_DIR/src/hexacopter_control/worlds/bihar_farm.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/bihar_farm.sdf"

# 1. Micro-XRCE-DDS Agent (New Window)
gnome-terminal --window --title="Micro-XRCE-DDS Agent" --geometry=100x24+0+0 -- bash -c "
echo 'Starting Agent...';
MicroXRCEAgent udp4 -p 8888; 
exec bash"

# 2. PX4 SITL (New Window)
gnome-terminal --window --title="PX4 SITL" --geometry=80x24+0+350 -- bash -c "
echo 'Starting PX4 SITL...';
source /opt/ros/jazzy/setup.bash;
cd $PX4_DIR;
export HEADLESS=0;

# Tell Gazebo where to find the custom Hexacopter & Bihar Farm
export GZ_SIM_RESOURCE_PATH=$WORKSPACE_DIR/src/hexacopter_control/models:$WORKSPACE_DIR/src/hexacopter_control/worlds:$GZ_SIM_RESOURCE_PATH
export PX4_GZ_WORLD=bihar_farm
export PX4_SIM_MODEL=gz_x500

# Set Location to Munger, Bihar
export PX4_HOME_LAT=25.3748
export PX4_HOME_LON=86.4735
export PX4_HOME_ALT=45.0

unset LD_LIBRARY_PATH
make px4_sitl gz_x500 -j2;
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

gnome-terminal --window --title="QGroundControl" --geometry=80x24+600+0 -- bash -c "
if [ -f \"$WORKSPACE_DIR/QGroundControl.AppImage\" ]; then
    echo 'Starting QGC...';
    $WORKSPACE_DIR/QGroundControl.AppImage; 
else
    echo '❌ QGC file missing. Check internet connection.';
fi
exec bash"

# 4. ROS 2 Mission (New Window)
gnome-terminal --window --title="ROS 2 Mission" --geometry=80x24+600+350 -- bash -c "
source /opt/ros/jazzy/setup.bash;
source $WORKSPACE_DIR/install/setup.bash;
cd $WORKSPACE_DIR;

echo '--- Launching Autonomous Mission (Z-Pattern) ---';
for i in {15..1}; do echo \"⏳ Mission Start in \$i...\"; sleep 1; done;
python3 src/hexacopter_control/hexacopter_control/mission_sprayer.py;
exec bash"

echo "✅ System launched! The drone will take off in ~15 seconds."