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

echo "🛑 Killing previous sessions..."
pkill -f MicroXRCEAgent
pkill -f px4
pkill -f gz-sim

# Ensure world file exists before symlinking
if [ ! -f "$WORKSPACE_DIR/src/hexacopter_control/worlds/bihar_farm.sdf" ]; then
    echo "⚠️  World file missing. Running setup_assets.sh..."
    $WORKSPACE_DIR/setup_assets.sh
fi

# --- FIX: Symlink World ---
rm "$PX4_DIR/Tools/simulation/gz/worlds/bihar_farm.sdf" 2>/dev/null
cp "$WORKSPACE_DIR/src/hexacopter_control/worlds/bihar_farm.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/bihar_farm.sdf"

# 1. Micro-XRCE-DDS Agent
gnome-terminal --window --title="Micro-XRCE-DDS Agent" -- bash -c "
echo 'Starting Agent...';
MicroXRCEAgent udp4 -p 8888; 
exec bash"

# 2. PX4 SITL (New Window)
gnome-terminal --window --title="PX4 SITL" --geometry=80x24+0+350 -- bash -c "
echo 'Starting PX4 SITL...';
source /opt/ros/jazzy/setup.bash;
cd $PX4_DIR;

# Export paths INSIDE the new terminal session
export GZ_SIM_RESOURCE_PATH=$WORKSPACE_DIR/src/hexacopter_control/models:$WORKSPACE_DIR/src/hexacopter_control/worlds:$GZ_SIM_RESOURCE_PATH
export PX4_GZ_WORLD=bihar_farm
export PX4_SIM_MODEL=gz_x500
unset LD_LIBRARY_PATH
make px4_sitl gz_x500 -j2; 
exec bash"

echo "✅ Simulation running. Now press F5 in VS Code to debug your node."