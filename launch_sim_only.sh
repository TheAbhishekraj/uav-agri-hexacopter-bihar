#!/bin/bash

# Project Paths
PX4_DIR=~/PX4-Autopilot

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

# 1. Micro-XRCE-DDS Agent
gnome-terminal --window --title="Micro-XRCE-DDS Agent" -- bash -c "
echo 'Starting Agent...';
MicroXRCEAgent udp4 -p 8888; 
exec bash"

# 2. PX4 SITL
gnome-terminal --tab --title="PX4 SITL" -- bash -c "
echo 'Starting PX4 SITL...';
cd $PX4_DIR;
export HEADLESS=0;
make px4_sitl gz_x500_depth; 
exec bash"

echo "✅ Simulation running. Now press F5 in VS Code to debug your node."