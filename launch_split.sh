#!/bin/bash

# Bihar Agricultural Drone - Launch Script
# Handles environment cleanup and multi-terminal launch

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

# --- FIX: Sanitize Environment ---
# Unset LD_LIBRARY_PATH to prevent gnome-terminal "symbol lookup error" (GLIBC conflict)
unset LD_LIBRARY_PATH
unset LD_PRELOAD
unset GTK_PATH
unset GIO_EXTRA_MODULES

# Deactivate virtual environment if present
if [[ "$VIRTUAL_ENV" != "" ]]; then
    PATH=${PATH/$VIRTUAL_ENV\/bin:/}
    unset VIRTUAL_ENV
fi

# 1. Cleanup
echo "🧹 Cleaning up previous sessions..."
killall -9 gz_sim px4 MicroXRCEAgent ruby QGroundControl flight_controller detection_node mission_commander 2>/dev/null

# 2. Check for QGroundControl
if [ ! -f "./QGroundControl.AppImage" ]; then
    echo "⚠️  QGroundControl.AppImage not found in project root!"
else
    chmod +x ./QGroundControl.AppImage
fi

# --- FIX: Symlink World to PX4 ---
# We symlink the custom world to PX4's folder so we can refer to it by name (avoiding path errors)
source ./setup_project_env.sh

# Check for outdated world file (missing crop rows or old sensors config)
if [ -f "$HOME/uav_agricultural_drone_project/src/hexacopter_control/worlds/bihar_farm.sdf" ] && ! grep -q "gz-sim-sensors-system" "$HOME/uav_agricultural_drone_project/src/hexacopter_control/worlds/bihar_farm.sdf"; then
    echo "⚠️  World file outdated (missing sensor plugins). Deleting to regenerate..."
    rm "$HOME/uav_agricultural_drone_project/src/hexacopter_control/worlds/bihar_farm.sdf"
fi

# Ensure world file exists before symlinking
if [ ! -f "$HOME/uav_agricultural_drone_project/src/hexacopter_control/worlds/bihar_farm.sdf" ]; then
    echo "⚠️  World file missing. Running setup_assets.sh..."
    ./setup_assets.sh
fi

# Ensure hexacopter model exists
if [ ! -f "$HOME/uav_agricultural_drone_project/src/hexacopter_control/models/hexacopter_agricultural/model.sdf" ]; then
    echo "⚠️  Hexacopter model missing! Please ensure model.sdf is in the project root and run setup_assets.sh"
    exit 1
fi

if [ -d "$PX4_DIR/Tools/simulation/gz/worlds" ]; then
    rm "$PX4_DIR/Tools/simulation/gz/worlds/bihar_farm.sdf" 2>/dev/null
    rm "$PX4_DIR/Tools/simulation/gz/worlds/bihar_farm.sdf.sdf" 2>/dev/null
    cp "$HOME/uav_agricultural_drone_project/src/hexacopter_control/worlds/bihar_farm.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/bihar_farm.sdf"
fi

# 3. Launch Components in Gnome Terminal Tabs
# Note: We explicitly unset VIRTUAL_ENV to avoid Gazebo crashes (See Learning Journey #5)

echo "🚀 Launching Bihar Agricultural Drone System..."

# Note: Updated syntax to avoid deprecated --command warning. 
# We use '--' to separate arguments for the command executed inside the tab.
# Splitting into separate windows to prevent tab loading errors in Snap environments.

gnome-terminal --window --title="1. Micro-XRCE-DDS" --geometry=80x24+0+0 -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"

gnome-terminal --window --title="2. PX4 SITL (Bihar)" --geometry=80x24+0+350 -- bash -c "source ./setup_project_env.sh; cd \$PX4_DIR; export GZ_SIM_RESOURCE_PATH=\$HOME/uav_agricultural_drone_project/src/hexacopter_control/models:\$HOME/uav_agricultural_drone_project/src/hexacopter_control/worlds:\$GZ_SIM_RESOURCE_PATH; export PX4_GZ_WORLD=bihar_farm; export PX4_SIM_MODEL=hexacopter_agricultural; export PX4_HOME_LAT=25.3748; export PX4_HOME_LON=86.4735; export PX4_HOME_ALT=45.0; unset LD_LIBRARY_PATH; make px4_sitl gz_x500 -j2; exec bash"

gnome-terminal --window --title="3. QGroundControl" --geometry=80x24+600+0 -- bash -c "echo 'Waiting for PX4 to boot...'; sleep 10; ./QGroundControl.AppImage; exec bash"

MISSION_SCRIPT="src/hexacopter_control/hexacopter_control/mission_sprayer.py"
if [ ! -f "$MISSION_SCRIPT" ]; then
    # Fallback to root if not found in package
    MISSION_SCRIPT="mission_sprayer.py"
fi

gnome-terminal --window --title="4. ROS 2 Mission" --geometry=80x24+600+350 -- bash -c "for i in {15..1}; do echo \"⏳ Mission Start in \$i...\"; sleep 1; done; source /opt/ros/jazzy/setup.bash; source install/setup.bash; python3 $MISSION_SCRIPT; exec bash"

echo "✅ Launch sequence initiated."
echo "   - Window 1: DDS Agent (Bridge)"
echo "   - Window 2: PX4 SITL (Simulation)"
echo "   - Window 3: QGroundControl"
echo "   - Window 4: Mission Commander (Counting down...)"