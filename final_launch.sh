#!/bin/bash

# =================================================================
# Final System Launch & Data Recording Script
# Project: uav-agri-hexacopter-bihar
# =================================================================

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$PROJECT_ROOT"

# 1. Environment Sanitization & Setup
source ./setup_project_env.sh

# 2. Cleanup & Directory Prep
echo "🧹 Cleaning up previous sessions..."
pkill -9 -f "gz_sim|px4|MicroXRCEAgent|ruby|QGroundControl|mission_commander" 2>/dev/null
mkdir -p "$PROJECT_ROOT/mission_logs"

# 3. Asset Verification
echo "🏗️  Verifying simulation assets..."
./setup_assets.sh

# --- FIX: Sync World to PX4 ---
if [ -d "$PX4_DIR/Tools/simulation/gz/worlds" ]; then
    echo "   🔗 Linking bihar_farm.sdf to PX4 worlds directory..."
    rm "$PX4_DIR/Tools/simulation/gz/worlds/bihar_farm.sdf" 2>/dev/null
    ln -sf "$PROJECT_ROOT/src/hexacopter_control/worlds/bihar_farm.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/bihar_farm.sdf"
fi

# 4. Launch Multi-Terminal System
echo "🚀 Launching Full System..."

# --- CRITICAL FIX: gnome-terminal GLIBC conflict ---
# We use 'env -u' to ensure the terminal starts with a clean system environment
# to avoid the GLIBC_PRIVATE symbol lookup error.
LAUNCH_CMD="env -u LD_LIBRARY_PATH -u LD_PRELOAD -u GTK_PATH -u GIO_EXTRA_MODULES gnome-terminal --window"

# Window 1: DDS Agent
$LAUNCH_CMD --title="1. Micro-XRCE-DDS" -- bash -c "source ./setup_project_env.sh; MicroXRCEAgent udp4 -p 8888; exec bash"

# Window 2: PX4 SITL (Bihar Farm)
$LAUNCH_CMD --title="2. PX4 SITL (Bihar)" -- bash -c "source ./setup_project_env.sh; export GZ_VERSION=harmonic; export GZ_SIM_RESOURCE_PATH=\"$PROJECT_ROOT/src/hexacopter_control/models:$PROJECT_ROOT/src/hexacopter_control/worlds:\$PX4_DIR/Tools/simulation/gz/worlds:\$GZ_SIM_RESOURCE_PATH\"; cd \$PX4_DIR; export PX4_GZ_WORLD=bihar_farm; export PX4_GZ_MODEL=hexacopter_agricultural; export PX4_HOME_LAT=25.3748; export PX4_HOME_LON=86.4735; export PX4_HOME_ALT=45.0; unset LD_LIBRARY_PATH; export PX4_SYS_AUTOSTART=4001; make px4_sitl gz_x500 -j2; exec bash"

# Window 3: QGroundControl
$LAUNCH_CMD --title="3. QGroundControl" -- bash -c "unset LD_LIBRARY_PATH; echo 'Waiting for PX4 to boot...'; sleep 10; ./QGroundControl.AppImage; exec bash"

# Window 4: Mission & Recording
LOG_NAME="mission_logs/log_$(date +%Y%m%d_%H%M%S)"
$LAUNCH_CMD --title="4. Mission & Recording" -- bash -c "
echo '⏳ Waiting for system stabilization...';
sleep 15;
echo '⏺️  Starting Rosbag Recording...';
source /opt/ros/jazzy/setup.bash;
source install/setup.bash;
ros2 bag record -o $LOG_NAME \
    /diagnostics \
    /fmu/out/vehicle_status \
    /fmu/out/vehicle_global_position \
    /fmu/out/vehicle_local_position \
    /fmu/out/sensor_combined \
    /fmu/out/battery_status \
    /fmu/out/vehicle_gps_position &
RECORD_PID=\$!;
MISSION_SCRIPT='$PROJECT_ROOT/src/hexacopter_control/hexacopter_control/mission_sprayer.py'
if [ ! -f \"\$MISSION_SCRIPT\" ]; then
    MISSION_SCRIPT='$PROJECT_ROOT/mission_sprayer.py'
fi
python3 \$MISSION_SCRIPT;
kill \$RECORD_PID;
exec bash"

echo "✅ System is live. Data is being recorded to $PROJECT_ROOT/mission_logs"