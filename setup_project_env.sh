#!/bin/bash
# setup_project_env.sh
# Connects the uav_agricultural_drone_project to the central DronePhD_Lab tools.
# Usage: source ./setup_project_env.sh

# Get the directory where this script is located
ENV_PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 1. Verify ROS 2: Check if ROS 2 Jazzy is active. If not, source it.
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        echo "🔄 ROS 2 not detected. Sourcing Jazzy..."
        source /opt/ros/jazzy/setup.bash
    else
        echo "❌ Error: ROS 2 Jazzy installation not found at /opt/ros/jazzy/setup.bash"
        return 1 2>/dev/null || exit 1
    fi
fi

if [ "$ROS_DISTRO" = "jazzy" ]; then
    echo "✅ ROS 2 Jazzy is active."
else
    echo "⚠️  Warning: Current ROS_DISTRO is '$ROS_DISTRO'. Expected 'jazzy'."
fi

# 2. Verify PX4 Path: Check if ~/DronePhD_Lab/PX4/PX4-Autopilot exists.
LAB_PX4_PATH="$HOME/DronePhD_Lab/PX4/PX4-Autopilot"
if [ ! -d "$LAB_PX4_PATH" ]; then
    echo "❌ Error: Central PX4 installation missing at $LAB_PX4_PATH"
    return 1 2>/dev/null || exit 1
fi

# 3. Prevent Duplicates: Check if there is a PX4-Autopilot folder inside my current project.
if [ -d "$ENV_PROJECT_ROOT/PX4-Autopilot" ]; then
    echo "⚠️  WARNING: Duplicate PX4-Autopilot folder detected in this project!"
    echo "   Please remove '$ENV_PROJECT_ROOT/PX4-Autopilot' to ensure you use the Lab version."
fi

# 4. Set Variables: Export the PX4_DIR variable.
export PX4_DIR="$LAB_PX4_PATH"
echo "✅ PX4_DIR set to: $PX4_DIR"

# 5. Gazebo Resource Paths
export GZ_SIM_RESOURCE_PATH="$ENV_PROJECT_ROOT/src/hexacopter_control/models:$ENV_PROJECT_ROOT/src/hexacopter_control/worlds:$GZ_SIM_RESOURCE_PATH"
export GAZEBO_MODEL_PATH="$ENV_PROJECT_ROOT/src/hexacopter_control/models:$GAZEBO_MODEL_PATH"

echo "🚀 Project environment configured successfully."