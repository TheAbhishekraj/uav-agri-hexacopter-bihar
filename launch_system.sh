#!/bin/bash
echo "🚀 Launching Agricultural Drone System..."

# Terminal 1: Gazebo + PX4
gnome-terminal -- bash -c "
cd ~/PX4-Autopilot && 
make px4_sitl_default gazebo &
sleep 5 &&
./build/px4_sitl_default/bin/px4;
exec bash
"

# Terminal 2: ROS 2
gnome-terminal -- bash -c "
cd $(dirname $0) &&
source install/setup.bash &&
ros2 launch launch full_system.launch.py;
exec bash
"

echo "✅ System launched! Monitor all terminals."
