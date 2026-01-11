#!/bin/bash
echo "🚁 DRONE SYSTEM HEALTH CHECK 🚁"

# Check 1: Is the Bridge Running?
if pgrep -f "MicroXRCEAgent" > /dev/null; then echo "✅ Agent: RUNNING"; else echo "❌ Agent: STOPPED"; fi

# Check 2: Is the Simulator Running?
if pgrep -f "gz-sim" > /dev/null; then echo "✅ Gazebo: RUNNING"; else echo "❌ Gazebo: STOPPED"; fi

# Check 3: Is QGC Running?
if pgrep -f "QGroundControl" > /dev/null; then echo "✅ QGC: RUNNING"; else echo "❌ QGC: STOPPED"; fi

# Check 4: Can we talk to the Drone?
source /opt/ros/jazzy/setup.bash
if [ -f "install/setup.bash" ]; then source install/setup.bash; fi

if timeout 2s ros2 topic hz /fmu/out/vehicle_odometry > /dev/null; then 
    echo "✅ ROS 2 Connection: EXCELLENT"; 
else 
    echo "❌ ROS 2 Connection: FAILED (Check Agent)"; 
fi