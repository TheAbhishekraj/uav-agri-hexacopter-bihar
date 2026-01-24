#!/bin/bash
echo "🏥 System Health Check"
echo "----------------------"

# Check Agent
if pgrep -x "MicroXRCEAgent" > /dev/null; then
    echo "✅ Micro-XRCE-DDS Agent: RUNNING"
else
    echo "❌ Micro-XRCE-DDS Agent: NOT RUNNING"
fi

# Check Gazebo
if pgrep -f "gz sim" > /dev/null || pgrep -x "ruby" > /dev/null; then
    echo "✅ Gazebo Simulation: RUNNING"
else
    echo "❌ Gazebo Simulation: NOT RUNNING"
fi

# Check QGC
if pgrep -f "QGroundControl" > /dev/null; then
    echo "✅ QGroundControl: RUNNING"
else
    echo "⚠️  QGroundControl: NOT RUNNING (Optional)"
fi

# Check ROS 2 Nodes
if pgrep -f "mission_commander" > /dev/null; then
    echo "✅ Mission Commander: RUNNING"
else
    echo "⏳ Mission Commander: WAITING (or not running)"
fi

echo "----------------------"
echo "💡 Tip: If Gazebo is black/empty, run 'unset VIRTUAL_ENV' before launching."