#!/bin/bash
# remediate_structure.sh
# Moves files to the new workspace and installs dependencies

SRC_OLD=~/uav_agricultural_drone_project
WS_NEW=~/agri_hexacopter_ws/src/agri_hexacopter

echo "🚀 Starting Remediation & Migration..."

# 1. Move Python Nodes
mkdir -p $WS_NEW/nodes
if [ -f "$SRC_OLD/src/hexacopter_control/hexacopter_control/mission_sprayer.py" ]; then
    cp "$SRC_OLD/src/hexacopter_control/hexacopter_control/mission_sprayer.py" "$WS_NEW/nodes/"
    chmod +x "$WS_NEW/nodes/mission_sprayer.py"
    echo "✅ Moved mission_sprayer.py to nodes/"
fi
# Ensure __init__.py exists for the 'nodes' package defined in setup.py
touch "$WS_NEW/nodes/__init__.py"

# 2. Move Launch Files
mkdir -p $WS_NEW/launch
if [ -d "$SRC_OLD/launch" ]; then
    cp "$SRC_OLD/launch/"*.launch.py "$WS_NEW/launch/" 2>/dev/null
    echo "✅ Migrated existing launch files"
fi

# 3. Move Simulation Assets
mkdir -p $WS_NEW/models $WS_NEW/worlds
if [ -d "$SRC_OLD/models" ]; then
    cp -r "$SRC_OLD/models/"* "$WS_NEW/models/" 2>/dev/null
    echo "✅ Migrated models"
fi
if [ -d "$SRC_OLD/worlds" ]; then
    cp "$SRC_OLD/worlds/"*.world "$WS_NEW/worlds/" 2>/dev/null
    echo "✅ Migrated worlds"
fi

# 4. Install Dependencies (Fixing the NumPy/TensorFlow issue)
echo "📦 Installing AI Dependencies..."
pip3 install "numpy<2" "tensorflow" "ultralytics"

echo "🎉 Remediation Complete. Run 'colcon build' in ~/agri_hexacopter_ws"