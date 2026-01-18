#!/bin/bash

echo "🚀 Preparing Agricultural Drone Product Release..."

# 1. Register all nodes
echo "   🔗 Registering Nodes..."
python3 register_planner.py
python3 register_decision_node.py
python3 register_spray_node.py
python3 register_watchdog_node.py
python3 register_mission_node.py

# 2. Setup Assets
echo "   📦 Setting up Assets..."
./setup_assets.sh
./setup_tflite.sh
./setup_field_bihar.sh

# 3. Final Build
echo "   🔨 Building Release Version..."
./clean_build.sh

echo "✅ Product Ready! Run './launch_split.sh' to start the full system."
echo "   (Or use 'docker build -t agri_drone .' to containerize)"