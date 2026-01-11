#!/bin/bash

# 1. Enforce Build Type (Requirement 1)
echo "🔧 Enforcing ament_python build type..."
python3 ~/uav_agricultural_drone_project/force_fix_python.py

# 2. Workspace Sourcing (Requirement 2)
if ! grep -q "source ~/uav_agricultural_drone_project/install/setup.bash" ~/.bashrc; then
    echo "source ~/uav_agricultural_drone_project/install/setup.bash" >> ~/.bashrc
    echo "✅ Added workspace sourcing to ~/.bashrc"
else
    echo "✅ Workspace sourcing already in ~/.bashrc"
fi

echo "✅ Setup complete. Please restart your terminal or run 'source ~/.bashrc'"