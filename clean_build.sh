#!/bin/bash

# 1. Ignore the virtual environment for colcon
if [ -d "hexacopter-env" ]; then
    touch hexacopter-env/COLCON_IGNORE
    echo "Ignored hexacopter-env for colcon."
fi

if [ -d ".venv" ]; then
    touch .venv/COLCON_IGNORE
    echo "Ignored .venv for colcon."
fi

# 2. Clean previous build artifacts to remove cached paths
echo "Cleaning build artifacts..."
rm -rf build/ install/ log/

# 3. Build with symlink install
echo "Building workspace..."
# We use --base-paths to explicitly tell colcon where to look, 
# avoiding the 'Duplicate package names' error if run from a parent dir.
colcon build --symlink-install --base-paths src

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo "Source the workspace: source install/setup.bash"
else
    echo "❌ Build failed."
fi