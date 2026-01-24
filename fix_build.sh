#!/bin/bash

WORKSPACE_DIR=~/uav_agricultural_drone_project
SRC_DIR=$WORKSPACE_DIR/src

echo "🧹 Starting Level 1 Cleanup: Enforcing ament_python..."

# Find all directories in src that have a setup.py (Implying they are Python packages)
find "$SRC_DIR" -name "setup.py" -print0 | while IFS= read -r -d '' setup_file; do
    pkg_dir=$(dirname "$setup_file")
    
    # Extract accurate package name from package.xml
    if [ -f "$pkg_dir/package.xml" ]; then
        pkg_name=$(grep "<name>" "$pkg_dir/package.xml" | sed -e 's/.*<name>\(.*\)<\/name>.*/\1/' | tr -d '[:space:]')
    else
        pkg_name=$(basename "$pkg_dir")
    fi
    
    echo "   🔧 Fixing Python Package: $pkg_name"

    # 1. Remove stray CMakeLists.txt (The "Wood Glue")
    if [ -f "$pkg_dir/CMakeLists.txt" ]; then
        echo "      - Removing conflicting CMakeLists.txt"
        rm "$pkg_dir/CMakeLists.txt"
    fi

    # 2. Fix package.xml build_type (The "Sticker")
    if [ -f "$pkg_dir/package.xml" ]; then
        # Replace ament_cmake with ament_python
        if grep -q "ament_cmake" "$pkg_dir/package.xml"; then
            echo "      - Updating package.xml to ament_python"
            sed -i 's/ament_cmake/ament_python/g' "$pkg_dir/package.xml"
        fi
    fi

    # 3. Create Resource Marker (The "Flag")
    # ROS 2 needs an empty file in resource/<pkg_name> to find the package at runtime
    if [ ! -d "$pkg_dir/resource" ]; then
        echo "      - Creating resource directory"
        mkdir -p "$pkg_dir/resource"
    fi
    
    if [ ! -f "$pkg_dir/resource/$pkg_name" ]; then
        echo "      - Creating resource marker: resource/$pkg_name"
        touch "$pkg_dir/resource/$pkg_name"
    fi

done

echo "✨ Cleanup Complete."
echo "👉 Now run: ./verify_workspace.py"
echo "👉 Then run: colcon build --symlink-install"