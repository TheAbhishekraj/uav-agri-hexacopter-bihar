#!/bin/bash

BASE_DIR="/home/abhishek/uav_agricultural_drone_project/src"

# Function to setup python package
setup_pkg() {
    PKG_NAME=$1
    PKG_DIR="$BASE_DIR/$PKG_NAME"
    
    echo "Configuring $PKG_NAME..."
    
    # 1. Fix Directory Structure (Move scripts into module folder)
    if [ ! -d "$PKG_DIR/$PKG_NAME" ]; then
        echo "  Creating module directory $PKG_DIR/$PKG_NAME..."
        mkdir -p "$PKG_DIR/$PKG_NAME"
        touch "$PKG_DIR/$PKG_NAME/__init__.py"
    fi

    # Move all .py files except setup.py into the module directory
    find "$PKG_DIR" -maxdepth 1 -name "*.py" ! -name "setup.py" -exec mv {} "$PKG_DIR/$PKG_NAME/" \;

    # Create setup.cfg
    cat > "$PKG_DIR/setup.cfg" << EOF
[develop]
script_dir=\$base/lib/$PKG_NAME
[install]
install_scripts=\$base/lib/$PKG_NAME
EOF

    # Create resource directory and marker
    mkdir -p "$PKG_DIR/resource"
    touch "$PKG_DIR/resource/$PKG_NAME"
    
    # Ensure package.xml uses ament_python
    if [ -f "$PKG_DIR/package.xml" ]; then
        if grep -q "ament_cmake" "$PKG_DIR/package.xml"; then
            echo "  Switching package.xml to ament_python..."
            sed -i 's/ament_cmake/ament_python/g' "$PKG_DIR/package.xml"
        fi
    fi
}

# Setup Python packages
setup_pkg "hexacopter_camera"
setup_pkg "hexacopter_control"
setup_pkg "yolov8_detection"
setup_pkg "sensor_fusion"

echo "✅ Build structure fixed."
echo "⚠️  Run these commands to rebuild:"
echo "  rm -rf build/ install/ log/"
echo "  colcon build --symlink-install"