import os

packages = ['hexacopter_camera', 'hexacopter_control', 'yolov8_detection', 'sensor_fusion']
base_path = os.path.expanduser('~/uav_agricultural_drone_project/src')

for pkg in packages:
    pkg_path = os.path.join(base_path, pkg)
    if not os.path.exists(pkg_path): continue
    
    # 1. Force fix package.xml
    xml_path = os.path.join(pkg_path, 'package.xml')
    with open(xml_path, 'r') as f:
        content = f.read()
    
    # Crucial change: Force build_type to ament_python
    content = content.replace('ament_cmake', 'ament_python')
    
    with open(xml_path, 'w') as f:
        f.write(content)

    # 2. Delete any residual CMakeLists.txt if they exist in Python folders
    cmake_file = os.path.join(pkg_path, 'CMakeLists.txt')
    if os.path.exists(cmake_file):
        os.remove(cmake_file)
        print(f"Removed conflicting CMake file from {pkg}")

    print(f"✅ {pkg} converted to ament_python")

print("\nAll packages updated. Now delete your build folder and try again.")