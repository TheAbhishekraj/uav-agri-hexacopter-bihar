import os

# Define paths
base_path = os.path.expanduser("~/uav_agricultural_drone_project")
install_path = os.path.join(base_path, "install")
packages = ["hexacopter_control", "yolov8_detection"]

print(f"🔍 Inspecting Install Directory: {install_path}\n")

if not os.path.exists(install_path):
    print("❌ CRITICAL: 'install' directory missing. You must run 'colcon build'.")
    exit(1)

for pkg in packages:
    pkg_path = os.path.join(install_path, pkg)
    
    # 1. Check if package folder exists
    if not os.path.exists(pkg_path):
        print(f"❌ {pkg}: Folder missing in install/. Build failed or skipped.")
        continue
        
    # 2. Check for share directory (where package.xml lives)
    share_path = os.path.join(pkg_path, "share", pkg)
    if not os.path.exists(os.path.join(share_path, "package.xml")):
        print(f"❌ {pkg}: package.xml missing in install/share. Installation incomplete.")
        continue

    # 3. Check for Ament Index Marker (Crucial for 'Package not found')
    marker_path = os.path.join(pkg_path, "share", "ament_index", "resource_index", "packages", pkg)
    if os.path.exists(marker_path):
        print(f"✅ {pkg}: Correctly installed and indexed.")
    else:
        print(f"❌ {pkg}: Resource marker missing! ROS 2 cannot find this package.")
        print(f"   -> Check if 'resource/{pkg}' exists in your src folder.")

print("\n💡 If you see any ❌, run: colcon build --symlink-install")