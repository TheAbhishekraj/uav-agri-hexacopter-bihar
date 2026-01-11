import os

base_dir = "/home/abhishek/uav_agricultural_drone_project/src"
target_folders = ["hexacopter_camera", "hexacopter_control", "yolov8_detection", "sensor_fusion"]

print("--- Scanning Python Nodes for C++ Artifacts ---")

for folder in target_folders:
    folder_path = os.path.join(base_dir, folder)
    
    # Check if folder exists
    if not os.path.exists(folder_path):
        print(f"Skipping {folder}: Directory not found.")
        continue

    # 1. Identify and Delete CMakeLists.txt
    cmake_file = os.path.join(folder_path, "CMakeLists.txt")
    if os.path.exists(cmake_file):
        os.remove(cmake_file)
        print(f"[{folder}] DELETED: CMakeLists.txt (Conflicting build file)")
    else:
        print(f"[{folder}] OK: No CMakeLists.txt found.")

    # 2. Verify setup.py and package.xml presence
    has_setup = os.path.exists(os.path.join(folder_path, "setup.py"))
    xml_path = os.path.join(folder_path, "package.xml")
    has_xml = os.path.exists(xml_path)

    if has_setup and has_xml:
        print(f"[{folder}] VERIFIED: setup.py and package.xml are present.")
        
        # 3. Verify build_type is ament_python
        with open(xml_path, 'r') as f:
            content = f.read()
            if "<build_type>ament_python</build_type>" in content:
                 print(f"[{folder}] VERIFIED: build_type is ament_python.")
            else:
                 print(f"[{folder}] ERROR: package.xml does not contain <build_type>ament_python</build_type>!")
    else:
        if not has_setup:
            print(f"[{folder}] ERROR: Missing setup.py!")
        if not has_xml:
            print(f"[{folder}] ERROR: Missing package.xml!")

print("\nCleanup complete. You can now run 'colcon build'.")