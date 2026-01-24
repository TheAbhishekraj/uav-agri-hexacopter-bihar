import os
import re

workspace_root = os.path.expanduser("~/uav_agricultural_drone_project")
src_dir = os.path.join(workspace_root, "src")
ignore_dirs = [".git", "build", "install", "log", ".vscode", "__pycache__"]

print(f"🔍 Scanning workspace: {workspace_root}")
print(f"   Target source directory: {src_dir}\n")

found_issues = False

for root, dirs, files in os.walk(workspace_root):
    # Skip ignored directories
    dirs[:] = [d for d in dirs if d not in ignore_dirs]
    
    if "package.xml" in files:
        full_path = os.path.join(root, "package.xml")
        
        # Check if this package is inside src/
        if not root.startswith(src_dir):
            # Check if it is ignored by COLCON_IGNORE in any parent directory up to workspace_root
            is_ignored = False
            curr = root
            while curr.startswith(workspace_root) and curr != workspace_root:
                if os.path.exists(os.path.join(curr, "COLCON_IGNORE")):
                    is_ignored = True
                    break
                curr = os.path.dirname(curr)
            
            if is_ignored:
                continue

            print(f"❌ WARNING: Found package outside src/: {full_path}")
            found_issues = True
            print("   -> Run './clean_build.sh' to ignore virtual environments.")
        else:
            print(f"✅ Found valid package: {os.path.relpath(full_path, workspace_root)}")
            
            # --- AUDIT CHECK: Build Type Consistency ---
            try:
                with open(full_path, 'r') as f:
                    content = f.read()
                    
                has_setup_py = os.path.exists(os.path.join(root, "setup.py"))
                has_cmakelists = os.path.exists(os.path.join(root, "CMakeLists.txt"))
                
                if has_setup_py:
                    # Robust check for ament_python ignoring whitespace
                    if not re.search(r"<build_type>\s*ament_python\s*</build_type>", content):
                        print(f"   ❌ ERROR: {os.path.basename(root)} has setup.py but package.xml is NOT ament_python!")
                        found_issues = True
                    if has_cmakelists:
                        print(f"   ⚠️  WARNING: {os.path.basename(root)} has both setup.py and CMakeLists.txt. This is ambiguous.")
                        found_issues = True
                    
                    # Check for Resource Marker (Critical for ROS 2 Python)
                    # Extract real package name
                    name_match = re.search(r"<name>\s*(.*?)\s*</name>", content)
                    pkg_name = name_match.group(1) if name_match else os.path.basename(root)
                    
                    resource_marker = os.path.join(root, "resource", pkg_name)
                    if not os.path.exists(resource_marker):
                        print(f"   ❌ ERROR: {pkg_name} missing resource marker at 'resource/{pkg_name}'. ROS 2 won't find this node!")
                        found_issues = True
                        
            except Exception as e:
                print(f"   ⚠️  Could not read package.xml: {e}")
            # -------------------------------------------

if not found_issues:
    print("\n✅ Workspace structure looks clean. Only src/ contains active packages.")
else:
    print("\n⚠️  Issues found. Please review warnings above.")