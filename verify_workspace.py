import os

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

if not found_issues:
    print("\n✅ Workspace structure looks clean. Only src/ contains active packages.")
else:
    print("\n⚠️  Issues found. Please review warnings above.")