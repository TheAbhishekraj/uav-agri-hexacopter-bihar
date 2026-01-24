import os

setup_path = os.path.expanduser("~/uav_agricultural_drone_project/src/hexacopter_control/setup.py")

if not os.path.exists(setup_path):
    print(f"❌ Error: Could not find {setup_path}")
    exit(1)

with open(setup_path, 'r') as f:
    lines = f.readlines()

new_entry = "            'thermal_decision = hexacopter_control.thermal_decision_node:main',\n"
modified = False

# Check if already registered
if any("thermal_decision" in line for line in lines):
    print("✅ thermal_decision is already registered.")
    exit(0)

# Insert into console_scripts
for i, line in enumerate(lines):
    if "'console_scripts': [" in line:
        lines.insert(i + 1, new_entry)
        modified = True
        break

if modified:
    with open(setup_path, 'w') as f:
        f.writelines(lines)
    print(f"✅ Successfully added thermal_decision to setup.py")
else:
    print("⚠️  Could not find 'console_scripts' in setup.py. Please edit manually.")