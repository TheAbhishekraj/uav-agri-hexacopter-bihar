# 🚀 Launch Instructions: Bihar Agricultural Drone

This guide details the two methods to launch the system:
1.  **Automatic (One-Click)** - Recommended for standard missions.
2.  **Manual (Step-by-Step)** - Recommended for debugging or development.

---

## 🟢 Option 1: The "One-Click" Launch (Recommended)

This script automates the setup of 4 terminals, sets the Bihar location, and starts the Z-Pattern mission.

### Steps:
1.  Open a terminal in your project folder:
    ```bash
    cd ~/uav_agricultural_drone_project
    ```
2.  Run the launch script:
    ```bash
    ./launch_drone.sh
    ```

### What it does:
*   **Kills** old processes (Gazebo, PX4, Agent).
*   **Starts** Micro-XRCE-DDS Agent.
*   **Starts** PX4 SITL (Gazebo) at Munger, Bihar coordinates.
*   **Starts** QGroundControl.
*   **Waits** 15 seconds, then runs the `mission_sprayer.py` node.

---

## 🟡 Option 2: Manual Launch (Debug Mode)

Use this method if you need to see logs from specific components or restart individual nodes.

### 🛑 Step 0: Clean Slate
Run this first to prevent port conflicts:
```bash
killall -9 gz_sim px4 MicroXRCEAgent ruby QGroundControl.AppImage flight_controller detection_node parameter_bridge 2>/dev/null
```

### 🖥️ Terminal 1: The Bridge
Connects ROS 2 to the Drone.
```bash
MicroXRCEAgent udp4 -p 8888
```

### 🖥️ Terminal 2: The Simulation
Launches the drone in Munger, Bihar.
```bash
cd ~/PX4-Autopilot
export PX4_HOME_LAT=25.3748
export PX4_HOME_LON=86.4735
export PX4_HOME_ALT=45.0
make px4_sitl gz_x500_depth
```

### 🖥️ Terminal 3: Ground Control
Launches the cockpit interface.
```bash
cd ~/uav_agricultural_drone_project
./QGroundControl.AppImage
```

### 🖥️ Terminal 4: The Mission
Runs the autonomous code.
```bash
cd ~/uav_agricultural_drone_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run the Z-Pattern Mission
python3 src/hexacopter_control/hexacopter_control/mission_sprayer.py
```

---

## 🔍 Verification

After launching (using either method), verify the system health:

```bash
cd ~/uav_agricultural_drone_project
./system_health_check.sh
```

**Expected Output:**
```text
✅ Agent: RUNNING
✅ Gazebo: RUNNING
✅ QGC: RUNNING
✅ ROS 2 Connection: EXCELLENT
```