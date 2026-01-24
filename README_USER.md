# 🚁 Agricultural Drone User Manual

## 🌟 Quick Start

1.  **Power On:** Turn on the drone and the ground control station.
2.  **Launch System:**
    Open a terminal and run:
    ```bash
    ./launch_split.sh
    ```
3.  **Start Mission:**
    In the "ROS 2 System" terminal, the mission commander will automatically wait for a path.
    Once the `coverage_planner` generates a path, the drone will:
    *   Take off to 5m.
    *   Fly the generated lawnmower pattern.
    *   Detect weeds using the camera.
    *   Spray automatically when a weed is found.
    *   Return to home when finished.

## ⚠️ Safety Features

*   **Geofence:** The drone will NOT spray outside the designated farm area (Munger, Bihar coordinates).
*   **Low Battery:** If battery drops below 20%, the drone will automatically Return to Launch (RTL).
*   **GPS Loss:** If GPS is lost for >2 seconds, the drone will land immediately.

## 🛠️ Troubleshooting

*   **Drone not taking off?** Check QGroundControl for error messages. Ensure the drone is armed.
*   **Not spraying?** Ensure you are inside the geofence and the "weed" confidence is >60%.
*   **Simulation lagging?** Close other applications or run in "Headless" mode.

## 📦 Updates

To update the software:
```bash
git pull
./setup_product.sh
```