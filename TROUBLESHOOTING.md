# 🛠️ Troubleshooting Log: Bihar Agricultural Drone

This file records errors encountered during the development and execution of the UAV simulation and their respective solutions.

## 🛑 Common Errors & Solutions

### 1. Gazebo World Not Found
**Error:** `Unable to find or download file` (often resulting in a blank Gazebo screen).
**Cause:** PX4 SITL often prepends its internal directory to absolute paths, causing a "Double Path" and "Double Extension" error (e.g., `.../worlds//home/abhishek/.../bihar_farm.sdf.sdf`).
**Fix:** Use the **Symlink Trick**. Create a symbolic link of your world file inside the PX4 directory and refer to it by its clean name (`bihar_farm`) in the launch script.

### 2. Sensor Error / EKF2 Fails to Initialize
**Error:** QGroundControl reports "Sensor Error" or "Preflight Fail: Accel Sensor 0 missing".
**Cause:** The Gazebo world file was missing the necessary system plugins for sensors (IMU, Magnetometer, Barometer, AirPressure) or lacked `<spherical_coordinates>`.
**Fix:** Updated `setup_assets.sh` to include `gz-sim-sensors-system`, `gz-sim-imu-system`, `gz-sim-magnetometer-system`, `gz-sim-air-pressure-system`, and correct Latitude/Longitude for Munger, Bihar.

### 3. GLIBC Symbol Lookup Error
**Error:** `/usr/bin/gnome-terminal.real: symbol lookup error: ... undefined symbol: __libc_pthread_init`
**Cause:** Conflicts between ROS 2 system libraries and environment variables (like `LD_LIBRARY_PATH`) often injected by Snaps or Virtual Environments.
**Fix:** Unset `LD_LIBRARY_PATH`, `LD_PRELOAD`, `GTK_PATH`, and `GIO_EXTRA_MODULES` immediately before calling `gnome-terminal`. Note that sourcing ROS setup files re-populates these, so they must be unset again if launching a GUI tool like Gazebo.

### 4. QGroundControl Disconnected
**Error:** QGC opens but shows no vehicle connection.
**Cause:** PX4 SITL hasn't finished booting or the MAVLink bridge isn't active.
**Fix:** Added a 10-second sleep delay in launch scripts before starting QGC to allow the PX4/Gazebo handshake to complete.

### 5. Python Package Not Found
**Error:** `ros2 run` fails to find `hexacopter_control` nodes.
**Cause:** The package was built as `ament_cmake` instead of `ament_python`, or the resource marker was missing.
**Fix:** Ensured `package.xml` uses `<build_type>ament_python</build_type>` and verified the existence of the `resource/<package_name>` marker.

### 6. AWS CloudFront Instability (QGC)
**Error:** QGroundControl download fails or the AppImage is corrupted.
**Cause:** Using AWS CloudFront links for QGC is unreliable in certain regions.
**Fix:** All scripts have been updated to use the official GitHub Releases link: `https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.2/QGroundControl.AppImage`.

### 7. Airframe Not Found (6001_*)
**Error:** `Error: no autostart file found ... airframes/6001_*`
**Cause:** The system is trying to load a hexacopter airframe that doesn't exist in the PX4 installation.
**Fix:** Explicitly export `PX4_SYS_AUTOSTART=4001` in launch scripts to use the stable x500 quadcopter profile as a base.

### 8. Build System Corruption (Ninja/Make)
**Error:** `ninja: build stopped: subcommand failed` or `make: *** [Makefile:227: px4_sitl] Error 1`
**Cause:** Previous failed builds left corrupted artifacts in the `build/` directory.
**Fix:** Run `make distclean` inside the `PX4-Autopilot` directory to wipe the cache.

### 9. Failed to load module "canberra-gtk-module"
**Error:** `# Failed to load module "canberra-gtk-module"` in terminal.
**Cause:** Missing GTK sound/event modules on the host system.
**Fix:** Run `sudo apt install libcanberra-gtk-module libcanberra-gtk3-module`.

### 7. Terminal Window Crashes on Launch
**Symptom:** A terminal window opens and immediately closes before you can read the error.
**Cause:** Usually a syntax error in the `bash -c` string or a missing dependency.
**Fix:** Temporarily change `exec bash` to `read -p "Press enter to close..."` in the launch script to freeze the window and read the error message.

## 🔍 Diagnostic Commands (The "Expert" Checklist)

### 1. Verify Gazebo is actually publishing sensors
Run this in a new terminal while the sim is running:
`gz topic -l | grep -E "mag|air_pressure|imu"`
*   **If empty:** The issue is in `setup_assets.sh` (missing world plugins) or `model.sdf` (missing sensor definitions).
*   **If topics exist:** The issue is in the PX4 `gz_bridge` mapping (check `PX4_GZ_MODEL` name).

## � How to record a new error
1. Run the launch command: `ros2 launch hexacopter_control launch_drone.launch.py 2>&1 | tee last_error.log`
2. Identify the error message in `last_error.log`.
3. Document the symptoms, the suspected cause, and the steps taken to resolve it in this file.

---
*Last Updated: $(date)*