# 🎮 Manual Flight Guide: Bihar Agricultural Drone

This guide explains how to manually fly the drone using QGroundControl and the PX4 simulator, ensuring a clean environment before starting.

---

## 🛑 Step 1: Clean Up & Deactivate (Run First)

This ensures you are out of any Python virtual environment and all drone processes are stopped.

```bash
# 1. Deactivate any hidden Python environment
deactivate 2>/dev/null || true

# 2. Kill all drone processes
killall -9 gz_sim px4 MicroXRCEAgent QGroundControl.AppImage ruby
```

---

## 🚀 Step 2: Launch for Manual Flight (3 Terminals)

Open 3 separate tabs (`Ctrl`+`Shift`+`T`) and run these commands.

### 🖥️ Terminal 1: The Bridge

```bash
deactivate 2>/dev/null || true  # Safety check
MicroXRCEAgent udp4 -p 8888
```

### 🖥️ Terminal 2: The Simulation (Munger, Bihar)

```bash
deactivate 2>/dev/null || true  # Safety check
cd ~/PX4-Autopilot
export PX4_HOME_LAT=25.3748
export PX4_HOME_LON=86.4735
export PX4_HOME_ALT=45.0
make px4_sitl gz_x500
```
*(Wait for the Gazebo window to appear)*

### 🖥️ Terminal 3: Ground Control

```bash
deactivate 2>/dev/null || true  # Safety check
cd ~/uav_agricultural_drone_project
./QGroundControl.AppImage
```

---

## 🎮 Step 3: Enable Virtual Joysticks

Since you don't have a physical remote, you must turn on the on-screen sticks to fly manually.

1.  In **QGroundControl**, click the **"Q" Icon** (Top Left) -> **Application Settings**.
2.  Scroll down to the **General** tab.
3.  Check the box: **Virtual Joystick**.
4.  Go back to **Fly View** (Paper Airplane icon).
5.  You will now see two white circles on the screen.
6.  **Arm** the drone (Click "Ready to Fly" -> "Arm" or use the slider).
7.  Push the **Left Stick UP** to take off!