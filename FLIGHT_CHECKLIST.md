# 🛫 Bihar Agri-Drone Flight Checklist

Follow this exact 4-terminal sequence to launch the full autonomous system.

## 1. Micro-XRCE-DDS Agent (Tab 1)
Start the communication bridge on port 8888.
```bash
MicroXRCEAgent udp4 -p 8888
```

## 2. PX4 SITL Simulation (Tab 2)
Launch the drone model (gz_x500).
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

## 3. QGroundControl (Tab 3)
Launch the Ground Control Station (AppImage).
```bash
./QGroundControl.AppImage
```

## 4. ROS 2 System (Tab 4)
Verify that ROS 2 is receiving data from the drone.
```bash
ros2 topic list | grep fmu
```
*Success Criteria: You should see `/fmu/out/vehicle_odometry`.*