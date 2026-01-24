#!/bin/bash

# --- COLOR CODES FOR READABILITY ---
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 Starting Bihar UAV Research Lab...${NC}"

# --- 1. THE HARD RESET ---
echo -e "${YELLOW}🧹 Killing old processes (GZ, ROS2, Agent, QGC)...${NC}"
pkill -9 gz || true
pkill -9 ruby || true
pkill -9 ros2 || true
pkill -9 MicroXRCEAgent || true
pkill -9 QGroundControl || true
sleep 3

# --- 2. START THE TRANSLATOR (MicroXRCE Agent) ---
echo -e "${GREEN}💓 Starting Heartbeat Agent (Port 8888)...${NC}"
# Running in a new terminal so you can see the scrolling 'Handshake'
gnome-terminal --title="MicroXRCE_Agent" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash" &
sleep 5

# --- 3. START THE WORLD (Gazebo with Auto-Play) ---
echo -e "${GREEN}🌍 Opening Bihar Farm Simulation (Physics Enabled)...${NC}"
source ~/uav_agricultural_drone_project/install/setup.bash
# The --gz-args "-r" forces the simulation to START immediately
ros2 launch hexacopter_control launch_drone.launch.py --gz-args "-r" &

# --- 4. THE LONG WAIT ---
echo -e "${YELLOW}⏳ Waiting 40 seconds for 3D Graphics and PX4 to stabilize...${NC}"
for i in {40..1}; do
    echo -ne "   Time remaining: $i seconds... \r"
    sleep 1
done
echo -e "\n${GREEN}Done waiting!${NC}"

# --- 5. START THE COCKPIT (QGroundControl) ---
echo -e "${GREEN}🎮 Launching QGroundControl from Project Folder...${NC}"
cd ~/uav_agricultural_drone_project
./QGroundControl.AppImage &
sleep 5

# --- 6. START THE DATA BRIDGE ---
echo -e "${GREEN}🌉 Building the Command Bridge (cmd_vel)...${NC}"
ros2 run ros_gz_bridge parameter_bridge /model/hexacopter/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &

echo -e "${BLUE}✅ ALL SYSTEMS GO! Check QGroundControl for 'Connected' status.${NC}"
