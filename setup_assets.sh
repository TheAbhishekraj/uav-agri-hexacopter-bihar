#!/bin/bash

WORKSPACE_DIR=~/uav_agricultural_drone_project
PKG_DIR=$WORKSPACE_DIR/src/hexacopter_control
MODEL_NAME="hexacopter_agricultural"
MODEL_DIR=$PKG_DIR/models/$MODEL_NAME
WORLD_DIR=$PKG_DIR/worlds

echo "🏗️  Setting up Simulation Assets..."

# 1. Create Directory Structure
mkdir -p "$MODEL_DIR"
mkdir -p "$WORLD_DIR"

# 2. Move/Copy model.sdf
if [ -f "$WORKSPACE_DIR/model.sdf" ]; then
    echo "   📦 Moving model.sdf to $MODEL_DIR"
    mv "$WORKSPACE_DIR/model.sdf" "$MODEL_DIR/model.sdf"
else
    echo "   ⚠️  model.sdf not found in root. Checking if it's already in place..."
fi

# 3. Create model.config (The Name Tag)
echo "   🏷️  Creating model.config..."
cat > "$MODEL_DIR/model.config" <<EOL
<?xml version="1.0"?>
<model>
  <name>$MODEL_NAME</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Abhishek</name>
    <email>user@example.com</email>
  </author>
  <description>Agricultural Hexacopter for Bihar Project</description>
</model>
EOL

# 4. Create a Basic World if missing
if [ ! -f "$WORLD_DIR/bihar_farm.sdf" ]; then
    echo "   🌾 Creating basic bihar_farm.sdf..."
    cat > "$WORLD_DIR/bihar_farm.sdf" <<EOL
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="bihar_farm">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-wind-system" name="gz::sim::systems::Wind"></plugin>
    <!-- Removed ogre2 tag to fix Sensor Missing error -->
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"></plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry></collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry>
          <material>
            <ambient>0.1 0.5 0.1 1</ambient>
            <diffuse>0.1 0.5 0.1 1</diffuse>
            <specular>0.1 0.5 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Crop Rows for Visual Realism -->
    <model name="crop_row_1">
      <static>true</static>
      <pose>5 0 0.2 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>1 20 0.4</size></box></geometry>
          <material><ambient>0.0 0.3 0.0 1</ambient><diffuse>0.0 0.3 0.0 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="crop_row_2">
      <static>true</static>
      <pose>-5 0 0.2 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>1 20 0.4</size></box></geometry>
          <material><ambient>0.0 0.3 0.0 1</ambient><diffuse>0.0 0.3 0.0 1</diffuse></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
EOL
fi

echo "✅ Assets organized. Ready for Level 2."