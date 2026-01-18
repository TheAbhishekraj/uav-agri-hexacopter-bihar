FROM ros:humble-perception

# 1. Install System Dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    wget \
    unzip \
    git \
    && rm -rf /var/lib/apt/lists/*

# 2. Install Python Dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

# 3. Setup Workspace
WORKDIR /root/uav_agricultural_drone_project
COPY . .

# 4. Install ROS Dependencies
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# 5. Setup AI Assets
RUN chmod +x setup_tflite.sh && ./setup_tflite.sh

# 6. Build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# 7. Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]