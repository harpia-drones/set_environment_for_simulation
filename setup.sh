#!/bin/bash

WS_DIRECTORY=$(find "$HOME" -type d -name "*_ws" -print -quit)

if [ -n "$WS_DIRECTORY" ]; then
    echo "=================================================================="
    echo "Starting setup.sh..."
    echo "=================================================================="

    # Update and upgrade the system
    echo "=================================================================="
    echo "Updating the system..."
    echo "=================================================================="
    apt-get upgrade -y && \
    apt-get update

    # Create a python virtual environment
    echo "=================================================================="
    echo "Creating a python virtual environment..."
    echo "==================================================================" 
    apt-get install -y python3-venv && \
    cd "/$HOME/" && \
    python3 -m venv harpia_venv && \
    source "/$HOME/harpia_venv/bin/activate"
    echo "source /$HOME/harpia_venv/bin/activate" >> "/$HOME/.bashrc"

    # Install the PX4 development toolchain to use the simulator
    echo "=================================================================="
    echo "Installing PX4..."
    echo "==================================================================" 
    cd "/$HOME/" && \
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive && \
    bash "/$HOME/PX4-Autopilot/Tools/setup/ubuntu.sh" --no-sim-tools

    # Install some dependencies for ROS2
    echo "=================================================================="
    echo "Installing some dependencies for ROS2..."
    echo "=================================================================="
    pip3 install -U empy pyros-genmsg setuptools catkin_pkg lark && \
    apt-get install -y ros-dev-tools 

    # Install the XRCE-DDS Agent
    echo "=================================================================="
    echo "Installing Micro-XRCE-DDS-Agent..."
    echo "==================================================================" 
    cd "/$HOME/" && \
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    curl -L "https://raw.githubusercontent.com/harpia-drones/set_environment_for_simulation/refs/heads/main/FindTinyXML2.cmake" -o "/$HOME/Micro-XRCE-DDS-Agent/cmake/modules/FindTinyXML2.cmake" && \
    cd "/$HOME/Micro-XRCE-DDS-Agent" && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig /usr/local/lib/

    # Clone the required repositories
    echo "=================================================================="
    echo "Cloning repositories..."
    echo "==================================================================" 
    cd "/$HOME/$WS_DIRECTORY/src/" && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git && \
    cd "/$HOME/$WS_DIRECTORY/" && \
    colcon build 
else
    echo "O diretório workspace padrão de projetos ROS não foi encontrado."
fi