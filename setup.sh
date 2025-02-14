#!/bin/bash

echo "=================================================================="
echo "Starting setup.sh..."
echo "=================================================================="

# Update and upgrade the system
echo "=================================================================="
echo "Updating the system..."
echo "=================================================================="
sudo apt-get upgrade -y && \
sudo apt-get update

# Create a python virtual environment
echo "=================================================================="
echo "Creating a python virtual environment..."
echo "==================================================================" 
sudo apt-get install -y python3-venv && \
cd /home/harpia/ && \
python3 -m venv harpia_venv && \
source harpia_venv/bin/activate 

# Install the PX4 development toolchain to use the simulator
echo "=================================================================="
echo "Installing PX4..."
echo "==================================================================" 
cd /home/harpia/ && \
git clone https://github.com/PX4/PX4-Autopilot.git --recursive && \
bash /home/harpia/PX4-Autopilot/Tools/setup/ubuntu.sh && \
cd /home/harpia/PX4-Autopilot/ && \
make px4_sitl

# Install some dependencies for ROS2
echo "=================================================================="
echo "Installing some dependencies for ROS2..."
echo "=================================================================="
pip3 install -U empy pyros-genmsg setuptools catkin_pkg lark && \
sudo apt-get install -y ros-dev-tools 

# Install the XRCE-DDS Agent
echo "=================================================================="
echo "Installing Micro-XRCE-DDS-Agent..."
echo "==================================================================" 
cd /home/harpia/ && \
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
cd /home/harpia/Micro-XRCE-DDS-Agent && \
mkdir build && \
cd build && \
cmake .. && \
make && \
make install && \
ldconfig /usr/local/lib/ && \

# Clone the required repositories
echo "=================================================================="
echo "Cloning repositories..."
echo "==================================================================" 
cd /home/harpia/estudos_ws/src/ && \
git clone https://github.com/PX4/px4_msgs.git && \
git clone https://github.com/PX4/px4_ros_com.git

# Function to try to run colcon build
build_with_retry() {
    while true; do
        echo "Running colcon build..."
        cd /home/harpia/estudos_ws/ && colcon build --packages-ignore bringup description interfaces
        if [ $? -eq 0 ]; then  # Checks whether the previous command was successful
            echo "colcon build succesfully completed!"
            break  # Exit the loop if the command is successful
        else
            echo "colcon build fails. Trying again..."
            sleep 2  # Wait 2 seconds before trying again (opcional)
        fi
    done
}

# Call the function
build_with_retry

echo "=================================================================="
echo "setup.sh succesfully completed!"
echo "=================================================================="