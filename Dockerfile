# Base image for ROS 2 Humble Desktop Full on Jammy
FROM osrf/ros:humble-desktop-full-jammy AS base

# Shell to be used during the build process and the container's default
SHELL ["/bin/bash", "-c"]

# Set the environment variables
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive

# Update and upgrade system
RUN apt update && DEBIAN_FRONTEND=noninteractive apt upgrade -y

# Install Gazebo Fortress  
RUN apt-get update && \
    apt-get install -y lsb-release gnupg curl && \
    curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y ignition-fortress

# Install TMUX (Terminal Multiplexer)
RUN apt-get install -y tmux 
    
# Source ROS 2 setup
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
RUN mkdir -p /root/estudos_ws/src
WORKDIR /root/estudos_ws

