# Base image for ROS 2 Jazzy Desktop Full on Jammy
FROM osrf/ros:jazzy-desktop-full AS base

# Shell to be used during the build process and the container's default
SHELL ["/bin/bash", "-c"]

############################################## Environment Setup ##############################################

RUN \
    # Create the workspace
    mkdir -p /root/estudos_ws/src; \
    \
    # Update and upgrade the system
    apt-get update && apt-get upgrade -y; \
    \
    # Install command line tools (tmux, unzip, gedit, vim)
    apt-get install -y tmux unzip gedit vim

############################################## Gazebo Harmonic Instalation ##############################################

RUN \
    # Download and add the Gazebo package GPG key
    curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    \
    # Add the Gazebo repository to apt sources
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    \
    # Install Gazebo Harmonic package
    apt-get update && apt-get install -y gz-harmonic

############################################## Tmux Setup ##############################################

RUN \
    # Create the tmux config file
    touch /root/.tmux.conf && \
    \
    # Enable mouse support
    echo 'set -g mouse on' >> /root/.tmux.conf && \
    \
    # Move to the pane on the left (bind Ctrl+Left to select the left pane)
    echo 'bind -n C-Left select-pane -L' >> /root/.tmux.conf && \
    \
    # Move to the pane on the right (bind Ctrl+Right to select the right pane)
    echo 'bind -n C-Right select-pane -R' >> /root/.tmux.conf && \
    \
    # Move to the pane above (bind Ctrl+Up to select the upper pane)
    echo 'bind -n C-Up select-pane -U' >> /root/.tmux.conf && \
    \
    # Move to the pane below (bind Ctrl+Down to select the bottom pane)
    echo 'bind -n C-Down select-pane -D' >> /root/.tmux.conf && \
    \
    # Avoid conflicts with modifier selection keys (set vi mode)
    echo 'setw -g mode-keys vi' >> /root/.tmux.conf

############################################## Terminal personalization Setup ##############################################

RUN \
    # Download and install OhMyPosh
    curl -s https://ohmyposh.dev/install.sh | bash -s; \
    \
    # Create the directory for the OhMyPosh themes
    mkdir /root/.poshthemes; \
    \
    # Download the theme from GitHub using curl
    curl -L "https://raw.githubusercontent.com/harpia-drones/Tema/refs/heads/main/theme.json" -o /root/.poshthemes/theme.json; \
    \
    # Give the necessary read-write permissions to the theme file
    chmod u+rw /root/.poshthemes/theme.json; \
    \
    # Configure OhMyPosh to use the downloaded theme in Bash
    echo 'eval "$(oh-my-posh init bash --config /root/.poshthemes/theme.json)"' >> /root/.bashrc; \
    \
    # Change the terminal color scheme using curl (replacing wget)
    echo "69" | bash -c "$(curl -sSL https://git.io/vQgMr)"

############################################## Colcon Setup #############################################

# Define the estudos_ws directory as the work directory
WORKDIR /root/estudos_ws/

# PS: Colcon instalation is using Debian packages

RUN \
    # Enable ROS 2 features (source ROS setup.bash)
    echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc; \
    source ~/.bashrc; \
    \
    # Add the ROS 2 repository to the apt sources list
    echo "deb [arch=$(dpkg --print-architecture)] http://repo.ros2.org/ubuntu/main $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list &&\
    \
    # Add the ROS GPG key to verify package integrity
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - &&\
    \
    # Install colcon and its common extensions
    apt update && apt install -y python3-colcon-common-extensions &&\
    \
    # Build the workspace
    colcon build; \
    \
    # Enable estudos_ws features (source custom workspace setup.bash)
    echo "source /root/estudos_ws/install/setup.bash" >> /root/.bashrc \
    \
    # Ensure the global bashrc file is updated with colcon configurations
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc; \
    echo "export _colcon_cd_root=/opt/ros/jazzy/" >> /root/.bashrc; \
    \
    # echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc; \
    \
    echo "alias bashrc='source /root/.bashrc'" >> /root/.bashrc; \
    \
    colcon build; \
    source ~/.bashrc;
    
############################################## Nav2 Setup ##############################################

RUN \ 
    # Update the package list and install Nav2 (navigation stack) and related packages
    apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb*; \
    \
    # Set environment variables for TurtleBot3 model and Gazebo model path in the global bashrc
    echo 'export TURTLEBOT3_MODEL=waffle' >> /root/.bashrc && \
    echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models' >> /root/.bashrc &&\
    \
    # Remove unused apt files after installation processes 
    rm -rf /var/lib/apt/lists/* 
