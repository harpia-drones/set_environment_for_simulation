# Base image for ROS 2 Humble Desktop Full on Jammy
FROM osrf/ros:humble-desktop-full-jammy AS base

# Shell to be used during the build process and the container's default
SHELL ["/bin/bash", "-c"]

# Update and upgrade system
RUN apt update && apt upgrade -y

# Install Gazebo Fortress  
RUN apt update && \
    apt-get install -y lsb-release gnupg curl && \
    curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y ignition-fortress

# Install command line tools
RUN apt update && \
    apt install -y \
    tmux \
    unzip \
    gedit

############################################## Tmux Setup ##############################################

# Create the tmux conf file
RUN touch /root/.tmux.conf

# Enable mouse suport 
RUN echo 'set -g mouse on' >> /root/.tmux.conf

# Move to the pane on the left
RUN echo 'bind -n C-Left select-pane -L' >> /root/.tmux.conf

# Move to the pane on the right
RUN echo 'bind -n C-Right select-pane -R' >> /root/.tmux.conf

# Move to the pane above
RUN echo 'bind -n C-Up select-pane -U' >> /root/.tmux.conf

# Move to the pane below
RUN echo 'bind -n C-Down select-pane -D' >> /root/.tmux.conf

# Avoid conflicts with modifier selection keys
RUN echo 'setw -g mode-keys vi' >> /root/.tmux.conf

############################################## Terminal personalization Setup ##############################################

# Install Oh My Posh
RUN curl -s https://ohmyposh.dev/install.sh | bash -s 

# Install OhMyPosh theme
RUN mkdir /root/.poshthemes 

# Donwload the theme for terminal from github
RUN curl -L "https://raw.githubusercontent.com/harpia-drones/Tema/refs/heads/main/theme.json" -o /root/.poshthemes/theme.json

# Give the required permissions
RUN chmod u+rw /root/.poshthemes/theme.json 

# Configure OhMyPosh for Bash
RUN echo 'eval "$(oh-my-posh init bash --config /root/.poshthemes/theme.json)"' >> /root/.bashrc 

# Change terminal color scheme
RUN echo "69" | bash -c "$(wget -qO- https://git.io/vQgMr)"

# Remove unused apt files after installation processes 
RUN rm -rf /var/lib/apt/lists/* 

############################################## ROS2 Setup ##############################################

# Create the workspace
RUN mkdir -p /root/estudos_ws/src 

# Define the estudos_ws directory as the work directory
WORKDIR /root/estudos_ws/

# Build the workspace
RUN colcon build

# Enable ros2 features
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

# Enable estudos_ws features
RUN echo 'source /root/estudos_ws/install/setup.bash' >> /root/.bashrc

# Install colcon
RUN apt install python3-colcon-common-extensions

# Enable colcon auto complete
RUN sudo chmod +x /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && \
    echo '/usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> /root/.bashrc

