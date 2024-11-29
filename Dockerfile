# Base image for ROS 2 Jazzy Desktop Full on Jammy
FROM osrf/ros:jazzy-desktop-full AS base

# Shell to be used during the build process and the container's default
SHELL ["/bin/bash", "-c"]

# Update and upgrade system
RUN apt-get update && apt-get upgrade -y

# Install Gazebo Harmonic   
RUN apt-get update && \
    apt-get install -y curl lsb-release gnupg 

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \ 
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y gz-harmonic

# Install command line tools
RUN apt update && \
    apt install -y \
    tmux \
    unzip \
    gedit \
    wget 

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

#RUN curl -s https://ohmyposh.dev/install.sh | bash -s 

RUN wget https://github.com/JanDeDobbeleer/oh-my-posh/releases/latest/download/posh-linux-amd64 -O /usr/local/bin/oh-my-posh && \
    chmod +x /usr/local/bin/oh-my-posh


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

############################################## ROS2 Setup ##############################################

# Create the workspace
RUN mkdir -p /root/estudos_ws/src 

# Define the estudos_ws directory as the work directory
WORKDIR /root/estudos_ws/

# Build the workspace
RUN colcon build

# Enable ros2 features
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

# Enable estudos_ws features
RUN echo 'source /root/estudos_ws/install/setup.bash' >> /root/.bashrc
RUN  sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install colcon
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-argcomplete

# Enable colcon cd
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc

# Enable colcon auto complete
# RUN chmod +x /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
# RUN echo '/usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> /root/.bashrc

############################################## Nav2 Setup ##############################################

# Install Nav2
RUN apt update && apt install -y ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb*

RUN echo 'export TURTLEBOT3_MODEL=waffle' >> /root/.bashrc && \
    echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models' >> /root/.bashrc 

# Remove unused apt files after installation processes 
RUN rm -rf /var/lib/apt/lists/* 
