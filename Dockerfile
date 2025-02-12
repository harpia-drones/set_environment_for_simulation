# Base image for ROS 2 Jazzy Desktop Full on Jammy
FROM osrf/ros:jazzy-desktop-full AS base

# Shell to be used during the build process and the container's default
SHELL ["/bin/bash", "-c"]

############################################## 
#              Environment Setup 
##############################################

RUN \
    # Create the workspace
    mkdir -p /root/estudos_ws/src; \
    \
    # Update and upgrade the system
    apt-get update && apt-get upgrade -y && \
    \
    # Install command line tools (tmux, unzip, gedit, vim)
    apt-get install -y tmux unzip gedit vim && \
    \
############################################## 
#            Gazebo Harmonic Setup 
##############################################
    \
    # Install Gazebo Harmonic package
    apt-get install -y ros-${ROS_DISTRO}-ros-gz; \
    \
############################################## 
#                 Tmux Setup 
##############################################
    \
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
    echo 'setw -g mode-keys vi' >> /root/.tmux.conf && \
    \
############################################## 
#       Terminal personalization Setup 
##############################################
    \
    # Download and install OhMyPosh
    curl -s https://ohmyposh.dev/install.sh | bash -s &&\
    \
    # Create the directory for the OhMyPosh themes
    mkdir /root/.poshthemes &&\
    \
    # Download the theme from GitHub using curl
    curl -L "https://raw.githubusercontent.com/harpia-drones/Tema/refs/heads/main/theme.json" -o /root/.poshthemes/theme.json &&\
    \
    # Give the necessary read-write permissions to the theme file
    chmod u+rw /root/.poshthemes/theme.json &&\
    \
    # Configure OhMyPosh to use the downloaded theme in Bash
    echo 'eval "$(oh-my-posh init bash --config /root/.poshthemes/theme.json)"' >> /root/.bashrc &&\
    \
    # Change the terminal color scheme using curl (replacing wget)
    echo "69" | bash -c "$(curl -sSL https://git.io/vQgMr)" && \
    \
############################################## 
#                 Colcon Setup 
##############################################
    \
    # Create the colcon-argcomplete missing file for jazzy \
    mkdir -p /usr/share/colcon_argcomplete/hook && \
    touch /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && \
    chmod +x /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && \
    echo 'if type register-python-argcomplete3 > /dev/null 2>&1; then' > /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && \
    echo '  eval "$(register-python-argcomplete3 colcon)"' >> /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && \
    echo 'elif type register-python-argcomplete > /dev/null 2>&1; then' >> /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && \
    echo '  eval "$(register-python-argcomplete colcon)"' >> /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && \
    echo 'fi' >> /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; \
    \
    # Enable ROS 2 features (source ROS setup.bash)
    echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    \
    # Add the ROS 2 repository to the apt sources list
    echo "deb [arch=$(dpkg --print-architecture)] http://repo.ros2.org/ubuntu/main $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list && \
    \
    # Add the ROS GPG key to verify package integrity
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    \
    # Install colcon and its common extensions
    apt update && apt install -y python3-colcon-common-extensions && \
    \
    # Build the workspace
    cd /root/estudos_ws/ && colcon build; \
    \
    # Enable estudos_ws features (source custom workspace setup.bash)
    echo "source /root/estudos_ws/install/setup.bash" >> /root/.bashrc; \
    \
    # Ensure the global bashrc file is updated with colcon configurations
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc; \
    echo "export _colcon_cd_root=/opt/ros/jazzy/" >> /root/.bashrc; \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc; \
    \
    # Create an alias to source the terminal
    echo "" >> /root/.bashrc; \
    \
    # Modifying the colcon function to avoid it undestands fonders in dependency/ as packages 
    echo "alias bashrc='source /root/.bashrc'" >> /root/.bashrc; \
    echo "" >> /root/.bashrc; \
    echo "colcon() {" >> /root/.bashrc; \
    echo "    if [[ \"\$1\" == \"build\" ]]; then" >> /root/.bashrc; \
    echo "        command colcon build --packages-ignore bringup description interfaces \"\${@:2}\"" >> /root/.bashrc; \
    echo "    else" >> /root/.bashrc; \
    echo "        command colcon \"\$@\"" >> /root/.bashrc; \
    echo "    fi" >> /root/.bashrc; \
    echo "}" >> /root/.bashrc; \
    echo "" >> /root/.bashrc

# Define the estudos_ws directory as the work directory
WORKDIR /root/estudos_ws/