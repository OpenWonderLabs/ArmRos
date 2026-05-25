#!/bin/bash
# Version: 1.0
# Date: $(date +%Y-%m-%d)
# Author: OneroArm Team
#
# Warning: This script assumes that the ubuntu20.04/22.04 system and ROS2 Humble have been installed correctly
# If not, please execute ros2_install.sh first.
#
set -e

# Get script directory
SCRIPT_DIR=$(dirname "$0")
# Get the username of the non-root user
USERNAME=$SUDO_USER
echo "Current user is: $USERNAME"

# Check if script is run as root (sudo)
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run with sudo privileges. for example: sudo bash moveit2_install.sh"
    read -p "Press any key to exit..."
    exit 1
fi
echo "sudo privileges check passed"

# Check if script is run in ubuntu20.04 or 22.04
if [ "$(lsb_release -sc)" != "focal" ]; then
    if [ "$(lsb_release -sc)" != "jammy" ]; then
        echo "This script must be run in ubuntu20.04 or 22.04"
        read -p "Press any key to exit..."
        exit 1
    fi
fi

echo "Ubuntu version check passed"

# Check if script is run in ROS2 Humble or Foxy
if [[ "$(sudo -u $USERNAME dpkg -l ros-foxy-desktop)" == *ii* ]]; then
    echo "ROS2 foxy check passed"
    ROS_DISTRO="foxy"
elif [[ "$(sudo -u $USERNAME dpkg -l ros-humble-desktop)" == *ii* ]]; then
    echo "ROS2 humble check passed"
    ROS_DISTRO="humble"
else
    echo "This script must be run with ROS2 Humble or Foxy-desktop-full"
    read -p "Press any key to exit..."
    exit 1
fi

sudo apt install nlohmann-json3-dev

# Save logs to files
LOG_FILE="${SCRIPT_DIR}/moveit2_install.log"
ERR_FILE="${SCRIPT_DIR}/moveit2_install.err"
rm -f ${LOG_FILE}
rm -f ${ERR_FILE}

# Redirect output to console and log files
exec 1> >(tee -a ${LOG_FILE} )
exec 2> >(tee -a ${ERR_FILE} >&2)

# Add GitHub520 Host to host for GitHub access in China
# https://github.com/521xueweihan/GitHub520
sudo apt-get install curl -y
sudo sed -i "/# GitHub520 Host Start/Q" /etc/hosts && curl https://raw.hellogithub.com/hosts >> /etc/hosts
echo "GitHub520 Host added to host file"
sudo systemctl restart systemd-resolved.service
echo "Refreshed network settings, sleep 5 seconds"
sleep 5

# Install wstool
sudo apt-get install python3-wstool -y

# Install moveit based on ROS distribution
if [ "$ROS_DISTRO" = "foxy" ]; then
    # Install moveit
    sudo apt-get install ros-foxy-moveit -y
    # Warning: Installing all subpackages of moveit may cause dependency conflicts, please do so with caution.
    sudo apt-get install ros-foxy-moveit-* -y
    # Install ros_control
    sudo apt-get install ros-foxy-controller-interface ros-foxy-controller-manager-msgs ros-foxy-controller-manager
elif [ "$ROS_DISTRO" = "humble" ]; then
    # Install moveit
    sudo apt-get install ros-humble-moveit -y
    # Warning: Installing all subpackages of moveit may cause dependency conflicts, please do so with caution.
    sudo apt-get install ros-humble-moveit-* -y
    # Install ros_control
    sudo apt-get install ros-humble-controller-interface ros-humble-controller-manager-msgs ros-humble-controller-manager
fi

# Install additional dependencies for OneroArm
sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher-gui -y
sudo apt-get install ros-$ROS_DISTRO-robot-state-publisher -y
sudo apt-get install ros-$ROS_DISTRO-xacro -y

clear
sleep 1

# Define the variables to be printed
TEXT0=""
TEXT1="Moveit installation completed!"
TEXT2="Please open a new terminal and run the following to verify the installation:"
TEXT3="cd /home/$USERNAME/oneroarm_ws && source install/setup.bash"
TEXT4="ros2 launch onero_bringup x1_l_moveit_bringup.launch.py"
TEXT5="1. Click 'Add' in the left panel, and add the following items:"
TEXT6="2. Add 'RobotModel', 'MotionPlanning' to the left panel"
TEXT7="3. Try to drag the end effector to see if the robot arm moves"
TEXT8="4. Click 'Plan & Execute' to see the robot arm move"
TEXT9="5. If you see the robot arm move, the installation is successful"

# Define the colors
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[1;32m'
NC='\033[0m'

# Calculate the center of the terminal window
TERMINAL_WIDTH=$(tput cols)
TEXT1_PADDING=$((($TERMINAL_WIDTH-${#TEXT1})/2))
TEXT2_PADDING=$((($TERMINAL_WIDTH-${#TEXT2})/2))
TEXT3_PADDING=$((($TERMINAL_WIDTH-${#TEXT3})/2))

# Finished
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${GREEN}$(printf '%*s' $TEXT1_PADDING)${TEXT1} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT2} ${NC}"
echo -e "${RED}$(printf '%*s' $TEXT3_PADDING)${TEXT3} ${NC}"
echo -e "${RED}$(printf '%*s' $TEXT3_PADDING)${TEXT4} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT5} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT6} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT7} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT8} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT9} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT1_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT1_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT1_PADDING)${TEXT0} ${NC}"

read -p "Press any key to exit..."
exit 0
