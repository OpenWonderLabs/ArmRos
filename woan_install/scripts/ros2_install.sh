#!/bin/bash
# Version: 1.0
# Date: $(date +%Y-%m-%d)
# Author: WoanArm Team
# Warning: This script is for ROS2 Humble/Foxy in ubuntu 20.04/22.04
set -e

# UBUNTU CONFIGURATION BEGINS HERE

# Check if script is run as root (sudo)
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run with sudo privileges. for example: sudo bash ros2_install.sh"
    read -p "Press any key to exit..."
    exit 1
fi
# Get script directory
SCRIPT_DIR=$(dirname "$0")
# Get the username of the non-root user
USERNAME=$SUDO_USER
Ubuntu_version=$(lsb_release -r --short)
echo "Current user is: $USERNAME"
# Save logs to files
LOG_FILE="${SCRIPT_DIR}/ros2_install.log"
ERR_FILE="${SCRIPT_DIR}/ros2_install.err"
rm -f ${LOG_FILE}
rm -f ${ERR_FILE}

# Redirect output to console and log files
exec 1> >(tee -a ${LOG_FILE} )
exec 2> >(tee -a ${ERR_FILE} >&2)

# Output log info to console
echo "ROS2 installation started!"  
echo "Installation logs will be saved to ${LOG_FILE}"
echo "Installation errors will be saved to ${ERR_FILE}"

# No Password sudo config
sudo sed -i 's/^%sudo.*/%sudo ALL=(ALL) NOPASSWD:ALL/g' /etc/sudoers

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

# Get architecture of the system
if [ $(uname -m) = "x86_64" ]; then
  MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ubuntu/"
else
  MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/"
fi
echo "Current system architecture is: $(uname -m)"
echo "Current mirror is: $MIRROR"
if [ $Ubuntu_version = "20.04" ]; then

# Backup original software sources
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup
sudo cp /etc/apt/sources.list /etc/apt/sources.list.d/woanarm_ros2.list
# Clear original software sources
sudo echo "" > /etc/apt/sources.list
sudo echo "" > /etc/apt/sources.list.d/woanarm_ros2.list

# Replace software sources
echo "deb $MIRROR focal main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb $MIRROR focal-updates main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb $MIRROR focal-backports main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb [arch=$(dpkg --print-architecture)] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" >> /etc/apt/sources.list.d/woanarm_ros2.list


if [ $(uname -m) = "x86_64" ]; then
  echo "deb http://security.ubuntu.com/ubuntu/ focal-security main restricted universe multiverse" >> /etc/apt/sources.list
else
  echo "deb http://ports.ubuntu.com/ubuntu-ports/ focal-security main restricted universe multiverse" >> /etc/apt/sources.list
fi

elif [ $Ubuntu_version = "22.04" ]; then
# Backup original software sources
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup
sudo cp /etc/apt/sources.list /etc/apt/sources.list.d/woanarm_ros2.list
# Clear original software sources
sudo echo "" > /etc/apt/sources.list
sudo echo "" > /etc/apt/sources.list.d/woanarm_ros2.list

# Replace software sources
echo "deb $MIRROR jammy main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb $MIRROR jammy-updates main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb $MIRROR jammy-backports main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb [arch=$(dpkg --print-architecture)] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" >> /etc/apt/sources.list.d/woanarm_ros2.list
if [ $(uname -m) = "x86_64" ]; then
  echo "deb http://security.ubuntu.com/ubuntu jammy-security main restricted universe multiverse" >> /etc/apt/sources.list
else
  echo "deb http://ports.ubuntu.com/ubuntu-ports/ jammy-security main restricted universe multiverse" >> /etc/apt/sources.list
fi

else
    echo "This script must be run with Ubuntu 20.04 or 22.04"
    read -p "Press any key to exit..."
    exit 1
fi

# Install Curl
sudo apt-get install curl -y # If you haven't already installed curl
sudo apt-get install software-properties-common -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# System update
sudo apt-get update
sudo apt-get upgrade -y

# Install pip
sudo apt-get install python3-dev -y
sudo apt-get install pip -y # If you haven't already installed pip

# Install gnome-terminal
sudo apt-get install gnome-terminal -y # If you haven't already installed gnome-terminal

# Set default pip source
pip config set global.index-url http://pypi.tuna.tsinghua.edu.cn/simple
pip config set global.trusted-host pypi.tuna.tsinghua.edu.cn

# First ensure that the Ubuntu Universe repository is enabled.
sudo apt-get install software-properties-common -y
sudo add-apt-repository universe

# Update the system packages index to the latest version
sudo apt-get update

# Install ROS2 based on Ubuntu version
if [ $Ubuntu_version = "20.04" ]; then
  sudo apt-get install ros-foxy-desktop python3-argcomplete -y
  sudo apt-get install gazebo11 -y
  sudo apt-get install ros-foxy-gazebo-* -y
  ROS_DISTRO="foxy"
elif [ $Ubuntu_version = "22.04" ]; then
  sudo apt-get install ros-humble-desktop -y
  sudo apt-get install gazebo -y
  sudo apt-get install ros-humble-gazebo-* -y
  ROS_DISTRO="humble"
fi

sudo apt-get install ros-dev-tools -y

# Environment setup
if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" /home/$USERNAME/.bashrc; then
    echo "# ROS2 $ROS_DISTRO Environment Setting" | sudo tee -a /home/$USERNAME/.bashrc
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" | sudo tee -a /home/$USERNAME/.bashrc
    echo "ROS2 $ROS_DISTRO environment setup added to /home/$USERNAME/.bashrc"
else
    echo "ROS2 $ROS_DISTRO environment is already set in /home/$USERNAME/.bashrc"
fi

source /home/$USERNAME/.bashrc

# Initialize rosdepc by fishros under BSD License
# https://pypi.org/project/rosdepc/#files
sudo pip install rosdep
sudo pip install rosdepc
# Init & update rosdep 
sudo rosdepc init > /dev/null
echo "rosdepc init completed!"

# System update again
sudo apt-get update
sudo apt-get dist-upgrade -y

# Install colcon build tool
sudo apt-get install python3-colcon-common-extensions -y

# Verifying ROS2 installation
clear

source /home/$USERNAME/.bashrc
# Define the variables to be printed
TEXT1="ROS2 installation completed!"
TEXT2="Please open a new terminal and run demo to verify the installation:"
TEXT3="ros2 run demo_nodes_cpp talker"
TEXT4="ros2 run demo_nodes_cpp listener"

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

# Print the text in the center of the screen in the desired colors
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo -e "${GREEN}$(printf '%*s' $TEXT1_PADDING)${TEXT1} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT2} ${NC}"
echo -e "${RED}$(printf '%*s' $TEXT3_PADDING)${TEXT3} ${NC}"
echo -e "${RED}$(printf '%*s' $TEXT3_PADDING)${TEXT4} ${NC}"
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""

read -p "Press any key to exit..."
exit 0
