#!/bin/bash

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

echo "Starting ROS2 ${ROS2_DISTRO} installation..."

# Validate ROS2 distribution
if [ "$ROS2_DISTRO" != "humble" ] && [ "$ROS2_DISTRO" != "jazzy" ]; then
  echo "Error: Invalid ROS2 distribution. Must be either 'humble' or 'jazzy'"
  exit 1
fi

# Install ROS2
echo "Installing ROS2 ${ROS2_DISTRO}..."
ensure_dir ~/Downloads
cd ~/Downloads
git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
cd ros2_setup_scripts_ubuntu
chmod +x "ros2-${ROS2_DISTRO}-desktop-main.sh"
./ros2-${ROS2_DISTRO}-desktop-main.sh
check_status "ROS2 ${ROS2_DISTRO} installation"

# Install Navigation2 and dependencies
echo "Installing Navigation2 and dependencies..."
if [ "$ROS2_DISTRO" = "humble" ]; then
  # Humble-specific packages
  sudo apt install -y ros-humble-navigation2
  sudo apt install -y ros-humble-nav2-bringup
  sudo apt install -y ros-humble-turtlebot3-gazebo
elif [ "$ROS2_DISTRO" = "jazzy" ]; then
  # Jazzy-specific packages
  sudo apt install -y ros-jazzy-navigation2
  sudo apt install -y ros-jazzy-nav2-bringup
  sudo apt install -y ros-jazzy-nav2-minimal-tb*
  sudo apt install -y ros-jazzy-nav2-loopback-sim
fi
check_status "Navigation2 installation"

# Add ROS2 and development environment to bashrc
echo "Configuring ROS2 environment..."
echo '' >>~/.bashrc
echo "# ROS2 ${ROS2_DISTRO} and development environment" >>~/.bashrc
echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >>~/.bashrc
echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >>~/.bashrc
echo 'source /usr/share/colcon_cd/function/colcon_cd.sh' >>~/.bashrc
echo "export _colcon_cd_root=/opt/ros/${ROS2_DISTRO}/" >>~/.bashrc

# Add Turtlebot3 configurations
if [ "$ROS2_DISTRO" = "humble" ]; then
  echo 'export TURTLEBOT3_MODEL=burger' >>~/.bashrc
  echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/${ROS2_DISTRO}/share/turtlebot3_gazebo/models" >>~/.bashrc
  echo "source /usr/share/gazebo/setup.bash" >>~/.bashrc
fi

check_status "ROS2 environment configuration"

# Source the ROS2 environment for the current session
source "/opt/ros/${ROS2_DISTRO}/setup.bash"

echo "ROS2 ${ROS2_DISTRO} installation completed successfully!"
echo ""
echo "Important: Please restart your terminal for the ROS2 environment to take effect."
