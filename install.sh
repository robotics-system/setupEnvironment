#!/bin/bash

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/scripts/common.sh"

# Available installation components
COMPONENTS=(
  "base"
  "docker"
  "vscode"
  "ros2"
)

# ROS2 distribution
ROS2_DISTRO=""

# Print usage information
print_usage() {
  echo "Usage: $0 [OPTIONS]"
  echo "Install development environment components"
  echo ""
  echo "Options:"
  echo "  -h, --help     Show this help message"
  echo "  -a, --all      Install all components"
  echo "  -l, --list     List available components"
  echo "  -c COMPONENT   Install specific component"
  echo "  --humble       Use ROS2 Humble distribution"
  echo "  --jazzy        Use ROS2 Jazzy distribution"
  echo ""
  echo "Available components:"
  for component in "${COMPONENTS[@]}"; do
    echo "  - $component"
  done
}

# Install specific component
install_component() {
  local component=$1
  local script="${SCRIPT_DIR}/scripts/${component}.sh"

  if [ ! -f "$script" ]; then
    echo "Error: Installation script for $component not found"
    exit 1
  fi

  echo "----------------------------------------"
  echo "Installing $component..."
  echo "----------------------------------------"

  # Pass ROS2_DISTRO as an environment variable to the script
  ROS2_DISTRO=$ROS2_DISTRO bash "$script"
}

# Install all components
install_all() {
  for component in "${COMPONENTS[@]}"; do
    install_component "$component"
  done
}

# Parse command line arguments
if [ $# -eq 0 ]; then
  echo "Error: Please specify ROS2 distribution (--humble or --jazzy)"
  print_usage
  exit 1
fi

while [[ $# -gt 0 ]]; do
  case $1 in
  -h | --help)
    print_usage
    exit 0
    ;;
  -l | --list)
    echo "Available components:"
    for component in "${COMPONENTS[@]}"; do
      echo "  - $component"
    done
    exit 0
    ;;
  -a | --all)
    if [ -z "$ROS2_DISTRO" ]; then
      echo "Error: Please specify ROS2 distribution (--humble or --jazzy) before --all"
      exit 1
    fi
    install_all
    shift
    ;;
  -c)
    if [ -z "$2" ]; then
      echo "Error: Component name required"
      exit 1
    fi
    if [ -z "$ROS2_DISTRO" ]; then
      echo "Error: Please specify ROS2 distribution (--humble or --jazzy) before -c"
      exit 1
    fi
    install_component "$2"
    shift 2
    ;;
  --humble)
    ROS2_DISTRO="humble"
    shift
    ;;
  --jazzy)
    ROS2_DISTRO="jazzy"
    shift
    ;;
  *)
    echo "Unknown option: $1"
    print_usage
    exit 1
    ;;
  esac
done

# If no installation command was given but distribution was set
if [ -n "$ROS2_DISTRO" ]; then
  echo "No installation command provided. Installing all components..."
  install_all
fi

echo "Installation completed!"
echo "Please restart your terminal for all changes to take effect."
