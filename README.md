# Ubuntu 22.04/24.04 - ROS2 Development Environment Setup

This repository contains scripts for setting up a development environment on Ubuntu. The scripts support both Ubuntu 22.04 (with ROS2 Humble) and Ubuntu 24.04 (with ROS2 Jazzy). The scripts are modular and can be run individually or all at once.

## Structure

```
.
├── install.sh              # Main installation script
└── scripts/
    ├── common.sh          # Common functions used by all scripts
    ├── base.sh            # Base development tools installation
    ├── docker.sh          # Docker installation
    ├── vscode.sh          # VS Code installation and configuration
    └── ros2.sh            # ROS2 installation and setup
```

Each script in the `scripts` directory is a self-contained module that can be run independently or through the main `install.sh` script.

## Supported Configurations

- Ubuntu 22.04 LTS with ROS2 Humble
- Ubuntu 24.04 LTS with ROS2 Jazzy

## Usage

Clone the repository and make the scripts executable:

```bash
git clone https://github.com/Swepz/ros2_ubuntu_setup.git
cd ros2_ubuntu_setup
chmod +x install.sh scripts/*.sh
```

### Running the Installation

You can run the installation in several ways:

1. Install everything:

```bash
# For Ubuntu 22.04 with ROS2 Humble
./install.sh --humble -a

# For Ubuntu 24.04 with ROS2 Jazzy
./install.sh --jazzy -a
```

2. Install a specific component:

```bash
# For Ubuntu 22.04 with ROS2 Humble
./install.sh --humble -c base    # Install base development tools
./install.sh --humble -c docker  # Install Docker
./install.sh --humble -c vscode  # Install VS Code
./install.sh --humble -c ros2    # Install ROS2 Humble

# For Ubuntu 24.04 with ROS2 Jazzy
./install.sh --jazzy -c base    # Install base development tools
./install.sh --jazzy -c docker  # Install Docker
./install.sh --jazzy -c vscode  # Install VS Code
./install.sh --jazzy -c ros2    # Install ROS2 Jazzy
```

3. View available options:

```bash
./install.sh -h
```

4. List available components:

```bash
./install.sh -l
```

## Adding New Components

To add a new installation component:

1. Create a new script in the `scripts` directory (e.g., `scripts/newcomponent.sh`)
2. Add the component name to the `COMPONENTS` array in `install.sh`
3. Make sure to source `common.sh` in your new script for access to shared functions

Example template for a new component:

```bash
#!/bin/bash

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

echo "Starting new component installation..."

# Your installation steps here
# Use check_status function to verify each step
```

## Post-Installation

After running the installation scripts:

1. Restart your terminal for all changes to take effect
2. Some components may require logging out and back in
3. Check the output of each script for component-specific instructions

### ROS2 Environment

The installation will automatically configure your environment for the selected ROS2 distribution:

- For Ubuntu 22.04: ROS2 Humble environment will be set up
- For Ubuntu 24.04: ROS2 Jazzy environment will be set up

### Testing Your Environment

After installation, you can test your ROS2 environment with the Navigation2:

```bash
source ~/.bashrc
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```
