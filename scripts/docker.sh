#!/bin/bash

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

echo "Starting Docker installation..."

# Install Docker
echo "Installing Docker prerequisites..."
sudo apt update
sudo apt install -y apt-transport-https ca-certificates curl software-properties-common
check_status "Docker prerequisites installation"

echo "Adding Docker repository..."
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
check_status "Docker repository configuration"

echo "Installing Docker CE..."
sudo apt update
sudo apt install -y docker-ce
check_status "Docker CE installation"

echo "Adding user to docker group..."
sudo usermod -aG docker ${USER}
check_status "Docker user configuration"

echo "Docker installation completed successfully!"
echo ""
echo "Important: You may need to log out and back in for docker group changes to take effect."
