#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

echo "Starting Visual Studio Code installation..."

# Install VSCode
echo "Installing Visual Studio Code..."
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor >packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/packages.microsoft.gpg
rm -f packages.microsoft.gpg

# Add the VS Code repository
echo "deb [arch=amd64,arm64,armhf signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list

# Update package cache and install VS Code
sudo apt update
sudo apt install -y code
check_status "VS Code installation"

# Install VSCode extensions
echo "Installing VSCode extensions..."
code --install-extension danielroedl.meld-diff
code --install-extension ms-azuretools.vscode-docker
code --install-extension ms-iot.vscode-ros
code --install-extension ms-python.debugpy
code --install-extension ms-python.python
code --install-extension ms-python.vscode-pylance
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.cpptools-extension-pack
code --install-extension ms-vscode.cpptools-themes
code --install-extension ms-vscode-remote.remote-containers
code --install-extension oderwat.indent-rainbow
code --install-extension robbowen.synthwave-vscode
code --install-extension analytic-signal.preview-pdf
check_status "VSCode extensions installation"

# Configure VSCode settings
echo "Configuring VSCode settings..."
VSCODE_CONFIG_DIR="/home/$USER/.config/Code/User"
mkdir -p "$VSCODE_CONFIG_DIR"

cat > "$VSCODE_CONFIG_DIR/settings.json" << EOF
{
    "workbench.colorTheme": "SynthWave '84",
    "python.defaultInterpreterPath": "/bin/python",
    "workbench.sideBar.location": "right"
}
EOF
check_status "VSCode settings configuration"

echo "Visual Studio Code installation completed successfully!"
