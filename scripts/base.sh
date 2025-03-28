#!/bin/bash

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

echo "Starting base installation process..."

# Update and install basic packages
echo "Updating system and installing basic packages..."
sudo apt update -y && sudo apt upgrade -y
sudo apt install git curl wget python-is-python3 build-essential xclip ssh -y
sudo systemctl enable ssh
sudo ufw allow ssh
sudo ufw allow in proto tcp from 192.169.1.0/24
sudo ufw allow in proto udp from 192.169.1.0/24
sudo ufw enable
check_status "Basic package installation"

# Install oh my bash with unattended flag - with improved error handling
echo "Installing Oh-My-Bash..."
if [ -d "$HOME/.oh-my-bash" ]; then
  echo "Removing existing Oh-My-Bash installation..."
  rm -rf "$HOME/.oh-my-bash"
fi

bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)" --unattended
if [ -d "$HOME/.oh-my-bash" ]; then
  echo "✓ Oh-My-Bash installation completed successfully"
else
  echo "✗ Error during Oh-My-Bash installation"
  exit 1
fi

# Install GitHub CLI
echo "Installing GitHub CLI..."
type -p wget >/dev/null || (sudo apt update && sudo apt-get install wget -y)
sudo mkdir -p -m 755 /etc/apt/keyrings
out=$(mktemp) && wget -nv -O$out https://cli.github.com/packages/githubcli-archive-keyring.gpg
cat $out | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg >/dev/null
sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list >/dev/null
sudo apt update
sudo apt install gh -y
check_status "GitHub CLI installation"

# Install Powerline Fonts
echo "Installing Powerline Fonts..."
git clone https://github.com/powerline/fonts.git --depth=1
cd fonts
./install.sh
cd ..
rm -rf fonts
check_status "Powerline Fonts installation"

# Install latest Neovim
echo "Installing latest Neovim..."
sudo add-apt-repository ppa:neovim-ppa/unstable -y
sudo apt update
sudo apt install neovim -y
check_status "Neovim installation"

# Configure vim alias
echo "Configuring vim alias..."
if ! grep -q "alias vim=nvim" ~/.bashrc; then
  echo "alias vim=nvim" >>~/.bashrc
fi
check_status "vim alias configuration"

# Install latest tmux
echo "Installing latest tmux..."
sudo apt install tmux -y
check_status "tmux installation"

# Configure tmux
echo "Configuring tmux..."
cd ~
git clone --single-branch https://github.com/gpakosz/.tmux.git
ln -s -f .tmux/.tmux.conf
cp .tmux/.tmux.conf.local .
check_status "tmux configuration"

# Install pip3
echo "Installing pip3..."
sudo apt install python3-pip -y
check_status "pip3 installation"

# Install Python development tools
echo "Installing Python development tools..."
sudo apt install pylint python3-yapf isort python3-neovim -y
check_status "Python development tools installation"

echo "Base installation completed successfully!"
echo ""
echo "Important post-installation steps:"
echo "1. Please restart your terminal for all changes to take effect"
echo "2. You may need to log out and back in for all font changes to take effect"
