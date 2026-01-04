#!/bin/bash

# ============================================
# GODVIEW - QUICK SETUP SCRIPT
# Installs all required dependencies
# ============================================

set -e

echo "╔════════════════════════════════════════════╗"
echo "║     GODVIEW - DEPENDENCY INSTALLER         ║"
echo "╚════════════════════════════════════════════╝"
echo ""
echo "This script will install:"
echo "  • Rust toolchain"
echo "  • Node.js 18+"
echo "  • OpenCV development libraries"
echo "  • Zenoh router"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Installation cancelled."
    exit 1
fi

# Update package list
echo ""
echo "--- Updating package list ---"
sudo apt-get update

# Install OpenCV and build dependencies
echo ""
echo "--- Installing OpenCV and build tools ---"
sudo apt-get install -y libopencv-dev clang libclang-dev pkg-config

# Install Rust
echo ""
echo "--- Installing Rust ---"
if ! command -v cargo &> /dev/null; then
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source "$HOME/.cargo/env"
    echo "Rust installed successfully"
else
    echo "Rust already installed"
fi

# Install Node.js
echo ""
echo "--- Installing Node.js ---"
if ! command -v node &> /dev/null; then
    curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
    sudo apt-get install -y nodejs
    echo "Node.js installed successfully"
else
    echo "Node.js already installed"
fi

# Install Zenoh Router
echo ""
echo "--- Installing Zenoh Router ---"
if ! command -v zenohd &> /dev/null; then
    ZENOH_VERSION="1.0.3"
    ZENOH_URL="https://github.com/eclipse-zenoh/zenoh/releases/download/${ZENOH_VERSION}/zenohd-${ZENOH_VERSION}-x86_64-unknown-linux-gnu.zip"
    
    echo "Downloading Zenoh ${ZENOH_VERSION}..."
    wget -q "$ZENOH_URL" -O /tmp/zenohd.zip
    
    echo "Extracting..."
    unzip -q /tmp/zenohd.zip -d /tmp/
    
    echo "Installing to /usr/local/bin..."
    sudo mv /tmp/zenohd /usr/local/bin/
    sudo chmod +x /usr/local/bin/zenohd
    
    rm /tmp/zenohd.zip
    echo "Zenoh router installed successfully"
else
    echo "Zenoh router already installed"
fi

# Download Haar Cascade
echo ""
echo "--- Downloading Haar Cascade model ---"
cd agent
if [ ! -f "haarcascade_frontalface_alt.xml" ]; then
    wget -q https://raw.githubusercontent.com/opencv/opencv/4.x/data/haarcascades/haarcascade_frontalface_alt.xml
    echo "Haar Cascade downloaded"
else
    echo "Haar Cascade already exists"
fi
cd ..

echo ""
echo "╔════════════════════════════════════════════╗"
echo "║        INSTALLATION COMPLETE! 🎉           ║"
echo "╚════════════════════════════════════════════╝"
echo ""
echo "Next steps:"
echo "  1. Restart your terminal (or run: source ~/.cargo/env)"
echo "  2. Run: ./check_requirements.sh"
echo "  3. Launch GodView: ./run_godview.sh"
echo ""
