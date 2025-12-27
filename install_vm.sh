#!/bin/bash
set -e

echo "=== STARTING INSTALLATION (PIPED) ==="

# 1. System Dependencies (Non-Interactive)
export DEBIAN_FRONTEND=noninteractive
# We use sudo -S to accept password from pipe if needed, but here we are in a piped script.
# If we run 'echo script | ssh', the script runs as user.
# User needs sudo.
# We will embed the password echo inside the commands.
PASS="password"

echo "Updating apt..."
echo "$PASS" | sudo -S -E apt-get update
echo "Installing libs..."
echo "$PASS" | sudo -S -E apt-get install -y libvulkan1 libomp5 xdg-user-dirs aria2 python3-pip unzip curl ffmpeg libx264-dev

# 2. Setup Workspace
echo "Setting up /workspace/carla..."
echo "$PASS" | sudo -S mkdir -p /workspace/carla
echo "$PASS" | sudo -S chown -R $(whoami) /workspace/carla
cd /workspace/carla

# 3. Download CARLA (Try 0.9.16, fallback 0.9.15)
URL_16="https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.16.tar.gz"
URL_15="https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.15.tar.gz"

if [ ! -f "CARLA_0.9.16.tar.gz" ] && [ ! -f "CARLA_0.9.15.tar.gz" ]; then
    echo "Checking 0.9.16..."
    if curl --output /dev/null --silent --head --fail "$URL_16"; then
        echo "Downloading 0.9.16..."
        aria2c -x 16 -s 16 "$URL_16"
        tar -xzf CARLA_0.9.16.tar.gz
    else
        echo "0.9.16 not found. Downloading 0.9.15..."
        aria2c -x 16 -s 16 "$URL_15"
        tar -xzf CARLA_0.9.15.tar.gz
    fi
else
    echo "CARLA tarball already exists. Skipping download."
    # We assume extraction?
    if [ ! -f "CarlaUE4.sh" ]; then
        echo "Extracting existing tarball..."
        tar -xzf CARLA*.tar.gz
    fi
fi

# 4. Python Dependencies
echo "Installing Python libs..."
# System wide pip or user? User is safer.
pip3 install --user ultralytics opencv-python numpy torch torchvision eclipse-zenoh pyzmq pyyaml matplotlib pandas
if [ -f "PythonAPI/carla/requirements.txt" ]; then
    pip3 install --user -r PythonAPI/carla/requirements.txt
fi

echo "=== INSTALLATION DONE ==="
