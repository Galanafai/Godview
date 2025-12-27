#!/bin/bash
set -e

# GodView CARLA Setup Script for RunPod (Clean Install)
# Usage: bash setup_carla.sh

echo "=== [1/5] Installing System Dependencies ==="
apt-get update -qq
apt-get install -y -qq xvfb libsdl2-2.0-0 xdg-user-dirs vulkan-tools libvulkan1

echo "=== [2/5] Creating Service User 'carla' ==="
# Create user if not exists
if ! id "carla" &>/dev/null; then
    useradd -m -s /bin/bash -u 1000 carla
fi
# Add to video group if exists
usermod -aG video carla || true

echo "=== [3/5] preparing Workspace ==="
# We install to a subdirectory we can control
INSTALL_DIR="/workspace/carla_sim"
mkdir -p "$INSTALL_DIR"

# Attempt to give ownership to carla user
# If this fails on Network Volume, we warn but proceed
chown carla:carla "$INSTALL_DIR" || echo "WARNING: Could not chown $INSTALL_DIR. Permissions might be tricky."

echo "=== [4/5] Downloading & Extracting CARLA ==="
# Run extraction AS the carla user to avoid permission issues later
su - carla -c "
    cd '$INSTALL_DIR'
    if [ ! -f 'CarlaUE4.sh' ]; then
        echo 'Downloading CARLA 0.9.15...'
        wget -q --show-progress https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.15.tar.gz
        
        echo 'Extracting...'
        tar -xzf CARLA_0.9.15.tar.gz
        rm CARLA_0.9.15.tar.gz
    else
        echo 'CARLA already found in $INSTALL_DIR'
    fi
"

echo "=== [5/5] Creating Headless Launcher ==="
cat <<EOF > "$INSTALL_DIR/run_headless.sh"
#!/bin/bash
export SDL_VIDEODRIVER=offscreen
export SDL_HINT_CUDA_DEVICE=0
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

echo "Starting CARLA Headless (Quality: Low, FPS: 20)..."
# Use xvfb-run to provide virtual display
xvfb-run -s "-screen 0 1280x720x24" \\
    ./CarlaUE4.sh -RenderOffScreen -quality-level=Low -benchmark -fps=20 -carla-rpc-port=2000
EOF

chmod +x "$INSTALL_DIR/run_headless.sh"
chown carla:carla "$INSTALL_DIR/run_headless.sh"

echo ""
echo "✅ Setup Complete!"
echo "To start CARLA:"
echo "  su - carla -c '$INSTALL_DIR/run_headless.sh'"
