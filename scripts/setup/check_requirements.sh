#!/bin/bash

# ============================================
# GODVIEW SYSTEM REQUIREMENTS CHECKER
# ============================================

echo "╔════════════════════════════════════════════╗"
echo "║   GODVIEW - SYSTEM REQUIREMENTS CHECK      ║"
echo "╚════════════════════════════════════════════╝"
echo ""

MISSING_DEPS=0

# Check Rust/Cargo
echo -n "Checking Rust/Cargo... "
if command -v cargo &> /dev/null; then
    RUST_VERSION=$(cargo --version)
    echo "✓ $RUST_VERSION"
else
    echo "✗ NOT FOUND"
    echo "  Install: curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
    MISSING_DEPS=1
fi

# Check Node.js
echo -n "Checking Node.js... "
if command -v node &> /dev/null; then
    NODE_VERSION=$(node --version)
    echo "✓ $NODE_VERSION"
else
    echo "✗ NOT FOUND"
    echo "  Install: curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash - && sudo apt-get install -y nodejs"
    MISSING_DEPS=1
fi

# Check npm
echo -n "Checking npm... "
if command -v npm &> /dev/null; then
    NPM_VERSION=$(npm --version)
    echo "✓ v$NPM_VERSION"
else
    echo "✗ NOT FOUND"
    MISSING_DEPS=1
fi

# Check Zenoh Router
echo -n "Checking Zenoh Router (zenohd)... "
if command -v zenohd &> /dev/null; then
    ZENOH_VERSION=$(zenohd --version 2>&1 | head -n1)
    echo "✓ $ZENOH_VERSION"
else
    echo "✗ NOT FOUND"
    echo "  Install: See README.md for Zenoh installation instructions"
    MISSING_DEPS=1
fi

# Check OpenCV (system library)
echo -n "Checking OpenCV (pkg-config)... "
if pkg-config --exists opencv4; then
    OPENCV_VERSION=$(pkg-config --modversion opencv4)
    echo "✓ v$OPENCV_VERSION"
elif pkg-config --exists opencv; then
    OPENCV_VERSION=$(pkg-config --modversion opencv)
    echo "✓ v$OPENCV_VERSION"
else
    echo "✗ NOT FOUND"
    echo "  Install: sudo apt-get install -y libopencv-dev clang libclang-dev"
    MISSING_DEPS=1
fi

# Check webcam device
echo -n "Checking webcam device... "
if [ -e /dev/video0 ]; then
    echo "✓ /dev/video0 exists"
else
    echo "⚠ /dev/video0 not found (may need to plug in webcam)"
fi

# Check Haar Cascade file
echo -n "Checking Haar Cascade model... "
if [ -f "agent/haarcascade_frontalface_alt.xml" ]; then
    echo "✓ Found"
else
    echo "✗ NOT FOUND"
    echo "  Run: cd agent && wget https://raw.githubusercontent.com/opencv/opencv/4.x/data/haarcascades/haarcascade_frontalface_alt.xml"
    MISSING_DEPS=1
fi

echo ""
echo "════════════════════════════════════════════"

if [ $MISSING_DEPS -eq 0 ]; then
    echo "✓ All dependencies satisfied!"
    echo ""
    echo "Ready to launch GodView:"
    echo "  ./run_godview.sh"
else
    echo "✗ Missing dependencies detected"
    echo ""
    echo "Please install the missing components above,"
    echo "then run this script again to verify."
fi

echo "════════════════════════════════════════════"
