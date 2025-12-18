#!/bin/bash

# GodView Agent v3 - Launch Script with Virtual GPS Configuration

set -e

echo "╔════════════════════════════════════════════╗"
echo "║   GODVIEW AGENT V3 - LAUNCHER              ║"
echo "╚════════════════════════════════════════════╝"
echo ""

# Parse command line arguments
AGENT_ID="${1:-agent_warehouse_1}"
GPS_LAT="${2:-37.7749}"
GPS_LON="${3:--122.4194}"
GPS_ALT="${4:-10.0}"
HEADING="${5:-0.0}"

echo "📍 Agent Configuration:"
echo "   ID: $AGENT_ID"
echo "   GPS: ($GPS_LAT, $GPS_LON, ${GPS_ALT}m)"
echo "   Heading: ${HEADING}° (0°=North)"
echo ""

# Export environment variables
export AGENT_ID="$AGENT_ID"
export AGENT_GPS_LAT="$GPS_LAT"
export AGENT_GPS_LON="$GPS_LON"
export AGENT_GPS_ALT="$GPS_ALT"
export AGENT_HEADING="$HEADING"

# Check if Rust is installed
if ! command -v cargo &> /dev/null; then
    echo "❌ Rust/Cargo not found!"
    echo ""
    echo "Install Rust:"
    echo "  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
    echo ""
    exit 1
fi

# Check if godview_core is built
if [ ! -d "../godview_core/target" ]; then
    echo "⚠️  GodView Core library not built yet"
    echo "Building godview_core..."
    cd ../godview_core
    cargo build --release
    cd ../agent
    echo "✅ GodView Core built successfully"
    echo ""
fi

# Build and run agent
echo "🔨 Building GodView Agent v3..."
cargo build --release

echo ""
echo "🚀 Launching agent..."
echo "════════════════════════════════════════════"
echo ""

cargo run --release
