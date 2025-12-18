#!/bin/bash

# Multi-Agent Simulation Script for GodView v3
# Launches two agents at different GPS positions to test global coordinate system

set -e

echo "╔════════════════════════════════════════════╗"
echo "║   GODVIEW V3 - MULTI-AGENT SIMULATION      ║"
echo "╚════════════════════════════════════════════╝"
echo ""

# Kill any existing agents
pkill -f "godview_agent" || true
sleep 1

echo "🌍 Simulation Scenario:"
echo "   Agent A: Corner camera facing North"
echo "   Agent B: Corner camera facing East (90° rotated)"
echo "   Both detect same person at different angles"
echo ""

# Agent A - Northwest corner, facing North (0°)
echo "🚀 Launching Agent A (Northwest corner, facing North)..."
AGENT_ID="agent_nw_corner" \
AGENT_GPS_LAT=37.7749 \
AGENT_GPS_LON=-122.4194 \
AGENT_GPS_ALT=10.0 \
AGENT_HEADING=0.0 \
cargo run --release &

AGENT_A_PID=$!
echo "   ✅ Agent A started (PID: $AGENT_A_PID)"
sleep 2

# Agent B - Northeast corner, facing East (90°)
echo "🚀 Launching Agent B (Northeast corner, facing East)..."
AGENT_ID="agent_ne_corner" \
AGENT_GPS_LAT=37.7749 \
AGENT_GPS_LON=-122.4193 \
AGENT_GPS_ALT=10.0 \
AGENT_HEADING=90.0 \
cargo run --release &

AGENT_B_PID=$!
echo "   ✅ Agent B started (PID: $AGENT_B_PID)"
echo ""

echo "════════════════════════════════════════════"
echo "✅ Multi-agent simulation running!"
echo ""
echo "Agent A PID: $AGENT_A_PID"
echo "Agent B PID: $AGENT_B_PID"
echo ""
echo "Press Ctrl+C to stop all agents"
echo "════════════════════════════════════════════"
echo ""

# Cleanup handler
cleanup() {
    echo ""
    echo "🛑 Stopping agents..."
    kill $AGENT_A_PID $AGENT_B_PID 2>/dev/null || true
    echo "✅ Agents stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Wait for agents
wait
