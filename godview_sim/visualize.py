#!/usr/bin/env python3
"""
Rerun visualization for GodView DST simulations.

Usage:
    # Run simulation with JSON export
    godview-sim --seed 42 --scenario time_warp --duration 10 --export sim_data.json
    
    # Visualize with Rerun
    python3 visualize.py sim_data.json
"""

import json
import sys
import rerun as rr
import numpy as np


def load_simulation_data(path: str) -> dict:
    """Load simulation data from JSON file."""
    with open(path, 'r') as f:
        return json.load(f)


def visualize(data: dict):
    """Visualize simulation data in Rerun."""
    rr.init("godview_sim", spawn=True)
    
    frames = data.get("frames", [])
    print(f"Visualizing {len(frames)} frames...")
    
    for frame in frames:
        time_sec = frame["time_sec"]
        # Use the new timeline API
        rr.set_time("sim_time", duration=time_sec)
        
        # Log ground truth (Oracle positions) - GREEN
        if "ground_truth" in frame:
            gt = frame["ground_truth"]
            if len(gt) > 0:
                positions = np.array([[e["x"], e["y"], e["z"]] for e in gt])
                rr.log(
                    "world/ground_truth",
                    rr.Points3D(
                        positions,
                        colors=[[0, 255, 0, 255]] * len(positions),  # Green
                        radii=[2.0] * len(positions)
                    )
                )
        
        # Log agent track estimates - different colors per agent
        agent_colors = [
            [255, 100, 100, 255],  # Red
            [100, 100, 255, 255],  # Blue
            [255, 255, 100, 255],  # Yellow
            [100, 255, 255, 255],  # Cyan
            [255, 100, 255, 255],  # Magenta
            [255, 165, 0, 255],    # Orange
        ]
        
        for i, agent in enumerate(frame.get("agents", [])):
            agent_id = agent.get("agent_id", i)
            tracks = agent.get("tracks", [])
            if tracks:
                positions = np.array([[t["x"], t["y"], t["z"]] for t in tracks])
                color = agent_colors[agent_id % len(agent_colors)]
                rr.log(
                    f"world/agents/agent_{agent_id}",
                    rr.Points3D(
                        positions,
                        colors=[color] * len(positions),
                        radii=[1.5] * len(positions)
                    )
                )
            
            # Log RMS error as scalar
            if "rms_error" in agent and agent["rms_error"] is not None:
                rr.log(
                    f"metrics/agent_{agent_id}/rms_error",
                    rr.Scalars([agent["rms_error"]])
                )
    
    # Print summary
    scenario = data.get("scenario", "unknown")
    seed = data.get("seed", 0)
    passed = data.get("passed", False)
    rms = data.get("final_rms_error", None)
    
    print(f"\n{'✓' if passed else '✗'} Scenario: {scenario}")
    print(f"  Seed: {seed}")
    if rms is not None:
        print(f"  Final RMS Error: {rms:.2f}m")
    print(f"\nVisualization complete! Check the Rerun viewer.")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 visualize.py <simulation_data.json>")
        print("\nGenerate data with:")
        print("  godview-sim --seed 42 --scenario time_warp --export sim_data.json")
        sys.exit(1)
    
    data = load_simulation_data(sys.argv[1])
    visualize(data)


if __name__ == "__main__":
    main()
