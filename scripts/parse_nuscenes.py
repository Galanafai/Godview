#!/usr/bin/env python3
"""
Parse nuScenes mini dataset to extract 3D object tracks for GodView demo.

Output: data/nuscenes_tracks.json
Format:
{
  "scenes": [
    {
      "name": "scene-0001",
      "frames": [
        {
          "timestamp": 1234567890.123,
          "ego_pose": [x, y, z, qw, qx, qy, qz],
          "objects": [
            {
              "instance_id": "abc123...",
              "category": "vehicle.car",
              "position": [x, y, z],
              "velocity": [vx, vy, vz],
              "size": [w, l, h]
            }
          ]
        }
      ]
    }
  ]
}
"""

import json
import os
from pathlib import Path
from collections import defaultdict

# nuScenes mini data path
NUSCENES_ROOT = Path("data/nuscenes")
OUTPUT_FILE = Path("data/nuscenes_tracks.json")

def load_json(filename):
    """Load a JSON file from the nuScenes v1.0-mini directory."""
    path = NUSCENES_ROOT / "v1.0-mini" / filename
    if not path.exists():
        print(f"WARNING: {path} not found")
        return []
    with open(path) as f:
        return json.load(f)

def main():
    print("🔍 Parsing nuScenes mini dataset...")
    
    # Load metadata tables
    scenes = load_json("scene.json")
    samples = load_json("sample.json")
    sample_annotations = load_json("sample_annotation.json")
    instances = load_json("instance.json")
    categories = load_json("category.json")
    ego_poses = load_json("ego_pose.json")
    sample_data = load_json("sample_data.json")
    
    print(f"  Scenes: {len(scenes)}")
    print(f"  Samples: {len(samples)}")
    print(f"  Annotations: {len(sample_annotations)}")
    print(f"  Instances: {len(instances)}")
    
    # Build lookup tables
    sample_by_token = {s['token']: s for s in samples}
    instance_by_token = {i['token']: i for i in instances}
    category_by_token = {c['token']: c for c in categories}
    ego_pose_by_token = {e['token']: e for e in ego_poses}
    
    # Map sample_token -> annotations
    annotations_by_sample = defaultdict(list)
    for ann in sample_annotations:
        annotations_by_sample[ann['sample_token']].append(ann)
    
    # Map sample_token -> ego_pose (via LIDAR_TOP sample_data)
    sample_to_ego = {}
    for sd in sample_data:
        # nuScenes uses 'channel' in older versions, 'sensor_channel' in newer
        channel = sd.get('channel', sd.get('sensor_channel', ''))
        if sd.get('is_key_frame', False) and 'LIDAR_TOP' in channel:
            sample_to_ego[sd['sample_token']] = sd['ego_pose_token']
    
    # Process each scene
    output = {"scenes": []}
    
    for scene in scenes:
        scene_name = scene['name']
        print(f"\n📍 Processing {scene_name}...")
        
        scene_data = {
            "name": scene_name,
            "description": scene.get('description', ''),
            "frames": []
        }
        
        # Walk through samples in this scene
        sample_token = scene['first_sample_token']
        frame_count = 0
        
        while sample_token:
            sample = sample_by_token.get(sample_token)
            if not sample:
                break
            
            # Get timestamp
            timestamp = sample['timestamp'] / 1e6  # Convert to seconds
            
            # Get ego pose
            ego_pose = None
            ego_token = sample_to_ego.get(sample_token)
            if ego_token and ego_token in ego_pose_by_token:
                ep = ego_pose_by_token[ego_token]
                ego_pose = {
                    "translation": ep['translation'],
                    "rotation": ep['rotation']
                }
            
            # Get all annotations for this sample
            anns = annotations_by_sample.get(sample_token, [])
            objects = []
            
            for ann in anns:
                # Get instance info
                inst = instance_by_token.get(ann['instance_token'], {})
                cat_token = inst.get('category_token')
                cat = category_by_token.get(cat_token, {})
                
                obj = {
                    "instance_id": ann['instance_token'][:12],  # Shortened
                    "category": cat.get('name', 'unknown'),
                    "position": ann['translation'],  # [x, y, z] in global coords
                    "velocity": ann.get('velocity', [0, 0]),  # [vx, vy] (2D only in nuScenes)
                    "size": ann['size']  # [w, l, h]
                }
                objects.append(obj)
            
            frame_data = {
                "timestamp": timestamp,
                "ego_pose": ego_pose,
                "objects": objects
            }
            scene_data["frames"].append(frame_data)
            
            frame_count += 1
            sample_token = sample.get('next', '')
        
        print(f"   Extracted {frame_count} frames with {len(scene_data['frames'][-1]['objects']) if scene_data['frames'] else 0} objects")
        output["scenes"].append(scene_data)
    
    # Write output
    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(OUTPUT_FILE, 'w') as f:
        json.dump(output, f, indent=2)
    
    print(f"\n✅ Saved to {OUTPUT_FILE}")
    print(f"   Total scenes: {len(output['scenes'])}")
    total_frames = sum(len(s['frames']) for s in output['scenes'])
    print(f"   Total frames: {total_frames}")

if __name__ == "__main__":
    main()
