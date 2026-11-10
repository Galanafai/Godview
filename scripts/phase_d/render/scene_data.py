"""Load a D4 scene package into per-track timelines for rendering.

No Blender dependency, so it can be checked with plain Python. Everything
rendered comes from GodView's ego view (`ego_view`) and the agents' poses;
dataset labels are only used to estimate the ground height.

Frames are the dataset's 10 Hz frames. Positions are in the package's odom
frame (east, north, up, metres).
"""

import json
import math
from pathlib import Path

import numpy as np

CLASS_NAMES = {1: "vehicle", 2: "pedestrian", 3: "truck"}

# Vehicle footprint from the D2 converter defaults (other vehicle's label):
# box centre 0.60 m ahead of and 0.12 m left of the lidar.
VEHICLE_SIZE = (4.57, 2.10, 2.29)
VEHICLE_CENTRE_FROM_LIDAR = (0.60, 0.12)


def unwrap_deg(seq):
    """Unwrap a list of angles in degrees so consecutive values differ by < 180."""
    out = []
    for a in seq:
        if not out:
            out.append(a)
            continue
        prev = out[-1]
        d = (a - prev + 180.0) % 360.0 - 180.0
        out.append(prev + d)
    return out


class Scene:
    def __init__(self, package_dir):
        self.dir = Path(package_dir)
        self.manifest = json.loads((self.dir / "manifest.json").read_text())
        files = sorted((self.dir / "frames").glob("*.json"))
        self.frames = [json.loads(f.read_text()) for f in files]
        for fr in self.frames:  # the full package stores labels as "truth"
            if "labels" not in fr and "truth" in fr:
                fr["labels"] = fr["truth"]
        self.frame_ids = [f["frame"] for f in self.frames]
        self.ego_index = self.manifest["vehicle_of_interest"]["index"]
        self.agents = {a["index"]: a for a in self.manifest["agents"]}
        self.ground_z = self._ground_z()
        self.tracks = self._tracks()
        self.agent_tracks = self._agent_tracks()

    # ------------------------------------------------------------------
    def _ground_z(self):
        zs = []
        for f in self.frames:
            for l in f["labels"]:
                if l["class_id"] in (1, 3):
                    zs.append(l["pos"][2] - l["size"][2] / 2.0)
        return float(np.median(zs)) if zs else 0.0

    def _tracks(self):
        """Per GodView object id: list of (frame, state) over the frames it exists.

        Agents' own vehicles (self objects) are skipped here; they are drawn
        from the agents' poses.
        """
        tracks = {}
        for f in self.frames:
            for o in f["ego_view"]:
                if o.get("self_agent") is not None:
                    continue
                tracks.setdefault(o["id"], []).append((f["frame"], o))
        out = {}
        for tid, seq in tracks.items():
            yaws = [s["box"]["yaw_deg"] if s.get("box") else 0.0 for _, s in seq]
            # Pedestrians: face the direction of motion when moving, since a
            # pedestrian box yaw is not meaningful.
            cls = seq[0][1]["class_id"]
            if cls == 2:
                yaws = []
                last = None
                for _, s in seq:
                    vx, vy = s["vel"][0], s["vel"][1]
                    if math.hypot(vx, vy) > 0.3:
                        last = math.degrees(math.atan2(vy, vx))
                    yaws.append(last if last is not None else 0.0)
                first = next((y for y in yaws if y is not None), 0.0)
                yaws = [first if y is None else y for y in yaws]
            yaws = unwrap_deg(yaws)
            states = []
            for (fr, s), yaw in zip(seq, yaws):
                b = s.get("box") or {}
                default = {1: (4.5, 2.0, 1.6), 2: (0.6, 0.6, 1.75), 3: (7.0, 2.6, 3.0)}[cls]
                size = (b.get("l", default[0]), b.get("w", default[1]), b.get("h", default[2]))
                if cls == 2:
                    size = (max(size[0], 0.45), max(size[1], 0.45), max(size[2], 1.5))
                cov = np.array(s["cov_pos"])
                states.append({
                    "frame": fr,
                    "pos": (s["x"], s["y"], s["z"]),
                    "yaw_deg": yaw,
                    "size": size,
                    "status": s["status"],
                    "cov_xy": cov[:2, :2],
                    "agents": list(s["contributing_agents"]),
                    "merged": bool(s.get("merged_labels")),
                    "labels": list(s.get("attributed_labels") or []),
                })
            out[tid] = {"class_id": cls, "states": states}
        return out

    def _agent_tracks(self):
        """Per agent index: per-frame sensor position, heading and body centre."""
        out = {}
        for f in self.frames:
            for a in f["agents"]:
                yaw = a["lidar_pose_map"][4]
                x, y, z = a["ego_odom"]
                entry = {"frame": f["frame"], "sensor": (x, y, z), "yaw_deg": yaw}
                if a["kind"] == "vehicle":
                    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
                    fx, fy = VEHICLE_CENTRE_FROM_LIDAR
                    entry["centre"] = (x + c * fx - s * fy, y + s * fx + c * fy,
                                       self.ground_z + VEHICLE_SIZE[2] / 2.0)
                out.setdefault(a["index"], {"kind": a["kind"], "folder": a["folder"],
                                            "states": []})["states"].append(entry)
        for t in out.values():
            ys = unwrap_deg([s["yaw_deg"] for s in t["states"]])
            for s, y in zip(t["states"], ys):
                s["yaw_deg"] = y
        return out

    # ------------------------------------------------------------------
    def static_points(self, max_points=None, seed=0):
        p = np.load(self.dir / "points" / "static_odom.npy")
        if max_points and len(p) > max_points:
            idx = np.random.default_rng(seed).choice(len(p), max_points, replace=False)
            p = p[np.sort(idx)]
        return p

    def ego_state(self, frame):
        for s in self.agent_tracks[self.ego_index]["states"]:
            if s["frame"] == frame:
                return s
        raise KeyError(frame)

    def focus_tracks(self, label_gids):
        """GodView object ids whose attributed labels include any of label_gids."""
        ids = set()
        for tid, t in self.tracks.items():
            for s in t["states"]:
                if set(s["labels"]) & set(label_gids):
                    ids.add(tid)
        return sorted(ids)

    def summary(self):
        n_ghost = sum(1 for t in self.tracks.values() for s in t["states"] if s["status"] == "ghost")
        return {
            "frames": (self.frame_ids[0], self.frame_ids[-1], len(self.frame_ids)),
            "ground_z": round(self.ground_z, 3),
            "tracks": len(self.tracks),
            "ghost_object_frames": n_ghost,
            "classes": {CLASS_NAMES[c]: sum(1 for t in self.tracks.values() if t["class_id"] == c)
                        for c in CLASS_NAMES},
            "agents": {i: t["kind"] for i, t in self.agent_tracks.items()},
        }


if __name__ == "__main__":
    import sys
    s = Scene(sys.argv[1])
    print(json.dumps(s.summary(), indent=1, default=str))
    print("focus", s.focus_tracks(["58", "30", "46"]))
    e = s.ego_state(s.frame_ids[0])
    print("ego", e)
