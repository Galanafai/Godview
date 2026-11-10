"""Draw GodView's ghost objects over a camera's real footage.

    python3 overlay.py --package DIR --outdir OUT [--camera 1/cam1] [--frames 130,160]
        [--max-range 35] [--focus 58,30,46] [--font PATH]

For each frame with an image, every ghost object in vehicle 1's ego view
(known only through other agents) within --max-range metres of the camera and
in front of it is projected into the image as a glowing box, drawn over
whatever hides it. Objects tied to the focus labels are drawn brighter and
tagged with the agents that reported them. Nothing else is drawn on the
footage. The projection follows scripts/phase_d/d4_scene_analysis.py.
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scene_data import Scene  # noqa: E402

CYAN = (64, 220, 255)
WHITE = (255, 255, 255)
AGENT_NAMES = {0: "roadside unit A", 1: "roadside unit B", 2: "this car", 3: "car 2"}


def rot(roll, yaw, pitch):
    cy, sy = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    cr, sr = np.cos(np.radians(roll)), np.sin(np.radians(roll))
    cp, sp = np.cos(np.radians(pitch)), np.sin(np.radians(pitch))
    return np.array([[cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
                     [sp, -cp * sr, cp * cr]])


def pose_matrix(p):
    m = np.eye(4)
    m[:3, :3] = rot(p[3], p[4], p[5])
    m[:3, 3] = p[:3]
    return m


def box_corners(centre, size, yaw_deg):
    half = np.array(size) / 2.0
    R = rot(0, yaw_deg, 0)
    return np.array([np.array(centre) + R @ (half * np.array(s)) for s in
                     [(1, 1, 1), (1, -1, 1), (-1, -1, 1), (-1, 1, 1),
                      (1, 1, -1), (1, -1, -1), (-1, -1, -1), (-1, 1, -1)]])


EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def project(points_odom, origin, cal):
    T = pose_matrix(cal["lidar_pose_map"])
    E = np.array(cal["extrinsic_cam_to_lidar"])
    K = np.array(cal["intrinsic"])
    M = np.linalg.inv(E) @ np.linalg.inv(T)
    out = []
    for p in points_odom:
        pc = (M @ np.r_[np.array(p) + origin, 1.0])[:3]
        out.append((K[0, 0] * pc[0] / pc[2] + K[0, 2], K[1, 1] * pc[1] / pc[2] + K[1, 2], pc[2]))
    return np.array(out)


def load_font(path, size):
    for p in [path, "/usr/share/fonts/opentype/inter/Inter-SemiBold.otf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"]:
        if p and Path(p).exists():
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def draw_frame(img, objs, font, scale, hud=True):
    w, h = img.size
    glow = Image.new("RGBA", img.size, (0, 0, 0, 0))
    fill = Image.new("RGBA", img.size, (0, 0, 0, 0))
    lines = Image.new("RGBA", img.size, (0, 0, 0, 0))
    gd, fd, ld = ImageDraw.Draw(glow), ImageDraw.Draw(fill), ImageDraw.Draw(lines)
    for o in objs:
        uvz = o["uvz"]
        if (uvz[:, 2] <= 0.5).any():
            continue
        uv = uvz[:, :2]
        if uv[:, 0].max() < 0 or uv[:, 0].min() > w or uv[:, 1].max() < 0 or uv[:, 1].min() > h:
            continue
        o["drawn"] = True
        strong = o["focus"]
        col = o.get("color", CYAN)
        a_fill = 70 if strong else 0
        a_line = 255 if strong else 90
        lw = max(2, int(round(3 * scale))) if strong else max(1, int(round(1.5 * scale)))
        # filled silhouette: convex hull of the projected corners
        hull = convex_hull(uv)
        fd.polygon([tuple(p) for p in hull], fill=col + (a_fill,))
        for i, j in EDGES:
            seg = [tuple(uv[i]), tuple(uv[j])]
            ld.line(seg, fill=col + (a_line,), width=lw)
            if strong:
                gd.line(seg, fill=col + (255,), width=lw * 4)
    glow = glow.filter(ImageFilter.GaussianBlur(radius=10 * scale))
    out = img.convert("RGBA")
    out = Image.alpha_composite(out, glow)
    out = Image.alpha_composite(out, fill)
    out = Image.alpha_composite(out, lines)
    if hud:
        draw_hud(out, objs, font, scale)
    return out.convert("RGB")


def draw_hud(img, objs, font, scale):
    """Bottom-left panel: how many objects are hidden from this car, and who saw them."""
    shown = [o for o in objs if o.get("drawn") and o.get("color", CYAN) == CYAN]
    if not shown:
        return
    agents = sorted({a for o in shown for a in o["agents"]})
    d = ImageDraw.Draw(img)
    small = font.font_variant(size=max(10, int(font.size * 0.72)))
    x0, y0 = 36 * scale, img.size[1] - 150 * scale
    lines = [("GODVIEW  ·  SHARED PERCEPTION", small, CYAN + (255,)),
             (f"{len(shown)} hidden from this car", font, (240, 250, 255, 255)),
             ("seen by " + ", ".join(AGENT_NAMES.get(a, f"agent {a}") for a in agents),
              small, (190, 225, 240, 255))]
    widths = [d.textbbox((0, 0), t, font=f)[2] for t, f, _ in lines]
    pad = 16 * scale
    panel = Image.new("RGBA", img.size, (0, 0, 0, 0))
    pd = ImageDraw.Draw(panel)
    pd.rounded_rectangle([x0 - pad, y0 - pad, x0 + max(widths) + pad, y0 + 112 * scale],
                         radius=10 * scale, fill=(6, 14, 22, 170))
    pd.line([x0 - pad, y0 - pad + 6 * scale, x0 - pad, y0 + 106 * scale], fill=CYAN + (255,),
            width=max(2, int(3 * scale)))
    img.alpha_composite(panel)
    d = ImageDraw.Draw(img)
    y = y0
    for text, f, col in lines:
        d.text((x0, y), text, font=f, fill=col)
        y += (f.size + 12 * scale)


def convex_hull(pts):
    pts = sorted(map(tuple, pts))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--package", required=True)
    p.add_argument("--outdir", default="overlay")
    p.add_argument("--camera", default="1/cam1")
    p.add_argument("--frames", default="")
    p.add_argument("--max-range", type=float, default=35.0)
    p.add_argument("--focus", default="58,30,46")
    p.add_argument("--font", default="")
    p.add_argument("--no-hud", action="store_true")
    args = p.parse_args()

    sc = Scene(args.package)
    pkg = Path(args.package)
    origin = np.array(sc.manifest["origin_map"])
    cal_all = json.loads((pkg / "calibration.json").read_text())[args.camera]
    focus = set(sc.focus_tracks([g for g in args.focus.split(",") if g]))
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)
    img_dir = pkg / "cameras" / args.camera
    wanted = {int(x) for x in args.frames.split(",") if x}
    by_frame = {f["frame"]: f for f in sc.frames}
    for img_path in sorted(img_dir.glob("*.jpeg")):
        fr = int(img_path.stem)
        if wanted and fr not in wanted:
            continue
        if fr not in by_frame:
            continue
        cal = cal_all if cal_all.get("static") else cal_all[f"{fr:06d}"]
        cam_pos = pose_matrix(cal["lidar_pose_map"])[:3, 3] - origin
        objs = []
        for tid, t in sc.tracks.items():
            s = next((s for s in t["states"] if s["frame"] == fr), None)
            if s is None:
                continue
            was_ghost = any(x["status"] == "ghost" and x["frame"] < fr for x in t["states"])
            revealed = s["status"] == "solid" and tid in focus and was_ghost
            if s["status"] != "ghost" and not revealed:
                continue
            if math.dist(s["pos"][:2], cam_pos[:2]) > args.max_range:
                continue
            corners = box_corners(s["pos"], s["size"], s["yaw_deg"])
            objs.append({"uvz": project(corners, origin, cal), "focus": tid in focus,
                         "agents": s["agents"], "color": WHITE if revealed else CYAN})
        img = Image.open(img_path).convert("RGB")
        scale = img.size[0] / 1920.0
        font = load_font(args.font, int(22 * scale))
        res = draw_frame(img, objs, font, scale, hud=not args.no_hud)
        res.save(out / f"overlay_{fr:06d}.jpg", quality=92)
        print("wrote", out / f"overlay_{fr:06d}.jpg", len(objs), "ghosts")


if __name__ == "__main__":
    main()
