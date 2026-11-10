"""Cut the GodView demo clip from rendered parts, with captions burned in.

    python3 make_clip.py --dashcam DIR --overlay DIR --orbit DIR --out clip.mp4
        [--freeze 145] [--start 118] [--end 205] [--size 1920x1080] [--font-dir DIR]

Parts:
  dashcam  raw front-camera frames NNNNNN.jpeg (10 Hz, dataset frame numbers)
  overlay  overlay.py output overlay_NNNNNN.jpg (run with --no-hud)
  orbit    build_scene.py --freeze N --shot orbit animation frames (30 fps)

Timeline (30 fps): real footage up to the paused frame, the paused-time 3D
orbit, the footage with GodView's ghosts from the paused frame on, an end card.
Real time is never stretched: the footage runs at its recorded speed, and the
3D part is labelled as paused time.
"""

import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter, ImageFont

CYAN = (64, 220, 255)
FPS = 30
XFADE = 8


def font(dirs, names, size):
    for d in dirs:
        for n in names:
            p = Path(d) / n
            if p.exists():
                return ImageFont.truetype(str(p), size)
    return ImageFont.load_default()


class Fonts:
    def __init__(self, extra):
        dirs = [d for d in [extra] if d] + ["/usr/share/fonts/opentype/inter",
                                            "/usr/share/fonts/truetype/dejavu"]
        self.dirs = dirs

    def get(self, weight, size):
        names = {"bold": ["Inter-Bold.otf", "DejaVuSans-Bold.ttf"],
                 "semi": ["Inter-SemiBold.otf", "DejaVuSans-Bold.ttf"],
                 "reg": ["Inter-Regular.otf", "DejaVuSans.ttf"]}[weight]
        return font(self.dirs, names, size)


def nearest(frames, n):
    """Closest available frame number at or before n (or the first one)."""
    best = None
    for k in frames:
        if k <= n and (best is None or k > best):
            best = k
    return best if best is not None else min(frames)


def fit(img, size):
    return img.convert("RGB").resize(size, Image.LANCZOS) if img.size != size else img.convert("RGB")


def caption(img, text, fonts, scale):
    """Large caption, lower third, on a soft dark band. Text in *stars* is cyan."""
    w, h = img.size
    f = fonts.get("semi", int(52 * scale))
    band = Image.new("RGBA", img.size, (0, 0, 0, 0))
    bd = ImageDraw.Draw(band)
    y0 = int(h * 0.76)
    for i in range(int(h * 0.24)):
        a = int(150 * min(1.0, i / (h * 0.10)))
        bd.line([(0, y0 + i), (w, y0 + i)], fill=(0, 4, 10, a))
    out = Image.alpha_composite(img.convert("RGBA"), band)
    d = ImageDraw.Draw(out)
    parts = text.split("*")
    widths = [d.textbbox((0, 0), p, font=f)[2] for p in parts]
    x = (w - sum(widths)) / 2
    y = int(h * 0.84)
    for i, (p, pw) in enumerate(zip(parts, widths)):
        d.text((x, y), p, font=f, fill=CYAN + (255,) if i % 2 else (255, 255, 255, 255))
        x += pw
    return out.convert("RGB")


def tag(img, text, fonts, scale):
    d = ImageDraw.Draw(img)
    f = fonts.get("semi", int(22 * scale))
    x, y = int(40 * scale), int(36 * scale)
    bb = d.textbbox((x, y), text, font=f)
    pad = int(10 * scale)
    d.rounded_rectangle([bb[0] - pad, bb[1] - pad, bb[2] + pad, bb[3] + pad], radius=int(8 * scale),
                        fill=(6, 14, 22), outline=CYAN, width=max(1, int(2 * scale)))
    d.text((x, y), text, font=f, fill=CYAN)
    return img


def end_card(size, fonts, scale):
    w, h = size
    img = Image.new("RGB", size, (4, 8, 16))
    glow = Image.new("RGB", size, (0, 0, 0))
    gd = ImageDraw.Draw(glow)
    gd.ellipse([w * 0.2, h * 0.05, w * 0.8, h * 0.75], fill=(10, 40, 70))
    img = Image.blend(img, glow.filter(ImageFilter.GaussianBlur(160 * scale)), 0.6)
    d = ImageDraw.Draw(img)

    def centred(text, f, y, fill):
        tw = d.textbbox((0, 0), text, font=f)[2]
        d.text(((w - tw) / 2, y), text, font=f, fill=fill)
    centred("GodView", fonts.get("bold", int(124 * scale)), h * 0.26, (255, 255, 255))
    centred("Robots and vehicles that share what they see", fonts.get("reg", int(40 * scale)),
            h * 0.44, CYAN)
    centred("Open source  ·  github.com/Galanafai/Godview", fonts.get("semi", int(32 * scale)),
            h * 0.56, (220, 230, 240))
    centred("Real camera footage and lidar from the V2X-Real dataset (UCLA). "
            "Detections from dataset annotations. 3D view: time paused.",
            fonts.get("reg", int(21 * scale)), h * 0.86, (130, 145, 160))
    return img


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dashcam", required=True)
    p.add_argument("--overlay", required=True)
    p.add_argument("--orbit", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--freeze", type=int, default=145)
    p.add_argument("--start", type=int, default=118)
    p.add_argument("--end", type=int, default=205)
    p.add_argument("--size", default="1920x1080")
    p.add_argument("--font-dir", default="")
    p.add_argument("--end-seconds", type=float, default=3.5)
    a = p.parse_args()
    size = tuple(int(v) for v in a.size.split("x"))
    scale = size[0] / 1920.0
    fonts = Fonts(a.font_dir)

    dash = {int(f.stem): f for f in Path(a.dashcam).glob("*.jpeg")}
    over = {int(f.stem.split("_")[-1]): f for f in Path(a.overlay).glob("overlay_*.jpg")}
    orbit = sorted(Path(a.orbit).glob("*.png"))
    if not dash or not over or not orbit:
        raise SystemExit("missing input frames")

    frames = []  # list of PIL images, 30 fps

    # A: real footage up to the paused moment
    n_a = (a.freeze - a.start) * 3
    for i in range(n_a):
        fr = a.start + i // 3
        img = fit(Image.open(dash[nearest(dash, fr)]), size)
        text = "Stopped at a red light." if i < n_a * 0.45 else "It can't see behind *this truck*."
        frames.append(caption(img, text, fonts, scale))

    # B: paused-time orbit
    n_b = len(orbit)
    b_frames = []
    for i, f in enumerate(orbit):
        img = fit(Image.open(f), size)
        img = tag(img, "TIME PAUSED  ·  3D VIEW OF WHAT GODVIEW KNOWS", fonts, scale)
        text = ("Nearby sensors share what they see." if i < n_b * 0.42
                else "Roadside units see *people behind the truck*.")
        b_frames.append(caption(img, text, fonts, scale))

    # C: footage again from the paused moment, with GodView's ghosts
    n_c = (a.end - a.freeze) * 3
    c_frames = []
    for i in range(n_c):
        fr = a.freeze + i // 3
        img = fit(Image.open(over[nearest(over, fr)]), size)
        text = ("GodView draws them *through the truck*..." if i < n_c * 0.4
                else "...so the car knows *before they step out*.")
        c_frames.append(caption(img, text, fonts, scale))

    # D: end card
    card = end_card(size, fonts, scale)
    d_frames = [card] * int(a.end_seconds * FPS)

    def join(seq, nxt):
        """Crossfade the last XFADE frames of seq into the first of nxt."""
        for k in range(XFADE):
            t = (k + 1) / (XFADE + 1)
            seq[-XFADE + k] = Image.blend(seq[-XFADE + k], nxt[k], t)
        return seq + nxt[XFADE:]

    frames = join(frames, b_frames)
    frames = join(frames, c_frames)
    frames = join(frames, d_frames)

    tmp = Path(tempfile.mkdtemp(prefix="clip_"))
    try:
        for i, img in enumerate(frames):
            img.save(tmp / f"{i:05d}.jpg", quality=94)
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS),
                        "-i", str(tmp / "%05d.jpg"), "-c:v", "libx264", "-preset", "veryfast",
                        "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.out],
                       check=True)
    finally:
        shutil.rmtree(tmp)
    print(f"wrote {a.out}: {len(frames)} frames, {len(frames) / FPS:.1f} s")


if __name__ == "__main__":
    main()
