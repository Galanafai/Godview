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


def draw_rich(d, text, f, cx, y, maxw):
    """Draw text centred at cx, wrapping to maxw; words inside *stars* are cyan."""
    words = []
    for i, chunk in enumerate(text.split("*")):
        for wd in chunk.split():
            if words and all(c in ".,;:!?" for c in wd):
                pw, phi = words[-1]
                words[-1] = (pw, phi, wd)
                continue
            words.append((wd, i % 2 == 1))
    words = [w if len(w) == 3 else (w[0], w[1], "") for w in words]
    lines, cur = [], []
    for wd in words:
        trial = " ".join(w + p for w, _, p in cur + [wd])
        if cur and d.textbbox((0, 0), trial, font=f)[2] > maxw:
            lines.append(cur)
            cur = [wd]
        else:
            cur.append(wd)
    if cur:
        lines.append(cur)
    lh = int(f.size * 1.25)
    space = d.textbbox((0, 0), "a b", font=f)[2] - d.textbbox((0, 0), "ab", font=f)[2]
    for li, line in enumerate(lines):
        widths = [d.textbbox((0, 0), w + p, font=f)[2] for w, _, p in line]
        x = cx - (sum(widths) + space * (len(line) - 1)) / 2
        for (w, hi, p), ww in zip(line, widths):
            d.text((x, y + li * lh), w, font=f, fill=CYAN if hi else (255, 255, 255))
            if p:
                d.text((x + d.textbbox((0, 0), w, font=f)[2], y + li * lh), p, font=f,
                       fill=(255, 255, 255))
            x += ww + space
    return len(lines) * lh


def compose(img, text, fonts, size, label=None, crop_centre=0.42):
    """16:9: caption over the picture. 4:5: picture in the middle, text above and below."""
    w, h = size
    if w / h > 1.5:
        scale = w / 1920.0
        out = caption(fit(img, size), text, fonts, scale)
        return tag(out, label, fonts, scale) if label else out
    canvas = Image.new("RGB", size, (5, 8, 15))
    src = img.convert("RGB")
    sw, sh = src.size
    cw = int(sh * 4 / 3)
    cx = int(sw * crop_centre)
    x0 = max(0, min(sw - cw, cx - cw // 2))
    src = src.crop((x0, 0, x0 + cw, sh))
    ph = int(w * 3 / 4)
    top = int(h * 0.10)
    canvas.paste(src.resize((w, ph), Image.LANCZOS), (0, top))
    d = ImageDraw.Draw(canvas)
    small = fonts.get("semi", int(w * 0.030))
    hdr = label or "GODVIEW  ·  SHARED PERCEPTION"
    tw = d.textbbox((0, 0), hdr, font=small)[2]
    d.text(((w - tw) / 2, top - int(w * 0.065)), hdr, font=small, fill=CYAN)
    big = fonts.get("bold", int(w * 0.060))
    draw_rich(d, text, big, w / 2, top + ph + int(w * 0.07), w * 0.88)
    return canvas


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
    scale = w / 1920.0 * (1.0 if w / h > 1.5 else 1.7)
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
    fine = fonts.get("reg", int(21 * scale))
    for k, line in enumerate(["Real camera footage and lidar from the V2X-Real dataset (UCLA).",
                              "Detections from dataset annotations. 3D view: time paused."]):
        centred(line, fine, h * 0.84 + k * fine.size * 1.4, (130, 145, 160))
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
    p.add_argument("--teaser", type=float, default=1.6)
    a = p.parse_args()
    size = tuple(int(v) for v in a.size.split("x"))
    scale = size[0] / 1920.0
    fonts = Fonts(a.font_dir)

    dash = {int(f.stem): f for f in Path(a.dashcam).glob("*.jpeg")}
    over = {int(f.stem.split("_")[-1]): f for f in Path(a.overlay).glob("overlay_*.jpg")}
    orbit = sorted(Path(a.orbit).glob("*.png"))
    if not dash or not over or not orbit:
        raise SystemExit("missing input frames")

    paused = "TIME PAUSED  ·  3D VIEW OF WHAT GODVIEW KNOWS"

    # Teaser: open on the payoff, then tell the story
    hook = Image.open(over[nearest(over, a.freeze + 5)])
    t_frames = [compose(hook, "This car knows who is *behind that truck*.", fonts, size)
                for _ in range(int(a.teaser * FPS))]

    # A: real footage up to the paused moment
    n_a = (a.freeze - a.start) * 3
    a_frames = []
    for i in range(n_a):
        fr = a.start + i // 3
        img = Image.open(dash[nearest(dash, fr)])
        text = ("Its own cameras and lidar can't see past it." if i < n_a * 0.5
                else "But it isn't the only sensor here.")
        a_frames.append(compose(img, text, fonts, size))

    # B: paused-time orbit
    n_b = len(orbit)
    b_frames = []
    for i, f in enumerate(orbit):
        text = ("Roadside units and another car share what they see." if i < n_b * 0.42
                else "They see *people behind the truck*.")
        b_frames.append(compose(Image.open(f), text, fonts, size, label=paused, crop_centre=0.5))

    # C: footage again from the paused moment, with GodView's ghosts
    n_c = (a.end - a.freeze) * 3
    c_frames = []
    for i in range(n_c):
        fr = a.freeze + i // 3
        img = Image.open(over[nearest(over, fr)])
        text = ("GodView fuses it into one shared picture," if i < n_c * 0.4
                else "so the car knows *before they step out*.")
        c_frames.append(compose(img, text, fonts, size))

    # D: end card
    card = end_card(size, fonts, scale)
    d_frames = [card] * int(a.end_seconds * FPS)

    def join(seq, nxt):
        """Crossfade the last XFADE frames of seq into the first of nxt."""
        for k in range(XFADE):
            t = (k + 1) / (XFADE + 1)
            seq[-XFADE + k] = Image.blend(seq[-XFADE + k], nxt[k], t)
        return seq + nxt[XFADE:]

    frames = join(t_frames, a_frames)
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
