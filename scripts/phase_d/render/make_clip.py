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
import json
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


def compose(img, text, fonts, size, label=None, crop_centre=0.42, crop=True):
    """16:9: caption over the picture. 4:5: picture in the middle, text above and below."""
    w, h = size
    if w / h > 1.5:
        scale = w / 1920.0
        out = caption(fit(img, size), text, fonts, scale)
        return tag(out, label, fonts, scale) if label else out
    canvas = Image.new("RGB", size, (5, 8, 15))
    src = img.convert("RGB")
    sw, sh = src.size
    if crop:
        cw = int(sh * 4 / 3)
        cx = int(sw * crop_centre)
        x0 = max(0, min(sw - cw, cx - cw // 2))
        src = src.crop((x0, 0, x0 + cw, sh))
    ph = int(w * src.size[1] / src.size[0])
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


def split(left, right, text, fonts, size):
    """Same moment side by side (16:9) or stacked (4:5): the car alone, and with GodView."""
    w, h = size
    canvas = Image.new("RGB", size, (5, 8, 15))
    d = ImageDraw.Draw(canvas)
    lab = fonts.get("bold", int(w * (0.022 if w / h > 1.5 else 0.040)))
    names = [("WHAT THE CAR SEES", (235, 240, 245)), ("WITH GODVIEW", CYAN)]
    if w / h > 1.5:
        pw = w // 2 - 12
        ph = int(pw * 9 / 16)
        y = int(h * 0.22)
        boxes = [(8, y), (w // 2 + 4, y)]
        for (x, yy), im, (name, col) in zip(boxes, (left, right), names):
            canvas.paste(fit(im, (pw, ph)), (x, yy))
            tw = d.textbbox((0, 0), name, font=lab)[2]
            d.text((x + (pw - tw) / 2, yy - lab.size - 18), name, font=lab, fill=col)
        big = fonts.get("bold", int(w * 0.028))
        draw_rich(d, text, big, w / 2, y + ph + int(h * 0.08), w * 0.9)
    else:
        pw, ph = w, int(w * 0.44)
        ys = [int(h * 0.045), int(h * 0.045) + ph + int(h * 0.07)]
        for yy, im, (name, col) in zip(ys, (left, right), names):
            src = im.convert("RGB")
            sw, sh = src.size
            ch = int(sw * ph / pw)
            y0 = int((sh - ch) * 0.65)
            canvas.paste(src.crop((0, y0, sw, y0 + ch)).resize((pw, ph), Image.LANCZOS), (0, yy))
            tw = d.textbbox((0, 0), name, font=lab)[2]
            d.text(((w - tw) / 2, yy + ph + 6), name, font=lab, fill=col)
        big = fonts.get("bold", int(w * 0.046))
        draw_rich(d, text, big, w / 2, ys[1] + ph + int(h * 0.05), w * 0.92)
    return canvas


def caption(img, text, fonts, scale):
    """Large caption, lower third, on a soft dark band. Text in *stars* is cyan."""
    if not text:
        return img.convert("RGB")
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
        centred(line, fine, h * 0.88 + k * fine.size * 1.4, (130, 145, 160))
    return img


def card_caption(card, text, fonts):
    """Voice caption on the end card, between the link and the fine print."""
    if not text:
        return card
    img = card.copy()
    w, h = img.size
    f = fonts.get("semi", int(w * (0.026 if w / h > 1.5 else 0.045)))
    draw_rich(ImageDraw.Draw(img), text, f, w / 2, h * 0.68, w * 0.85)
    return img


GLUE = {"a", "an", "the", "and", "or", "but", "of", "to", "at", "in", "on", "it", "its", "it's",
        "is", "are", "was", "they", "that", "with", "for", "can't", "can", "what", "who", "this",
        "my", "our", "your", "their", "be", "by", "from", "as", "so", "if"}


def phrases(words, max_words=7):
    """Caption phrases: break after punctuation, or near max_words but never after a small word
    and never just before the last one or two words of a sentence."""
    out, cur = [], []
    for i, w in enumerate(words):
        cur.append(w)
        word = w["word"].lower().strip(".,?!;:\"'")
        ahead = [x["word"] for x in words[i + 1:i + 3]]
        ends_soon = any(x[-1:] in ".,?!;:" for x in ahead) or len(words) - i - 1 <= 2
        if w["word"][-1:] in ".,?!;:":
            brk = True
        elif len(cur) >= max_words + 3:
            brk = True
        else:
            brk = len(cur) >= max_words and word not in GLUE and not ends_soon
        if brk:
            out.append(cur)
            cur = []
    if cur:
        out.append(cur)
    return out


def synth_bed(path, total, cuts, ping_at, voice_spans, sr=48000):
    """Original sound bed: low drone, soft whooshes at cuts, one ping. Ducked under the voice."""
    import wave
    import numpy as np
    n = int(total * sr)
    t = np.arange(n) / sr
    rng = np.random.default_rng(7)
    drone = sum(a * np.sin(2 * np.pi * f * t + ph) for f, a, ph in
                [(55.0, 1.0, 0.0), (82.4, 0.6, 1.1), (110.0, 0.35, 2.3), (164.8, 0.15, 0.4)])
    drone *= 0.75 + 0.25 * np.sin(2 * np.pi * 0.07 * t)
    noise = np.convolve(rng.standard_normal(n), np.ones(400) / 400, mode="same") * 6
    bed = 0.020 * drone / 2.1 + 0.004 * noise
    fade = np.clip(t / 1.5, 0, 1) * np.clip((total - t) / 2.0, 0, 1)
    bed *= fade
    for c in cuts:  # whoosh: band-limited noise swell
        i0, i1 = int((c - 0.35) * sr), int((c + 0.15) * sr)
        if i0 < 0 or i1 > n:
            continue
        seg = np.convolve(rng.standard_normal(i1 - i0), np.ones(25) / 25, mode="same")
        env = np.sin(np.linspace(0, np.pi, i1 - i0)) ** 2
        bed[i0:i1] += 0.05 * seg * env
    if ping_at is not None and 0 < ping_at < total - 1:
        i0 = int(ping_at * sr)
        tt = np.arange(int(1.2 * sr)) / sr
        ping = (np.sin(2 * np.pi * 1318.5 * tt) + 0.5 * np.sin(2 * np.pi * 1975.5 * tt)) * np.exp(-tt * 4.5)
        bed[i0:i0 + len(tt)] += 0.07 * ping[:max(0, min(len(tt), n - i0))]
    duck = np.ones(n)
    for s0, s1 in voice_spans:
        a0, a1 = int(max(0, s0 - 0.2) * sr), int(min(total, s1 + 0.3) * sr)
        duck[a0:a1] = 0.45
    duck = np.convolve(duck, np.ones(4800) / 4800, mode="same")
    bed *= duck
    pcm = (np.clip(bed, -1, 1) * 32767).astype(np.int16)
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sr)
        wf.writeframes(pcm.tobytes())


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dashcam", required=True)
    p.add_argument("--overlay", required=True)
    p.add_argument("--orbit", required=True)
    p.add_argument("--live", default="", help="build_scene.py --shot live frames (30 fps)")
    p.add_argument("--out", required=True)
    p.add_argument("--freeze", type=int, default=145)
    p.add_argument("--start", type=int, default=118)
    p.add_argument("--end", type=int, default=205)
    p.add_argument("--size", default="1920x1080")
    p.add_argument("--font-dir", default="")
    p.add_argument("--end-seconds", type=float, default=3.5)
    p.add_argument("--teaser", type=float, default=1.6)
    p.add_argument("--voice", default="", help="voice.json from voice.py; captions become the words")
    p.add_argument("--audio", default="", help="clean.wav from voice.py")
    p.add_argument("--sound", action="store_true", help="add the sound bed (also without a voice)")
    a = p.parse_args()
    size = tuple(int(v) for v in a.size.split("x"))
    scale = size[0] / 1920.0
    fonts = Fonts(a.font_dir)

    dash = {int(f.stem): f for f in Path(a.dashcam).glob("*.jpeg")}
    over = {int(f.stem.split("_")[-1]): f for f in Path(a.overlay).glob("overlay_*.jpg")}
    orbit = sorted(Path(a.orbit).glob("*.png"))
    live = sorted(Path(a.live).glob("*.png")) if a.live else []
    if not dash or not over or not orbit:
        raise SystemExit("missing input frames")
    first_data = min(dash)
    last_data = max(dash)
    paused = "TIME PAUSED  ·  3D VIEW OF WHAT GODVIEW KNOWS"
    hook = Image.open(over[nearest(over, a.freeze + 5)])
    card = end_card(size, fonts, scale)
    cache = {}

    def load(path):
        if path not in cache:
            if len(cache) > 64:
                cache.clear()
            cache[path] = Image.open(path).convert("RGB")
        return cache[path]

    # Each section: (name, default length s, min s, max s, frame function(i, n) -> image without caption)
    def f_hook(i, n):
        return hook

    def f_dash(i, n):
        fr = max(first_data, a.freeze - -(-(n - i) // 3))
        return load(dash[nearest(dash, fr)])

    def f_orbit(i, n):
        x = i * (len(orbit) - 1) / max(n - 1, 1)
        k = int(x)
        if k + 1 >= len(orbit) or x == k:
            return load(orbit[min(k, len(orbit) - 1)])
        return Image.blend(load(orbit[k]), load(orbit[k + 1]), x - k)

    def f_split(i, n):
        fr = min(last_data, a.freeze + i // 3)
        return (load(dash[nearest(dash, fr)]), load(over[nearest(over, fr)]))

    def f_live(i, n):
        return load(live[min(i, len(live) - 1)])

    def f_card(i, n):
        return card

    max_a = (a.freeze - first_data) / 10.0
    max_c = (last_data - a.freeze) / 10.0
    sections = [("hook", a.teaser, 1.2, 6.5, f_hook),
                ("dash", (a.freeze - a.start) / 10.0, 1.5, max_a, f_dash),
                ("orbit", len(orbit) / FPS, 3.5, 10.0, f_orbit),
                ("split", (a.end - a.freeze) / 10.0, 3.0, max_c, f_split)]
    if live:
        sections.append(("live", len(live) / FPS, 2.5, len(live) / FPS, f_live))
    sections.append(("card", a.end_seconds, 3.0, 9.0, f_card))

    # Voice: section lengths follow the beats, captions are the spoken words
    beats = json.loads(Path(a.voice).read_text())["beats"] if a.voice else None
    lead, tail = 0.15, 0.45
    if beats is not None and len(beats) != len(sections):
        raise SystemExit(f"{len(beats)} beats for {len(sections)} sections; record one beat per section")
    lengths, placed = [], []  # placed: (beat index, clip time where that beat's audio starts)
    t_sec, t_free = 0.0, 0.0
    for k, (name, dflt, lo, hi, fn) in enumerate(sections):
        if beats is None:
            lengths.append(min(max(dflt, lo), hi))
            continue
        b = beats[k]
        t0 = max(t_sec + lead, t_free)
        placed.append((k, t0))
        t_free = t0 + (b["end"] - b["start"]) + 0.25
        want = t_free - 0.25 + tail - t_sec
        lengths.append(min(max(want, lo), hi))
        t_sec += lengths[-1]
    if beats is not None:
        overflow = t_free + 0.6 - sum(lengths)
        if overflow > 0:
            lengths[-1] += overflow  # the end card holds until the voice is done
    total = sum(lengths)

    # caption timeline: (start, end, text)
    caps = []
    if beats is not None:
        for k, t0 in placed:
            b = beats[k]
            ph = phrases(b["words"])
            for j, words in enumerate(ph):
                s0 = t0 + words[0]["start"] - b["start"] - 0.08
                s1 = (t0 + ph[j + 1][0]["start"] - b["start"] - 0.08) if j + 1 < len(ph) \
                    else t0 + b["end"] - b["start"] + 0.5
                caps.append((s0, s1, " ".join(w["word"] for w in words)))
    defaults = {"hook": ["This car knows who is *behind that truck*."],
                "dash": ["Its own cameras and lidar can't see past it.",
                         "But it isn't the only sensor here."],
                "orbit": ["Roadside units and another car share what they see.",
                          "They see *people behind the truck*."],
                "split": ["Same moment: the car alone, and *with GodView*.",
                          "It knows they're there *before they step out*."],
                "live": ["Same scene from above, in real time.", "Every sensor feeds *one shared map*."],
                "card": [""]}

    def text_at(t, k, i, n):
        if beats is None:
            opts = defaults[sections[k][0]]
            return opts[min(len(opts) - 1, int(len(opts) * i / max(n, 1)))]
        for s0, s1, txt in caps:
            if s0 <= t < s1:
                return txt
        return ""

    tmp = Path(tempfile.mkdtemp(prefix="clip_"))
    n_frames = 0
    k_card = sum(int(round(l * FPS)) for l in lengths[:-1])
    try:
        for k, (name, dflt, lo, hi, fn) in enumerate(sections):
            n = int(round(lengths[k] * FPS))
            for i in range(n):
                t = n_frames / FPS
                txt = text_at(t, k, i, n)
                src = fn(i, n)
                if name == "split":
                    img = split(src[0], src[1], txt, fonts, size)
                elif name == "card":
                    img = card_caption(src, txt, fonts)
                else:
                    label = paused if name == "orbit" else (
                        "REAL TIME  ·  3D VIEW" if name == "live" else None)
                    img = compose(src, txt, fonts, size, label=label, crop=name in ("hook", "dash"))
                back = k_card - n_frames  # fade the last XFADE frames into the end card
                if 0 < back <= XFADE:
                    img = Image.blend(img, card, (XFADE + 1 - back) / (XFADE + 1))
                img.save(tmp / f"{n_frames:05d}.jpg", quality=94)
                n_frames += 1
        video_only = tmp / "video.mp4"
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS),
                        "-i", str(tmp / "%05d.jpg"), "-c:v", "libx264", "-preset", "veryfast",
                        "-crf", "18", "-pix_fmt", "yuv420p", str(video_only)], check=True)
        dur = n_frames / FPS
        if beats is None and not a.sound:
            shutil.move(str(video_only), a.out)
        else:
            cuts = [sum(lengths[:k]) for k in range(1, len(lengths))]
            orbit_k = [s[0] for s in sections].index("orbit")
            ping = sum(lengths[:orbit_k]) + 0.46 * lengths[orbit_k]
            spans = [(t0, t0 + beats[k]["end"] - beats[k]["start"]) for k, t0 in placed] if beats else []
            bed = tmp / "bed.wav"
            synth_bed(bed, dur, cuts, ping, spans)
            inputs = ["-i", str(video_only), "-i", str(bed)]
            chains, labels = [], ["[1:a]"]
            if beats is not None:
                inputs += ["-i", a.audio]
                for j, (k, t0) in enumerate(placed):
                    b = beats[k]
                    s0 = b.get("audio_start", max(0.0, b["start"] - 0.06))
                    s1 = b.get("audio_end", b["end"] + 0.3)
                    ms = int(t0 * 1000) - int((b["start"] - s0) * 1000)
                    chains.append(f"[2:a]atrim=start={s0:.3f}:end={s1:.3f},asetpts=PTS-STARTPTS,"
                                  f"afade=t=in:d=0.03,afade=t=out:st={s1 - s0 - 0.08:.3f}:d=0.08,"
                                  f"adelay={max(ms, 0)}[v{j}]")
                    labels.append(f"[v{j}]")
            mix = "".join(labels) + (f"amix=inputs={len(labels)}:normalize=0:duration=longest,"
                                     f"apad,atrim=0:{dur:.3f},loudnorm=I=-14:TP=-1.0:LRA=9[aout]")
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *inputs, "-filter_complex",
                            ";".join(chains + [mix]), "-map", "0:v", "-map", "[aout]",
                            "-c:v", "copy", "-c:a", "aac", "-b:a", "192k", "-ar", "48000",
                            "-movflags", "+faststart", a.out], check=True)
    finally:
        shutil.rmtree(tmp)
    print(f"wrote {a.out}: {n_frames} frames, {n_frames / FPS:.1f} s")
    if beats is not None:
        for (k, t0), (name, *_r) in zip(placed, sections):
            print(f"  {name:6s} {lengths[k]:5.2f} s  voice at {t0:5.2f} s: {beats[k]['text']}")


if __name__ == "__main__":
    main()
