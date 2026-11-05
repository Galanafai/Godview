"""Render a top-down demo video from two replay dumps (sharing and --no-sharing).

Three panels: "Car A alone" (car 0's world without sharing), "Car B alone"
(car 1's world without sharing), "Shared view" (car 0's world with sharing).
Labels are thin gray boxes; world estimates are filled boxes colored by world
ID, so an object that keeps its ID keeps its color. The two cars are marked.

Usage:
  python -I render_demo.py SHARE.dump ALONE.dump OUT_DIR NAME \
      --title "..." [--still-frame 100] [--xlim -110 65 --ylim -42 64]
Writes OUT_DIR/NAME.mp4, NAME.gif and NAME_still.png.
"""

import argparse
import json
import os
import shutil
import subprocess
import tempfile
import zlib

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

PALETTE = plt.get_cmap('tab20').colors + plt.get_cmap('tab20b').colors


def corners(x, y, l, w, yaw_deg):
    a = np.radians(yaw_deg)
    c, s = np.cos(a), np.sin(a)
    d = np.array([[l, w], [l, -w], [-l, -w], [-l, w]]) / 2.0
    return np.c_[x + c * d[:, 0] - s * d[:, 1], y + s * d[:, 0] + c * d[:, 1]]


def color_of(world_id):
    return PALETTE[zlib.crc32(world_id.encode()) % len(PALETTE)]


def draw_panel(ax, title, labels, objs, ego, xlim, ylim):
    ax.set_facecolor('#fafafa')
    for o in labels:
        ax.add_patch(Polygon(corners(o['x'], o['y'], o['l'], o['w'], o['yaw_deg']), closed=True,
                             fill=False, edgecolor='#9a9a9a', linewidth=0.6))
    n = 0
    for o in objs:
        if not (xlim[0] <= o['x'] <= xlim[1] and ylim[0] <= o['y'] <= ylim[1]):
            continue
        n += 1
        col = color_of(o['id'])
        b = o['box']
        if b:
            ax.add_patch(Polygon(corners(o['x'], o['y'], b['l'], b['w'], b['yaw_deg']), closed=True,
                                 facecolor=col, edgecolor='black', linewidth=0.4, alpha=0.85))
        else:
            ax.plot(o['x'], o['y'], 'o', color=col, markersize=4)
    for k, (e, name) in enumerate(zip(ego, ('A', 'B'))):
        ax.plot(e[0], e[1], marker='*', color='crimson', markersize=13, markeredgecolor='black', zorder=5)
        ax.text(e[0] + 3, e[1] + 3, 'Car ' + name, fontsize=9, fontweight='bold', color='crimson', zorder=6)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title, fontsize=12, fontweight='bold')
    ax.text(0.02, 0.97, 'objects seen: %d' % n, transform=ax.transAxes, fontsize=10, va='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='#cccccc'))
    return n


def render_frame(path, share, alone, args, idx, total):
    fig, axes = plt.subplots(1, 3, figsize=(18, 4.5), dpi=100)
    labels = share['labels']
    ego = [e[:2] for e in share['ego']]
    xlim, ylim = args.xlim, args.ylim
    draw_panel(axes[0], 'Car A alone', labels, alone['worlds'][0], ego, xlim, ylim)
    draw_panel(axes[1], 'Car B alone', labels, alone['worlds'][1], ego, xlim, ylim)
    draw_panel(axes[2], 'Shared view', labels, share['worlds'][0], ego, xlim, ylim)
    # 20 m scale bar in the last panel.
    axes[2].plot([xlim[1] - 28, xlim[1] - 8], [ylim[0] + 6] * 2, color='black', linewidth=2)
    axes[2].text(xlim[1] - 18, ylim[0] + 8, '20 m', ha='center', fontsize=9)
    fig.text(0.5, 0.045, '%s   frame %d / %d   (10 Hz)   gray: labels   filled: GodView world objects, color = world ID'
             % (args.title, idx + 1, total), ha='center', fontsize=10)
    fig.text(0.5, 0.008, 'Detections are labels with added noise, not a real detector.', ha='center', fontsize=9,
             style='italic', color='#444444')
    fig.subplots_adjust(left=0.01, right=0.99, top=0.91, bottom=0.1, wspace=0.03)
    fig.savefig(path)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('share')
    ap.add_argument('alone')
    ap.add_argument('out_dir')
    ap.add_argument('name')
    ap.add_argument('--title', default='')
    ap.add_argument('--still-frame', type=int, default=100)
    ap.add_argument('--xlim', type=float, nargs=2, default=[-110.0, 65.0])
    ap.add_argument('--ylim', type=float, nargs=2, default=[-42.0, 64.0])
    ap.add_argument('--max-frames', type=int, default=300)
    ap.add_argument('--gif-seconds', type=float, default=8.0)
    args = ap.parse_args()

    share = [json.loads(l) for l in open(args.share) if l.strip()]
    alone = [json.loads(l) for l in open(args.alone) if l.strip()]
    assert len(share) == len(alone)
    n = min(len(share), args.max_frames)
    os.makedirs(args.out_dir, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix='godview_demo_')
    try:
        for i in range(n):
            render_frame(os.path.join(tmp, 'f%04d.png' % i), share[i], alone[i], args, i, n)
        mp4 = os.path.join(args.out_dir, args.name + '.mp4')
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', '10', '-i', os.path.join(tmp, 'f%04d.png'),
                        '-vf', 'scale=1600:-2', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '26',
                        '-movflags', '+faststart', mp4], check=True)
        gif = os.path.join(args.out_dir, args.name + '.gif')
        pal = os.path.join(tmp, 'pal.png')
        start = max(0, min(args.still_frame - 20, n - int(args.gif_seconds * 10))) / 10.0
        filt = 'fps=5,scale=960:-1:flags=lanczos'
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-ss', str(start), '-t', str(args.gif_seconds),
                        '-i', mp4, '-vf', filt + ',palettegen=max_colors=64', pal], check=True)
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-ss', str(start), '-t', str(args.gif_seconds),
                        '-i', mp4, '-i', pal, '-lavfi', filt + ' [x]; [x][1:v] paletteuse=dither=bayer:bayer_scale=4',
                        gif], check=True)
        shutil.copy(os.path.join(tmp, 'f%04d.png' % args.still_frame),
                    os.path.join(args.out_dir, args.name + '_still.png'))
    finally:
        shutil.rmtree(tmp)
    for f in (args.name + '.mp4', args.name + '.gif', args.name + '_still.png'):
        p = os.path.join(args.out_dir, f)
        print('%s %.2f MB' % (p, os.path.getsize(p) / 1e6))


if __name__ == '__main__':
    main()
