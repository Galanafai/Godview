"""Render the 4:5 (1080x1350) LinkedIn version of the demo from the same dumps.

Two panels stacked: "One car alone" (car B, i.e. car 1, from the
--no-sharing dump) on top and "Shared view" (car 1's world in the normal
replay) below, so both panels show the same car without and with sharing.
Larger fonts and outlines for a phone screen.

Usage:
  python -I render_linkedin.py SHARE.dump ALONE.dump OUT.mp4 --start 40 --frames 150
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_demo import color_of, corners  # noqa: E402

XLIM = (-105.0, 60.0)
YLIM = (-38.0, 62.0)
FOOTER = 'Detections: dataset labels with added noise, not a real detector'


def panel(ax, title, labels, objs, ego):
    ax.set_facecolor('#f7f7f7')
    for o in labels:
        ax.add_patch(Polygon(corners(o['x'], o['y'], o['l'], o['w'], o['yaw_deg']), closed=True,
                             fill=False, edgecolor='#8a8a8a', linewidth=1.3))
    n = 0
    for o in objs:
        if not (XLIM[0] <= o['x'] <= XLIM[1] and YLIM[0] <= o['y'] <= YLIM[1]):
            continue
        n += 1
        b = o['box']
        if b:
            ax.add_patch(Polygon(corners(o['x'], o['y'], b['l'], b['w'], b['yaw_deg']), closed=True,
                                 facecolor=color_of(o['id']), edgecolor='black', linewidth=1.1, alpha=0.9))
        else:
            ax.plot(o['x'], o['y'], 'o', color=color_of(o['id']), markersize=7)
    for e, name in zip(ego, ('A', 'B')):
        ax.plot(e[0], e[1], marker='*', color='crimson', markersize=26, markeredgecolor='black',
                markeredgewidth=1.2, zorder=5)
        ax.text(e[0] + 4, e[1] + 4, 'Car ' + name, fontsize=20, fontweight='bold', color='crimson', zorder=6,
                bbox=dict(boxstyle='round,pad=0.15', facecolor='white', alpha=0.75, edgecolor='none'))
    ax.set_xlim(*XLIM)
    ax.set_ylim(*YLIM)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_linewidth(1.5)
    ax.set_title(title, fontsize=28, fontweight='bold', pad=8)
    ax.text(0.025, 0.96, 'objects seen: %d' % n, transform=ax.transAxes, fontsize=24, va='top', fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.95, edgecolor='#999999'))


def frame(path, share, alone):
    fig = plt.figure(figsize=(10.8, 13.5), dpi=100)
    ax1 = fig.add_axes([0.02, 0.516, 0.96, 0.402])
    ax2 = fig.add_axes([0.02, 0.070, 0.96, 0.402])
    ego = [e[:2] for e in share['ego']]
    panel(ax1, 'One car alone (car B)', share['labels'], alone['worlds'][1], ego)
    panel(ax2, 'Shared view', share['labels'], share['worlds'][1], ego)
    ax2.plot([XLIM[1] - 30, XLIM[1] - 10], [YLIM[0] + 7] * 2, color='black', linewidth=4)
    ax2.text(XLIM[1] - 20, YLIM[0] + 10, '20 m', ha='center', fontsize=18)
    fig.text(0.5, 0.988, 'Two real cars sharing what they see (V2V4Real)', ha='center', va='top', fontsize=24)
    fig.text(0.5, 0.037, 'gray: labels   filled: GodView world objects (color = ID)', ha='center', fontsize=19)
    fig.text(0.5, 0.008, FOOTER, ha='center', fontsize=19, style='italic', color='#333333')
    fig.savefig(path)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('share')
    ap.add_argument('alone')
    ap.add_argument('out')
    ap.add_argument('--start', type=int, default=40)
    ap.add_argument('--frames', type=int, default=150)
    args = ap.parse_args()
    share = [json.loads(l) for l in open(args.share) if l.strip()]
    alone = [json.loads(l) for l in open(args.alone) if l.strip()]
    assert len(share) == len(alone)
    idx = range(args.start, min(args.start + args.frames, len(share)))
    tmp = tempfile.mkdtemp(prefix='godview_li_')
    try:
        for k, i in enumerate(idx):
            frame(os.path.join(tmp, 'f%04d.png' % k), share[i], alone[i])
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', '10', '-i', os.path.join(tmp, 'f%04d.png'),
                        '-vf', 'scale=1080:1350', '-c:v', 'libx264', '-profile:v', 'high', '-pix_fmt', 'yuv420p',
                        '-crf', '22', '-movflags', '+faststart', args.out], check=True)
        shutil.copy(os.path.join(tmp, 'f%04d.png' % min(60, len(idx) - 1)), args.out.replace('.mp4', '_still.png'))
    finally:
        shutil.rmtree(tmp)
    print('%s %.2f MB, frames %d to %d' % (args.out, os.path.getsize(args.out) / 1e6, idx[0], idx[-1]))


if __name__ == '__main__':
    main()
