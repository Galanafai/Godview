"""Phase D4: merge the candidate files, rank them (rule order) and render front-camera thumbnails
(start and end of each interval) for the top N, with the object's projected centre marked.

Usage (python -I, needs cv2): d4_thumbnails.py CAND_DIR OUT_DIR N RAW_DIR [RAW_DIR ...]
RAW_DIRs are searched for <scenario>/<vehicle>/<frame>_cam1.jpeg.
"""
import glob
import json
import os
import sys

import cv2

cand_dir, out_dir, n = sys.argv[1], sys.argv[2], int(sys.argv[3])
raws = sys.argv[4:]
os.makedirs(out_dir, exist_ok=True)
allc = []
for f in sorted(glob.glob(os.path.join(cand_dir, 'cand_*.json'))):
    allc += json.load(open(f))
allc.sort(key=lambda c: (not c['preferred'], -c['length_frames'], c['scenario'], c['vehicle'], c['gid']))
for r, c in enumerate(allc, 1):
    c['rank'] = r
json.dump(allc, open(os.path.join(out_dir, 'candidates.json'), 'w'), indent=1)


def find(c, frame):
    for raw in raws:
        p = os.path.join(raw, c['scenario'], c['vehicle'], frame + '_cam1.jpeg')
        if os.path.exists(p):
            return p
    return None


def tile(c, which):
    p = find(c, c['start_file'] if which == 'start' else c['end_file'])
    img = cv2.imread(p)
    uv = c['uv_start'] if which == 'start' else c['uv_end']
    if uv:
        cv2.circle(img, (int(uv[0]), int(uv[1])), 38, (0, 0, 255), 6)
        cv2.circle(img, (int(uv[0]), int(uv[1])), 5, (0, 0, 255), -1)
    d = c['range_m_start_end'][0 if which == 'start' else 1]
    img = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA)
    label = '#%d %s: frame %s, %.0f m' % (c['rank'], which, c['start_file'] if which == 'start' else c['end_file'], d)
    cv2.rectangle(img, (0, 0), (640, 26), (0, 0, 0), -1)
    cv2.putText(img, label, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return img


rows = []
for c in allc[:n]:
    row = cv2.hconcat([tile(c, 'start'), tile(c, 'end')])
    cv2.imwrite(os.path.join(out_dir, 'rank%d.jpg' % c['rank']), row, [cv2.IMWRITE_JPEG_QUALITY, 85])
    rows.append(row)
print('wrote', len(rows), 'thumbnails;', len(allc), 'candidates')
