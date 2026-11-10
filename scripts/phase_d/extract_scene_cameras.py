"""Extract the camera images of chosen agents for one scene and a frame range from a V2X-Real zip.

Refuses unsafe member names. Usage (python -I):
  extract_scene_cameras.py ZIP OUT_DIR SPLIT SCENARIO FIRST_FRAME AGENT:CAMS [AGENT:CAMS ...]
e.g. ... test 2023-04-03-18-19-32_13_0 100 1:1234 -1:12 -2:12
"""
import os
import sys
import zipfile

zp, out, split, scen, first = sys.argv[1:6]
want = {}
for a in sys.argv[6:]:
    ag, cams = a.split(':')
    want[ag] = set(cams)
z = zipfile.ZipFile(zp)
n = 0
for i in z.infolist():
    p = i.filename.split('/')
    if len(p) != 4 or p[0] != split or p[1] != scen or not p[3].endswith('.jpeg'):
        continue
    agent, name = p[2], p[3]
    frame, cam = name[:6], name.split('_cam')[1][0]
    if agent not in want or cam not in want[agent] or int(frame) < int(first):
        continue
    assert '..' not in p and not i.filename.startswith('/')
    dest = os.path.join(out, i.filename)
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    with z.open(i) as s, open(dest, 'wb') as d:
        d.write(s.read())
    n += 1
print('extracted', n)
