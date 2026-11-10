"""Selective, safe extraction of a V2X-Real (LiDAR-Cameras) zip.

Extracts every `.yaml` and `.bin`, and the front-camera image (`_cam1.jpeg`) of the vehicles
(agent folders with a positive id); skips the other images (roadside-unit cameras and the
vehicles' other cameras). Refuses absolute paths, `..` components and symlinks. The zip is
left untouched.

Usage (python -I): extract_v2xreal_zip.py ZIP OUT_DIR
"""
import os
import sys
import zipfile


def main():
    z = zipfile.ZipFile(sys.argv[1])
    out = os.path.abspath(sys.argv[2])
    names = z.namelist()
    bad = [n for n in names if n.startswith('/') or '..' in n.split('/') or '\\' in n]
    links = [i.filename for i in z.infolist() if (i.external_attr >> 16) & 0o170000 == 0o120000]
    if bad or links:
        sys.exit('refusing: unsafe members %r %r' % (bad[:3], links[:3]))
    keep = []
    for i in z.infolist():
        n = i.filename
        if i.is_dir():
            continue
        parts = n.split('/')
        agent = parts[-2] if len(parts) >= 2 else ''
        if n.endswith('.yaml') or n.endswith('.bin'):
            keep.append(i)
        elif n.endswith('_cam1.jpeg') and agent.lstrip('-').isdigit() and int(agent) > 0:
            keep.append(i)
    total = sum(i.file_size for i in keep)
    print('%d of %d members, %.2f GB to extract' % (len(keep), len(names), total / 1e9), flush=True)
    for k, i in enumerate(keep):
        dest = os.path.join(out, i.filename)
        assert os.path.abspath(dest).startswith(out + os.sep)
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        with z.open(i) as src, open(dest, 'wb') as dst:
            while True:
                b = src.read(1 << 20)
                if not b:
                    break
                dst.write(b)
        if k % 2000 == 0:
            print(k, flush=True)
    print('done', flush=True)


if __name__ == '__main__':
    main()
