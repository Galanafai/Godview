# List a zip: entry count, size, unsafe paths, symlinks, top folders.
import sys, zipfile, os, collections
z = zipfile.ZipFile(sys.argv[1])
bad = [n for n in z.namelist() if n.startswith('/') or '..' in n.split('/') or '\\' in n]
links = [i.filename for i in z.infolist() if (i.external_attr >> 16) & 0o170000 == 0o120000]
tops = collections.Counter(n.split('/')[0] for n in z.namelist())
print(sys.argv[1], 'entries', len(z.namelist()), 'uncompressed GB %.2f' % (sum(i.file_size for i in z.infolist())/1e9))
print(' bad paths', bad[:5], 'symlinks', links[:5]); print(' top', dict(tops))
exts = collections.Counter(os.path.splitext(n)[1] for n in z.namelist()); print(' exts', dict(exts))
