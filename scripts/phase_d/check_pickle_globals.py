# List the global imports in a torch .pth pickle without loading it.
import sys, zipfile, pickletools, collections
z = zipfile.ZipFile(sys.argv[1]); name = [n for n in z.namelist() if n.endswith('data.pkl')][0]
data = z.read(name); g = collections.Counter(); prev = []
for op, arg, pos in pickletools.genops(data):
    if op.name in ('GLOBAL', 'INST'): g[arg] += 1
    if op.name == 'STACK_GLOBAL': g[' '.join(map(str, prev[-2:]))] += 1
    if op.name in ('SHORT_BINUNICODE', 'BINUNICODE', 'UNICODE'): prev.append(arg)
    if op.name in ('REDUCE','BUILD') : pass
print(name); [print(' ', k, v) for k, v in g.items()]
