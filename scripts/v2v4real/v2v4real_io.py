"""Safe readers for V2V4Real (OPV2V format) frame files.

The dataset's YAML files contain numpy objects serialized with Python tags
(numpy scalars, dtypes and arrays). yaml.unsafe_load would rebuild them by
calling arbitrary Python, so this module uses a SafeLoader that understands
only those few tags and decodes the raw bytes with numpy.frombuffer.
"""

import numpy as np
import yaml

_ALLOWED_DTYPES = {"f4", "f8", "i4", "i8", "u1", "b1"}


class _NdarrayMarker:
    pass


class V2VLoader(yaml.SafeLoader):
    pass


def _apply_parts(loader, node):
    if isinstance(node, yaml.SequenceNode):
        return loader.construct_sequence(node, deep=True), None
    m = loader.construct_mapping(node, deep=True)
    return m.get("args", []), m.get("state")


def _dtype(loader, node):
    args, _state = _apply_parts(loader, node)
    name = args[0]
    if name not in _ALLOWED_DTYPES:
        raise ValueError("unexpected dtype %r" % name)
    return np.dtype(name).newbyteorder("<")


def _scalar(loader, node):
    args, _ = _apply_parts(loader, node)
    dt, raw = args[0], args[1]
    return np.frombuffer(raw, dtype=dt)[0].item()


def _reconstruct(loader, node):
    args, state = _apply_parts(loader, node)
    if not isinstance(args[0], _NdarrayMarker):
        raise ValueError("only numpy.ndarray reconstruction is supported")
    _ver, shape, dt, fortran, raw = state
    arr = np.frombuffer(raw, dtype=dt).copy()
    return arr.reshape(shape, order="F" if fortran else "C")


def _tuple(loader, node):
    return tuple(loader.construct_sequence(node, deep=True))


def _ndarray_name(loader, node):
    return _NdarrayMarker()


P = "tag:yaml.org,2002:python/"
V2VLoader.add_constructor(P + "object/apply:numpy.dtype", _dtype)
V2VLoader.add_constructor(P + "object/apply:numpy.core.multiarray.scalar", _scalar)
V2VLoader.add_constructor(P + "object/apply:numpy.core.multiarray._reconstruct", _reconstruct)
V2VLoader.add_constructor(P + "tuple", _tuple)
V2VLoader.add_constructor(P + "name:numpy.ndarray", _ndarray_name)


def load_yaml(path):
    with open(path, "rb") as f:
        return yaml.load(f, Loader=V2VLoader)


def load_pcd_xyz(path):
    """Read an ASCII .pcd file and return an (N, 3) float array of x, y, z."""
    with open(path, "r") as f:
        header = {}
        for line in f:
            parts = line.split()
            if not parts or parts[0].startswith("#"):
                continue
            header[parts[0]] = parts[1:]
            if parts[0] == "DATA":
                break
        if header.get("DATA", [""])[0] != "ascii":
            raise ValueError("only ascii pcd is supported: %r" % header.get("DATA"))
        fields = header["FIELDS"]
        data = np.loadtxt(f, dtype=np.float64, ndmin=2)
    idx = [fields.index(c) for c in ("x", "y", "z")]
    return data[:, idx]
