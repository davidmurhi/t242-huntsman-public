"""
inventor.py

Reads Autodesk Inventor 2025+ part (.ipt) and assembly (.iam) files well
enough to rebuild the robot in simulation: the solid-model vertices of
each part, and where every part is placed in each assembly.

Inventor files are OLE compound documents. Since Inventor 2025 each data
segment is Zstandard-compressed. The B-rep segment holds an ACIS
"ASM BinaryFile4" solid; 'point' records in it carry vertex coordinates.
Assemblies store each occurrence's placement as a sparse 4x4 matrix
(two 16-bit masks, then only the entries that are not 0, 1 or -1), the
layout documented by the open-source InventorLoader project.

Units in the files are centimetres.

    pip install olefile zstandard numpy
"""

from __future__ import annotations

import re
import struct

import numpy as np
import olefile

_ZSTD_MAGIC = b"\x28\xb5\x2f\xfd"


def _zstd(data: bytes) -> bytes:
    try:
        import zstandard
        return zstandard.ZstdDecompressor().decompressobj().decompress(data)
    except ImportError:
        pass
    try:                                    # Python 3.14+
        from compression import zstd
        return zstd.ZstdDecompressor().decompress(data)
    except ImportError:
        pass
    return _zstd_ctypes(data)


def _zstd_ctypes(data: bytes) -> bytes:
    """Last resort: the system libzstd, which most Linux machines have."""
    import ctypes
    import ctypes.util
    lib = ctypes.util.find_library("zstd") or "libzstd.so.1"
    try:
        z = ctypes.CDLL(lib)
    except OSError:
        raise SystemExit("needs a zstd decoder:  pip install zstandard")

    class Buf(ctypes.Structure):
        _fields_ = [("ptr", ctypes.c_void_p), ("size", ctypes.c_size_t),
                    ("pos", ctypes.c_size_t)]
    z.ZSTD_createDStream.restype = ctypes.c_void_p
    z.ZSTD_initDStream.argtypes = [ctypes.c_void_p]
    z.ZSTD_freeDStream.argtypes = [ctypes.c_void_p]
    z.ZSTD_decompressStream.argtypes = [ctypes.c_void_p] * 3
    z.ZSTD_decompressStream.restype = ctypes.c_size_t
    z.ZSTD_isError.argtypes = [ctypes.c_size_t]
    ds = z.ZSTD_createDStream()
    z.ZSTD_initDStream(ds)
    src = ctypes.create_string_buffer(data, len(data))
    inb = Buf(ctypes.cast(src, ctypes.c_void_p), len(data), 0)
    out, chunk = bytearray(), ctypes.create_string_buffer(1 << 20)
    try:
        while True:
            ob = Buf(ctypes.cast(chunk, ctypes.c_void_p), 1 << 20, 0)
            r = z.ZSTD_decompressStream(ds, ctypes.byref(ob), ctypes.byref(inb))
            if z.ZSTD_isError(r):
                raise ValueError("bad zstd frame")
            out += chunk.raw[:ob.pos]
            if r == 0 or (inb.pos >= inb.size and ob.pos == 0):
                return bytes(out)
    finally:
        z.ZSTD_freeDStream(ds)


def segments(path):
    """Yield (stream name, decompressed bytes) for every compressed segment."""
    ole = olefile.OleFileIO(path)
    for entry in ole.listdir():
        name = "/".join(entry)
        if not name.startswith("RSeStorage/"):
            continue
        raw = ole.openstream(entry).read()
        for m in re.finditer(re.escape(_ZSTD_MAGIC), raw):
            try:
                yield name.split("/")[-1], _zstd(raw[m.start():])
            except Exception:
                continue
    ole.close()


def part_vertices(path) -> np.ndarray:
    """All solid-model vertices of a part, in cm, part coordinates."""
    pts = []
    for _name, blob in segments(path):
        if b"ASM" not in blob or b"point" not in blob:
            continue
        for m in re.finditer(rb"point", blob):
            window = blob[m.end():m.end() + 80]
            k = window.find(b"\x13")              # ACIS position tag
            while k != -1 and k + 25 <= len(window):
                xyz = struct.unpack_from("<3d", window, k + 1)
                if all(abs(v) < 1e4 for v in xyz):
                    pts.append(xyz)
                    break
                k = window.find(b"\x13", k + 1)
    return np.unique(np.round(np.array(pts), 5), axis=0)


def _read_sparse_matrix(blob, i):
    d1, d2 = struct.unpack_from("<HH", blob, i)
    i += 4
    m = np.zeros((4, 4))
    for row in range(4):
        for col in range(4):
            bit = 1 << (col + 4 * row)
            if d2 & bit == 0:
                if d1 & bit == 0:
                    if i + 8 > len(blob):
                        return None, i
                    v = struct.unpack_from("<d", blob, i)[0]
                    i += 8
                    m[row, col] = 0.0 if abs(v) < 1e-6 else v
                else:
                    m[row, col] = 1.0
            else:
                m[row, col] = 0.0 if d1 & bit == 0 else -1.0
    return m, i


def _is_placement(m):
    if m is None or not np.allclose(m[3], [0, 0, 0, 1]):
        return False
    r = m[:3, :3]
    if not np.all(np.abs(r) <= 1.000001):
        return False
    return (np.allclose(r @ r.T, np.eye(3), atol=1e-5)
            and np.all(np.abs(m[:3, 3]) < 1e3))


def occurrence_placements(path):
    """Placements of an assembly's direct occurrences, in browser order.

    They sit at the end of the assembly's largest segment, one per
    occurrence. Masks whose last row encodes 0 0 0 1 mark them.
    """
    best = None
    for _name, blob in segments(path):
        if len(blob) < 100_000:
            continue
        found, i = [], 0
        while i < len(blob) - 4:
            d1, d2 = struct.unpack_from("<HH", blob, i)
            if (d2 >> 12) == 0x7 and (d1 >> 12) == 0x8:
                m, end = _read_sparse_matrix(blob, i)
                if _is_placement(m):
                    found.append(m)
                    i = end
                    continue
            i += 1
        if found:
            best = found
    return best or []


def occurrence_names(path):
    """Occurrence names ('full leg:3', ...) in browser order."""
    names = []
    for _name, blob in segments(path):
        hits = re.findall(rb"((?:[\x20-\x7e]\x00){3,}?:\x00\d\x00(?:\d\x00)?)",
                          blob)
        cand = [h.decode("utf-16le") for h in hits]
        cand = [c for c in cand if not c.startswith(("PmDCSegment", "Mate",
                                                     "Flush",
                                                     "Rotational", "Insert",
                                                     "Angle"))]
        if len(cand) > len(names):
            names = cand
    return names

def part_circles(path) -> np.ndarray:
    """Every circular edge of a part (hole rims, bosses, horn discs).

    ACIS 'ellipse-curve' records hold the centre (position tag 0x13), the
    normal and the major axis (vector tag 0x14) and the axis ratio. Rows
    are centre xyz, normal xyz, radius, all cm in part coordinates; only
    true circles (ratio 1) are returned.
    """
    out = []
    for _name, blob in segments(path):
        if b"ASM" not in blob or b"ellipse" not in blob:
            continue
        for m in re.finditer(rb"ellipse", blob):
            w = blob[m.end():m.end() + 160]
            k = w.find(b"\x13")
            if k == -1 or k + 1 + 24 + 2 * 25 + 9 > len(w):
                continue
            c = struct.unpack_from("<3d", w, k + 1)
            if w[k + 25] != 0x14 or w[k + 50] != 0x14:
                continue
            nrm = struct.unpack_from("<3d", w, k + 26)
            major = struct.unpack_from("<3d", w, k + 51)
            ratio = struct.unpack_from("<d", w, k + 76)[0] \
                if w[k + 75] == 0x06 else 1.0
            r = float(np.linalg.norm(major))
            if abs(ratio - 1.0) < 1e-6 and 0 < r < 100:
                out.append(list(c) + list(nrm) + [r])
    if not out:
        return np.zeros((0, 7))
    return np.unique(np.round(np.array(out), 5), axis=0)



def plate_loops(path, axis, value, uv):
    """Closed outlines of the face lying in the plane coordinate[axis] ==
    value: [outer outline, hole, hole, ...] as (u, v) arrays, cm. Read from
    the solid's real edges (straight lines and arcs), so concave outlines
    and rectangular cut-outs come out exactly."""
    import sab
    for _name, blob in segments(path):
        if b"ASM" in blob and b"point" in blob:
            edges = sab.plane_edges(blob, axis, value)
            return sab.loops([e[:, list(uv)] for e in edges])
    return []
