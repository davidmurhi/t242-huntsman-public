"""Minimal reader for ACIS binary (SAB) records inside Inventor parts:
enough to follow edges, vertices and curves and trace a face's outline."""
import struct


def records(blob):
    i = blob.find(b"\r\tasmheader")
    recs, cur, name = [], [], []
    n = len(blob)
    while i < n:
        t = blob[i]; i += 1
        if t in (0x0d, 0x0e):
            L = blob[i]; i += 1
            s = blob[i:i + L].decode("latin-1"); i += L
            name.append(s)
            if t == 0x0d:
                cur.append(("name", "-".join(name))); name = []
        elif t == 0x0c:
            cur.append(("ptr", struct.unpack_from("<i", blob, i)[0])); i += 4
        elif t in (0x04, 0x15):
            cur.append(("int", struct.unpack_from("<i", blob, i)[0])); i += 4
        elif t == 0x06:
            cur.append(("dbl", struct.unpack_from("<d", blob, i)[0])); i += 8
        elif t in (0x13, 0x14):
            cur.append(("pos" if t == 0x13 else "vec",
                        struct.unpack_from("<3d", blob, i))); i += 24
        elif t == 0x16:
            cur.append(("v2", struct.unpack_from("<2d", blob, i))); i += 16
        elif t in (0x07, 0x12):
            L = blob[i]; i += 1 + L
        elif t == 0x08:
            L = struct.unpack_from("<H", blob, i)[0]; i += 2 + L
        elif t == 0x09:
            L = struct.unpack_from("<I", blob, i)[0]; i += 4 + L
        elif t in (0x0a, 0x0b, 0x0f, 0x10):
            cur.append(("flag", t))
        elif t == 0x02:
            i += 1
        elif t == 0x03:
            i += 2
        elif t == 0x05:
            i += 4
        elif t == 0x11:
            if kind(cur).startswith(("Begin-of-ASM-History", "End-of-ASM")):
                break
            recs.append(cur)
            cur = []
        else:
            break
    return recs


def kind(rec):
    for k, v in rec:
        if k == "name":
            return v
    return ""


def _fields(rec, k):
    return [v for t, v in rec if t == k]


def plane_edges(blob, axis, value, tol=1e-3, steps=24):
    """Every edge lying in the plane coordinate[axis] == value, as a list
    of sampled 3D polylines (cm, part coordinates)."""
    import numpy as np
    R = records(blob)

    def point_of_vertex(vi):
        pt = [p for p in _fields(R[vi], "ptr") if p >= 0 and kind(R[p]) == "point"]
        return np.array(_fields(R[pt[-1]], "pos")[0])

    out = []
    for r in R:
        if kind(r) != "edge":
            continue
        ptrs = _fields(r, "ptr")
        dbls = _fields(r, "dbl")
        vs = [p for p in ptrs if p >= 0 and kind(R[p]) == "vertex"]
        cs = [p for p in ptrs if p >= 0 and kind(R[p]).endswith("curve")]
        if len(vs) < 1 or not cs:
            continue
        v0, v1, curve = vs[0], vs[-1], cs[0]
        a, b = point_of_vertex(v0), point_of_vertex(v1)
        if abs(a[axis] - value) > tol or abs(b[axis] - value) > tol:
            continue
        ck = kind(R[curve])
        if ck.startswith("straight"):
            out.append(np.array([a, b]))
        elif ck.startswith("ellipse"):
            c = np.array(_fields(R[curve], "pos")[0])
            n, m = (np.array(v) for v in _fields(R[curve], "vec")[:2])
            ratio = _fields(R[curve], "dbl")[0]
            if abs(c[axis] - value) > tol:
                continue
            span = abs(dbls[1] - dbls[0]) if len(dbls) >= 2 else 2 * np.pi
            mu = m / np.linalg.norm(m)
            qu = np.cross(n / np.linalg.norm(n), mu)
            r1, r2 = np.linalg.norm(m), np.linalg.norm(m) * ratio
            ang = lambda p: np.arctan2((p - c) @ qu / r2, (p - c) @ mu / r1)  # noqa: E731
            f = lambda t: c + mu * r1 * np.cos(t) + qu * r2 * np.sin(t)  # noqa: E731
            if np.linalg.norm(a - b) < 1e-6:          # full circle
                ts = np.linspace(ang(a), ang(a) + 2 * np.pi, steps * 2)
            else:
                d = (ang(b) - ang(a)) % (2 * np.pi)
                if abs(d - span) <= abs((2 * np.pi - d) - span):
                    ts = np.linspace(ang(a), ang(a) + d, steps)
                else:
                    ts = np.linspace(ang(a), ang(a) - (2 * np.pi - d), steps)
            out.append(np.array([f(t) for t in ts]))
    return out


def loops(polylines, tol=0.02):
    """Join edge polylines end to end into closed loops (2D (u, v)
    arrays). Edges are first joined only where their ends meet; any
    chains left open (an edge that could not be read) are then joined to
    each other across the smallest gaps. Largest loop first,
    anticlockwise."""
    import numpy as np

    def join(segs, gap):
        used = [False] * len(segs)
        closed, open_ = [], []
        for i in range(len(segs)):
            if used[i]:
                continue
            used[i] = True
            chain = list(segs[i])
            while np.linalg.norm(chain[0] - chain[-1]) >= tol:
                end = chain[-1]
                best, bj, flip = None, -1, False
                for j, s in enumerate(segs):
                    if used[j]:
                        continue
                    for f, q in ((False, s[0]), (True, s[-1])):
                        d = np.linalg.norm(q - end)
                        if best is None or d < best:
                            best, bj, flip = d, j, f
                if bj < 0 or best > gap:
                    break
                used[bj] = True
                s = segs[bj][::-1] if flip else segs[bj]
                chain.extend(s[1:] if best < tol else s)
            c = np.array(chain)
            if np.linalg.norm(c[0] - c[-1]) < tol:
                closed.append(c[:-1])
            else:
                open_.append(c)
        return closed, open_

    segs = [np.asarray(p, float) for p in polylines if len(p) >= 2]
    closed, open_ = join(segs, tol)
    if open_:
        more, rest = join(open_, 1e9)
        closed += more + rest

    def area(p):
        x, y = p[:, 0], p[:, 1]
        return 0.5 * (np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
    out = [p for p in closed if len(p) >= 3]
    out.sort(key=lambda p: -abs(area(p)))
    return [p if area(p) > 0 else p[::-1] for p in out]
