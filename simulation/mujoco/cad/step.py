"""
step.py

A small STEP (ISO 10303-21) reader: enough to turn the B-rep solids in a
part or assembly exported from Inventor into triangle meshes, with each
solid's colour and its placement in the assembly.

Planar and cylindrical faces are triangulated exactly from their edge
loops (straight lines, circles, ellipses and B-spline edges); tori,
cones and B-spline surfaces (plain or rational) are triangulated in their
own (u, v) parameters and evaluated back onto the true surface.
"""

from __future__ import annotations

import re

import numpy as np

# ---------------------------------------------------------------- parse


def _split(s):
    """Split a STEP argument list at top-level commas."""
    out, depth, cur, q = [], 0, [], False
    for ch in s:
        if ch == "'" :
            q = not q
        if not q:
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
            elif ch == "," and depth == 0:
                out.append("".join(cur).strip())
                cur = []
                continue
        cur.append(ch)
    if cur:
        out.append("".join(cur).strip())
    return out


def _val(tok):
    tok = tok.strip()
    if tok.startswith("#"):
        return int(tok[1:])
    if tok.startswith("("):
        return [_val(t) for t in _split(tok[1:-1])] if tok[1:-1].strip() else []
    if tok.startswith("'"):
        return tok[1:-1]
    if tok.startswith("."):
        return tok
    if tok in ("$", "*"):
        return None
    try:
        return float(tok)
    except ValueError:
        return tok


def parse(path):
    text = open(path, encoding="latin-1").read()
    data = text[text.index("DATA;") + 5:text.index("ENDSEC;", text.index("DATA;"))]
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|$)", data, re.S):
        eid, body = int(m.group(1)), " ".join(m.group(2).split())
        mm = re.match(r"([A-Z0-9_]+)\s*\((.*)\)$", body)
        if mm:
            ents[eid] = (mm.group(1), [_val(t) for t in _split(mm.group(2))])
        else:
            ents[eid] = ("COMPLEX", body)
    return ents


def _complex(body):
    """Sub-records of a complex entity: {NAME: args}."""
    inner = body.strip()[1:-1]
    out = {}
    for m in re.finditer(r"([A-Z0-9_]+)\s*\(", inner):
        if m.start() and inner[m.start() - 1] not in " )":
            continue
        i, depth = m.end(), 1
        while depth:
            depth += {"(": 1, ")": -1}.get(inner[i], 0)
            i += 1
        args = inner[m.end():i - 1]
        out.setdefault(m.group(1), [_val(t) for t in _split(args)] if args.strip() else [])
    return out


def _knots(mults, knots):
    return np.repeat(np.array(knots, float), np.array(mults, int))


def _basis(x, t, k):
    from scipy.interpolate import BSpline
    x = np.clip(np.asarray(x, float), t[k], t[-k - 1])
    return BSpline.design_matrix(x, t, k).toarray()


class _Nurbs:
    """A B-spline (optionally rational) surface S(u, v)."""

    def __init__(self, du, dv, P, tu, tv, W=None):
        self.du, self.dv, self.P, self.tu, self.tv = du, dv, P, tu, tv
        self.W = np.ones(P.shape[:2]) if W is None else W
        self.ur = (tu[du], tu[-du - 1])
        self.vr = (tv[dv], tv[-dv - 1])

    def grid(self, us, vs):
        Nu, Nv = _basis(us, self.tu, self.du), _basis(vs, self.tv, self.dv)
        w = Nu @ self.W @ Nv.T
        xyz = np.einsum("ai,ijc,bj->abc", Nu, self.P * self.W[..., None], Nv)
        return xyz / w[..., None]

    def at(self, uv):
        Nu = _basis(uv[:, 0], self.tu, self.du)
        Nv = _basis(uv[:, 1], self.tv, self.dv)
        w = np.einsum("ai,ij,aj->a", Nu, self.W, Nv)
        xyz = np.einsum("ai,ijc,aj->ac", Nu, self.P * self.W[..., None], Nv)
        return xyz / w[:, None]


def _nurbs_curve(deg, P, t, W=None, n=None):
    W = np.ones(len(P)) if W is None else np.asarray(W, float)
    n = n or max(8, 6 * (len(P) - 1))
    xs = np.linspace(t[deg], t[-deg - 1], n)
    N = _basis(xs, t, deg)
    return (N @ (P * W[:, None])) / (N @ W)[:, None]


# ---------------------------------------------------------------- geometry


class Step:
    def __init__(self, path):
        self.e = parse(path)

    def pt(self, i):
        return np.array(self.e[i][1][1], float)

    def dirn(self, i):
        d = np.array(self.e[i][1][1], float)
        return d / np.linalg.norm(d)

    def axis(self, i):
        """AXIS2_PLACEMENT_3D -> origin, x, y, z."""
        _t, a = self.e[i]
        o = self.pt(a[1])
        z = self.dirn(a[2]) if a[2] else np.array([0, 0, 1.0])
        x = self.dirn(a[3]) if len(a) > 3 and a[3] else np.array([1.0, 0, 0])
        x = x - z * (x @ z)
        x /= np.linalg.norm(x)
        return o, x, np.cross(z, x), z

    def matrix(self, i):
        o, x, y, z = self.axis(i)
        M = np.eye(4)
        M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = x, y, z, o
        return M

    # ---- edges
    def edge_points(self, eid, step=1.0):
        """Points along an EDGE_CURVE from its start to its end vertex."""
        _t, a = self.e[eid]
        v0 = self.pt(self.e[a[1]][1][1])
        v1 = self.pt(self.e[a[2]][1][1])
        ctype, ca = self.e[a[3]]
        same = a[4] == ".T."
        if ctype in ("CIRCLE", "ELLIPSE"):
            o, x, y, _z = self.axis(ca[1])
            r1 = ca[2]
            r2 = ca[3] if ctype == "ELLIPSE" else r1
            ang = lambda p: np.arctan2((p - o) @ y / r2, (p - o) @ x / r1)  # noqa
            t0, t1 = ang(v0), ang(v1)
            if same:
                if t1 <= t0 + 1e-9:
                    t1 += 2 * np.pi
            else:
                if t1 >= t0 - 1e-9:
                    t1 -= 2 * np.pi
            n = max(6, int(abs(t1 - t0) * max(r1, r2) / step) + 1,
                    int(abs(t1 - t0) / 0.2) + 1)
            ts = np.linspace(t0, t1, n)
            return o + np.outer(r1 * np.cos(ts), x) + np.outer(r2 * np.sin(ts), y)
        if ctype in ("B_SPLINE_CURVE_WITH_KNOTS", "COMPLEX"):
            if ctype == "COMPLEX":
                c = _complex(ca)
                if "B_SPLINE_CURVE" not in c:
                    return np.array([v0, v1])
                deg, cp = c["B_SPLINE_CURVE"][0], c["B_SPLINE_CURVE"][1]
                kn = c["B_SPLINE_CURVE_WITH_KNOTS"]
                t = _knots(kn[0], kn[1])
                W = c.get("RATIONAL_B_SPLINE_CURVE", [None])[0]
            else:
                deg, cp = int(ca[1]), ca[2]
                t, W = _knots(ca[6], ca[7]), None
            pts = _nurbs_curve(int(deg), np.array([self.pt(i) for i in cp]),
                               t, W)
            if not same:
                pts = pts[::-1]
            pts[0], pts[-1] = v0, v1
            return pts
        return np.array([v0, v1])                       # LINE and the rest

    def loop_points(self, lid):
        pts = []
        for oe in self.e[lid][1][1]:
            _t, a = self.e[oe]
            p = self.edge_points(a[3])
            if a[4] == ".F.":
                p = p[::-1]
            pts.extend(p[:-1] if len(p) > 1 else p)
        return np.array(pts)

    # ---- faces
    def face_mesh(self, fid):
        """Triangles (n, 3, 3) for one ADVANCED_FACE."""
        _t, a = self.e[fid]
        bounds, surf, same = a[1], a[2], a[3] == ".T."
        loops, outer = [], None
        for b in bounds:
            bt, ba = self.e[b]
            pts = self.loop_points(ba[1])
            if len(pts) < 3:
                continue
            loops.append(pts)
            if bt == "FACE_OUTER_BOUND":
                outer = len(loops) - 1
        if not loops:
            return np.zeros((0, 3, 3))
        stype, sa = self.e[surf]
        if stype == "PLANE":
            o, x, y, z = self.axis(sa[1])
            to2 = lambda P: np.stack([(P - o) @ x, (P - o) @ y], 1)  # noqa
            to3 = lambda Q: o + np.outer(Q[:, 0], x) + np.outer(Q[:, 1], y)  # noqa
            normal = lambda P: np.tile(z, (len(P), 1))           # noqa
            grid = None
        elif stype == "CYLINDRICAL_SURFACE":
            o, x, y, z = self.axis(sa[1])
            r = sa[2]

            def to2(P, ref=None):
                th = np.unwrap(np.arctan2((P - o) @ y, (P - o) @ x))
                return np.stack([th * r, (P - o) @ z], 1)

            def to3(Q):
                th = Q[:, 0] / r
                return (o + np.outer(r * np.cos(th), x)
                        + np.outer(r * np.sin(th), y) + np.outer(Q[:, 1], z))

            def normal(P):
                d = (P - o) - np.outer((P - o) @ z, z)
                return d / np.linalg.norm(d, axis=1, keepdims=True)
            grid = 1.5
        else:
            return self._param_face(loops, outer or 0, stype, sa, same)

        loops2 = [to2(L) for L in loops]
        if stype == "CYLINDRICAL_SURFACE" and len(loops2) > 1:
            # keep all loops on the same wrap of the angle
            ref = loops2[outer or 0][:, 0].mean()
            for L in loops2:
                L[:, 0] += np.round((ref - L[:, 0].mean()) / (2 * np.pi * r)) \
                    * 2 * np.pi * r
        tris2 = _triangulate(loops2, outer or 0, grid)
        if not len(tris2):
            return np.zeros((0, 3, 3))
        flat = tris2.reshape(-1, 2)
        P = to3(flat).reshape(-1, 3, 3)
        # Face the triangles the way the surface points out of the solid.
        want = normal(P.mean(1)) * (1 if same else -1)
        if stype not in ("PLANE", "CYLINDRICAL_SURFACE"):
            want = None
        n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
        if want is not None:
            flip = np.einsum("ij,ij->i", n, want) < 0
            P[flip] = P[flip][:, [0, 2, 1]]
        return P

    def _surface(self, stype, sa):
        """(eval(uv)->xyz, invert(xyz)->uv, u-period or None, outward(xyz, uv))."""
        if stype == "TOROIDAL_SURFACE":
            o, x, y, z = self.axis(sa[1])
            R, r = sa[2], sa[3]

            def ev(Q):
                u, v = Q[:, 0], Q[:, 1]
                rad = R + r * np.cos(v)
                return (o + np.outer(rad * np.cos(u), x)
                        + np.outer(rad * np.sin(u), y) + np.outer(r * np.sin(v), z))

            def inv(P):
                d = P - o
                u = np.arctan2(d @ y, d @ x)
                v = np.arctan2(d @ z, np.hypot(d @ x, d @ y) - R)
                return np.stack([u, v], 1)
            return ev, inv, (2 * np.pi, 2 * np.pi), (R + r, r)
        if stype == "CONICAL_SURFACE":
            o, x, y, z = self.axis(sa[1])
            r0, ta = sa[2], np.tan(sa[3])

            def ev(Q):
                u, v = Q[:, 0], Q[:, 1]
                rad = r0 + v * ta
                return (o + np.outer(rad * np.cos(u), x)
                        + np.outer(rad * np.sin(u), y) + np.outer(v, z))

            def inv(P):
                d = P - o
                return np.stack([np.arctan2(d @ y, d @ x), d @ z], 1)
            return ev, inv, (2 * np.pi, None), (r0, 1.0)
        # B-spline, plain or rational (complex entity)
        if stype == "COMPLEX":
            c = _complex(sa)
            du, dv, cp = c["B_SPLINE_SURFACE"][:3]
            kn = c["B_SPLINE_SURFACE_WITH_KNOTS"]
            tu, tv = _knots(kn[0], kn[2]), _knots(kn[1], kn[3])
            W = c.get("RATIONAL_B_SPLINE_SURFACE", [None])[0]
            W = None if W is None else np.array(W, float)
        else:
            du, dv, cp = sa[1], sa[2], sa[3]
            tu, tv = _knots(sa[8], sa[10]), _knots(sa[9], sa[11])
            W = None
        P = np.array([[self.pt(i) for i in row] for row in cp])
        S = _Nurbs(int(du), int(dv), P, tu, tv, W)
        us = np.linspace(*S.ur, 120)
        vs = np.linspace(*S.vr, 120)
        G = S.grid(us, vs)
        from scipy.spatial import cKDTree
        tree = cKDTree(G.reshape(-1, 3))
        UV = np.stack(np.meshgrid(us, vs, indexing="ij"), -1).reshape(-1, 2)
        span_u = np.linalg.norm(np.diff(G, axis=0), axis=2).sum(0).max()
        span_v = np.linalg.norm(np.diff(G, axis=1), axis=2).sum(1).max()
        lu, lv = S.ur[1] - S.ur[0], S.vr[1] - S.vr[0]

        def inv(Pts):
            uv = UV[tree.query(Pts)[1]].copy()
            h = 1e-6
            for _ in range(6):                      # Gauss-Newton polish
                f = S.at(uv)
                Su = (S.at(uv + [h * lu, 0]) - f) / (h * lu)
                Sv = (S.at(uv + [0, h * lv]) - f) / (h * lv)
                r = Pts - f
                a11, a12, a22 = (Su * Su).sum(1), (Su * Sv).sum(1), (Sv * Sv).sum(1)
                b1, b2 = (Su * r).sum(1), (Sv * r).sum(1)
                det = a11 * a22 - a12 ** 2 + 1e-18
                uv[:, 0] += (a22 * b1 - a12 * b2) / det
                uv[:, 1] += (a11 * b2 - a12 * b1) / det
                uv[:, 0] = np.clip(uv[:, 0], *S.ur)
                uv[:, 1] = np.clip(uv[:, 1], *S.vr)
            return uv
        return S.at, inv, (None, None), (span_u / lu, span_v / lv)

    def _param_face(self, loops, outer, stype, sa, same):
        ev, inv, period, scale = self._surface(stype, sa)
        su, sv = scale
        loops2 = []
        for L in loops:
            dense = []
            ring = np.vstack([L, L[:1]])
            for p, q in zip(ring[:-1], ring[1:]):
                n = max(1, int(np.linalg.norm(q - p) / 0.8))
                dense.append(p + np.outer(np.linspace(0, 1, n, endpoint=False), q - p))
            uv = inv(np.vstack(dense))
            for k in (0, 1):
                if period[k]:
                    uv[:, k] = np.unwrap(uv[:, k], period=period[k])
            loops2.append(uv * [su, sv])
        # keep inner loops on the outer loop's wrap
        for k in (0, 1):
            if period[k]:
                ref = loops2[outer][:, k].mean()
                P = period[k] * (su, sv)[k]
                for L in loops2:
                    L[:, k] += np.round((ref - L[:, k].mean()) / P) * P
        tris2 = _triangulate(loops2, outer, 0.8)
        if not len(tris2):
            return np.zeros((0, 3, 3))
        T = ev((tris2.reshape(-1, 2) / [su, sv])).reshape(-1, 3, 3)
        # orient by the surface normal Su x Sv (STEP's convention)
        c = tris2.mean(1) / [su, sv]
        h = 1e-5
        f = ev(c)
        nrm = np.cross(ev(c + [h, 0]) - f, ev(c + [0, h]) - f)
        if not same:
            nrm = -nrm
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        flip = np.einsum("ij,ij->i", n, nrm) < 0
        T[flip] = T[flip][:, [0, 2, 1]]
        return T

    # ---- solids and assembly
    def solids(self):
        """[(name, colour rgb, triangles in assembly mm)] for every solid."""
        e = self.e
        colour = {}
        for i, (t, a) in e.items():
            if t == "STYLED_ITEM":
                c = self._colour(a[1])
                if c is not None:
                    colour[a[2]] = c
        place = self._placements()
        out = []
        for i, (t, a) in e.items():
            if t != "MANIFOLD_SOLID_BREP":
                continue
            shell = e[a[1]][1][1]
            tris = [self.face_mesh(f) for f in shell]
            tris = np.vstack([x for x in tris if len(x)]) if tris else np.zeros((0, 3, 3))
            M = place.get(i, np.eye(4))
            tris = tris @ M[:3, :3].T + M[:3, 3]
            out.append((a[0], colour.get(i, (0.2, 0.2, 0.2)), tris))
        return out

    def _colour(self, styles):
        """Dig the first COLOUR_RGB out of a style tree."""
        stack = list(styles) if isinstance(styles, list) else [styles]
        seen = set()
        while stack:
            s = stack.pop()
            if isinstance(s, list):
                stack.extend(s)
                continue
            if not isinstance(s, int) or s in seen or s not in self.e:
                continue
            seen.add(s)
            t, a = self.e[s]
            if t == "COLOUR_RGB":
                return tuple(a[1:4])
            if isinstance(a, list):
                stack.extend(a)
        return None

    def _placements(self):
        """Solid id -> 4x4 transform into the top assembly."""
        e = self.e
        rep_of_solid = {}
        for i, (t, a) in e.items():
            if t.endswith("SHAPE_REPRESENTATION") and isinstance(a, list) \
                    and len(a) > 1 and isinstance(a[1], list):
                for item in a[1]:
                    if isinstance(item, int) and e.get(item, ("",))[0] == \
                            "MANIFOLD_SOLID_BREP":
                        rep_of_solid[item] = i
        # Representation links: child rep -> (parent rep, transform).
        parent = {}
        for i, (t, a) in e.items():
            if t == "COMPLEX" and "REPRESENTATION_RELATIONSHIP" in a:
                refs = [int(x) for x in re.findall(r"#(\d+)", a)]
                if len(refs) >= 3:
                    r1, r2, idt = refs[0], refs[1], refs[2]
                    if e.get(idt, ("",))[0] == "ITEM_DEFINED_TRANSFORMATION":
                        A1 = self.matrix(e[idt][1][2])
                        A2 = self.matrix(e[idt][1][3])
                        parent[r1] = (r2, A2 @ np.linalg.inv(A1))
            elif t == "SHAPE_REPRESENTATION_RELATIONSHIP":
                parent.setdefault(a[2], (a[3], np.eye(4)))
                parent.setdefault(a[3], (a[2], np.eye(4)))  # brep <-> shape rep
        place = {}
        for solid, rep in rep_of_solid.items():
            M, r, hops = np.eye(4), rep, 0
            seen = {r}
            while r in parent and hops < 20:
                r2, T = parent[r]
                if r2 in seen:
                    break
                seen.add(r2)
                M = T @ M
                r = r2
                hops += 1
            place[solid] = M
        return place


def _triangulate(loops, outer, grid=None):
    """Triangles (n, 3, 2) filling the outer loop minus the others."""
    from scipy.spatial import Delaunay
    dense = []
    for L in loops:
        ring = np.vstack([L, L[:1]])
        for p, q in zip(ring[:-1], ring[1:]):
            n = max(1, int(np.linalg.norm(q - p) / 1.0))
            dense.append(p + np.outer(np.linspace(0, 1, n, endpoint=False),
                                      q - p))
    pts = np.vstack(dense)
    if grid:
        lo, hi = pts.min(0), pts.max(0)
        gx = np.arange(lo[0] + grid / 2, hi[0], grid)
        gy = np.arange(lo[1] + grid / 2, hi[1], grid)
        if len(gx) and len(gy):
            G = np.stack(np.meshgrid(gx, gy), -1).reshape(-1, 2)
            pts = np.vstack([pts, G])
    pts = np.unique(np.round(pts, 6), axis=0)
    if len(pts) < 3:
        return np.zeros((0, 3, 2))
    try:
        tri = Delaunay(pts)
    except Exception:
        return np.zeros((0, 3, 2))
    T = pts[tri.simplices]
    c = T.mean(1)
    keep = _inside(loops[outer], c)
    for k, L in enumerate(loops):
        if k != outer:
            keep &= ~_inside(L, c)
    return T[keep]


def _inside(poly, pts):
    """Even-odd point-in-polygon test for many points at once."""
    poly = np.asarray(poly, float)
    x, y = pts[:, 0][:, None], pts[:, 1][:, None]
    x0, y0 = poly[:, 0][None, :], poly[:, 1][None, :]
    x1, y1 = np.roll(poly[:, 0], -1)[None, :], np.roll(poly[:, 1], -1)[None, :]
    crosses = (y0 > y) != (y1 > y)
    with np.errstate(divide="ignore", invalid="ignore"):
        xs = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
    return (crosses & (x < xs)).sum(axis=1) % 2 == 1


def write_stl(path, tris):
    """Binary STL of (n, 3, 3) triangles."""
    import struct
    tris = np.asarray(tris, np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)),
                                         ("a", "<u2")])
        rec["n"], rec["v"] = n, tris
        fh.write(rec.tobytes())


def main():
    """python cad/step.py part.step out_dir  -> one STL per visible colour
    plus parts.json (file, colour), in the part's own frame, metres."""
    import json
    import os
    import sys
    src, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    groups = {}
    for name, colour, tris in Step(src).solids():
        key = tuple(round(c, 3) for c in colour)
        groups.setdefault(key, []).append(tris)
    index = []
    for k, (colour, parts) in enumerate(sorted(groups.items())):
        fn = f"part{k}.stl"
        write_stl(os.path.join(out, fn), np.vstack(parts) / 1000.0)
        index.append({"file": fn, "rgb": list(colour),
                      "triangles": int(sum(len(p) for p in parts))})
    with open(os.path.join(out, "parts.json"), "w") as fh:
        json.dump(index, fh, indent=1)
    print(json.dumps(index, indent=1))


if __name__ == "__main__":
    main()
