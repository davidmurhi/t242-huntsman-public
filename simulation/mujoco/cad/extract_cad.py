#!/usr/bin/env python3
"""
extract_cad.py

Turns V Source's Inventor CAD into huntsman_cad.json, which spider.py
builds the simulation from. Run it again whenever Ronan sends new CAD;
nothing about the robot's shape is typed in by hand.

    python3 cad/extract_cad.py path/to/Huntsman/     (folder of .ipt/.iam)

What it does
    1. Reads every part's solid-model vertices (cm).
    2. Reads where each part sits in 'mounted servo', 'full leg' and 'bot'.
    3. Composes those placements, so every part of every leg is in one
       frame.
    4. Finds the three joint axes from the servo horn frames.
    5. Converts to the simulator's frame (metres, x forward, battery at
       the back, y left, z up) with the origin at the centre of the eight
       hip axes, level with the top of the body plate.
    6. Keeps only each part's convex-hull vertices, which is all MuJoCo
       needs for a collision mesh.

Parts the assembly references but the CAD pack did not include
(case2, rear rail v2, Makitabattery) are reported, not invented.
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import inventor  # noqa: E402

# Weights from Ronan's 'Spider Part Weight.xlsx' (15 Oct 2026), grams.
MASS_G = {
    "body": 180, "Battery LxT 18V v7": 671, "acrylic leg": 27,
    "9imod 55kg": 75, "Servo Bracket - U Shaped bis": 22,
    "Multi Purpose Bracket": 15, "Servo Mount - Horn Frame": 0,
    "payload": 867,
}

# Sheet-metal parts are open frames, and a convex hull would fill them in
# solid. Each is split into its plates, in the part's own coordinates (cm),
# using the plate planes found in the part files.
PIECES = {
    "Servo Bracket - U Shaped bis": [            # base plate and two arms
        lambda p: p[:, 1] <= 0.001,
        lambda p: p[:, 2] >= -0.301,
        lambda p: p[:, 2] <= -5.299,
    ],
    "Multi Purpose Bracket": [                   # spine and two flanges
        lambda p: p[:, 1] <= 0.301,
        lambda p: (p[:, 2] >= 2.95) & (p[:, 2] <= 3.40),
        lambda p: p[:, 2] <= 0.251,
    ],
    # The acrylic lower leg is a truss: a post at the servo end, a strip
    # under the servo notch, a column, then a top and a bottom strip either
    # side of a long triangular window running to the foot.
    "acrylic leg": [
        lambda p: p[:, 0] <= -17.99,                               # end post
        lambda p: (p[:, 0] <= -12.70) & (p[:, 2] <= -6.00),        # under notch
        lambda p: (p[:, 0] >= -13.95) & (p[:, 0] <= -12.70),       # column
        lambda p: (p[:, 0] >= -13.95) & (p[:, 2] >= -4.81),        # top strip
        lambda p: (p[:, 0] >= -13.30) & ((p[:, 2] <= -5.18)
                                         | (p[:, 0] >= -2.30)),    # bottom strip
    ],
    "body": [                                    # main deck, battery tab
        lambda p: p[:, 0] <= 12.40,
        lambda p: (p[:, 0] >= 12.50) & (np.abs(p[:, 2]) <= 3.40),
    ],
    "9imod 55kg": [                              # case, mounting tabs
        lambda p: (np.abs(p[:, 0]) <= 2.001) & (p[:, 1] <= 3.96),
        lambda p: (p[:, 1] >= 2.72) & (p[:, 1] <= 3.06),
    ],
}

# Details the solid model only implies. The servo's output spline is at
# x = -1.05 on its top face; Ronan's render shows the round horn there.
def _disc(cx, cz, y0, y1, r, k=20):
    a = np.linspace(0, 2 * np.pi, k, endpoint=False)
    ring = np.stack([cx + r * np.cos(a), np.zeros(k), cz + r * np.sin(a)], 1)
    return np.vstack([ring + [0, y0, 0], ring + [0, y1, 0]])


# Points added to a piece so it meets its neighbour: the battery tab runs
# back into the main deck rather than stopping 1 mm short of it.
PIECE_EXTRA = {
    ("body", 1): np.array([[12.2, y, z] for y in (0.0, 0.5)
                           for z in (-3.35, 3.35)]),
}

# Output shaft centre from the servo's own circles: x = -1.02, z = 0.
VISUALS = {
    "9imod 55kg": [("horn", _disc(-1.02, 0.0, 4.07, 4.30, 1.05)),
                   ("spline", _disc(-1.02, 0.0, 3.95, 4.60, 0.32))],
}

# Skins: what the robot looks like, as opposed to what it collides with.
# Each listed piece is a flat plate (or the servo case seen side-on). Its
# outline, thickness and every hole are read from the part file, and it
# is drawn as that outline extruded through its thickness, with a texture
# on both faces carrying the holes, the edge lines and, on the servo, the
# '9imod 55KG DSC55MG' lettering. Axes: (normal, across, up), right-handed.
SKINS = {
    ("9imod 55kg", 0): (2, 0, 1),
    ("9imod 55kg", 1): (1, 2, 0),
    ("Multi Purpose Bracket", 0): (1, 2, 0),
    ("Multi Purpose Bracket", 1): (2, 0, 1),
    ("Multi Purpose Bracket", 2): (2, 0, 1),
    ("Servo Bracket - U Shaped bis", 0): (1, 2, 0),
    ("Servo Bracket - U Shaped bis", 1): (2, 0, 1),
    ("Servo Bracket - U Shaped bis", 2): (2, 0, 1),
    ("body", 0): (1, 2, 0),
}
# The body plate is drawn as one outline: the main deck and the battery
# tab together, with the two concave fillets where they meet (the large
# circles at x = 12.5 cm in body.ipt) cut out of it.
SKIN_UNION = {("body", 0): (0, 1)}


def _ear_clip(poly):
    """Triangles (index triples) of a simple anticlockwise polygon."""
    idx = list(range(len(poly)))
    tris = []

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    guard = 0
    while len(idx) > 3 and guard < 20000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if cross(a, b, c) <= 1e-12:
                continue
            inside = False
            for j in idx:
                if j in (i0, i1, i2):
                    continue
                p = poly[j]
                if (cross(a, b, p) >= 0 and cross(b, c, p) >= 0
                        and cross(c, a, p) >= 0):
                    inside = True
                    break
            if not inside:
                tris.append((i0, i1, i2))
                idx.pop(k)
                break
        else:
            break
    if len(idx) == 3:
        tris.append(tuple(idx))
    return tris


def concave_outline(groups_uv, notches, px=40):
    """Outline of the union of convex pieces minus notch discs, traced on
    a fine grid (0.25 mm) and simplified to 0.1 mm."""
    import cv2
    from scipy.spatial import ConvexHull
    allp = np.vstack(groups_uv)
    lo = allp.min(0) - 0.5
    size = np.ceil((allp.max(0) + 0.5 - lo) * px).astype(int)
    mask = np.zeros((size[1], size[0]), np.uint8)
    for g in groups_uv:
        h = g[ConvexHull(g).vertices]
        cv2.fillPoly(mask, [np.round((h - lo) * px).astype(np.int32)], 255)
    for cu, cv_, r in notches:
        cv2.circle(mask, (int(round((cu - lo[0]) * px)),
                          int(round((cv_ - lo[1]) * px))),
                   int(round(r * px)), 0, -1)
    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cs, key=cv2.contourArea)
    c = cv2.approxPolyDP(c, 0.01 * px, True)[:, 0, :]
    poly = c / px + lo
    area = 0.5 * np.sum(poly[:, 0] * np.roll(poly[:, 1], -1)
                        - np.roll(poly[:, 0], -1) * poly[:, 1])
    if area < 0:
        poly = poly[::-1]
    return poly

# The servo label, read from the raised lettering on the case's +z face
# (bounding boxes of the letter outlines, cm, part coordinates).
SERVO_TEXT = [("9imod", -1.547, 0.124, 3.159, 3.627),
              ("55KG", -1.679, 0.059, 1.674, 2.201),
              ("DSC55MG", -1.653, 1.673, 0.784, 1.311)]


def skin_of(g, circles, axes):
    """Outline, thickness and holes of one plate, in its own 2D frame.

    Circles lying in the plate's plane are either holes (wholly inside
    the plate) or the rounded ends of its outline (which run past the
    corner vertices); the second kind are sampled into the outline so
    rounded ends stay round.
    """
    from scipy.spatial import ConvexHull
    a, u, v = axes
    uv = g[:, [u, v]]
    lo, hi = uv.min(0), uv.max(0)
    w0, w1 = float(g[:, a].min()), float(g[:, a].max())
    flat = np.zeros((0, 7))
    if len(circles):
        on = (np.abs(circles[:, 3 + a]) > 0.99) \
            & (circles[:, a] >= w0 - 0.01) & (circles[:, a] <= w1 + 0.01)
        flat = circles[on]

    def inside(ring, p, margin):
        h = ConvexHull(ring)
        return bool(np.all(h.equations[:, :2] @ p + h.equations[:, 2]
                           <= -margin))

    base = uv[ConvexHull(uv).vertices]
    holes, arcs = [], []
    for c in flat:
        cu, cv, r = c[u], c[v], c[6]
        if inside(base, np.array([cu, cv]), r * 0.98):
            holes.append((round(cu, 4), round(cv, 4), round(r, 4)))
        elif lo[0] - r < cu < hi[0] + r and lo[1] - r < cv < hi[1] + r:
            t = np.linspace(0, 2 * np.pi, 72, endpoint=False)
            pts = np.stack([cu + r * np.cos(t), cv + r * np.sin(t)], 1)
            keep = np.all((pts >= lo - 1e-3) & (pts <= hi + 1e-3), axis=1)
            arcs.append(pts[keep])
    if arcs:
        uv = np.vstack([uv] + arcs)
    ring = uv[ConvexHull(uv).vertices]          # anticlockwise
    holes = sorted(set(holes))
    return {"axes": list(axes), "w": [w0, w1],
            "outline": np.round(ring, 4).tolist(), "holes": holes}


def skin_of_union(groups, circles, axes):
    """skin_of for a plate drawn from several pieces, with notches."""
    a, u, v = axes
    sk = skin_of(np.vstack(groups), circles, axes)
    uvs = [g[:, [u, v]] for g in groups]
    allp = np.vstack(uvs)
    lo, hi = allp.min(0), allp.max(0)
    notches = []
    for c in circles:
        if abs(c[3 + a]) > 0.99 and c[6] > 1.0:
            if lo[0] < c[u] < hi[0] and lo[1] < c[v] < hi[1]:
                notches.append((c[u], c[v], c[6]))
    poly = concave_outline(uvs, notches)
    sk["outline"] = np.round(poly, 4).tolist()
    sk["tris"] = [list(map(int, t)) for t in _ear_clip(poly.tolist())]
    sk["holes"] = [h for h in sk["holes"] if h[2] <= 1.0]
    return sk


# Assembly frame (Inventor): x along the body (battery at +x), y up, z
# across. Simulator frame: x forward, y left, z up. The front of the robot
# is the end WITHOUT the battery (the battery rides at the back), so
# forward is the assembly's -x.
SKIN_PANEL = 0.48      # each face's share of the texture atlas width

TO_SIM = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]], dtype=float)


def hull(points):
    try:
        from scipy.spatial import ConvexHull
        return points[ConvexHull(points).vertices]
    except Exception:
        return points


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    folder = sys.argv[1]
    f = lambda name: os.path.join(folder, name)          # noqa: E731

    parts, circles = {}, {}
    for fn in sorted(os.listdir(folder)):
        if fn.endswith(".ipt"):
            parts[fn[:-4]] = inventor.part_vertices(f(fn))
            circles[fn[:-4]] = inventor.part_circles(f(fn))

    def placed(asm):
        names = inventor.occurrence_names(f(asm))
        mats = inventor.occurrence_placements(f(asm))
        names = names[-len(mats):]
        return list(zip(names, mats))

    mounted = placed("mounted servo.iam")
    leg = placed("full leg.iam")
    bot = placed("bot.iam")

    # Flatten the leg: sub-assembly 'mounted servo' expands to its parts.
    def flatten(occ):
        out = []
        for name, M in occ:
            base = name.rsplit(":", 1)[0]
            if base == "mounted servo":
                tag = name.replace(" ", "_")
                for n2, M2 in mounted:
                    out.append((f"{tag}/{n2.rsplit(':', 1)[0]}", M @ M2))
            else:
                out.append((name.rsplit(":", 1)[0] + "#" + name.rsplit(":", 1)[1],
                            M))
        return out
    leg_parts = flatten(leg)

    def part_of(key):
        return key.split("/")[-1].split("#")[0]

    # Which link each leg part belongs to, from how the servos are built:
    #   hip servo body is fixed to the chassis; its horn frame turns with
    #   the leg. The knee servo ('mounted servo:1') rides the hip horn;
    #   its horn frame drives the U-bracket femur. The ankle servo
    #   ('mounted servo:2') is bolted to the acrylic lower leg and turns
    #   with it; its horn frame is fixed to the end of the femur. That is
    #   how the built robot is assembled: the ankle servo hangs down with
    #   the lower leg.
    link_of = {
        "9imod 55kg#1": "body",
        "Servo Mount - Horn Frame#1": "coxa",
        "mounted_servo:1/9imod 55kg": "coxa",
        "mounted_servo:1/Multi Purpose Bracket": "coxa",
        "mounted_servo:1/Servo Mount - Horn Frame": "femur",
        "Servo Bracket - U Shaped bis#1": "femur",
        "Servo Bracket - U Shaped bis#2": "femur",
        "mounted_servo:2/9imod 55kg": "tibia",
        "mounted_servo:2/Multi Purpose Bracket": "tibia",
        "mounted_servo:2/Servo Mount - Horn Frame": "femur",
        "acrylic leg#1": "tibia",
    }
    missing = [k for k, _ in leg_parts if k not in link_of]
    if missing:
        raise SystemExit(f"unexpected parts in full leg: {missing}")

    lp = dict(leg_parts)
    # Horn frames: origin on the servo output shaft, shaft along local y.
    joints_leg = {
        "hip": lp["Servo Mount - Horn Frame#1"],
        "knee": lp["mounted_servo:1/Servo Mount - Horn Frame"],
        "ankle": lp["mounted_servo:2/Servo Mount - Horn Frame"],
    }

    # Body-level placements.
    botd = {n.rsplit(":", 1)[0] + "#" + n.rsplit(":", 1)[1]: M for n, M in bot}
    legs = [(k, M) for k, M in botd.items() if k.startswith("full leg#")]
    body_M = botd["body#1"]

    # Simulator origin: centre of the hip axes, top of the plate.
    hip_world = np.array([(M @ joints_leg["hip"])[:3, 3] for _, M in legs])
    plate = (parts["body"] @ body_M[:3, :3].T) + body_M[:3, 3]
    origin = np.array([hip_world[:, 0].mean(), plate[:, 1].max(),
                       hip_world[:, 2].mean()])

    def to_sim(pts_cm):
        return ((np.asarray(pts_cm) - origin) @ TO_SIM.T) / 100.0

    def dir_sim(v):
        return TO_SIM @ np.asarray(v)

    out = {"source": "V Source CAD, Huntsman.zip (15 Oct 2026)",
           "units": "metres, x forward (battery at the back), y left, z up",
           "parts_missing_from_pack": ["case2", "rear rail v2",
                                       "Makitabattery"],
           "body": [], "legs": [], "skins": {}}

    def skin_mesh(sk, M):
        """Extruded outline of a skin, placed by M, in simulator metres.

        Vertices: top ring, bottom ring, then the side walls (their own
        copies, so the walls can take the plain edge colour). Texture
        coordinates follow the atlas spiders.py draws: front face panel,
        back face panel (mirrored, so it reads the right way round from
        behind), then a narrow plain strip for the walls.
        """
        a, u, v = sk["axes"]
        ring = np.array(sk["outline"])
        n = len(ring)
        lo, hi = ring.min(0), ring.max(0)
        U = (ring[:, 0] - lo[0]) / (hi[0] - lo[0])
        V = (ring[:, 1] - lo[1]) / (hi[1] - lo[1])
        pf = SKIN_PANEL
        def pts(w):
            p = np.zeros((n, 3))
            p[:, u], p[:, v], p[:, a] = ring[:, 0], ring[:, 1], w
            return p
        top, bot = pts(sk["w"][1] + 0.002), pts(sk["w"][0] - 0.002)
        local = np.vstack([top, bot, top, bot])
        tex = ([(U[i] * pf, 1 - V[i]) for i in range(n)]
               + [((2 - U[i]) * pf, 1 - V[i]) for i in range(n)]
               + [(2 * pf + 0.5 * (1 - 2 * pf), 0.5)] * (2 * n))
        faces = []
        tris = sk.get("tris") or [(0, i, i + 1) for i in range(1, n - 1)]
        for i0, i1, i2 in tris:
            faces.append((i0, i1, i2))                   # front, +normal
            faces.append((n + i0, n + i2, n + i1))       # back
        for i in range(n):
            j = (i + 1) % n
            t0, t1, b0, b1 = 2 * n + i, 2 * n + j, 3 * n + i, 3 * n + j
            faces += [(t0, b0, b1), (t0, b1, t1)]
        world = local @ M[:3, :3].T + M[:3, 3]
        verts = to_sim(world)
        # Make every wall face point away from the plate's middle.
        mid = verts.mean(0)
        fixed = []
        for f3 in faces:
            p0, p1, p2 = verts[list(f3)]
            nrm = np.cross(p1 - p0, p2 - p0)
            cen = (p0 + p1 + p2) / 3
            if f3[0] >= 2 * n and nrm @ (cen - mid) < 0:
                f3 = (f3[0], f3[2], f3[1])
            fixed.append(f3)
        return {"vertices": np.round(verts, 6).tolist(),
                "faces": [list(map(int, f3)) for f3 in fixed],
                "texcoord": np.round(np.array(tex), 5).tolist()}

    def add(lst, key, base, M):
        local = parts[base]
        groups = ([local[sel(local)] for sel in PIECES[base]]
                  if base in PIECES else [local])
        groups = [np.vstack([g, PIECE_EXTRA[(base, k)]])
                  if (base, k) in PIECE_EXTRA else g
                  for k, g in enumerate(groups)]
        groups = [g for g in groups if len(g) >= 4]
        mass = MASS_G.get(base, 0) / 1000.0
        # The part's own frame in the simulator, so details (labels,
        # holes) can be laid on its faces: origin in metres, axes as rows.
        frame = {"pos": np.round(to_sim(M[:3, 3][None, :])[0], 6).tolist(),
                 "rot": np.round(TO_SIM @ M[:3, :3], 6).tolist()}
        for k, g in enumerate(groups):
            world = g @ M[:3, :3].T + M[:3, 3]
            lo, hi = g.min(0) / 100.0, g.max(0) / 100.0
            lst.append({"name": f"{key}.{k}" if len(groups) > 1 else key,
                        "part": base, "piece": k, "mass": mass / len(groups),
                        "frame": frame,
                        "local_box": [np.round(lo, 6).tolist(),
                                      np.round(hi, 6).tolist()],
                        "vertices": np.round(hull(to_sim(world)), 5).tolist()})
            if (base, k) in SKINS:
                skey = f"{base}.{k}"
                if skey not in out["skins"]:
                    cs = circles.get(base, np.zeros((0, 7)))
                    if (base, k) in SKIN_UNION:
                        sk = skin_of_union([groups[j] for j in
                                            SKIN_UNION[(base, k)]], cs,
                                           SKINS[(base, k)])
                    else:
                        sk = skin_of(g, cs, SKINS[(base, k)])
                    if base == "9imod 55kg" and k == 0:
                        sk["text"] = SERVO_TEXT
                    out["skins"][skey] = sk
                lst[-1]["skin"] = dict(key=skey,
                                       **skin_mesh(out["skins"][skey], M))
        for label, verts in VISUALS.get(base, []):
            world = verts @ M[:3, :3].T + M[:3, 3]
            lst.append({"name": f"{key}.{label}", "part": label, "mass": 0.0,
                        "visual": True,
                        "vertices": np.round(hull(to_sim(world)), 5).tolist()})

    add(out["body"], "plate", "body", body_M)
    add(out["body"], "battery", "Battery LxT 18V v7",
        botd["Battery LxT 18V v7#1"])
    for slot, (name, M) in enumerate(legs):
        add(out["body"], f"hip_servo_{slot}", "9imod 55kg",
            M @ lp["9imod 55kg#1"])

    # Unsupplied parts: keep their placements so they can be added later.
    out["placements_without_geometry"] = {
        k: {"pos": to_sim(M[:3, 3][None, :])[0].round(4).tolist(),
            "rot": np.round(TO_SIM @ M[:3, :3], 4).tolist()}
        for k, M in botd.items()
        if k.split("#")[0] in ("case2", "rear rail v2", "Makitabattery")}

    for name, M in legs:
        leg = {"cad_name": name, "links": {"coxa": [], "femur": [],
                                             "tibia": []}, "joints": {}}
        for key, Mp in leg_parts:
            link = link_of[key]
            if link == "body":
                continue
            add(leg["links"][link], key, part_of(key), M @ Mp)
        for j, J in joints_leg.items():
            W = M @ J
            leg["joints"][j] = {
                "pivot": to_sim(W[:3, 3][None, :])[0].round(5).tolist(),
                "axis": np.round(dir_sim(W[:3, 1]), 5).tolist()}
        # Foot tip: the acrylic leg's farthest point from the ankle axis.
        tib = np.array([v for p in leg["links"]["tibia"]
                        if p["part"] == "acrylic leg" for v in p["vertices"]])
        ank = np.array(leg["joints"]["ankle"]["pivot"])
        tip = tib[np.argmax(np.linalg.norm((tib - ank)[:, :2], axis=1))]
        leg["tip"] = np.round(tip, 5).tolist()
        out["legs"].append(leg)

    path = os.path.join(HERE, "huntsman_cad.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)
    total = (sum(p["mass"] for p in out["body"])
             + sum(p["mass"] for l in out["legs"]
                   for ps in l["links"].values() for p in ps))
    print(f"wrote {os.path.relpath(path)}")
    print(f"  parts read     {len(parts)}  ({', '.join(sorted(parts))})")
    print(f"  legs           {len(out['legs'])}")
    print(f"  mass in parts  {total:.3f} kg  (+ {MASS_G['payload'] / 1000:.3f}"
          f" kg payload, whose CAD was not in the pack)")
    print(f"  not in pack    {', '.join(out['parts_missing_from_pack'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
