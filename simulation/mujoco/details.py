"""
details.py

The body-mounted pieces that are not plates: the battery in its cradle
at the back and the payload case (case2) at the front. Both are drawn
only; spider.py gives them their collision shapes and mass.

    battery     an 18 V 5.0 Ah power-tool pack (115.6 x 74.6 x 67.8 mm):
                black body, rail block on top, red hood with the charge
                gauge, rating on the sides
    cradle      125 x 80 x 40 mm, 4 mm walls, ends full height and the
                sides cut down in the middle, on the plate's battery tab
    case2       not in the CAD pack: sized from Ronan's renders so it sits
                between the hip servos without touching them, in the
                black speckled 3D-print finish
"""

from __future__ import annotations

import os

import numpy as np

# --------------------------------------------------------------------
# Colours (r g b), Ronan's renders
# --------------------------------------------------------------------
CASE_BLACK = (0.075, 0.075, 0.08)
DARK = (0.173, 0.176, 0.184)     # the legs' and plate's finish

# --------------------------------------------------------------------
# Payload case (case2), from the renders: front end narrowing in plan
# over its last 24 mm and its top-front edge chamfered.
# --------------------------------------------------------------------
CASE_TAPER = (0.026, 0.009)     # length of taper, inset each side
CASE_CHAMFER = 0.014            # top-front edge
CASE_REAR_CHAMFER = 0.003

HERE = os.path.dirname(os.path.abspath(__file__))


def _read_stl(path):
    """(n, 3, 3) triangles from a binary STL."""
    raw = open(path, "rb").read()
    n = int.from_bytes(raw[80:84], "little")
    rec = np.frombuffer(raw[84:84 + n * 50], dtype=[("n", "<f4", 3),
                                                    ("v", "<f4", (3, 3)),
                                                    ("a", "<u2")])
    return rec["v"].astype(float)


def _write_stl(path, tris):
    import struct
    tris = np.asarray(tris, np.float32)
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)),
                                     ("a", "<u2")])
    rec["n"], rec["v"] = nrm, tris
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80 + struct.pack("<I", len(tris)) + rec.tobytes())


def _n(v):
    return f"{v:.6g}"


def _vec(v):
    return " ".join(_n(x) for x in v)


def _rgba(c, a=1.0):
    return f"{_n(c[0])} {_n(c[1])} {_n(c[2])} {_n(a)}"


def _vis(attrs):
    return (f"<geom {attrs} density=\"0\" contype=\"0\" conaffinity=\"0\" "
            f"group=\"1\"/>")


def _mesh(name, pts):
    return (f'    <mesh name="{name}" vertex="'
            + " ".join(_n(x) for x in np.asarray(pts).ravel()) + '"/>')


def _slab(points, lo, hi, axis=2):
    """Convex hull of `points` cut to lo <= coordinate <= hi."""
    from scipy.spatial import ConvexHull, HalfspaceIntersection
    h = ConvexHull(points)
    eq = list(h.equations)
    e = np.zeros(4)
    e[axis], e[3] = -1.0, lo
    eq.append(e.copy())
    e[axis], e[3] = 1.0, -hi
    eq.append(e)
    eq = np.array(eq)
    inside = points.mean(0).copy()
    inside[axis] = (lo + hi) / 2
    return HalfspaceIntersection(eq, inside).intersections


def _rounded_box(lo, hi, r, k=6):
    """Points of a box with all edges rounded by r (convex)."""
    lo, hi = np.asarray(lo, float) + r, np.asarray(hi, float) - r
    pts = []
    for u in np.linspace(0, np.pi / 2, k):
        for w in np.linspace(0, np.pi / 2, k):
            d = np.array([np.cos(u) * np.cos(w), np.cos(u) * np.sin(w),
                          np.sin(u)]) * r
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        c = np.array([hi[0] if sx > 0 else lo[0],
                                      hi[1] if sy > 0 else lo[1],
                                      hi[2] if sz > 0 else lo[2]])
                        pts.append(c + d * [sx, sy, sz])
    return np.array(pts)


def _textured_mesh(name, pts, scale=0.06):
    """Convex hull of pts as a mesh with flat texture coordinates on every
    face (projected on the face's own plane), so a 2D texture tiles evenly
    with no smearing. scale = metres per texture repeat."""
    from scipy.spatial import ConvexHull
    pts = np.asarray(pts, float)
    h = ConvexHull(pts)
    verts, uvs, faces = [], [], []
    centre = pts.mean(0)
    for simplex, eq in zip(h.simplices, h.equations):
        tri = pts[simplex]
        nrm = eq[:3]
        if np.dot(np.cross(tri[1] - tri[0], tri[2] - tri[0]), nrm) < 0:
            tri = tri[[0, 2, 1]]
        ax = int(np.argmax(np.abs(nrm)))
        u, v = [k for k in range(3) if k != ax]
        base = len(verts)
        for q in tri:
            verts.append(q)
            uvs.append(((q[u] - centre[u]) / scale, (q[v] - centre[v]) / scale))
        faces.append((base, base + 1, base + 2))
    return (f'    <mesh name="{name}" inertia="shell" vertex="'
            + " ".join(_n(x) for x in np.ravel(verts)) + '" face="'
            + " ".join(str(i) for f in faces for i in f) + '" texcoord="'
            + " ".join(_n(x) for x in np.ravel(uvs)) + '"/>')


def speckle_tex(path, size=512, seed=7):
    """Black 3D-print finish with fine grey flecks, as in Ronan's render."""
    from PIL import Image
    rng = np.random.default_rng(seed)
    a = rng.normal(22, 3, (size, size))
    flecks = rng.random((size, size))
    a[flecks > 0.985] = rng.uniform(45, 85, (flecks > 0.985).sum())
    a = np.clip(a, 8, 90).astype("uint8")
    Image.fromarray(np.stack([a] * 3, -1)).save(path)


# --------------------------------------------------------------------
# Battery (18 V 5.0 Ah power-tool pack) and the cradle it sits in.
# Battery 115.6 x 74.6 x 67.8 mm; cradle 125 x 80 x 40 mm, 4 mm walls,
# ends full height, sides cut down in the middle. Placed where the CAD
# puts the battery, sitting on the plate's battery tab.
# --------------------------------------------------------------------
BAT_L, BAT_W, BAT_H = 0.1156, 0.0746, 0.0678
BODY_H = 0.048                  # main body; the rail block sits on top
CR_L, CR_W, CR_H, CR_T = 0.125, 0.082, 0.040, 0.004
CR_LOW = 0.014                  # height of the cut-down middle of the walls
PACK_BLACK = (0.09, 0.09, 0.095)
CRADLE_BLACK = (0.09, 0.09, 0.095)
PLATE_GREY = (0.17, 0.17, 0.18)      # same as the legs
INTERNAL = ("part1.stl", "part2.stl", "part4.stl")   # BMS, contacts
PLATE_X0 = -0.1985               # ramp end of the battery plate
PLATE_FLOOR = 0.007             # its pocket floor, above the base


def _box_pts(lo, hi):
    return np.array([[x, y, z] for x in (lo[0], hi[0])
                     for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def build(cad, folder):
    """The battery in its cradle. Returns (asset lines, body geom lines)."""
    os.makedirs(folder, exist_ok=True)
    bat = next(p for p in cad["body"] if p["part"] == "Battery LxT 18V v7")
    v = np.array(bat["vertices"])
    cx = float((v[:, 0].min() + v[:, 0].max()) / 2)
    assets, geoms = [], []

    def piece(name, pts, colour, material="satin"):
        assets.append(_mesh(f"m_{name}", pts))
        geoms.append(_vis(f'name="{name}" type="mesh" mesh="m_{name}" '
                          f'rgba="{_rgba(colour)}" material="{material}"'))

    def box(name, centre, half, colour, extra=""):
        geoms.append(_vis(f'name="{name}" type="box" size="{_vec(half)}" '
                          f'pos="{_vec(centre)}" rgba="{_rgba(colour)}" '
                          f'material="satin" {extra}'))

    # ---- battery plate: the printed adapter, from Battery Plate.stl ----
    # 72 x 90 x 22 mm (cad/parts, same part as Makita Battery Mounting
    # 04.step). Flat face screwed to the base tab; the pack slides in
    # along its 90 mm length, terminals down, latch at the ramped end.
    # Plate frame: x across (0..72), y along the slide (0 = ramp end),
    # z up from the base. Its latch pocket sits under the pack's latch.
    pl = _read_stl(os.path.join(HERE, "cad", "parts", "Battery Plate.stl"))
    pl = pl / 1000.0
    x0 = PLATE_X0
    x1 = x0 + 0.090
    placed = np.stack([x0 + pl[:, :, 1], pl[:, :, 0] - 0.036, pl[:, :, 2]], -1)
    placed = placed[:, ::-1]                 # swapping x, y mirrors: rewind
    pdir = os.path.join(folder, "..", "meshes", "battery_placed")
    os.makedirs(pdir, exist_ok=True)
    _write_stl(os.path.join(pdir, "plate.stl"), placed)
    assets.append('    <mesh name="m_battery_plate" '
                  'file="meshes/battery_placed/plate.stl" inertia="shell"/>')
    geoms.append(_vis(f'name="battery_plate" type="mesh" mesh="m_battery_plate" '
                      f'rgba="{_rgba(PLATE_GREY)}" material="satin"'))

    # ---- battery: the real pack, from Battery LxT 18V v7.step ----
    # The STEP (in cad/parts, exported to meshes/battery by cad/step.py)
    # is in the same frame as the Inventor part, so the CAD placement puts
    # it exactly where bot.iam does: terminals down into the cradle, the
    # flat label face up, latch button towards the robot. Lifted onto the
    # cradle floor.
    import json
    R = np.array(bat["frame"]["rot"])
    P = np.array(bat["frame"]["pos"])
    src = os.path.join(HERE, "meshes", "battery")
    out_dir = os.path.join(folder, "..", "meshes", "battery_placed")
    os.makedirs(out_dir, exist_ok=True)
    parts = json.load(open(os.path.join(src, "parts.json")))
    loaded = []
    for part in parts:
        tris = _read_stl(os.path.join(src, part["file"])) @ R.T + P
        loaded.append((part, tris))
    # The pack's terminal face rests on the plate's 7 mm floor; its latch
    # drops into the pocket at the ramped end.
    lift = PLATE_FLOOR - loaded[0][1][:, :, 2].min()
    for k, (part, tris) in enumerate(loaded):
        tris = tris + [0.0, 0.0, lift]
        if part["file"] in INTERNAL:
            continue
        fn = f"placed{k}.stl"
        _write_stl(os.path.join(out_dir, fn), tris)
        assets.append(f'    <mesh name="m_battery{k}" '
                      f'file="meshes/battery_placed/{fn}" inertia="shell"/>')
        geoms.append(_vis(f'name="battery{k}" type="mesh" mesh="m_battery{k}" '
                          f'rgba="{_rgba(part["rgb"])}" material="satin"'))
    # Solid dark core inside the case, so the side grip recesses (whose
    # curved backs the STEP does not carry) read as closed, not see-through.
    zc = lift + loaded[0][1][:, :, 2].min()          # case bottom
    box("pack_core", (-0.1585, 0, zc + 0.0265), (0.046, 0.033, 0.0215),
        PACK_BLACK)
    box("pack_core_rail", (-0.1685, 0, zc + 0.0075), (0.030, 0.033, 0.006),
        PACK_BLACK)
    allv = np.vstack([t.reshape(-1, 3) for _p, t in loaded]) + [0, 0, lift]
    top, bw = allv[:, 2].max(), allv[:, 1].max()
    # Printed rating label on the flat face, and the rating on one side.
    label_tex(os.path.join(folder, "bat_label.png"))
    side_tex(os.path.join(folder, "bat_side.png"))
    assets += [
        '    <texture name="bat_label" type="2d" file="textures/bat_label.png"/>',
        '    <material name="bat_label" texture="bat_label" specular="0.1"/>',
        '    <texture name="bat_side" type="2d" file="textures/bat_side.png"/>',
        '    <material name="bat_side" texture="bat_side" specular="0.1"/>']
    geoms.append(_vis(
        f'name="pack_label" type="box" size="0.038 0.025 0.0002" '
        f'pos="{_n(cx)} 0 {_n(top + 0.0002)}" material="bat_label"'))
    geoms.append(_vis(
        f'name="pack_side_l" type="box" size="0.036 0.0110 0.0002" '
        f'pos="{_n(cx)} {_n(bw + 0.0003)} {_n(top - 0.022)}" '
        f'xyaxes="-1 0 0 0 0 1" material="bat_side"'))

    # The plate's battery tab carried on under the cradle to its back end.
    tab_end = -0.197
    if x0 < tab_end:
        # Overlaps the real tab by 8 mm and sits 0.2 mm lower, so its
        # underside runs on from the tab's without a seam.
        x1e = tab_end + 0.008
        box("plate_tab_ext", ((x0 + x1e) / 2, 0, -0.0026),
            ((x1e - x0) / 2, 0.0335, 0.0026), DARK)
    return assets, geoms


def label_tex(path):
    """The pack's rating label: white sticker, title line, three symbol
    boxes and lines of small print (no brand mark)."""
    from PIL import Image, ImageDraw
    W, H = 640, 420
    img = Image.new("RGB", (W, H), (232, 232, 228))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([3, 3, W - 4, H - 4], 14, outline=(70, 70, 70), width=4)
    d.rectangle([24, 22, 300, 52], fill=(40, 40, 40))           # title bar
    for y in (70, 88):
        d.line([24, y, W - 24, y], fill=(110, 110, 110), width=6)
    for k in range(3):                                         # symbol boxes
        x = 24 + k * 200
        d.rectangle([x, 110, x + 180, 280], outline=(50, 50, 50), width=4)
    d.ellipse([54, 135, 174, 255], outline=(200, 30, 30), width=8)
    d.line([70, 240, 160, 150], fill=(200, 30, 30), width=8)
    d.ellipse([274, 140, 364, 230], outline=(40, 40, 40), width=6)
    d.line([474, 140, 564, 250], fill=(40, 40, 40), width=6)
    d.line([564, 140, 474, 250], fill=(40, 40, 40), width=6)
    for y in range(300, 400, 18):
        d.line([24, y, W - 24, y], fill=(120, 120, 120), width=5)
    img.save(path)


def side_tex(path):
    """'18V  5.0Ah  LITHIUM-ION' in white on black, the battery's side."""
    from PIL import Image, ImageDraw, ImageFont
    W, H = 1000, 160
    TEXT = "18V  5.0Ah  LITHIUM-ION"
    img = Image.new("RGB", (W, H), (19, 19, 20))
    d = ImageDraw.Draw(img)
    font = None
    for f in ("/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
              "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
              "C:/Windows/Fonts/arialbd.ttf"):
        if os.path.exists(f):
            size = 96
            while size > 20:                          # fit it with margins
                font = ImageFont.truetype(f, size)
                box = d.textbbox((0, 0), TEXT, font=font)
                if box[2] - box[0] <= W * 0.9 and box[3] - box[1] <= H * 0.7:
                    break
                size -= 4
            break
    font = font or ImageFont.load_default()
    d.text((W / 2, H / 2), TEXT, fill=(235, 235, 235), font=font,
           anchor="mm")
    img.save(path)


def case_and_sled(size, centre_x, folder):
    """Payload case, as XML lines."""
    assets, geoms = [], []
    L, W, H = size
    xr, xf = centre_x - L / 2, centre_x + L / 2
    tl, ti = CASE_TAPER
    c, cr = CASE_CHAMFER, CASE_REAR_CHAMFER
    pts = []
    for z in (0.0, H - c):
        pts += [(xr, -W / 2, z), (xr, W / 2, z), (xf - tl, -W / 2, z),
                (xf - tl, W / 2, z), (xf, -W / 2 + ti, z), (xf, W / 2 - ti, z)]
    pts += [(xr, -W / 2, H - cr), (xr, W / 2, H - cr),
            (xr + cr, -W / 2, H), (xr + cr, W / 2, H),
            (xf - tl, -W / 2, H), (xf - tl, W / 2, H),
            (xf - c, -W / 2 + ti * 0.6, H), (xf - c, W / 2 - ti * 0.6, H)]
    speckle_tex(os.path.join(folder, "case_speckle.png"))
    assets += ['    <material name="satin" specular="0.18" shininess="0.35"/>',
               '    <texture name="case_speckle" type="2d" '
               'file="textures/case_speckle.png"/>',
               '    <material name="case_speckle" texture="case_speckle" '
               'specular="0.06" shininess="0.1"/>',
               _textured_mesh("m_case_look", pts)]
    geoms.append(_vis('name="case_look" type="mesh" mesh="m_case_look" '
                      'material="case_speckle"'))

    return assets, geoms
