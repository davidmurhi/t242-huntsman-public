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

    # ---- cradle: floor, two end walls, two side walls (U-shaped) ----
    x0, x1 = cx - CR_L / 2, cx + CR_L / 2
    w2 = CR_W / 2
    piece("cradle_floor", _rounded_box([x0, -w2, 0.0], [x1, w2, CR_T], 0.002),
          CRADLE_BLACK)
    post = 0.018                                   # tall corner length
    ramp = 0.010
    for sx, tag in ((-1, "rear"), (1, "front")):
        xe = cx + sx * (CR_L / 2 - CR_T / 2)
        # End wall: tall posts at both corners, lower in the middle.
        for sy in (-1, 1):
            yi = sy * (w2 - post)
            piece(f"cradle_end_{tag}_{'l' if sy > 0 else 'r'}", np.vstack([
                _box_pts([xe - CR_T / 2, min(yi, sy * w2), 0],
                         [xe + CR_T / 2, max(yi, sy * w2), CR_H]),
                _box_pts([xe - CR_T / 2, yi - sy * ramp, 0],
                         [xe + CR_T / 2, yi - sy * ramp, CR_LOW])]),
                CRADLE_BLACK)
        box(f"cradle_end_{tag}_mid", (xe, 0, CR_LOW / 2),
            (CR_T / 2, w2 - post - ramp + 0.0005, CR_LOW / 2), CRADLE_BLACK)
    for sy, tag in ((1, "l"), (-1, "r")):
        ye = sy * (w2 - CR_T / 2)
        for sx in (-1, 1):
            xi = cx + sx * (CR_L / 2 - post)
            piece(f"cradle_side_{tag}_{'f' if sx > 0 else 'b'}", np.vstack([
                _box_pts([min(xi, cx + sx * CR_L / 2), ye - CR_T / 2, 0],
                         [max(xi, cx + sx * CR_L / 2), ye + CR_T / 2, CR_H]),
                _box_pts([xi - sx * ramp, ye - CR_T / 2, 0],
                         [xi - sx * ramp, ye + CR_T / 2, CR_LOW])]),
                CRADLE_BLACK)
        box(f"cradle_side_{tag}_mid", (cx, ye, CR_LOW / 2),
            (CR_L / 2 - post - ramp + 0.0005, CR_T / 2, CR_LOW / 2),
            CRADLE_BLACK)

    # Locating blocks inside the short ends, as in the cradle drawing.
    for sx, tag in ((-1, "rear"), (1, "front")):
        box(f"cradle_stop_{tag}", (cx + sx * (CR_L / 2 - CR_T - 0.004), 0,
                                   CR_T + 0.005), (0.004, 0.011, 0.005),
            CRADLE_BLACK)

    # ---- battery: rounded black pack, label up, as in the photo ----
    z0 = CR_T
    bl, bw = BAT_L / 2, BAT_W / 2
    gap = CR_L / 2 - CR_T - 0.008 - bl            # clear of the stops
    piece("pack_body", _rounded_box([cx - bl, -bw, z0], [cx + bl, bw, z0 + BAT_H],
                                    0.007), PACK_BLACK)
    top = z0 + BAT_H
    # Rating label on top, inside the four corner screws.
    label_tex(os.path.join(folder, "bat_label.png"))
    side_tex(os.path.join(folder, "bat_side.png"))
    assets += [
        '    <texture name="bat_label" type="2d" file="textures/bat_label.png"/>',
        '    <material name="bat_label" texture="bat_label" specular="0.1"/>',
        '    <texture name="bat_side" type="2d" file="textures/bat_side.png"/>',
        '    <material name="bat_side" texture="bat_side" specular="0.1"/>']
    geoms.append(_vis(
        f'name="pack_label" type="box" size="0.040 0.026 0.0002" '
        f'pos="{_n(cx)} 0 {_n(top + 0.0002)}" material="bat_label"'))
    for i, (dx, dy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        geoms.append(_vis(
            f'name="pack_screw{i}" type="cylinder" size="0.0022 0.0004" '
            f'pos="{_n(cx + dx * (bl - 0.008))} {_n(dy * (bw - 0.007))} '
            f'{_n(top + 0.0003)}" rgba="0.03 0.03 0.03 1"'))
    # Rating on the logo side (no brand mark); a plain slot on the other.
    geoms.append(_vis(
        f'name="pack_side_l" type="box" size="0.040 0.0125 0.0002" '
        f'pos="{_n(cx)} {_n(bw + 0.0003)} {_n(z0 + BAT_H * 0.42)}" '
        f'xyaxes="-1 0 0 0 0 1" material="bat_side"'))
    geoms.append(_vis(
        f'name="pack_slot_r" type="box" size="0.030 0.0003 0.0018" '
        f'pos="{_n(cx)} {_n(-bw - 0.0001)} {_n(z0 + BAT_H * 0.55)}" '
        f'rgba="0.02 0.02 0.02 1"'))
    # Slide rails showing on both ends.
    for sx, tag in ((-1, "rear"), (1, "front")):
        box(f"pack_rail_{tag}", (cx + sx * (bl + 0.0005), 0, z0 + 0.010),
            (0.0008, 0.020, 0.006), (0.04, 0.04, 0.045))
    del gap

    # The plate's battery tab carried on under the cradle to its back end.
    tab_end = -0.197
    if x0 < tab_end:
        box("plate_tab_ext", ((x0 + tab_end + 0.002) / 2, 0, -0.0025),
            ((tab_end + 0.002 - x0) / 2, 0.0335, 0.0025), DARK)
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
