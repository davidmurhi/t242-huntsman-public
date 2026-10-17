"""
details.py

The body-mounted pieces that are not plates: the Makita battery and its
mount, the payload case on top (case2) and the rail sled underneath
(rear rail v2). All are drawn only; none of them adds collision shapes
beyond what spider.py already gives the battery and payload.

Where the CAD has it, the CAD decides:
    battery     outline from Battery LxT 18V v7.ipt, cut into its
                colour bands; release button and the four yellow screw
                heads at the circles in the part file
Where it does not (case2, rear rail v2 and Makitabattery were not in
the pack), sizes come from Ronan's renders, scaled by the body plate
(230 mm wide), and positions from where bot.iam places them.
"""

from __future__ import annotations

import os

import numpy as np
from PIL import Image, ImageDraw

# --------------------------------------------------------------------
# Colours (r g b), Ronan's renders
# --------------------------------------------------------------------
CHARCOAL = (0.25, 0.25, 0.26)
BAT_CAP = (0.19, 0.19, 0.20)
YELLOW = (0.98, 0.72, 0.10)
MOUNT_GREY = (0.78, 0.79, 0.80)
SLED_GREY = (0.82, 0.83, 0.84)
WHITE = (0.95, 0.95, 0.96)

# --------------------------------------------------------------------
# Battery bands, by height above the plate (m): the slide mount covers
# the rails, then a yellow band, then the charcoal housing.
# --------------------------------------------------------------------
MOUNT_RISE = 0.0165
BAND = 0.010

# --------------------------------------------------------------------
# Payload case (case2), from the renders: 129 x 93 x 60 mm, front end
# narrowing in plan over its last 24 mm and its top-front edge chamfered.
# --------------------------------------------------------------------
CASE_TAPER = (0.024, 0.008)     # length of taper, inset each side
CASE_CHAMFER = 0.014            # top-front edge
CASE_REAR_CHAMFER = 0.003

# --------------------------------------------------------------------
# Rail sled under the plate (rear rail v2), from the underside render.
# --------------------------------------------------------------------
SLED_X = (-0.203, 0.047)        # rear and front ends
SKI_Y = 0.030                   # ski centre lines either side
SKI_W, SKI_H = 0.024, 0.020
SLED_TOP = -0.005               # underside of the plate


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


def speckle_texture(path, size=512, seed=7):
    """Black 3D-print grain for the case."""
    rng = np.random.default_rng(seed)
    base = rng.normal(20, 6, (size, size))
    for _ in range(3):
        base = (base + np.roll(base, 1, 0) + np.roll(base, 1, 1)) / 3
    base += rng.normal(0, 4, (size, size))
    flecks = rng.random((size, size)) > 0.996
    base[flecks] = 70
    a = np.clip(base, 4, 90).astype("uint8")
    Image.fromarray(np.stack([a] * 3, -1)).save(path)


def case_top_texture(path, L, W, px=8):
    """Top of the case: grain, the recessed panel and its cable slots."""
    w, h = int(L * 1000 * px), int(W * 1000 * px)
    rng = np.random.default_rng(3)
    a = np.clip(rng.normal(21, 7, (h, w)), 4, 80)
    a[rng.random((h, w)) > 0.996] = 70
    img = Image.fromarray(np.stack([a.astype("uint8")] * 3, -1))
    d = ImageDraw.Draw(img)
    mm = lambda x, y: (x * px, y * px)                      # noqa: E731
    # Recessed panel: 82 x 62 mm, set back 18 mm from the rear edge.
    x0, y0 = 18, (W * 1000 - 62) / 2
    d.rectangle([mm(x0, y0), mm(x0 + 82, y0 + 62)], outline=(46, 46, 48),
                width=px)
    d.rectangle([mm(x0 + 1, y0 + 1), mm(x0 + 81, y0 + 61)],
                outline=(10, 10, 11), width=max(1, px // 2))
    # Cable slots seen in the render.
    for sx, sy, sl in ((40, y0 + 14, 16), (70, y0 + 40, 10), (56, y0 + 50, 8)):
        d.rounded_rectangle([mm(sx, sy), mm(sx + sl, sy + 3)], radius=px,
                            fill=(150, 150, 152))
    img.save(path)


def build(cad, folder):
    """Returns (asset lines, body geom lines)."""
    os.makedirs(folder, exist_ok=True)
    assets, geoms = [], []

    # ---------------- battery ----------------
    bat = next(p for p in cad["body"] if p["part"] == "Battery LxT 18V v7")
    v = np.array(bat["vertices"])
    z0 = v[:, 2].min()
    bands = [("bat_band", z0 + MOUNT_RISE - 0.001, z0 + MOUNT_RISE + BAND,
              YELLOW),
             ("bat_body", z0 + MOUNT_RISE + BAND, v[:, 2].max(), CHARCOAL)]
    for name, lo, hi, col in bands:
        pts = _slab(v, lo, hi)
        if name == "bat_band":                       # stand proud 0.6 mm
            c = pts.mean(0)
            pts[:, :2] = c[:2] + (pts[:, :2] - c[:2]) * np.array(
                [1 + 0.0012 / max(1e-6, np.ptp(pts[:, 0])),
                 1 + 0.0012 / max(1e-6, np.ptp(pts[:, 1]))])
        assets.append(_mesh(f"m_{name}", pts))
        geoms.append(_vis(f'name="{name}" type="mesh" mesh="m_{name}" '
                          f'rgba="{_rgba(col)}"'))
    # The rear cap (the end past the plate) is a shade darker in the render.
    xr = v[:, 0].min()
    cap = _slab(v, z0 + MOUNT_RISE + BAND, v[:, 2].max())
    cap = _slab(cap, xr - 0.0005, xr + 0.014, axis=0)
    cap[:, 0] -= 0.0004
    assets.append(_mesh("m_bat_cap", cap))
    geoms.append(_vis(f'name="bat_cap" type="mesh" mesh="m_bat_cap" '
                      f'rgba="{_rgba(BAT_CAP)}"'))

    # Mount: light grey slide plate the battery sits in.
    lo, hi = v.min(0), v.max(0)
    mid, half = (lo + hi) / 2, (hi - lo) / 2
    geoms.append(_vis(
        f'name="battery_mount" type="box" size="{_n(half[0] - 0.006)} '
        f'{_n(half[1] + 0.0025)} {_n((lo[2] + MOUNT_RISE) / 2)}" '
        f'pos="{_n(mid[0] + 0.004)} {_n(mid[1])} '
        f'{_n((lo[2] + MOUNT_RISE) / 2)}" rgba="{_rgba(MOUNT_GREY)}"'))

    # Button and screw heads, at the part file's circles (cm, part frame).
    R = np.array(bat["frame"]["rot"])
    P = np.array(bat["frame"]["pos"])
    to_sim = lambda c: R @ (np.array(c) / 100.0) + P            # noqa: E731
    xdir = R @ np.array([1.0, 0, 0])
    for name, c, r, col, out in (
            ("bat_button", (-0.06, -0.35, -2.025), 0.425, WHITE, -1),
            ("bat_screw_rl", (11.1, -1.15, -0.10), 0.35, YELLOW, 1),
            ("bat_screw_rr", (11.1, 1.15, -0.10), 0.35, YELLOW, 1),
            ("bat_screw_fl", (-0.12, -1.53, -0.28), 0.28, YELLOW, -1),
            ("bat_screw_fr", (-0.12, 1.53, -0.28), 0.28, YELLOW, -1)):
        p = to_sim(c) + out * xdir * 0.0006
        zaxis = xdir * out
        geoms.append(_vis(
            f'name="{name}" type="cylinder" size="{_n(r / 100)} 0.0008" '
            f'pos="{_vec(p)}" zaxis="{_vec(zaxis)}" rgba="{_rgba(col)}"'))

    # Release latch: yellow, at the battery's tail, under the plate line.
    geoms.append(_vis(
        f'name="bat_latch" type="box" size="0.010 0.015 0.0035" '
        f'pos="{_n(lo[0] + 0.009)} 0 {_n(lo[2] - 0.0045)}" '
        f'euler="0 -14 0" rgba="{_rgba(YELLOW)}"'))
    return assets, geoms


def case_and_sled(size, centre_x, folder):
    """Case mesh points, texture assets and the sled, as XML lines."""
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
    speckle_texture(os.path.join(folder, "case_grain.png"))
    case_top_texture(os.path.join(folder, "case_top.png"),
                     xf - c - (xr + cr), W)
    assets += [
        '    <texture name="case_grain" type="cube" file="textures/case_grain.png"/>',
        '    <material name="case_grain" texture="case_grain" specular="0.05" '
        'shininess="0.05"/>',
        '    <texture name="case_top" type="2d" file="textures/case_top.png"/>',
        '    <material name="case_top" texture="case_top" specular="0.05" '
        'shininess="0.05"/>',
        _mesh("m_case_look", pts)]
    geoms.append(_vis('name="case_look" type="mesh" mesh="m_case_look" '
                      'material="case_grain"'))
    top_l = xf - c - (xr + cr)
    geoms.append(_vis(
        f'name="case_top" type="box" size="{_n(top_l / 2)} '
        f'{_n(W / 2 - ti * 0.6)} 0.0002" pos="{_n(xr + cr + top_l / 2)} 0 '
        f'{_n(H + 0.0001)}" material="case_top"'))

    # ---------------- rail sled underneath ----------------
    x0, x1 = SLED_X
    lx, cx = (x1 - x0) / 2, (x0 + x1) / 2
    zc = SLED_TOP - SKI_H / 2
    for side, s in (("l", 1), ("r", -1)):
        y = s * SKI_Y
        geoms.append(_vis(
            f'name="sled_ski_{side}" type="box" size="{_n(lx - SKI_W / 2)} '
            f'{_n(SKI_W / 2)} {_n(SKI_H / 4)}" pos="{_n(cx)} {_n(y)} '
            f'{_n(SLED_TOP - SKI_H / 4)}" rgba="{_rgba(SLED_GREY)}"'))
        geoms.append(_vis(
            f'name="sled_keel_{side}" type="capsule" size="{_n(SKI_W / 2)} '
            f'{_n(lx - SKI_W / 2)}" pos="{_n(cx)} {_n(y)} '
            f'{_n(SLED_TOP - SKI_H / 2)}" zaxis="1 0 0" '
            f'rgba="{_rgba(SLED_GREY)}"'))
    geoms.append(_vis(
        f'name="sled_deck" type="box" size="{_n(lx - 0.012)} '
        f'{_n(SKI_Y)} 0.0015" pos="{_n(cx - 0.004)} 0 {_n(SLED_TOP - 0.0015)}" '
        f'rgba="{_rgba(SLED_GREY)}"'))
    geoms.append(_vis(
        f'name="sled_slot" type="box" size="{_n(lx * 0.55)} 0.006 0.0002" '
        f'pos="{_n(cx + 0.02)} 0 {_n(SLED_TOP - 0.0031)}" '
        f'rgba="0.30 0.31 0.33 1"'))
    geoms.append(_vis(
        f'name="sled_step" type="box" size="0.016 0.016 0.005" '
        f'pos="{_n(cx - 0.012)} 0 {_n(SLED_TOP - 0.008)}" '
        f'rgba="{_rgba((0.92, 0.92, 0.93))}"'))
    return assets, geoms
