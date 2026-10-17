"""
details.py

The body-mounted pieces that are not plates: the Makita battery and its
slide mount, the payload case on top (case2) and the rail sled under the
plate (rear rail v2). All are drawn only; none of them adds collision
shapes beyond what spider.py already gives the battery and payload.

Where the CAD has it, the CAD decides:
    battery     its own outline from Battery LxT 18V v7.ipt, coloured in
                bands like Ronan's render, release button and screw heads
                at the circles in the part file
Where it does not (case2, rear rail v2 and Makitabattery were not in
the pack), shapes and sizes come from Ronan's renders, scaled by the
body plate (230 mm wide), at the positions bot.iam places them.
"""

from __future__ import annotations

import os

import numpy as np

# --------------------------------------------------------------------
# Colours (r g b), Ronan's renders
# --------------------------------------------------------------------
CHARCOAL = (0.27, 0.27, 0.28)
YELLOW = (0.98, 0.72, 0.10)
MOUNT_GREY = (0.80, 0.81, 0.82)
SLED_GREY = (0.80, 0.81, 0.82)
SLED_WHITE = (0.93, 0.93, 0.94)
CASE_BLACK = (0.075, 0.075, 0.08)
WHITE = (0.95, 0.95, 0.96)
SLOT = (0.12, 0.12, 0.13)

# --------------------------------------------------------------------
# Battery bands, by height above the plate (m): the slide mount covers
# the rails, then the yellow band, then the charcoal housing.
# --------------------------------------------------------------------
MOUNT_RISE = 0.0165
MOUNT_MARGIN = 0.003            # mount stands this far outside the battery
BAND = 0.010

# --------------------------------------------------------------------
# Payload case (case2), from the renders: front end narrowing in plan
# over its last 24 mm and its top-front edge chamfered.
# --------------------------------------------------------------------
CASE_TAPER = (0.024, 0.008)     # length of taper, inset each side
CASE_CHAMFER = 0.014            # top-front edge
CASE_REAR_CHAMFER = 0.003

# --------------------------------------------------------------------
# Rail sled under the plate (rear rail v2), from the underside render:
# two flat runners, rounded in section, running from under the front of
# the plate back past the battery, their front ends curving up into the
# plate like sled runners; a recessed deck between them with a long slot
# and a raised block.
# --------------------------------------------------------------------
SLED_X = (-0.212, 0.100)        # rear and front ends of the runners
RUNNER_Y = 0.034                # runner centre lines either side
RUNNER_W = 0.024                # width
RUNNER_D = 0.022                # depth below the plate
RUNNER_TIP = 0.060              # length of the curved-up front
RUNNER_EDGE = 0.005             # radius of the rounded edges
DECK_DROP = 0.007               # deck sits this far below the plate
PLATE_UNDER = -0.005            # underside of the plate


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


def build(cad, folder):
    """Battery and its mount. Returns (asset lines, body geom lines)."""
    os.makedirs(folder, exist_ok=True)
    assets, geoms = [], []

    bat = next(p for p in cad["body"] if p["part"] == "Battery LxT 18V v7")
    v = np.array(bat["vertices"])
    lo, hi = v.min(0), v.max(0)
    z0 = lo[2]
    band_lo, band_hi = z0 + MOUNT_RISE, z0 + MOUNT_RISE + BAND

    # One clean housing and one yellow band, both cut from the battery's
    # own outline so their sides line up exactly.
    for name, a, b, col in (("bat_band", band_lo - 0.0005, band_hi, YELLOW),
                            ("bat_body", band_hi, hi[2], CHARCOAL)):
        assets.append(_mesh(f"m_{name}", _slab(v, a, b)))
        geoms.append(_vis(f'name="{name}" type="mesh" mesh="m_{name}" '
                          f'rgba="{_rgba(col)}" material="satin"'))

    # Slide mount: a light grey block round the battery's foot, its top
    # edges rounded, sitting on the plate.
    m_lo = [lo[0] - MOUNT_MARGIN, lo[1] - MOUNT_MARGIN, 0.0]
    m_hi = [hi[0] + MOUNT_MARGIN, hi[1] + MOUNT_MARGIN, z0 + MOUNT_RISE]
    assets.append(_mesh("m_bat_mount", _rounded_box(m_lo, m_hi, 0.002)))
    geoms.append(_vis(f'name="battery_mount" type="mesh" mesh="m_bat_mount" '
                      f'rgba="{_rgba(MOUNT_GREY)}" material="satin"'))

    # Button, screw heads, the grooves either side of the button and the
    # vents in the band, on the end faces (button and screws at the
    # circles in the part file; cm, part frame).
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
        geoms.append(_vis(
            f'name="{name}" type="cylinder" size="{_n(r / 100)} 0.0008" '
            f'pos="{_vec(p)}" zaxis="{_vec(xdir * out)}" '
            f'rgba="{_rgba(col)}"'))
    front_x = hi[0] if xdir[0] < 0 else lo[0]       # face towards the robot
    button = to_sim((-0.06, -0.35, -2.025))
    for k, dy in enumerate((-0.022, 0.022)):
        geoms.append(_vis(
            f'name="bat_groove{k}" type="capsule" size="0.0035 0.010" '
            f'pos="{_n(front_x + 0.0004)} {_n(button[1] + dy)} '
            f'{_n(button[2] + 0.004)}" rgba="0.17 0.17 0.18 1"'))
    for k in range(4):
        geoms.append(_vis(
            f'name="bat_vent{k}" type="box" size="0.0006 0.0016 0.0032" '
            f'pos="{_n(front_x + 0.0003)} {_n(button[1] - 0.0075 + 0.005 * k)} '
            f'{_n((band_lo + band_hi) / 2)}" rgba="0.15 0.15 0.16 1"'))

    # Release latch: the yellow tab at the battery's tail, seen from below.
    tail = lo[0] if xdir[0] < 0 else hi[0]
    geoms.append(_vis(
        f'name="bat_latch" type="capsule" size="0.0045 0.013" '
        f'pos="{_n(tail - 0.003)} 0 {_n(PLATE_UNDER - 0.004)}" zaxis="0 1 0" '
        f'rgba="{_rgba(YELLOW)}"'))
    return assets, geoms


def _runner(y):
    """One sled runner: a flat bar with rounded edges whose front end
    curves up into the plate, like the runner of a sled (convex points)."""
    x0, x1 = SLED_X
    top, bottom = PLATE_UNDER, PLATE_UNDER - RUNNER_D
    r = RUNNER_EDGE
    zt, zb = top - r, bottom + r                   # core, inside the rounding
    tail = (zt - zb) / 2
    core = [(x0 + tail, zt), (x1 - r, zt)]
    for t in np.linspace(0.0, -np.pi / 2, 18):       # the curved-up tip
        core.append((x1 - RUNNER_TIP + (RUNNER_TIP - r) * np.cos(t),
                     zt + (zt - zb) * np.sin(t)))
    for t in np.linspace(-np.pi / 2, -3 * np.pi / 2, 10):   # rounded tail
        core.append((x0 + tail + tail * np.cos(t), (zt + zb) / 2
                     + tail * np.sin(t)))
    dirs = []
    for u in np.linspace(-np.pi / 2, np.pi / 2, 7):
        for w in np.linspace(0, 2 * np.pi, 12, endpoint=False):
            dirs.append((np.cos(u) * np.cos(w), np.cos(u) * np.sin(w),
                         np.sin(u)))
    half = RUNNER_W / 2 - r
    pts = [(cx + r * dx, y + sy * half + r * dy, cz + r * dz)
           for cx, cz in core for dx, dy, dz in dirs for sy in (-1, 1)]
    pts = np.array(pts)
    pts[:, 2] = np.minimum(pts[:, 2], top)
    return pts


def case_and_sled(size, centre_x, folder):
    """Payload case and the sled under the plate, as XML lines."""
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
    assets += ['    <material name="satin" specular="0.18" shininess="0.35"/>',
               '    <material name="matte" specular="0.06" shininess="0.1"/>',
               _mesh("m_case_look", pts)]
    geoms.append(_vis(f'name="case_look" type="mesh" mesh="m_case_look" '
                      f'rgba="{_rgba(CASE_BLACK)}" material="matte"'))

    # ---------------- rail sled underneath ----------------
    x0, x1 = SLED_X
    for side, s in (("l", 1), ("r", -1)):
        assets.append(_mesh(f"m_runner_{side}", _runner(s * RUNNER_Y)))
        geoms.append(_vis(f'name="sled_runner_{side}" type="mesh" '
                          f'mesh="m_runner_{side}" rgba="{_rgba(SLED_GREY)}" '
                          f'material="satin"'))
    deck_x0, deck_x1 = x0 + 0.004, x1 - RUNNER_TIP * 0.55
    inner = RUNNER_Y - RUNNER_W / 2 + 0.001
    # Side walls from the plate down to the deck, then the deck itself.
    geoms.append(_vis(
        f'name="sled_deck" type="box" size="{_n((deck_x1 - deck_x0) / 2)} '
        f'{_n(inner)} 0.0015" pos="{_n((deck_x0 + deck_x1) / 2)} 0 '
        f'{_n(PLATE_UNDER - DECK_DROP)}" rgba="{_rgba(SLED_GREY)}" '
        f'material="satin"'))
    for side, s in (("l", 1), ("r", -1)):
        geoms.append(_vis(
            f'name="sled_wall_{side}" type="box" '
            f'size="{_n((deck_x1 - deck_x0) / 2)} 0.0012 {_n(DECK_DROP / 2)}" '
            f'pos="{_n((deck_x0 + deck_x1) / 2)} {_n(s * (inner - 0.0012))} '
            f'{_n(PLATE_UNDER - DECK_DROP / 2)}" rgba="{_rgba(SLED_GREY)}"'))
    slot_l = (deck_x1 - deck_x0) * 0.42
    geoms.append(_vis(
        f'name="sled_slot" type="box" size="{_n(slot_l / 2)} 0.0055 0.0002" '
        f'pos="{_n(deck_x0 + (deck_x1 - deck_x0) * 0.62)} 0 '
        f'{_n(PLATE_UNDER - DECK_DROP - 0.0016)}" rgba="{_rgba(SLOT)}"'))
    geoms.append(_vis(
        f'name="sled_block" type="box" size="0.017 {_n(inner * 0.8)} 0.0045" '
        f'pos="{_n(deck_x0 + (deck_x1 - deck_x0) * 0.30)} 0 '
        f'{_n(PLATE_UNDER - DECK_DROP - 0.0045)}" rgba="{_rgba(SLED_WHITE)}" '
        f'material="satin"'))
    return assets, geoms
