"""
details.py

The body-mounted pieces that are not plates: the Makita battery and the
payload case on top (case2). Both are drawn only; spider.py gives them
their collision shapes and mass.

    battery     its own outline from Battery LxT 18V v7.ipt, one colour,
                sitting straight on the plate
    case2       not in the CAD pack: sized from Ronan's renders, scaled by
                the body plate (230 mm wide), where bot.iam places it
"""

from __future__ import annotations

import os

import numpy as np

# --------------------------------------------------------------------
# Colours (r g b), Ronan's renders
# --------------------------------------------------------------------
BATTERY = (0.075, 0.075, 0.08)   # black, like the front case
CASE_BLACK = (0.075, 0.075, 0.08)
RAILS = 0.0165                   # m, battery slide rails below the housing

# --------------------------------------------------------------------
# Payload case (case2), from the renders: front end narrowing in plan
# over its last 24 mm and its top-front edge chamfered.
# --------------------------------------------------------------------
CASE_TAPER = (0.024, 0.008)     # length of taper, inset each side
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


def build(cad, folder):
    """The battery: its CAD outline in one piece, sitting on the plate.
    Returns (asset lines, body geom lines)."""
    os.makedirs(folder, exist_ok=True)
    bat = next(p for p in cad["body"] if p["part"] == "Battery LxT 18V v7")
    v = np.array(bat["vertices"])
    # Its slide rails (the narrow bottom 16 mm) went into the mount, which
    # is not drawn; without it the housing would float above the plate.
    # Draw the housing only, resting on the plate top (z = 0).
    rails = v[:, 2].min() + RAILS
    body = _slab(v, rails, v[:, 2].max())
    body[:, 2] -= rails
    assets = [_mesh("m_bat_look", body)]
    geoms = [_vis(f'name="battery_look" type="mesh" mesh="m_bat_look" '
                  f'rgba="{_rgba(BATTERY)}" material="satin"')]
    return assets, geoms


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
    assets += ['    <material name="satin" specular="0.18" shininess="0.35"/>',
               '    <material name="matte" specular="0.06" shininess="0.1"/>',
               _mesh("m_case_look", pts)]
    geoms.append(_vis(f'name="case_look" type="mesh" mesh="m_case_look" '
                      f'rgba="{_rgba(CASE_BLACK)}" material="matte"'))

    return assets, geoms
