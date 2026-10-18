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
MOUNT_MARGIN = 0.003             # m, mount stands this far outside the battery
DARK = (0.173, 0.176, 0.184)     # the legs' and plate's finish
LATCH = (0.24, 0.24, 0.25)
LEAD_RED = (0.70, 0.08, 0.06)
LEAD_BLACK = (0.04, 0.04, 0.04)
BEZEL = (0.05, 0.05, 0.055)
ROCKER = (0.12, 0.12, 0.13)

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
    lo, hi = v.min(0), v.max(0)
    rails = lo[2] + RAILS                  # top of the slide rails
    # Housing: the battery above its rails, where the CAD puts it.
    assets = [_mesh("m_bat_look", _slab(v, rails, hi[2]))]
    geoms = [_vis(f'name="battery_look" type="mesh" mesh="m_bat_look" '
                  f'rgba="{_rgba(BATTERY)}" material="satin"')]
    # Adapter dock: the block the battery slides onto, as on the real
    # robot, in the legs' dark finish, from the plate up to the housing.
    m_lo = [lo[0] - MOUNT_MARGIN, lo[1] - MOUNT_MARGIN, 0.0]
    m_hi = [hi[0] + MOUNT_MARGIN, hi[1] + MOUNT_MARGIN, rails]
    assets.append(_mesh("m_bat_mount", _rounded_box(m_lo, m_hi, 0.002)))
    geoms.append(_vis(f'name="battery_mount" type="mesh" mesh="m_bat_mount" '
                      f'rgba="{_rgba(DARK)}" material="satin"'))
    # Adapter dock details, after the common Makita 18V battery docks:
    # T-rails along both sides that the battery's slide locks into, a
    # back wall at the terminal end with the power leads coming out of
    # it, the latch notch at the open end, and the on/off rocker switch
    # on the front.
    length = m_hi[0] - m_lo[0]
    cx = (m_lo[0] + m_hi[0]) / 2
    for side, sy in (("l", 1), ("r", -1)):
        geoms.append(_vis(                                   # rail upright
            f'name="dock_rail_{side}" type="box" size="{_n(length / 2 - 0.006)} '
            f'0.0020 0.0045" pos="{_n(cx + 0.002)} '
            f'{_n(sy * (hi[1] + 0.0010))} {_n(rails + 0.0040)}" '
            f'rgba="{_rgba(DARK)}" material="satin"'))
        geoms.append(_vis(                                   # rail lip
            f'name="dock_lip_{side}" type="box" size="{_n(length / 2 - 0.006)} '
            f'0.0032 0.0012" pos="{_n(cx + 0.002)} '
            f'{_n(sy * (hi[1] - 0.0010))} {_n(rails + 0.0080)}" '
            f'rgba="{_rgba(DARK)}" material="satin"'))
    # Back wall at the battery's tail (its terminal end), with the leads.
    wall_x = m_lo[0] + 0.004
    geoms.append(_vis(
        f'name="dock_wall" type="box" size="0.004 {_n(hi[1] + 0.003)} 0.010" '
        f'pos="{_n(wall_x)} 0 {_n(rails + 0.006)}" rgba="{_rgba(DARK)}" '
        f'material="satin"'))
    for k, (dy, col) in enumerate(((0.008, LEAD_RED), (-0.008, LEAD_BLACK))):
        geoms.append(_vis(
            f'name="dock_lead{k}" type="capsule" size="0.0028 0.010" '
            f'pos="{_n(wall_x - 0.006)} {_n(dy)} {_n(rails + 0.004)}" '
            f'euler="0 70 0" rgba="{_rgba(col)}"'))
    # Latch notch at the open (front) end of the slide.
    geoms.append(_vis(
        f'name="dock_notch" type="box" size="0.006 0.012 0.0004" '
        f'pos="{_n(m_hi[0] - 0.010)} 0 {_n(rails + 0.0004)}" '
        f'rgba="0.06 0.06 0.065 1"'))
    # On/off rocker switch on the front face.
    front = m_hi[0]
    zc = rails / 2
    geoms.append(_vis(
        f'name="switch_bezel" type="box" size="0.0015 0.0095 0.0062" '
        f'pos="{_n(front + 0.0015)} 0 {_n(zc)}" rgba="{_rgba(BEZEL)}" '
        f'material="satin"'))
    geoms.append(_vis(
        f'name="switch_rocker" type="box" size="0.0014 0.0078 0.0048" '
        f'pos="{_n(front + 0.0033)} 0 {_n(zc)}" euler="0 12 0" '
        f'rgba="{_rgba(ROCKER)}" material="satin"'))
    geoms.append(_vis(
        f'name="switch_mark" type="box" size="0.0002 0.0007 0.0024" '
        f'pos="{_n(front + 0.0049)} 0 {_n(zc + 0.0008)}" euler="0 12 0" '
        f'rgba="0.92 0.92 0.92 1"'))

    # Battery markings: the white rating label on its top face and the
    # voltage/capacity line on both sides, as on a BL1850B (no brand mark).
    label_tex(os.path.join(folder, "bat_label.png"))
    side_tex(os.path.join(folder, "bat_side.png"))
    assets += [
        '    <texture name="bat_label" type="2d" file="textures/bat_label.png"/>',
        '    <material name="bat_label" texture="bat_label" specular="0.1"/>',
        '    <texture name="bat_side" type="2d" file="textures/bat_side.png"/>',
        '    <material name="bat_side" texture="bat_side" specular="0.1"/>']
    geoms.append(_vis(
        f'name="bat_label" type="box" size="0.030 0.026 0.0002" '
        f'pos="-0.1580 0 {_n(hi[2] + 0.0002)}" material="bat_label"'))
    for side, sy in (("l", 1), ("r", -1)):
        x_axis = "-1 0 0" if sy > 0 else "1 0 0"
        geoms.append(_vis(
            f'name="bat_side_{side}" type="box" size="0.031 0.0052 0.0002" '
            f'pos="-0.1610 {_n(sy * (hi[1] + 0.0004))} 0.0470" '
            f'xyaxes="{x_axis} 0 0 1" material="bat_side"'))
    # The plate's battery tab carried on under the mount to its back end,
    # so nothing hangs off the end of the base.
    tab_end = -0.197                       # where body.ipt's tab stops
    if m_lo[0] < tab_end:
        x0, x1 = m_lo[0], tab_end + 0.002
        geoms.append(_vis(
            f'name="plate_tab_ext" type="box" size="{_n((x1 - x0) / 2)} '
            f'0.0335 0.0025" pos="{_n((x0 + x1) / 2)} 0 -0.0025" '
            f'rgba="{_rgba(DARK)}" material="satin"'))
    return assets, geoms


def label_tex(path):
    """The battery's rating label: white, ruled, with a few blocks of
    small print and the recycling marks as simple shapes."""
    from PIL import Image, ImageDraw
    W, H = 600, 520
    img = Image.new("RGB", (W, H), (226, 226, 222))
    d = ImageDraw.Draw(img)
    d.rectangle([4, 4, W - 5, H - 5], outline=(60, 60, 60), width=4)
    for y in range(40, 200, 22):                     # small print
        d.line([30, y, W - 30, y], fill=(120, 120, 120), width=6)
    for i, x in enumerate((40, 180, 320, 460)):        # symbols
        d.ellipse([x, 260, x + 100, 360], outline=(40, 40, 40), width=6)
    d.rectangle([40, 400, W - 40, 470], outline=(40, 40, 40), width=5)
    img.save(path)


def side_tex(path):
    """'18V  5.0Ah  LITHIUM-ION' in white on black, the battery's side."""
    from PIL import Image, ImageDraw, ImageFont
    W, H = 1000, 160
    TEXT = "18V   5.0Ah   LITHIUM-ION"
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
    assets += ['    <material name="satin" specular="0.18" shininess="0.35"/>',
               '    <material name="matte" specular="0.06" shininess="0.1"/>',
               _mesh("m_case_look", pts)]
    geoms.append(_vis(f'name="case_look" type="mesh" mesh="m_case_look" '
                      f'rgba="{_rgba(CASE_BLACK)}" material="matte"'))

    return assets, geoms
