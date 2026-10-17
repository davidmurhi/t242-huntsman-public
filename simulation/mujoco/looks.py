"""
looks.py

Textures for the Huntsman model: what each plate looks like on its faces.

spider.py builds every bracket, servo case and the body plate as its CAD
outline extruded through its thickness (cad/extract_cad.py reads the
outline, thickness and hole positions from the part files). This module
paints the faces those meshes carry:

    - the part's colour, matched to Ronan's Inventor renders
    - every hole in the part file, dark with a light rim
    - the light edge line Inventor draws round each face
    - on the servo's +z face, the '9imod / 55KG / DSC55MG' lettering,
      sized to the raised letters in 9imod 55kg.ipt

Atlas layout (matches SKIN_PANEL in extract_cad.py): front face in the
left 48%, back face (mirrored so it reads correctly from behind) in the
next 48%, and a plain strip on the right for the side walls.
"""

from __future__ import annotations

import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

PANEL = 0.48
SUPER = 3                       # draw at 3x, then shrink: smooth edges

# Face colours (0-255), from Ronan's renders.
FACE = {
    "9imod 55kg": (88, 89, 93),
    "Multi Purpose Bracket": (46, 47, 49),
    "Servo Bracket - U Shaped bis": (44, 45, 47),
    "body": (104, 106, 109),
}
LINE = {                         # the light edge lines
    "9imod 55kg": (205, 207, 210),
    "Multi Purpose Bracket": (128, 130, 134),
    "Servo Bracket - U Shaped bis": (124, 126, 130),
    "body": (190, 192, 195),
}
HOLE = (16, 16, 17)
TEXT = (236, 237, 238)

FONTS = ["/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
         "/System/Library/Fonts/Supplemental/Arial.ttf",
         "C:/Windows/Fonts/arial.ttf",
         "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]


def _font(px):
    for f in FONTS:
        if os.path.exists(f):
            return ImageFont.truetype(f, px)
    return ImageFont.load_default()


def _px_per_cm(w, h):
    return min(120.0, 1400.0 / max(w, h))


def _draw_text(img, word, box, to_px):
    """Draw one word so its ink fills box = (u0, u1, v0, v1) in cm."""
    u0, u1, v0, v1 = box
    (x0, y1), (x1, y0) = to_px(u0, v0), to_px(u1, v1)
    w, h = int(round(x1 - x0)), int(round(y1 - y0))
    if w < 2 or h < 2:
        return
    f = _font(200)
    tmp = Image.new("L", (1400, 300), 0)
    ImageDraw.Draw(tmp).text((20, 20), word, font=f, fill=255)
    ink = tmp.crop(tmp.getbbox()).resize((w, h), Image.LANCZOS)
    img.paste(Image.new("RGB", (w, h), TEXT), (int(round(x0)),
                                                int(round(y0))), ink)


def paint(part, skin, mirror=False, text=True):
    """One face of a skin, as an RGB image at SUPER x resolution."""
    ring = np.array(skin["outline"])
    lo, hi = ring.min(0), ring.max(0)
    w, h = hi - lo
    s = _px_per_cm(w, h) * SUPER
    W, H = max(8, int(round(w * s))), max(8, int(round(h * s)))

    def to_px(u, v):
        x = (u - lo[0]) / w * W
        if mirror:
            x = W - x
        return x, (1.0 - (v - lo[1]) / h) * H

    face, line = FACE.get(part, (90, 90, 90)), LINE.get(part, (180, 180, 180))
    img = Image.new("RGB", (W, H), face)
    d = ImageDraw.Draw(img)
    edge = max(2, int(round(0.028 * s)))            # 0.28 mm line
    poly = [to_px(u, v) for u, v in ring]
    d.polygon(poly, fill=face)
    d.line(poly + [poly[0]], fill=line, width=edge, joint="curve")
    for cu, cv, r in skin.get("holes", []):
        x, y = to_px(cu, cv)
        rp = r * s
        d.ellipse([x - rp, y - rp, x + rp, y + rp], fill=HOLE,
                  outline=line, width=max(1, edge - 1))
    for hp in skin.get("hole_polys", []):
        pts = [to_px(u, v) for u, v in hp]
        d.polygon(pts, fill=HOLE)
        d.line(pts + [pts[0]], fill=line, width=max(1, edge - 1), joint="curve")
    if text and not mirror:
        for word, u0, u1, v0, v1 in skin.get("text", []):
            _draw_text(img, word, (u0, u1, v0, v1), to_px)
    return img


def atlas(part, skin):
    """Front, mirrored back and wall strip side by side."""
    front = paint(part, skin)
    back = paint(part, skin, mirror=True)
    W, H = front.size
    total = int(round(W / PANEL))
    img = Image.new("RGB", (total, H), FACE.get(part, (90, 90, 90)))
    img.paste(front, (0, 0))
    img.paste(back, (int(round(PANEL * total)), 0))
    return img.resize((max(4, total // SUPER), max(4, H // SUPER)),
                      Image.LANCZOS)


def write_all(skins, folder):
    """Write one PNG per skin; returns {skin key: file name}."""
    os.makedirs(folder, exist_ok=True)
    names = {}
    for key, sk in skins.items():
        part = key.rsplit(".", 1)[0]
        fn = "skin_" + key.replace(" ", "_").replace(".", "_") + ".png"
        atlas(part, sk).save(os.path.join(folder, fn))
        names[key] = fn
    return names
