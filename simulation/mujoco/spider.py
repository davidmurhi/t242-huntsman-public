#!/usr/bin/env python3
"""
spider.py

Project Huntsman in MuJoCo, built from V Source's own CAD.

Every part's shape and position comes from cad/huntsman_cad.json, which
cad/extract_cad.py reads straight out of Ronan's Inventor files
(Huntsman.zip, 15 Oct 2026). Nothing about the robot's geometry is typed
in by hand here. The only numbers below are the ones the CAD cannot give
us: the standing pose, the servo control gains, and the payload box,
whose part file was not in the pack.

    body plate   349 x 230 x 5 mm      from body.ipt
    battery      Makita LXT 18V 5.0Ah  from Battery LxT 18V v7.ipt
    servos       24 x 9imod DSC55MG    from 9imod 55kg.ipt
    femur        2 U-brackets          from Servo Bracket - U Shaped bis.ipt
    lower leg    acrylic truss         from acrylic leg.ipt
    leg layout   front pair 23 deg forward, middle pairs straight out,
                 rear pair 30 deg back, hips 62 mm apart along each side
    front        the end without the battery; the battery rides at the back
    mass         4.326 kg              Ronan's part-weight sheet

RUN
    macOS            mjpython spider.py
    Windows / Linux  python spider.py
    python3 spider.py --check     report only, no viewer
    python3 spider.py --save      write spider.xml and exit

REQUIREMENTS
    pip install mujoco numpy
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.join(HERE, "cad", "huntsman_cad.json")

# =====================================================================
# WHAT THE CAD DOES NOT TELL US
# =====================================================================

# Standing pose. The CAD has every leg laid out flat. Ronan's photos of
# the built robot stand with the femur roughly level and the acrylic
# lower leg dropping steeply to the floor.
# Hip yaw in the standing pose, degrees forward (+) or back (-) of the
# CAD layout, per pair front to back. In the CAD the two middle legs on
# each side point straight out with their femur brackets only 6 mm apart,
# so they could never step independently. Splaying them apart, as the
# built robot in Ronan's photos does, gives every leg room of its own.
STANCE_YAW = (8.0, 14.0, -6.0, -12.0)
FEMUR_ANGLE_UP = 10.0          # knee raises the femur this far above level
TIBIA_ANGLE_DOWN = 72.0        # lower leg this far below level

# Payload enclosure ('case2' in bot.iam). Its part file was not in the
# pack, so it is a box sized from Ronan's top-view render, placed where
# bot.iam puts it. Its mass is the 867 g 'Payload' line on the sheet.
PAYLOAD_SIZE = (0.129, 0.093, 0.045)
PAYLOAD_CENTRE_X = 0.040
PAYLOAD_MASS = 0.867

# Servos: 9imod DSC55MG, 55 kg.cm = 5.39 N.m at stall. Confirm at the 8 V
# rail Craig is moving to before trusting the margins in the report.
SERVO_TORQUE = 55.0 * 0.0980665
HIP_GAIN, HIP_DAMPING = 80.0, 1.60
KNEE_GAIN, KNEE_DAMPING = 80.0, 1.40
ANKLE_GAIN, ANKLE_DAMPING = 45.0, 0.70
HIP_RANGE = (-40.0, 40.0)
KNEE_RANGE = (-35.0, 45.0)     # positive raises the leg
ANKLE_RANGE = (-60.0, 60.0)    # positive raises the foot

FOOT_RADIUS = 0.005
FOOT_CLEARANCE = 0.003
MIN_LEG_GAP = 0.004            # closest neighbouring legs may come

# Colours, matched to Ronan's renders (MuJoCo lights brighter than
# Inventor, so these sit a shade darker than the render looks).
COLOURS = {
    "body": "0.46 0.47 0.49 1",
    "Battery LxT 18V v7": "0.17 0.17 0.18 1",
    "9imod 55kg": "0.21 0.21 0.22 1",
    "Servo Bracket - U Shaped bis": "0.17 0.17 0.18 1",
    "Multi Purpose Bracket": "0.18 0.18 0.19 1",
    "Servo Mount - Horn Frame": "0.70 0.71 0.73 1",
    "acrylic leg": "0.03 0.03 0.035 1",
    "horn": "0.92 0.92 0.93 1",
    "spline": "0.62 0.63 0.66 1",
}
# Parts drawn but not collided with: horn hardware is inside the envelope
# of the parts it is bolted to.
VISUAL_ONLY = {"Servo Mount - Horn Frame", "horn", "spline"}


# =====================================================================

LEG_NAMES = [f"{s}{p}" for p in (1, 2, 3, 4) for s in ("l", "r")]
JOINTS = ("hip", "knee", "ankle")
GRAVITY = 9.81


def n(v):
    return f"{v:.6g}"


def vec(v):
    return " ".join(n(x) for x in v)


def rot(axis, deg):
    """Rotation matrix about a unit axis."""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    t = math.radians(deg)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


def load_cad():
    if not os.path.exists(CAD):
        sys.exit("cad/huntsman_cad.json is missing. Run cad/extract_cad.py.")
    with open(CAD, encoding="utf-8") as fh:
        return json.load(fh)


def name_legs(cad):
    """CAD legs -> l1..l4, r1..r4. Left is +y; pair 1 is the front, the
    end without the battery."""
    named = {}
    for side, letter in ((1, "l"), (-1, "r")):
        group = [lg for lg in cad["legs"]
                 if np.sign(lg["joints"]["hip"]["pivot"][1]) == side]
        group.sort(key=lambda lg: -lg["joints"]["hip"]["pivot"][0])
        for i, lg in enumerate(group):
            lg["_pair"] = i
            named[f"{letter}{i + 1}"] = lg
    return named


def pose_leg(lg):
    """Put one leg in the standing pose.

    Returns joint pivots, joint axes (positive = hip anticlockwise from
    above, knee and ankle raise), the parts of each link as vertex arrays
    in world coordinates, and the foot tip.
    """
    hip = np.array(lg["joints"]["hip"]["pivot"])
    knee = np.array(lg["joints"]["knee"]["pivot"])
    ankle = np.array(lg["joints"]["ankle"]["pivot"])
    tip = np.array(lg["tip"])
    links = {k: [(p["name"], p["part"], p["mass"], np.array(p["vertices"]))
                 for p in v] for k, v in lg["links"].items()}
    # Skins (the textured look of each plate) ride with their link.
    skins = {k: [(p["skin"]["key"], np.array(p["skin"]["vertices"]),
                  p["skin"]["faces"], p["skin"]["texcoord"], p["name"])
                 for p in v if "skin" in p] for k, v in lg["links"].items()}

    # Which way the leg points comes from the knee servo's shaft in the
    # CAD, which is square to the leg, rather than from where the foot
    # happens to be.
    shaft = np.array(lg["joints"]["knee"]["axis"], float)
    shaft[2] = 0.0
    out = np.cross(shaft, [0.0, 0.0, 1.0])
    out /= np.linalg.norm(out)
    if out[:2] @ (tip - hip)[:2] < 0:
        out = -out
    lift = np.cross(out, [0.0, 0.0, 1.0])      # rotating +about this raises

    def turn(points, pivot, axis, deg):
        return (points - pivot) @ rot(axis, deg).T + pivot

    def turn_skins(link, pivot, axis, deg):
        skins[link] = [(k, turn(v, pivot, axis, deg), f, t, nm)
                       for k, v, f, t, nm in skins[link]]

    # Hip: splay the whole leg to its standing direction. Positive yaw
    # swings the foot forward on both sides.
    side = 1.0 if out[1] > 0 else -1.0
    yaw = -side * STANCE_YAW[int(lg["_pair"])]
    for link in ("coxa", "femur", "tibia"):
        links[link] = [(a, b, m, turn(v, hip, [0, 0, 1], yaw))
                       for a, b, m, v in links[link]]
        turn_skins(link, hip, [0, 0, 1], yaw)
    knee = turn(knee, hip, [0, 0, 1], yaw)
    ankle = turn(ankle, hip, [0, 0, 1], yaw)
    tip = turn(tip, hip, [0, 0, 1], yaw)
    out = rot([0, 0, 1], yaw) @ out
    lift = np.cross(out, [0.0, 0.0, 1.0])

    # Knee: raise the femur and everything beyond it.
    for link in ("femur", "tibia"):
        links[link] = [(a, b, m, turn(v, knee, lift, FEMUR_ANGLE_UP))
                       for a, b, m, v in links[link]]
        turn_skins(link, knee, lift, FEMUR_ANGLE_UP)
    ankle = turn(ankle, knee, lift, FEMUR_ANGLE_UP)
    tip = turn(tip, knee, lift, FEMUR_ANGLE_UP)

    # Ankle: drop the lower leg to the standing angle.
    d = tip - ankle
    now = math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
    drop = -(TIBIA_ANGLE_DOWN + now)
    links["tibia"] = [(a, b, m, turn(v, ankle, lift, drop))
                      for a, b, m, v in links["tibia"]]
    turn_skins("tibia", ankle, lift, drop)
    tip = turn(tip, ankle, lift, drop)

    return {"pivot": {"hip": hip, "knee": knee, "ankle": ankle},
            "axis": {"hip": np.array([0.0, 0.0, 1.0]), "knee": lift,
                     "ankle": lift},
            "links": links, "skins": skins, "tip": tip, "out": out,
            "stand": {"hip": yaw, "knee": FEMUR_ANGLE_UP, "ankle": drop}}


def build_robot():
    cad = load_cad()
    legs = {k: pose_leg(v) for k, v in name_legs(cad).items()}
    ground = min(L["tip"][2] for L in legs.values()) - FOOT_RADIUS
    return cad, legs, ground


def leg_fan(L):
    """Degrees the leg points forward of straight out to its side."""
    side = 1.0 if L["out"][1] > 0 else -1.0
    return math.degrees(math.atan2(L["out"][0], side * L["out"][1]))


def total_mass(cad):
    return (sum(p["mass"] for p in cad["body"]) + PAYLOAD_MASS
            + sum(p["mass"] for lg in cad["legs"]
                  for ps in lg["links"].values() for p in ps))


# =====================================================================
# CHECKS
# =====================================================================

def leg_cloud(L, swing=0.0):
    """All solid vertices of a leg, optionally swung at the hip."""
    pts = np.vstack([v for link in ("coxa", "femur", "tibia")
                     for _a, part, _m, v in L["links"][link]
                     if part not in VISUAL_ONLY])
    if swing:
        pts = (pts - L["pivot"]["hip"]) @ rot([0, 0, 1], swing).T \
            + L["pivot"]["hip"]
    return pts


def gap(a, b, swing_a=0.0, swing_b=0.0):
    pa, pb = leg_cloud(a, swing_a), leg_cloud(b, swing_b)
    d = np.linalg.norm(pa[:, None, :] - pb[None, :, :], axis=2)
    return float(d.min())


def run_checks(cad, legs, ground):
    problems, notes = [], []
    mass = total_mass(cad)
    if abs(mass - 4.326) > 0.002:
        problems.append(f"parts add to {mass:.3f} kg, the sheet says 4.326")
    else:
        notes.append(f"parts add to {mass:.3f} kg, matching Ronan's sheet")

    for feet in (6, 4):
        per_foot = mass * GRAVITY / feet
        arm = max(math.hypot(*(L["tip"] - L["pivot"]["knee"])[:2])
                  for L in legs.values())
        torque = per_foot * arm
        line = (f"{feet} feet down: knee holds up to {torque:.2f} N.m, "
                f"{torque / SERVO_TORQUE:.0%} of the servo")
        (problems if torque / SERVO_TORQUE > 0.6 else notes).append(line)

    for side in ("l", "r"):
        for p in (1, 2, 3):
            a, b = legs[f"{side}{p}"], legs[f"{side}{p + 1}"]
            rest = gap(a, b)
            sign = 1.0 if side == "l" else -1.0
            allowed = 0.0
            s = 0.5
            while s <= 20.0 and gap(a, b, sign * s, -sign * s) >= MIN_LEG_GAP:
                allowed = s
                s += 0.5
            line = (f"{side}{p}-{side}{p + 1}: {rest * 1000:.0f} mm apart "
                    f"standing, {allowed:.1f} deg each towards each other")
            if rest < MIN_LEG_GAP:
                problems.append(line + " - touching while standing")
            else:
                notes.append(line + ("  << tight" if allowed < 3.0 else ""))
    low = min(min(v[2] for v in p["vertices"]) for p in cad["body"])
    clearance = low - ground
    line = f"belly {clearance * 1000:.0f} mm off the floor standing"
    (notes if clearance > 0.03 else problems).append(line)
    return problems, notes


# =====================================================================
# MODEL
# =====================================================================

def mesh_asset(name, verts, origin):
    local = np.asarray(verts) - origin
    return (f'    <mesh name="{name}" vertex="'
            + " ".join(n(x) for x in local.ravel()) + '"/>')


def part_geom(gname, mesh, part, mass, collide=True, hidden=False):
    attrs = (f'name="{gname}" type="mesh" mesh="{mesh}" '
             f'rgba="{COLOURS.get(part, "0.5 0.5 0.5 1")}"')
    attrs += f' mass="{n(mass)}"' if mass > 0 else ' density="0"'
    attrs += ' class="part"' if collide else ' contype="0" conaffinity="0"'
    # A part drawn by its skin keeps its collision shape in group 3, which
    # the viewer hides by default (tick 'Geom groups 3' to see it).
    attrs += ' group="3"' if hidden else ""
    return f"<geom {attrs}/>"


def skin_asset(name, verts, faces, tex, origin):
    local = np.asarray(verts) - origin
    return (f'    <mesh name="{name}" inertia="shell" vertex="'
            + " ".join(n(x) for x in local.ravel()) + '" face="'
            + " ".join(str(i) for f in faces for i in f) + '" texcoord="'
            + " ".join(n(x) for x in np.asarray(tex).ravel()) + '"/>')


def skin_geom(gname, mesh, key):
    return (f'<geom name="{gname}" type="mesh" mesh="{mesh}" '
            f'material="{skin_material(key)}" density="0" contype="0" '
            f'conaffinity="0" group="1"/>')


def skin_material(key):
    return "skin_" + key.replace(" ", "_").replace(".", "_")


def build_model():
    cad, legs, ground = build_robot()
    lift = -ground + FOOT_CLEARANCE
    assets, body = [], []

    import looks
    tex_files = looks.write_all(cad["skins"], os.path.join(HERE, "textures"))
    for key, fn in tex_files.items():
        mat = skin_material(key)
        assets.append(f'    <texture name="{mat}" type="2d" '
                      f'file="textures/{fn}"/>')
        assets.append(f'    <material name="{mat}" texture="{mat}" '
                      f'specular="0.25" shininess="0.3"/>')

    for p in cad["body"]:
        m = f"m_{p['name']}"
        assets.append(mesh_asset(m, p["vertices"], np.zeros(3)))
        body.append("      " + part_geom(p["name"], m, p["part"], p["mass"],
                                          p["part"] not in VISUAL_ONLY,
                                          hidden="skin" in p or p["part"]
                                          == "Battery LxT 18V v7"))
        if "skin" in p:
            sk = p["skin"]
            assets.append(skin_asset(f"s_{p['name']}", sk["vertices"],
                                     sk["faces"], sk["texcoord"], np.zeros(3)))
            body.append("      " + skin_geom(f"skin_{p['name']}",
                                             f"s_{p['name']}", sk["key"]))
    # Battery, payload case and the sled underneath: see details.py.
    import details
    tex_dir = os.path.join(HERE, "textures")
    for part in details.build(cad, tex_dir), details.case_and_sled(
            PAYLOAD_SIZE, PAYLOAD_CENTRE_X, tex_dir):
        assets.extend(part[0])
        body.extend("      " + g for g in part[1])
    pl, pw, ph = PAYLOAD_SIZE
    body.append(f'      <geom name="payload" type="box" size="{n(pl / 2)} '
                f'{n(pw / 2)} {n(ph / 2)}" pos="{n(PAYLOAD_CENTRE_X)} 0 '
                f'{n(ph / 2)}" mass="{n(PAYLOAD_MASS)}" '
                f'rgba="0.07 0.07 0.08 1" class="part" group="3"/>')
    body.append('      <site name="body_centre" pos="0 0 0" size="0.004" '
                'rgba="0 0 0 0"/>')

    def emit_leg(leg, L):
        lines = []
        piv, ax = L["pivot"], L["axis"]
        chain = [("hip", "coxa", np.zeros(3)),
                 ("knee", "femur", piv["hip"]),
                 ("ankle", "tibia", piv["knee"])]
        ranges = {"hip": HIP_RANGE, "knee": KNEE_RANGE, "ankle": ANKLE_RANGE}
        damping = {"hip": HIP_DAMPING, "knee": KNEE_DAMPING,
                   "ankle": ANKLE_DAMPING}
        armature = {"hip": 0.003, "knee": 0.003, "ankle": 0.002}
        indent = "      "
        for depth, (joint, link, parent) in enumerate(chain):
            origin = piv[joint]
            bname = leg if joint == "hip" else f"{leg}_{joint}"
            lines.append(f'{indent}<body name="{bname}" '
                         f'pos="{vec(origin - parent)}">')
            lines.append(f'{indent}  <joint name="{leg}_{joint}" '
                         f'type="hinge" axis="{vec(ax[joint])}" '
                         f'range="{n(ranges[joint][0])} {n(ranges[joint][1])}"'
                         f' damping="{n(damping[joint])}" '
                         f'armature="{armature[joint]}"/>')
            skinned = {nm for *_x, nm in L["skins"][link]}
            for k, (pname, part, mass, verts) in enumerate(L["links"][link]):
                m = f"m_{leg}_{link}_{k}"
                assets.append(mesh_asset(m, verts, origin))
                collide = part not in VISUAL_ONLY
                lines.append(f"{indent}  " + part_geom(
                    f"{leg}_{link}_{k}", m, part, mass, collide,
                    hidden=pname in skinned))
            for k, (key, verts, faces, tex, _nm) in enumerate(
                    L["skins"][link]):
                m = f"s_{leg}_{link}_{k}"
                assets.append(skin_asset(m, verts, faces, tex, origin))
                lines.append(f"{indent}  " + skin_geom(
                    f"{leg}_{link}_skin{k}", m, key))
            if link == "tibia":
                lines.append(f'{indent}  <geom name="{leg}_tip" '
                             f'type="sphere" size="{n(FOOT_RADIUS)}" '
                             f'pos="{vec(L["tip"] - origin)}" density="0" '
                             f'rgba="0.02 0.02 0.02 1" class="tip" '
                             f'group="3"/>')
            indent += "  "
        for _ in chain:
            indent = indent[:-2]
            lines.append(f"{indent}</body>")
        return lines

    for leg in LEG_NAMES:
        body.extend(emit_leg(leg, legs[leg]))

    xml = [
        '<?xml version="1.0" encoding="utf-8"?>',
        "<!-- Project Huntsman, built from V Source CAD (15 Oct 2026).",
        "     Generated by spider.py from cad/huntsman_cad.json. -->",
        '<mujoco model="huntsman">',
        '  <compiler angle="degree" autolimits="true"/>',
        '  <option timestep="0.002" solver="Newton" integrator="implicitfast"',
        '          iterations="100" ls_iterations="50"/>',
        '  <visual><global offwidth="1600" offheight="1000"/>',
        '    <headlight ambient="0.20 0.20 0.20" diffuse="0.30 0.30 0.30"',
        '               specular="0.05 0.05 0.05"/>',
        '    <quality shadowsize="8192"/></visual>',
        "  <default>",
        '    <default class="part">',
        '      <geom condim="3" solimp="0.90 0.96 0.001" solref="0.01 1"/>',
        "    </default>",
        '    <default class="tip">',
        '      <geom condim="4" friction="1.6 0.05 0.001"',
        '            solimp="0.95 0.99 0.001" solref="0.008 1"/>',
        "    </default>",
        "  </default>",
        "  <asset>",
        '    <texture name="sky" type="skybox" builtin="gradient"',
        '             rgb1="0.24 0.28 0.35" rgb2="0.10 0.12 0.16"',
        '             width="512" height="512"/>',
        '    <texture name="grid" type="2d" builtin="checker"',
        '             rgb1="0.26 0.28 0.32" rgb2="0.30 0.32 0.36"',
        '             width="512" height="512"/>',
        '    <material name="ground" texture="grid" texrepeat="16 16"',
        '              reflectance="0.05"/>',
    ] + assets + [
        "  </asset>",
        "  <worldbody>",
        '    <light name="key" pos="1 -1 3" dir="-0.3 0.3 -1"',
        '           directional="true" diffuse="0.58 0.58 0.58"',
        '           specular="0.15 0.15 0.15"/>',
        '    <light name="fill" pos="-2 2 3" dir="0.4 -0.4 -1"',
        '           directional="true" diffuse="0.16 0.16 0.16"',
        '           specular="0 0 0" castshadow="false"/>',
        '    <light name="under" pos="0 0 -2" dir="0.1 0.2 1"',
        '           directional="true" diffuse="0.18 0.18 0.18"',
        '           specular="0 0 0" castshadow="false"/>',
        '    <geom name="ground" type="plane" size="20 20 0.1"',
        '          material="ground" friction="1.5 0.05 0.001" condim="4"/>',
        f'    <body name="body" pos="0 0 {n(lift)}">',
        '      <freejoint name="root"/>',
    ] + body + ["    </body>", "  </worldbody>", "", "  <actuator>"]

    gains = {"hip": HIP_GAIN, "knee": KNEE_GAIN, "ankle": ANKLE_GAIN}
    ranges = {"hip": HIP_RANGE, "knee": KNEE_RANGE, "ankle": ANKLE_RANGE}
    for leg in LEG_NAMES:
        for j in JOINTS:
            # MuJoCo converts joint ranges to radians but NOT actuator
            # control ranges: servo targets are always radians.
            lo, hi = (math.radians(a) for a in ranges[j])
            xml.append(f'    <position name="{leg}_{j}" joint="{leg}_{j}" '
                       f'kp="{n(gains[j])}" ctrlrange="{n(lo)} {n(hi)}" '
                       f'forcerange="-{n(SERVO_TORQUE)} {n(SERVO_TORQUE)}"/>')
    xml += [
        "  </actuator>",
        "  <sensor>",
        '    <framepos name="body_pos" objtype="body" objname="body"/>',
        '    <framequat name="body_tilt" objtype="body" objname="body"/>',
        '    <velocimeter name="body_speed" site="body_centre"/>',
        '    <gyro name="body_turn" site="body_centre"/>',
        "  </sensor>",
        "</mujoco>",
    ]
    return "\n".join(xml) + "\n"


# Exposed for walk.py: fan angle of each pair, averaged left and right.
def _fans():
    try:
        _c, legs, _g = build_robot()
        return tuple(round((leg_fan(legs[f"l{p}"]) + leg_fan(legs[f"r{p}"]))
                           / 2.0, 1) for p in (1, 2, 3, 4))
    except SystemExit:
        return (22.7, 0.0, 0.0, -30.0)


LEG_FAN = _fans()


def report():
    cad, legs, ground = build_robot()
    problems, notes = run_checks(cad, legs, ground)
    L = legs["l2"]
    a = L["axis"]["knee"] / np.linalg.norm(L["axis"]["knee"])

    def across(d):
        """Length perpendicular to the joint axis (ignores horn offsets)."""
        return float(np.linalg.norm(d - (d @ a) * a))
    femur = across(L["pivot"]["ankle"] - L["pivot"]["knee"])
    tibia = across(L["tip"] - L["pivot"]["ankle"])

    print()
    print("Project Huntsman - built from V Source CAD (15 Oct 2026)")
    print("=" * 66)
    print(f"  source       {cad['source']}")
    print(f"  mass         {total_mass(cad):.3f} kg")
    print(f"  femur        {femur * 1000:.0f} mm knee to ankle")
    print(f"  lower leg    {tibia * 1000:.0f} mm ankle to tip")
    print(f"  standing     femur {FEMUR_ANGLE_UP:.0f} deg up, lower leg "
          f"{TIBIA_ANGLE_DOWN:.0f} deg down")
    print(f"  body plate   {(-ground) * 1000:.0f} mm off the floor")
    print()
    print("  leg   fan     hip position (x, y) mm")
    for leg in LEG_NAMES:
        h = legs[leg]["pivot"]["hip"]
        print(f"  {leg}  {leg_fan(legs[leg]):+6.1f}   "
              f"({h[0] * 1000:+5.0f}, {h[1] * 1000:+5.0f})")
    print()
    print("  Hardware zero (CAD, legs flat) -> simulation zero (standing)")
    print(f"    knee +{legs['l1']['stand']['knee']:.0f} deg, ankle "
          f"{legs['l1']['stand']['ankle']:+.0f} deg, hip 0 deg")
    print()
    print("  Not in the CAD pack, approximated: "
          + ", ".join(cad["parts_missing_from_pack"]))
    print()
    print("  Motor order the hardware must use:")
    for i, leg in enumerate(LEG_NAMES):
        print("    " + "  ".join(f"{i * 3 + j:2d} {leg}_{jn:<5}"
                                 for j, jn in enumerate(JOINTS)))
    print()
    print("Checks")
    for line in notes:
        print(f"  ok    {line}")
    for line in problems:
        print(f"  WRONG {line}")
    print()
    print("All checks pass." if not problems else f"{len(problems)} problem(s).")
    print("=" * 66)
    return not problems


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--save", action="store_true")
    args = ap.parse_args()
    if not report():
        return 1
    if args.check:
        return 0
    path = os.path.join(HERE, "spider.xml")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(build_model())
    print(f"\nwrote {os.path.basename(path)}")
    if args.save:
        return 0
    import mujoco
    import mujoco.viewer
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    print(f"loaded {model.nu} servos, {sum(model.body_mass):.3f} kg")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.0, 0.08]
        viewer.cam.distance, viewer.cam.elevation = 1.1, -24.0
        viewer.cam.azimuth = 135.0
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
    return 0


if __name__ == "__main__":
    sys.exit(main())
