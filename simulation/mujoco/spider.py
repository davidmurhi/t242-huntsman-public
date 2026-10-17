#!/usr/bin/env python3
"""
spider.py

Project Huntsman - the robot, calibrated to V Source's CAD.

Builds a MuJoCo model of the spider from the numbers at the top of this
file and opens the viewer. There is no gait here; walk.py drives it.

WHERE THE NUMBERS COME FROM
    Masses    Ronan Donahoe's part-weight sheet, sent with the CAD on
              15 Oct 2026 (Spider Part Weight.xlsx). Exact. 4.326 kg total.
    Lengths   Measured off the CAD top view, scaled against two parts of
              known size that agree with each other: the Makita 18V battery
              (76 mm wide) and the servo case (40 mm). About 3.64 px/mm.
              Estimates, good to roughly +/-5 mm, until Ronan exports a
              STEP file with exact geometry. See docs/cad-calibration.md.
    Servos    9imod DSC55MG, 55 kg.cm, read off the CAD model labels.

WHAT CHANGED FROM THE PLACEHOLDER MODEL (commit 7e1f102)
    mass           1.9 kg (as specified)   ->  4.326 kg (weighed)
    leg            femur + tibia + foot    ->  coxa + femur + tibia, as built
    hip            at the body flank       ->  inside the body, under the plate
    leg fan        56 / 19 deg             ->  29 / 0 deg
    torque limit   12 N.m                  ->  5.4 N.m (the real servo)
    leg contacts   switched off for testing ->  on, so collisions are real

CRAIG'S SPECIFICATION (unchanged)
    8 legs, 3 joints each: hip turns the leg forward and back, knee and
    ankle raise and lower it. 24 servos.

RUN
    macOS            mjpython spider.py
    Windows / Linux  python spider.py

    python3 spider.py --check     build report only, no viewer
    python3 spider.py --save      write spider.xml and exit

REQUIREMENTS
    pip install mujoco numpy
"""

from __future__ import annotations

import argparse
import math
import os
import sys

# =====================================================================
# THE ROBOT
# Every number V Source sends us gets changed here, and only here.
# Lengths in metres, masses in kilograms, torque in N.m, angles in deg.
# Body frame: x forward (battery end), y left, z up. Origin is the centre
# of the eight hip axes, on the top face of the plate.
# =====================================================================

# --- Part masses, from Ronan's sheet (15 Oct 2026) ---------------------
MASS_PLATE = 0.180
MASS_BATTERY = 0.671          # Makita LXT 18V 5.0Ah
MASS_PAYLOAD = 0.867          # Pi 5, ESP32, electronics enclosure
MASS_SERVO = 0.075            # DSC55MG with horn, x24
MASS_U_BRACKET = 0.022        # x16, two per leg
MASS_MP_BRACKET = 0.015       # x16, two per leg
MASS_ACRYLIC_LEG = 0.027      # x8, the lower leg

# --- Body: the base plate ---------------------------------------------
PLATE_LENGTH = 0.220          # front to back
PLATE_WIDTH = 0.213           # side to side
PLATE_THICKNESS = 0.005
PLATE_OFFSET_X = 0.005        # plate sits slightly forward of hip centre

# Battery overhangs the front of the plate, payload box sits behind centre.
BATTERY_SIZE = (0.115, 0.076, 0.064)    # L x W x H, Makita BL1850B
BATTERY_X = 0.160
PAYLOAD_SIZE = (0.120, 0.086, 0.060)
PAYLOAD_X = -0.042

# --- Where each pair's hip axis sits, (forward, sideways) -------------
# Pair 1 is the front pair. Measured from the CAD top view.
HIP_POSITIONS = ((0.088, 0.061), (0.031, 0.058),
                 (-0.032, 0.058), (-0.088, 0.061))

# --- Which way each pair points, degrees forward of straight out -------
LEG_FAN = (29.0, 0.0, 0.0, -29.0)

# --- Leg segments (CAD zero pose has them all horizontal) --------------
COXA_LENGTH = 0.044           # hip axis to knee axis
FEMUR_LENGTH = 0.110          # knee axis to ankle axis (U-bracket pair)
TIBIA_LENGTH = 0.152          # ankle axis to foot tip (acrylic leg)
FEMUR_WIDTH = 0.056           # U-bracket outside width, across the leg
FEMUR_DEPTH = 0.024
TIBIA_THICKNESS = 0.008
SERVO_CASE = (0.040, 0.020, 0.0405)     # standard-size case, L x W x H
FOOT_RADIUS = 0.006

# --- Standing pose ------------------------------------------------------
# The simulation's zero is the standing pose, so a policy outputting zero
# stands still. On the hardware the CAD flat pose is zero, so these two
# angles are the offsets the PWM mapping has to add. See the report.
FEMUR_ANGLE_UP = 20.0         # femur raised above horizontal
TIBIA_ANGLE_DOWN = 75.0       # tibia below horizontal

# --- Servos -------------------------------------------------------------
# DSC55MG, 55 kg.cm = 5.4 N.m. Rated stall torque; confirm at the 8 V rail
# Craig is moving to before trusting the margin below.
SERVO_TORQUE = 55.0 * 0.0980665
HIP_GAIN, HIP_DAMPING = 80.0, 1.60
KNEE_GAIN, KNEE_DAMPING = 80.0, 1.40
ANKLE_GAIN, ANKLE_DAMPING = 45.0, 0.70

HIP_RANGE = (-40.0, 40.0)
KNEE_RANGE = (-35.0, 45.0)    # positive raises the leg
ANKLE_RANGE = (-60.0, 60.0)   # positive raises the foot

# Leg-to-leg contacts are modelled. Earlier versions could switch them off
# for testing; a model that cannot detect its own collisions cannot tell us
# whether the real robot would foul itself.
MIN_LEG_GAP = 0.005           # smallest gap between neighbouring legs
FOOT_CLEARANCE = 0.004        # feet start this far above the floor

# =====================================================================
# WORKED OUT FROM THE ABOVE - no numbers to change past this point
# =====================================================================

LEG_NAMES = [f"{side}{pair}" for pair in (1, 2, 3, 4) for side in ("l", "r")]
JOINTS = ("hip", "knee", "ankle")
GRAVITY = 9.81


def n(value):
    return f"{value:.6g}"


def total_mass():
    return (MASS_PLATE + MASS_BATTERY + MASS_PAYLOAD
            + 24 * MASS_SERVO + 16 * MASS_U_BRACKET
            + 16 * MASS_MP_BRACKET + 8 * MASS_ACRYLIC_LEG)


def segment_masses():
    """How the weighed parts are split between the moving links."""
    body = MASS_PLATE + MASS_BATTERY + MASS_PAYLOAD + 8 * MASS_SERVO
    coxa = MASS_SERVO + MASS_MP_BRACKET            # knee servo + horn frame
    femur = 2 * MASS_U_BRACKET + MASS_MP_BRACKET + MASS_SERVO
    tibia = MASS_ACRYLIC_LEG
    return body, coxa, femur, tibia


def femur_reach():
    a = math.radians(FEMUR_ANGLE_UP)
    return FEMUR_LENGTH * math.cos(a), FEMUR_LENGTH * math.sin(a)


def tibia_reach():
    a = math.radians(TIBIA_ANGLE_DOWN)
    return TIBIA_LENGTH * math.cos(a), -TIBIA_LENGTH * math.sin(a)


def leg_reach():
    """Horizontal distance from hip axis to foot tip, standing."""
    return COXA_LENGTH + femur_reach()[0] + tibia_reach()[0]


def standing_height():
    """Height of the hip plane above the floor, standing."""
    return -(femur_reach()[1] + tibia_reach()[1]) + FOOT_RADIUS


def leg_heading(pair, side):
    """Direction the leg points, degrees anticlockwise from forward."""
    return side * (90.0 - LEG_FAN[pair])


def hip_point(pair, side):
    x, y = HIP_POSITIONS[pair]
    return x, side * y


# --- plan-view clearance between neighbouring legs -----------------------

def _seg_dist(p1, p2, q1, q2):
    """Shortest distance between two 2D segments."""
    def pt_seg(p, a, b):
        ax, ay = b[0] - a[0], b[1] - a[1]
        L = ax * ax + ay * ay
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax
                                                  + (p[1] - a[1]) * ay) / L))
        return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    d1, d2 = cross(q1, q2, p1), cross(q1, q2, p2)
    d3, d4 = cross(p1, p2, q1), cross(p1, p2, q2)
    if d1 * d2 < 0 and d3 * d4 < 0:
        return 0.0
    return min(pt_seg(p1, q1, q2), pt_seg(p2, q1, q2),
               pt_seg(q1, p1, p2), pt_seg(q2, p1, p2))


def _leg_parts(pair, side, swing):
    """Plan-view femur and tibia of one leg as (start, end, half-width)."""
    hx, hy = hip_point(pair, side)
    h = math.radians(leg_heading(pair, side) + swing)
    ux, uy = math.cos(h), math.sin(h)
    fo, _ = femur_reach()
    to, _ = tibia_reach()
    knee = (hx + ux * COXA_LENGTH, hy + uy * COXA_LENGTH)
    ankle = (knee[0] + ux * fo, knee[1] + uy * fo)
    tip = (ankle[0] + ux * to, ankle[1] + uy * to)
    return [(knee, ankle, FEMUR_WIDTH / 2), (ankle, tip, TIBIA_THICKNESS)]


def leg_gap(a, b, side, swing_a=0.0, swing_b=0.0):
    """Clearance between neighbouring legs a and b (pair indices)."""
    best = 1e9
    for p1, p2, ra in _leg_parts(a, side, swing_a):
        for q1, q2, rb in _leg_parts(b, side, swing_b):
            best = min(best, _seg_dist(p1, p2, q1, q2) - ra - rb)
    return best


def opposing_swing_allowed(a, b):
    """Largest swing two neighbours can make towards each other at once
    (each by this many degrees, in opposite directions) and still clear
    MIN_LEG_GAP. Pair a is in front of pair b."""
    side = 1.0
    best = 0.0
    step = 0.1
    s = step
    while s < 45.0:
        # leg a turns back, leg b turns forward: they close up
        gap = leg_gap(a, b, side, swing_a=s, swing_b=-s)
        if gap < MIN_LEG_GAP:
            break
        best = s
        s += step
    return best


# =====================================================================
# CHECKS - these run every time, so a broken robot cannot go unnoticed
# =====================================================================

def torque_budget(feet_down=6):
    """Static knee torque standing on feet_down legs, vs servo rating."""
    weight = total_mass() * GRAVITY
    per_leg = weight / feet_down
    fo, _ = femur_reach()
    to, _ = tibia_reach()
    knee_arm = fo + to
    ankle_arm = to
    return per_leg, per_leg * knee_arm, per_leg * ankle_arm


def run_checks():
    problems, notes = [], []

    mass = total_mass()
    if abs(mass - 4.326) > 0.001:
        problems.append(f"masses add to {mass:.3f} kg, the sheet says 4.326")
    else:
        notes.append(f"masses add to {mass:.3f} kg, matching Ronan's sheet")

    for feet in (6, 4):
        _force, knee, _ankle = torque_budget(feet)
        share = knee / SERVO_TORQUE
        line = (f"{feet} feet down: knee holds {knee:.2f} N.m, "
                f"{share:.0%} of the {SERVO_TORQUE:.1f} N.m servo")
        if share > 0.6:
            problems.append(line + " - too little margin to walk")
        else:
            notes.append(line)

    for a in range(3):
        rest = leg_gap(a, a + 1, 1.0)
        allowed = opposing_swing_allowed(a, a + 1)
        line = (f"legs {a + 1} and {a + 2}: {rest * 1000:.0f} mm apart "
                f"standing; can swing {allowed:.1f} deg each towards "
                f"each other")
        if rest < MIN_LEG_GAP:
            problems.append(line + " - touching while standing")
        elif allowed < 4.0:
            notes.append(line + "  << must move in step")
        else:
            notes.append(line)

    # Support polygon: is the centre of mass inside the feet?
    body, coxa, femur, tibia = segment_masses()
    com_x = (MASS_BATTERY * BATTERY_X + MASS_PAYLOAD * PAYLOAD_X
             + MASS_PLATE * PLATE_OFFSET_X) / mass
    feet_x = []
    for pair in range(4):
        hx, _ = HIP_POSITIONS[pair]
        h = math.radians(90.0 - LEG_FAN[pair])
        feet_x.append(hx + math.cos(h) * leg_reach())
    margin = min(max(feet_x) - com_x, com_x - min(feet_x))
    notes.append(f"centre of mass {com_x * 1000:+.0f} mm forward of hip "
                 f"centre (battery forward, payload aft), "
                 f"{margin * 1000:.0f} mm inside the feet")

    underside = PLATE_THICKNESS + SERVO_CASE[2]
    clearance = standing_height() - underside
    if clearance < 0.04:
        problems.append(f"belly only {clearance * 1000:.0f} mm off the floor")
    else:
        notes.append(f"belly (hip servos) {clearance * 1000:.0f} mm off the "
                     f"floor standing")
    return problems, notes


# =====================================================================
# BUILDING THE MODEL
# =====================================================================

def box(name, size, pos, material, mass=None, collide=True, euler=None):
    """size is full lengths; MuJoCo wants half lengths."""
    attrs = [f'name="{name}" type="box"',
             f'size="{n(size[0] / 2)} {n(size[1] / 2)} {n(size[2] / 2)}"',
             f'pos="{n(pos[0])} {n(pos[1])} {n(pos[2])}"',
             f'material="{material}"']
    if euler:
        attrs.append(f'euler="{euler}"')
    if mass is None:
        attrs.append('density="0"')
    else:
        attrs.append(f'mass="{n(mass)}"')
    if collide:
        attrs.append('class="limb"')
    else:
        attrs.append('contype="0" conaffinity="0"')
    return "<geom " + " ".join(attrs) + "/>"


def build_leg(leg, pair):
    side = 1.0 if leg.startswith("l") else -1.0
    hx, hy = hip_point(pair, side)
    heading = leg_heading(pair, side)
    _body, m_coxa, m_femur, m_tibia = segment_masses()

    fo, fu = femur_reach()
    to, td = tibia_reach()
    femur_mid = (fo / 2, fu / 2)
    sx, sy, sz = SERVO_CASE

    lines = [
        # Leg frame: local x points along the leg, local z up.
        f'      <body name="{leg}" pos="{n(hx)} {n(hy)} 0" '
        f'euler="0 0 {n(heading)}">',
        # Joint 1: hip, turns the leg forward and back.
        f'        <joint name="{leg}_hip" type="hinge" axis="0 0 1" '
        f'range="{n(HIP_RANGE[0])} {n(HIP_RANGE[1])}" '
        f'damping="{n(HIP_DAMPING)}" armature="0.003"/>',
        # Coxa: the knee servo and its horn frame, hip to knee.
        "        " + box(f"{leg}_coxa", (COXA_LENGTH + sx / 2, sz, sy),
                         ((COXA_LENGTH + sx / 2) / 2 - sx / 4, 0, 0),
                         "servo", mass=m_coxa),

        f'        <body name="{leg}_knee" pos="{n(COXA_LENGTH)} 0 0">',
        # Joint 2: knee, raises and lowers the leg.
        f'          <joint name="{leg}_knee" type="hinge" axis="0 -1 0" '
        f'range="{n(KNEE_RANGE[0])} {n(KNEE_RANGE[1])}" '
        f'damping="{n(KNEE_DAMPING)}" armature="0.003"/>',
        # Femur: the U-bracket pair, carrying the ankle servo at its end.
        "          " + box(f"{leg}_femur",
                           (FEMUR_LENGTH, FEMUR_WIDTH, FEMUR_DEPTH),
                           (femur_mid[0], 0, femur_mid[1]), "alloy",
                           mass=2 * MASS_U_BRACKET + MASS_MP_BRACKET,
                           euler=f"0 {n(-FEMUR_ANGLE_UP)} 0"),
        "          " + box(f"{leg}_ankle_servo", (sx, sz, sy),
                           (fo - sx / 4, 0, fu), "servo",
                           mass=MASS_SERVO, collide=False,
                           euler=f"0 {n(-FEMUR_ANGLE_UP)} 0"),

        f'          <body name="{leg}_ankle" pos="{n(fo)} 0 {n(fu)}">',
        # Joint 3: ankle, raises and lowers the foot.
        f'            <joint name="{leg}_ankle" type="hinge" axis="0 -1 0" '
        f'range="{n(ANKLE_RANGE[0])} {n(ANKLE_RANGE[1])}" '
        f'damping="{n(ANKLE_DAMPING)}" armature="0.002"/>',
        # Tibia: the acrylic leg.
        f'            <geom name="{leg}_tibia" type="capsule" '
        f'fromto="0 0 0 {n(to)} 0 {n(td)}" size="{n(TIBIA_THICKNESS)}" '
        f'mass="{n(m_tibia)}" material="acrylic" class="limb"/>',
        f'            <geom name="{leg}_tip" type="sphere" '
        f'pos="{n(to)} 0 {n(td)}" size="{n(FOOT_RADIUS)}" '
        f'density="0" material="rubber" class="tip"/>',
        "          </body>",
        "        </body>",
        "      </body>",
    ]
    del m_coxa, m_femur
    return lines


def build_model():
    body_m, _c, _f, _t = segment_masses()
    sx, sy, sz = SERVO_CASE
    bl, bw, bh = BATTERY_SIZE
    pl, pw, ph = PAYLOAD_SIZE

    lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        "<!-- Project Huntsman, calibrated to V Source CAD (15 Oct 2026).",
        "     Generated by spider.py. Change the numbers there, not here. -->",
        '<mujoco model="huntsman_cad">',
        "",
        '  <compiler angle="degree" autolimits="true"/>',
        '  <option timestep="0.002" solver="Newton" integrator="implicitfast"',
        '          iterations="100" ls_iterations="50"/>',
        "",
        "  <default>",
        '    <default class="limb">',
        '      <geom condim="3" solimp="0.90 0.96 0.001" solref="0.01 1"/>',
        "    </default>",
        '    <default class="tip">',
        '      <geom condim="4" friction="2.0 0.05 0.001"',
        '            solimp="0.95 0.99 0.001" solref="0.008 1"/>',
        "    </default>",
        "  </default>",
        "",
        "  <asset>",
        '    <texture name="grid" type="2d" builtin="checker"',
        '             rgb1="0.24 0.24 0.27" rgb2="0.30 0.30 0.33"',
        '             width="512" height="512"/>',
        '    <material name="ground" texture="grid" texrepeat="14 14"/>',
        '    <material name="plate" rgba="0.55 0.56 0.58 1" specular="0.4"/>',
        '    <material name="servo" rgba="0.16 0.16 0.17 1" specular="0.3"/>',
        '    <material name="alloy" rgba="0.22 0.22 0.20 1" specular="0.6"/>',
        '    <material name="acrylic" rgba="0.05 0.05 0.06 1"'
        ' specular="0.8" shininess="0.8"/>',
        '    <material name="rubber" rgba="0.03 0.03 0.03 1"/>',
        '    <material name="battery" rgba="0.12 0.45 0.55 1"/>',
        '    <material name="battery_top" rgba="0.08 0.08 0.09 1"/>',
        '    <material name="payload" rgba="0.10 0.10 0.11 1"/>',
        "  </asset>",
        "",
        "  <worldbody>",
        '    <light name="key" pos="1 -1 3" dir="-0.3 0.3 -1"',
        '           directional="true" diffuse="0.7 0.7 0.7"/>',
        '    <light name="fill" pos="-2 2 3" dir="0.4 -0.4 -1"',
        '           directional="true" diffuse="0.25 0.25 0.25"',
        '           castshadow="false"/>',
        '    <geom name="ground" type="plane" size="20 20 0.1"',
        '          material="ground" friction="1.5 0.05 0.001" condim="4"/>',
        "",
        f'    <body name="body" pos="0 0 '
        f'{n(standing_height() + FOOT_CLEARANCE)}">',
        '      <freejoint name="root"/>',
        # Every geom with mass below is a real part from Ronan's sheet.
        "      " + box("body_plate",
                       (PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS),
                       (PLATE_OFFSET_X, 0, -PLATE_THICKNESS / 2), "plate",
                       mass=MASS_PLATE),
        "      " + box("battery", (bl, bw, bh),
                       (BATTERY_X, 0, bh / 2), "battery",
                       mass=MASS_BATTERY),
        "      " + box("battery_cap", (bl * 0.55, bw * 1.01, bh * 0.35),
                       (BATTERY_X - bl * 0.18, 0, bh * 0.9), "battery_top",
                       collide=False),
        "      " + box("payload", (pl, pw, ph),
                       (PAYLOAD_X, 0, ph / 2), "payload",
                       mass=MASS_PAYLOAD),
    ]

    # Hip servos hang under the plate, one per leg, turning about z.
    for pair in range(4):
        for side, letter in ((1.0, "l"), (-1.0, "r")):
            hx, hy = hip_point(pair, side)
            heading = leg_heading(pair, side)
            lines.append("      " + box(
                f"{letter}{pair + 1}_hip_servo", (sx, sy, sz),
                (hx - math.cos(math.radians(heading)) * sx * 0.25,
                 hy - math.sin(math.radians(heading)) * sx * 0.25,
                 -PLATE_THICKNESS - sz / 2),
                "servo", mass=MASS_SERVO,
                euler=f"0 0 {n(heading)}"))

    lines += [
        '      <site name="body_centre" pos="0 0 0" size="0.004" '
        'rgba="0 0 0 0"/>',
    ]

    for leg in LEG_NAMES:
        lines.extend(build_leg(leg, int(leg[1]) - 1))

    lines += ["    </body>", "  </worldbody>", "", "  <actuator>"]

    servo = {
        "hip": (HIP_RANGE, HIP_GAIN),
        "knee": (KNEE_RANGE, KNEE_GAIN),
        "ankle": (ANKLE_RANGE, ANKLE_GAIN),
    }
    for leg in LEG_NAMES:
        for joint in JOINTS:
            rng, gain = servo[joint]
            lines.append(
                f'    <position name="{leg}_{joint}" joint="{leg}_{joint}" '
                f'kp="{n(gain)}" ctrlrange="{n(rng[0])} {n(rng[1])}" '
                f'forcerange="-{n(SERVO_TORQUE)} {n(SERVO_TORQUE)}"/>')

    lines += [
        "  </actuator>",
        "",
        "  <sensor>",
        '    <framepos name="body_pos" objtype="body" objname="body"/>',
        '    <framequat name="body_tilt" objtype="body" objname="body"/>',
        '    <velocimeter name="body_speed" site="body_centre"/>',
        '    <gyro name="body_turn" site="body_centre"/>',
        "  </sensor>",
        "",
        "</mujoco>",
    ]
    return "\n".join(lines) + "\n"


# =====================================================================
# REPORT
# =====================================================================

def report():
    problems, notes = run_checks()
    body, coxa, femur, tibia = segment_masses()
    fo, fu = femur_reach()
    to, td = tibia_reach()

    print()
    print("Project Huntsman - calibrated to V Source CAD (15 Oct 2026)")
    print("=" * 64)
    print("Mass, from Ronan's part-weight sheet")
    print(f"  body         {body:.3f} kg  (plate, battery, payload, "
          f"8 hip servos)")
    print(f"  each leg     {coxa + femur + tibia:.3f} kg  "
          f"(coxa {coxa:.3f}, femur {femur:.3f}, tibia {tibia:.3f})")
    print(f"  total        {total_mass():.3f} kg")
    print()
    print("Leg, measured from the CAD top view (+/- ~5 mm)")
    print(f"  coxa         {COXA_LENGTH * 1000:.0f} mm  hip to knee, level")
    print(f"  femur        {FEMUR_LENGTH * 1000:.0f} mm  at "
          f"{FEMUR_ANGLE_UP:.0f} deg up   -> out {fo * 1000:.0f}, "
          f"up {fu * 1000:+.0f}")
    print(f"  tibia        {TIBIA_LENGTH * 1000:.0f} mm  at "
          f"{TIBIA_ANGLE_DOWN:.0f} deg down -> out {to * 1000:.0f}, "
          f"down {td * 1000:+.0f}")
    print(f"  reach        {leg_reach() * 1000:.0f} mm from hip to foot")
    print(f"  hips stand   {standing_height() * 1000:.0f} mm off the floor")
    print()
    print("Servos: 9imod DSC55MG, 55 kg.cm = "
          f"{SERVO_TORQUE:.2f} N.m stall (confirm at 8 V)")
    for feet in (6, 4):
        force, knee, ankle = torque_budget(feet)
        print(f"  on {feet} feet  {force:.1f} N per foot, knee "
              f"{knee:.2f} N.m, ankle {ankle:.2f} N.m")
    print()
    print("Hardware zero offsets (CAD flat pose -> simulation standing pose)")
    print(f"  knee   +{FEMUR_ANGLE_UP:.0f} deg     ankle "
          f"-{FEMUR_ANGLE_UP + TIBIA_ANGLE_DOWN:.0f} deg     hip 0 deg")
    print()
    print("Motor order - the hardware must send commands in this order")
    for i, leg in enumerate(LEG_NAMES):
        print("  " + "  ".join(f"{i * 3 + j:2d} {leg}_{jn:<5}"
                               for j, jn in enumerate(JOINTS)))
    print()
    print("Checks")
    for note in notes:
        print(f"  ok    {note}")
    for problem in problems:
        print(f"  WRONG {problem}")
    print()
    print(f"{len(problems)} problem(s)." if problems else
          "The model matches the CAD and the checks pass.")
    print("=" * 64)
    return not problems


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--save", action="store_true")
    args = parser.parse_args()

    ok = report()
    if not ok:
        return 1
    if args.check:
        return 0

    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "spider.xml")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(build_model())
    print(f"\nwrote {os.path.basename(path)}")
    if args.save:
        return 0

    try:
        import mujoco
        import mujoco.viewer
    except ImportError:
        print("\nmujoco is not installed.  pip install mujoco numpy")
        return 1

    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    print(f"loaded  {model.nu} motors, {sum(model.body_mass):.3f} kg")
    for _ in range(800):
        mujoco.mj_step(model, data)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")
    print(f"settled at {data.xpos[body][2]:.4f} m "
          f"(expected about {standing_height():.4f})")
    mujoco.mj_resetData(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.0, standing_height() * 0.6]
        viewer.cam.distance = 1.0
        viewer.cam.elevation = -22.0
        viewer.cam.azimuth = 135.0
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
    return 0


if __name__ == "__main__":
    sys.exit(main())
