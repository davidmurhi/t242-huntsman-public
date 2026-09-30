#!/usr/bin/env python3
"""
spider.py

Project Huntsman - the spider body, and nothing else.

Builds the robot to Craig Schneider's specification and opens the MuJoCo
viewer so you can look at it. There is no gait, no obstacle course and no
training environment here on purpose. The body has to be right before
anything is built on top of it.

CRAIG'S SPECIFICATION
    A square block body.
    8 legs.
    3 joints per leg:
        joint 1  horizontal  - swings the leg forward and back
        joint 2  vertical    - raises and lowers the leg
        joint 3  vertical    - raises and lowers the foot
    24 motors total.

RUN
    macOS            mjpython spider.py
    Windows / Linux  python spider.py

    Drag to rotate. Scroll to zoom. Press Escape to close.

    python spider.py --check     print the build report, no viewer
    python spider.py --save      write spider.xml and exit

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
# Every dimension Craig sends through gets changed here, and only here.
# Lengths in metres, mass in kilograms, torque in newton-metres,
# angles in degrees.
# =====================================================================

# --- Body: the square block at the centre ----------------------------
BODY_LENGTH = 0.22
BODY_WIDTH = 0.14
BODY_HEIGHT = 0.04
BODY_MASS = 0.90

# --- Leg segment 1: body to first joint (the "femur") ----------------
# Angles upward and outward from the body, as an arachnid leg does.
FEMUR_LENGTH = 0.12
FEMUR_ANGLE_UP = 30.0
FEMUR_THICKNESS = 0.007
FEMUR_MASS = 0.035

# --- Leg segment 2: first joint to second joint (the "tibia") --------
# Angles downward and outward, carrying the leg back to the ground.
TIBIA_LENGTH = 0.15
TIBIA_ANGLE_DOWN = 60.0
TIBIA_THICKNESS = 0.005
TIBIA_MASS = 0.028

# --- Leg segment 3: second joint to the foot tip ---------------------
# Hangs straight down. This is what touches the ground.
FOOT_LENGTH = 0.05
FOOT_THICKNESS = 0.004
FOOT_MASS = 0.010

# --- How far each pair of legs fans forward (+) or backward (-) ------
# Front pairs point forward, rear pairs point backward, so the legs
# spread around the body instead of all sticking out sideways.
# Test setting. When true, neighbouring legs pass through each other
# instead of colliding. Use it to find out whether the walk works once
# the legs stop shoving one another; it does not fix anything.
EXCLUDE_NEIGHBOURING_LEGS = True

# How far out from the body's side each leg's joint sits.
#
# The joint sits this far out from the body, measured ALONG the direction
# the leg points rather than straight out sideways.
#
# That distinction matters. A fanned leg leaves the body at an angle, so a
# sideways offset is not covered by the segment's own rounded end and
# leaves a visible gap at the shoulder. Offsetting along the leg's own
# direction means the rounded end covers it exactly, and the leg reads as
# joined on.
#
# Keep it at or below the segment thickness, or a gap opens up again.
LEG_STANDOFF = 0.006

# How far each leg may swing before it would reach into its neighbour's
# space. Every pair gets its own wedge, reaching halfway to the leg in
# front and halfway to the one behind, less this margin. The hips are
# given those limits in the model, so no gait can drive them into each
# other however it is written.
NEIGHBOUR_MARGIN = 6.0

LEG_FAN = (56.0, 19.0, -19.0, -56.0)

# --- Joint travel limits ---------------------------------------------
HIP_RANGE = (-45.0, 45.0)      # horizontal joint
KNEE_RANGE = (-50.0, 50.0)     # first vertical joint
ANKLE_RANGE = (-50.0, 50.0)    # second vertical joint

# --- Servos -----------------------------------------------------------
# These were originally far too weak and far too lightly damped. Measuring
# showed the hip saturating 99% of the time with 50 degrees of tracking
# error and peak speeds eleven times what the gait asked for. The legs were
# not being moved, they were being flung, and that is what drove them into
# each other. Higher torque stops the saturation, much higher damping stops
# the oscillation, and armature adds rotor inertia so the solver stays
# steady.
HIP_TORQUE, HIP_GAIN, HIP_DAMPING = 8.0, 60.0, 1.20
KNEE_TORQUE, KNEE_GAIN, KNEE_DAMPING = 8.0, 60.0, 1.00
ANKLE_TORQUE, ANKLE_GAIN, ANKLE_DAMPING = 4.0, 35.0, 0.50

# --- How far the hip will swing once a gait drives it -----------------
# Only used to check that neighbouring legs cannot collide.
PLANNED_HIP_SWING = 14.0

FOOT_CLEARANCE = 0.005         # feet start this far above the floor

# =====================================================================
# WORKED OUT FROM THE ABOVE - no numbers to change past this point
# =====================================================================

LEG_NAMES = [f"{side}{pair}" for pair in (1, 2, 3, 4) for side in ("l", "r")]
JOINTS = ("hip", "knee", "ankle")


def femur_reach():
    """How far out and how far up the femur goes."""
    a = math.radians(FEMUR_ANGLE_UP)
    return FEMUR_LENGTH * math.cos(a), FEMUR_LENGTH * math.sin(a)


def tibia_reach():
    """How far out and how far down the tibia goes."""
    a = math.radians(TIBIA_ANGLE_DOWN)
    return TIBIA_LENGTH * math.cos(a), -TIBIA_LENGTH * math.sin(a)


def hip_range_for(pair):
    """The wedge this pair of legs is allowed to swing within.

    Halfway to the neighbour in front and halfway to the one behind, less
    a margin, expressed relative to the leg's own fan angle. The outermost
    pairs have no neighbour on one side, so they get a fixed allowance
    there instead.
    """
    fan = LEG_FAN[pair]
    if pair > 0:
        towards_front = (LEG_FAN[pair - 1] + fan) / 2.0
    else:
        towards_front = fan + 22.0
    if pair < len(LEG_FAN) - 1:
        towards_back = (fan + LEG_FAN[pair + 1]) / 2.0
    else:
        towards_back = fan - 22.0
    return (towards_back + NEIGHBOUR_MARGIN - fan,
            towards_front - NEIGHBOUR_MARGIN - fan)


def standing_height():
    """How high the body sits when the robot is standing."""
    _, up = femur_reach()
    _, down = tibia_reach()
    return -(up + down - FOOT_LENGTH)


def leg_reach():
    """How far the foot sits from the body, horizontally."""
    return femur_reach()[0] + tibia_reach()[0]


def leg_mount_positions():
    """Where along the body each pair of legs attaches, front to back."""
    usable = BODY_LENGTH * 0.75
    spacing = usable / 3.0
    return [(1.5 - i) * spacing for i in range(4)]


def density_for(mass, thickness, length):
    """Density that gives a capsule the mass we asked for."""
    volume = (math.pi * thickness ** 2 * length
              + 4.0 / 3.0 * math.pi * thickness ** 3)
    return mass / volume


def n(value):
    return f"{value:.6g}"


# =====================================================================
# CHECKS - these run every time, so a broken robot cannot go unnoticed
# =====================================================================

def run_checks():
    """Returns (problems, notes). Problems mean the robot is wrong."""
    problems, notes = [], []

    # Can neighbouring legs hit each other once the hips start swinging?
    arcs = [(i + 1, fan - PLANNED_HIP_SWING, fan + PLANNED_HIP_SWING)
            for i, fan in enumerate(LEG_FAN)]
    for (a, a_lo, a_hi), (b, b_lo, b_hi) in zip(arcs, arcs[1:]):
        gap = max(a_lo, b_lo) - min(a_hi, b_hi)
        if gap < 0:
            problems.append(
                f"legs {a} and {b} overlap by {-gap:.0f} degrees and will "
                f"collide - spread LEG_FAN wider or lower PLANNED_HIP_SWING")
        else:
            notes.append(f"legs {a} and {b} clear each other by {gap:.0f} deg")

    # Is it wide enough not to tip over?
    half_span = BODY_WIDTH / 2 + leg_reach()
    stability = half_span / standing_height()
    if stability < 1.5:
        problems.append(
            f"stance is narrow for its height (ratio {stability:.2f}, want "
            f"1.5 or more) - it will tip easily")
    else:
        notes.append(f"stance ratio {stability:.2f}, stable")

    # Are the legs mounted far enough apart not to look tangled?
    # Each leg reaches a long way out but they attach close together.
    # If the reach is many times the spacing, the legs overlap visually
    # from most angles even when they never actually touch.
    spacing = abs(leg_mount_positions()[0] - leg_mount_positions()[1])
    crowding = leg_reach() / spacing
    if crowding > 3.5:
        problems.append(
            f"legs are crowded (each reaches {crowding:.1f}x the gap between "
            f"mounts) - they will look tangled. lengthen BODY_LENGTH or "
            f"shorten TIBIA_LENGTH")
    else:
        notes.append(f"leg reach is {crowding:.1f}x the mount spacing, "
                     f"not crowded")

    # Do the legs actually clear the body, right through their swing?
    # A leg mounted flat on the surface has its first segment half buried
    # in the body, and the front and rear pairs then scrape along the
    # side as they swing.
    showing = LEG_STANDOFF - FEMUR_THICKNESS
    if showing > 0.001:
        problems.append(
            f"a {showing * 1000:.0f} mm gap shows at the shoulder - "
            f"lower LEG_STANDOFF to at most the femur thickness "
            f"({FEMUR_THICKNESS * 1000:.0f} mm)")
    else:
        notes.append("legs join the body with no gap showing")

    # Does it look like a spider, or like a box with stubs?
    span = 2 * half_span
    proportion = span / BODY_LENGTH
    if proportion < 2.0:
        problems.append(
            f"legs are short for the body (span is {proportion:.1f}x body "
            f"length, real spiders are 3-4x)")
    else:
        notes.append(f"leg span {proportion:.1f}x body length")

    # Do the legs reach the ground at all?
    if standing_height() <= 0:
        problems.append(
            "legs do not reach below the body - check FEMUR_ANGLE_UP and "
            "TIBIA_ANGLE_DOWN")

    return problems, notes


# =====================================================================
# BUILDING THE MODEL
# =====================================================================

def build_leg(leg, pair, mounts):
    """One leg: three segments, three joints, foot on the end."""
    side = 1.0 if leg.startswith("l") else -1.0

    # The whole leg is rotated so the pair fans forward or backward.
    # Positive means forward for both sides, so the sign flips per side.
    fan = -side * LEG_FAN[pair]

    hip_low, hip_high = hip_range_for(pair)

    # Step the joint out along the way the leg points, not straight out
    # sideways, so the segment's rounded end covers the gap.
    lean = math.radians(LEG_FAN[pair])
    mount_x = mounts[pair] + LEG_STANDOFF * math.sin(lean)
    mount_y = side * (BODY_WIDTH / 2 + LEG_STANDOFF * math.cos(lean))
    femur_out, femur_up = femur_reach()
    tibia_out, tibia_down = tibia_reach()

    return [
        f'      <body name="{leg}" pos="{n(mount_x)} {n(mount_y)} 0" '
        f'euler="0 0 {n(fan)}">',

        # Joint 1: horizontal. Swings the leg forward and back.
        f'        <joint name="{leg}_hip" type="hinge" axis="0 0 1" '
        f'range="{n(hip_low)} {n(hip_high)}" '
        f'damping="{n(HIP_DAMPING)}" armature="0.003"/>',

        f'        <geom name="{leg}_femur" type="capsule" '
        f'fromto="0 0 0 0 {n(side * femur_out)} {n(femur_up)}" '
        f'size="{n(FEMUR_THICKNESS)}" '
        f'density="{n(density_for(FEMUR_MASS, FEMUR_THICKNESS, FEMUR_LENGTH))}" '
        f'rgba="0.30 0.32 0.38 1" class="limb"/>',

        f'        <body name="{leg}_knee" pos="0 {n(side * femur_out)} '
        f'{n(femur_up)}">',

        # Joint 2: vertical. Raises and lowers the leg.
        f'          <joint name="{leg}_knee" type="hinge" axis="1 0 0" '
        f'range="{n(KNEE_RANGE[0])} {n(KNEE_RANGE[1])}" '
        f'damping="{n(KNEE_DAMPING)}" armature="0.003"/>',

        f'          <geom name="{leg}_tibia" type="capsule" '
        f'fromto="0 0 0 0 {n(side * tibia_out)} {n(tibia_down)}" '
        f'size="{n(TIBIA_THICKNESS)}" '
        f'density="{n(density_for(TIBIA_MASS, TIBIA_THICKNESS, TIBIA_LENGTH))}" '
        f'rgba="0.22 0.24 0.28 1" class="limb"/>',

        f'          <body name="{leg}_ankle" pos="0 {n(side * tibia_out)} '
        f'{n(tibia_down)}">',

        # Joint 3: vertical. Raises and lowers the foot.
        f'            <joint name="{leg}_ankle" type="hinge" axis="1 0 0" '
        f'range="{n(ANKLE_RANGE[0])} {n(ANKLE_RANGE[1])}" '
        f'damping="{n(ANKLE_DAMPING)}" armature="0.002"/>',

        f'            <geom name="{leg}_foot" type="capsule" '
        f'fromto="0 0 0 0 0 {n(-FOOT_LENGTH)}" size="{n(FOOT_THICKNESS)}" '
        f'density="{n(density_for(FOOT_MASS, FOOT_THICKNESS, FOOT_LENGTH))}" '
        f'rgba="0.18 0.20 0.24 1" class="limb"/>',

        # The tip is the only part meant to touch the ground.
        f'            <geom name="{leg}_tip" type="sphere" '
        f'pos="0 0 {n(-FOOT_LENGTH)}" size="{n(FOOT_THICKNESS * 1.6)}" '
        f'density="{n(density_for(FOOT_MASS, FOOT_THICKNESS, FOOT_LENGTH))}" '
        f'rgba="0.05 0.05 0.06 1" class="tip"/>',

        "          </body>",
        "        </body>",
        "      </body>",
    ]


def build_model():
    mounts = leg_mount_positions()
    body_density = BODY_MASS / (BODY_LENGTH * BODY_WIDTH * BODY_HEIGHT)

    lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        "<!-- Project Huntsman spider body. Generated by spider.py.",
        "     Change the numbers in spider.py, not this file. -->",
        '<mujoco model="huntsman_spider">',
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
        '    <material name="ground" texture="grid" texrepeat="14 14"',
        '              reflectance="0" shininess="0" specular="0"/>',
        "  </asset>",
        "",
        "  <worldbody>",
        '    <light name="key" pos="1 -1 3" dir="-0.3 0.3 -1"',
        '           directional="true" diffuse="0.7 0.7 0.7"/>',
        '    <light name="fill" pos="-2 2 3" dir="0.4 -0.4 -1"',
        '           directional="true" diffuse="0.25 0.25 0.25"',
        '           castshadow="false"/>',
        "",
        "    <!-- Flat ground. Nothing else in the scene by design. -->",
        '    <geom name="ground" type="plane" size="20 20 0.1"',
        '          material="ground" friction="1.5 0.05 0.001" condim="4"/>',
        "",
        f'    <body name="body" pos="0 0 '
        f'{n(standing_height() + FOOT_CLEARANCE)}">',
        '      <freejoint name="root"/>',
        f'      <geom name="body_block" type="box" '
        f'size="{n(BODY_LENGTH / 2)} {n(BODY_WIDTH / 2)} {n(BODY_HEIGHT / 2)}" '
        f'density="{n(body_density)}" rgba="0.80 0.34 0.12 1" class="limb"/>',
        f'      <geom name="front_marker" type="sphere" '
        f'pos="{n(BODY_LENGTH / 2)} 0 {n(BODY_HEIGHT / 2)}" size="0.014" '
        f'rgba="0.95 0.85 0.20 1" contype="0" conaffinity="0"/>',
        '      <site name="body_centre" pos="0 0 0" size="0.004" '
        'rgba="0 0 0 0"/>',
    ]

    for pair in range(4):
        for side in ("l", "r"):
            lines.extend(build_leg(f"{side}{pair + 1}", pair, mounts))

    lines += ["    </body>", "  </worldbody>", ""]

    if EXCLUDE_NEIGHBOURING_LEGS:
        lines += [
            "  <!-- Neighbouring legs are told to ignore each other.",
            "       This is a test setting, not a fix. It shows whether the",
            "       walk underneath is sound once the legs stop shoving one",
            "       another. Turn it off again before believing any result",
            "       about whether the real robot would foul itself. -->",
            "  <contact>",
        ]
        for side in ("l", "r"):
            for pair in range(1, 4):
                lines.append(f'    <exclude body1="{side}{pair}" '
                             f'body2="{side}{pair + 1}"/>')
                lines.append(f'    <exclude body1="{side}{pair}_knee" '
                             f'body2="{side}{pair + 1}"/>')
                lines.append(f'    <exclude body1="{side}{pair}" '
                             f'body2="{side}{pair + 1}_knee"/>')
                lines.append(f'    <exclude body1="{side}{pair}_knee" '
                             f'body2="{side}{pair + 1}_knee"/>')
                lines.append(f'    <exclude body1="{side}{pair}_ankle" '
                             f'body2="{side}{pair + 1}_ankle"/>')
                lines.append(f'    <exclude body1="{side}{pair}_knee" '
                             f'body2="{side}{pair + 1}_ankle"/>')
                lines.append(f'    <exclude body1="{side}{pair}_ankle" '
                             f'body2="{side}{pair + 1}_knee"/>')
        lines += ["  </contact>", ""]

    lines += ["  <actuator>"]

    servo = {
        "hip": (HIP_RANGE, HIP_GAIN, HIP_TORQUE),
        "knee": (KNEE_RANGE, KNEE_GAIN, KNEE_TORQUE),
        "ankle": (ANKLE_RANGE, ANKLE_GAIN, ANKLE_TORQUE),
    }
    for index, leg in enumerate(LEG_NAMES):
        for joint in JOINTS:
            rng, gain, torque = servo[joint]
            if joint == "hip":
                rng = hip_range_for(index // 2)
            lines.append(
                f'    <position name="{leg}_{joint}" joint="{leg}_{joint}" '
                f'kp="{n(gain)}" ctrlrange="{n(rng[0])} {n(rng[1])}" '
                f'forcerange="-{n(torque)} {n(torque)}"/>')

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
    femur_out, femur_up = femur_reach()
    tibia_out, tibia_down = tibia_reach()
    problems, notes = run_checks()

    print()
    print("Project Huntsman - spider body")
    print("=" * 58)
    print("Craig's specification")
    print("  square block body, 8 legs, 3 joints per leg")
    print("  joint 1 horizontal, joints 2 and 3 vertical")
    print("  24 motors")
    print()
    print("How this build meets it")
    print(f"  body         {BODY_LENGTH:.3f} x {BODY_WIDTH:.3f} x "
          f"{BODY_HEIGHT:.3f} m, {BODY_MASS:.2f} kg")
    print(f"  legs         8, in 4 pairs")
    print(f"  joints/leg   3  (1 horizontal, 2 vertical)")
    print(f"  motors       {len(LEG_NAMES) * len(JOINTS)}")
    print()
    print("Leg shape")
    print(f"  femur        {FEMUR_LENGTH:.3f} m at {FEMUR_ANGLE_UP:.0f} deg up"
          f"    -> out {femur_out:.3f}, up {femur_up:+.3f}")
    print(f"  tibia        {TIBIA_LENGTH:.3f} m at {TIBIA_ANGLE_DOWN:.0f} deg "
          f"down  -> out {tibia_out:.3f}, down {tibia_down:+.3f}")
    print(f"  foot         {FOOT_LENGTH:.3f} m straight down")
    print(f"  stands       {standing_height():.3f} m high")
    print(f"  reaches      {leg_reach():.3f} m out from the body")
    print()
    print("Leg fan (positive is forward)")
    for i, fan in enumerate(LEG_FAN):
        print(f"  pair {i + 1}       {fan:+6.1f} deg")
    print()
    print("Motor order - the hardware must send commands in this order")
    print("  legs  l1 r1 l2 r2 l3 r3 l4 r4")
    print("  each  hip, knee, ankle")
    for i, leg in enumerate(LEG_NAMES):
        row = "  ".join(f"{i * 3 + j:2d} {leg}_{j_name}"
                        for j, j_name in enumerate(JOINTS))
        print(f"  {row}")
    print()
    print("Checks")
    for note in notes:
        print(f"  ok    {note}")
    for problem in problems:
        print(f"  WRONG {problem}")
    print()
    if problems:
        print(f"{len(problems)} problem(s). Fix the numbers at the top of "
              f"spider.py and run again.")
    else:
        print("The body is sound.")
    print("=" * 58)
    return not problems


# =====================================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true",
                        help="print the report and exit")
    parser.add_argument("--save", action="store_true",
                        help="write spider.xml and exit")
    args = parser.parse_args()

    ok = report()
    if not ok:
        return 1
    if args.check:
        return 0

    xml = build_model()
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "spider.xml")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(xml)
    print(f"\nwrote {os.path.basename(path)}")

    if args.save:
        return 0

    try:
        import mujoco
        import mujoco.viewer
    except ImportError:
        print("\nmujoco is not installed.  pip install mujoco numpy")
        return 1

    try:
        model = mujoco.MjModel.from_xml_path(path)
    except ValueError as exc:
        print(f"\nMuJoCo rejected the model:\n  {exc}")
        return 1

    data = mujoco.MjData(model)
    print(f"loaded  {model.njnt} joints, {model.nu} motors, "
          f"{sum(model.body_mass):.3f} kg total")

    # Let it settle so it opens standing, not falling.
    for _ in range(600):
        mujoco.mj_step(model, data)

    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")
    print(f"settled at {data.xpos[body][2]:.4f} m "
          f"(expected about {standing_height():.4f})")

    on_ground = sum(
        1 for i in range(data.ncon)
        for pair in [{data.contact[i].geom1, data.contact[i].geom2}]
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        in pair and any(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{leg}_tip")
            in pair for leg in LEG_NAMES))
    print(f"feet on the ground: {on_ground} of 8")
    print("\nopening viewer - drag to rotate, scroll to zoom")

    mujoco.mj_resetData(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.0, standing_height() * 0.5]
        viewer.cam.distance = 1.0
        viewer.cam.elevation = -20.0
        viewer.cam.azimuth = 135.0
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()

    return 0


if __name__ == "__main__":
    sys.exit(main())
