#!/usr/bin/env python3
"""
poses.py

Separates two very different problems that look the same.

Either the gait commands poses where the legs genuinely overlap, which is
a geometry problem, or the poses are all fine and the robot is going
unstable and flailing its legs into each other, which is a control
problem. They need opposite fixes, so this tells them apart.

It does that by stepping the robot through one whole gait cycle with the
physics frozen - joints placed exactly where the gait asks, no gravity,
no momentum, no falling. If legs overlap here, the poses themselves are
bad. If nothing overlaps here but they collide while walking, the poses
are fine and the robot is losing its balance.

    python3 poses.py
"""

import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)          # spider.py and walk.py live one level up
XML = os.path.join(HERE, "spider.xml")

LEGS = ["l1", "r1", "l2", "r2", "l3", "r3", "l4", "r4"]
SAMPLES = 72          # poses tested around one full cycle


def part_of(name):
    if name is None or name == "ground":
        return "ground"
    for leg in LEGS:
        if name.startswith(leg + "_"):
            return leg
    return "body"


def ease(x):
    return x * x * (3.0 - 2.0 * x)


def main():
    if not os.path.exists(XML):
        print("spider.xml not found. run: python3 spider.py --save")
        return 1
    try:
        import walk
    except ImportError:
        print("walk.py must be beside this file")
        return 1

    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    gname = {i: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
             for i in range(model.ngeom)}

    # Map each joint to its slot in qpos so we can place it directly.
    slot = {}
    for leg in LEGS:
        for joint in ("hip", "knee", "ankle"):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                    f"{leg}_{joint}")
            slot[f"{leg}_{joint}"] = model.jnt_qposadr[jid]

    print()
    print("Gait poses, physics frozen")
    print("=" * 66)
    print(f"  gait: reach {walk.STEP_REACH:.0f}, lift {walk.STEP_LIFT:.0f}, "
          f"swing share {walk.SWING_SHARE:.2f}")
    print(f"  testing {SAMPLES} poses around one cycle")
    print()

    clashes = {}
    worst_pose = None

    for s in range(SAMPLES):
        cycle = s / SAMPLES

        # Park the body well clear of the ground so only the legs matter.
        data.qpos[:] = model.qpos0
        data.qpos[2] = 1.0
        data.qvel[:] = 0.0

        for leg, side, offset, scale in walk.LEGS:
            sweep, lift = walk.leg_targets((cycle + offset) % 1.0,
                                           walk.STEP_REACH * scale)
            data.qpos[slot[f"{leg}_hip"]] = math.radians(-side * sweep)
            data.qpos[slot[f"{leg}_knee"]] = math.radians(lift)
            data.qpos[slot[f"{leg}_ankle"]] = math.radians(
                -lift * walk.ANKLE_FOLLOW)

        # Place the bodies and find contacts. No stepping, no dynamics.
        mujoco.mj_forward(model, data)

        hits = 0
        for c in range(data.ncon):
            con = data.contact[c]
            a = part_of(gname.get(con.geom1))
            b = part_of(gname.get(con.geom2))
            if a == "ground" or b == "ground":
                continue
            hits += 1
            key = tuple(sorted((a, b)))
            depth = -con.dist * 1000.0
            if key not in clashes or depth > clashes[key][0]:
                clashes[key] = (depth, cycle,
                                gname.get(con.geom1), gname.get(con.geom2))

        if worst_pose is None or hits > worst_pose[1]:
            worst_pose = (cycle, hits)

    if not clashes:
        print("  NO POSE IN THE GAIT OVERLAPS.")
        print()
        print("  Every position the gait asks for is geometrically fine.")
        print("  So the legs are not being commanded into each other - the")
        print("  robot is losing its balance and flailing them together.")
        print()
        print("  That is a control problem, not a shape problem. The body")
        print("  does not need changing. What needs changing is how many")
        print("  legs leave the ground at once, and how hard it steps.")
        print()
        print("  Next thing to try: a slower, more careful gait where only")
        print("  two legs lift at a time instead of four.")
    else:
        print(f"  {len(clashes)} pair(s) of legs overlap in some pose.")
        print(f"  Worst pose is at {worst_pose[0]:.0%} through the cycle, "
              f"with {worst_pose[1]} contacts.")
        print()
        print(f"  {'legs':<14} {'deepest':>9}  {'at cycle':>9}   parts")
        print("  " + "-" * 62)
        for key, (depth, cycle, g1, g2) in sorted(
                clashes.items(), key=lambda kv: -kv[1][0]):
            same = " (same leg folding on itself)" if key[0] == key[1] else ""
            print(f"  {key[0] + ' - ' + key[1]:<14} {depth:7.1f} mm  "
                  f"{cycle:8.0%}   {g1} / {g2}{same}")
        print()
        deep = max(d for d, _c, _a, _b in clashes.values())
        print(f"  The poses themselves are bad, by up to {deep:.0f} mm.")
        print("  Reduce STEP_LIFT in walk.py first - lifting the knee is")
        print("  what swings each leg outward into its neighbour.")

    print()
    print("=" * 66)
    print("Paste this whole output back.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
