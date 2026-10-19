#!/usr/bin/env python3
"""
rest.py

Finds out what is overlapping while the robot just stands there.

No gait, no walking. Settles the robot and asks MuJoCo which parts are
touching, where they are, and how far into each other they have sunk.

    python3 rest.py
"""

import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "spider.xml")

LEGS = ["l1", "r1", "l2", "r2", "l3", "r3", "l4", "r4"]


def main():
    if not os.path.exists(XML):
        print("spider.xml not found. run: python3 spider.py --save")
        return 1

    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    name = {i: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
            for i in range(model.ngeom)}

    print()
    print("Standing still - what is touching what")
    print("=" * 66)

    # --- before any physics: the pose as built ---
    mujoco.mj_forward(model, data)
    print(f"\nAs built, before gravity: {data.ncon} contact(s)")
    for c in range(data.ncon):
        con = data.contact[c]
        print(f"  {name.get(con.geom1)} <-> {name.get(con.geom2)}   "
              f"overlap {-con.dist * 1000:.1f} mm")

    # --- settle under gravity, no motor commands ---
    data.ctrl[:] = 0.0
    for _ in range(1500):
        mujoco.mj_step(model, data)

    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")
    print(f"\nAfter settling: body at {data.xpos[body][2]:.4f} m, "
          f"{data.ncon} contact(s)")
    print()

    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    feet, others = [], []
    for c in range(data.ncon):
        con = data.contact[c]
        n1, n2 = name.get(con.geom1), name.get(con.geom2)
        if ground in (con.geom1, con.geom2):
            feet.append(n1 if con.geom2 == ground else n2)
        else:
            others.append((n1, n2, -con.dist * 1000, con.pos.copy()))

    print(f"  feet on the ground: {len(feet)}")
    for f in sorted(feet):
        print(f"    {f}")

    print()
    if not others:
        print("  nothing else is touching. the legs are clear when standing.")
        print("  so the overlap only happens once it starts moving.")
    else:
        print(f"  PARTS OVERLAPPING WHILE STANDING STILL: {len(others)}")
        print()
        print(f"  {'part A':<14} {'part B':<14} {'sunk in':>9}   where")
        print("  " + "-" * 60)
        for n1, n2, depth, pos in sorted(others, key=lambda r: -r[2]):
            print(f"  {n1:<14} {n2:<14} {depth:7.1f} mm   "
                  f"({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})")

    # --- where each leg's parts actually are ---
    print()
    print("  Where the parts are (world coordinates, after settling)")
    print(f"  {'geom':<14} {'x':>8} {'y':>8} {'z':>8}")
    print("  " + "-" * 44)
    for leg in ("l1", "l2"):
        for part in ("femur", "tibia", "foot", "tip"):
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                    f"{leg}_{part}")
            if gid == -1:
                continue
            p = data.geom_xpos[gid]
            print(f"  {leg + '_' + part:<14} {p[0]:8.4f} {p[1]:8.4f} "
                  f"{p[2]:8.4f}")
        print()

    # --- how close are neighbouring legs, measured by MuJoCo ---
    print("  Distance between neighbouring leg parts (centre to centre)")
    for a, b in (("l1", "l2"), ("l2", "l3"), ("l3", "l4")):
        worst = None
        for pa in ("femur", "tibia", "foot"):
            for pb in ("femur", "tibia", "foot"):
                ga = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                       f"{a}_{pa}")
                gb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                       f"{b}_{pb}")
                if ga == -1 or gb == -1:
                    continue
                d = float(np.linalg.norm(
                    data.geom_xpos[ga] - data.geom_xpos[gb]))
                if worst is None or d < worst[0]:
                    worst = (d, f"{a}_{pa}", f"{b}_{pb}")
        print(f"    {a}-{b}: closest parts {worst[1]} and {worst[2]}, "
              f"{worst[0] * 1000:.0f} mm apart")

    print()
    print("=" * 66)
    print("Paste this whole output back.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
