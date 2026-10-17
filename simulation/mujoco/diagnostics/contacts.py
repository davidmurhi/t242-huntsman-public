#!/usr/bin/env python3
"""
contacts.py

Dumps exactly what is touching what, with positions and distances, so
there is no guessing about which parts are really involved.

    python3 contacts.py
"""

import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)          # spider.py and walk.py live one level up
XML = os.path.join(HERE, "spider.xml")


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

    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
             for i in range(model.ngeom)]
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")

    act = {}
    for leg, _s, _o, _sc in walk.LEGS:
        for joint in ("hip", "knee", "ankle"):
            key = f"{leg}_{joint}"
            act[key] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                         key)

    print()
    print("Every geom in the model")
    print("=" * 66)
    for i, name in enumerate(names):
        print(f"  {i:3d}  {name}")

    for _ in range(600):
        mujoco.mj_step(model, data)

    print()
    print("Walking for 3 seconds, then listing what touches")
    print("=" * 66)

    seen = {}
    force = np.zeros(6)
    steps = int(3.0 / model.opt.timestep)
    for i in range(steps):
        walk.drive(data, act, i * model.opt.timestep, walk.STEP_TIME)
        mujoco.mj_step(model, data)
        for c in range(data.ncon):
            con = data.contact[c]
            one, two = names[con.geom1], names[con.geom2]
            key = (one, two)
            mujoco.mj_contactForce(model, data, c, force)
            entry = seen.setdefault(key, {"n": 0, "deep": 0.0, "peak": 0.0,
                                          "pos": None, "gap": 0.0})
            entry["n"] += 1
            depth = -float(con.dist) * 1000.0
            if depth > entry["deep"]:
                entry["deep"] = depth
                entry["pos"] = con.pos.copy()
            entry["peak"] = max(entry["peak"], float(abs(force[0])))

    print()
    print(f"  {'geom A':<14} {'geom B':<14} {'frames':>7} {'deepest':>9} "
          f"{'force':>9}   where the contact is")
    print("  " + "-" * 76)
    for (one, two), e in sorted(seen.items(), key=lambda kv: -kv[1]["n"]):
        pos = e["pos"]
        where = (f"({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})"
                 if pos is not None else "")
        print(f"  {str(one):<14} {str(two):<14} {e['n']:>7} "
              f"{e['deep']:7.2f}mm {e['peak']:7.2f}N   {where}")

    # where the foot tips actually are
    print()
    print("  Foot tip positions right now")
    print(f"  {'leg':<5} {'x':>9} {'y':>9} {'z':>9}")
    print("  " + "-" * 36)
    tips = {}
    for leg in ("l1", "r1", "l2", "r2", "l3", "r3", "l4", "r4"):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{leg}_tip")
        p = data.geom_xpos[gid].copy()
        tips[leg] = p
        print(f"  {leg:<5} {p[0]:9.4f} {p[1]:9.4f} {p[2]:9.4f}")

    print()
    print("  Distance between each pair of foot tips")
    pairs = [("l1", "l2"), ("l2", "l3"), ("l3", "l4"),
             ("r1", "r2"), ("r2", "r3"), ("r3", "r4")]
    for a, b in pairs:
        d = float(np.linalg.norm(tips[a] - tips[b]))
        flag = "  <-- touching?" if d < 0.02 else ""
        print(f"    {a} to {b}: {d * 1000:6.1f} mm{flag}")

    print()
    print(f"  body at {data.xpos[body][2]:.4f} m")
    print("=" * 66)
    print()
    print("Paste this back.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
