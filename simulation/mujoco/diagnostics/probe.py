#!/usr/bin/env python3
"""
probe.py

The full diagnostic. Records everything, then reports the numbers that
separate the possible causes.

It answers four questions:

  1. Are the actuators actually following the commanded trajectory, or
     are they saturating and doing something else?
  2. When a foot lifts, does it go up, or does it also swing sideways?
  3. What was happening at the moment of the FIRST leg-to-leg contact,
     before the robot had a chance to fall over and confuse things?
  4. Do the feet trace clean paths with gravity off and the body held
     still, which would separate a kinematic fault from a balance one?

    python3 probe.py
"""

from __future__ import annotations

import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)          # spider.py and walk.py live one level up
XML = os.path.join(HERE, "spider.xml")
LEGS = ["l1", "r1", "l2", "r2", "l3", "r3", "l4", "r4"]
JOINTS = ("hip", "knee", "ankle")
SECONDS = 8.0


def part_of(name):
    if name is None or name == "ground":
        return "ground"
    for leg in LEGS:
        if name.startswith(leg + "_"):
            return leg
    return "body"


def rpy(quat):
    w, x, y, z = quat
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


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
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")

    act, qadr, dadr, tip = {}, {}, {}, {}
    for leg in LEGS:
        tip[leg] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                     f"{leg}_tip")
        for joint in JOINTS:
            key = f"{leg}_{joint}"
            act[key] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                         key)
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, key)
            qadr[key] = model.jnt_qposadr[jid]
            dadr[key] = model.jnt_dofadr[jid]

    limits = {k: float(abs(model.actuator_forcerange[v][1]))
              for k, v in act.items()}

    # ---------------------------------------------------------------
    # PART A - walking, with everything recorded
    # ---------------------------------------------------------------
    for _ in range(600):
        mujoco.mj_step(model, data)

    stats = {j: {"err": 0.0, "vel": 0.0, "tau": 0.0, "sat": 0, "n": 0}
             for j in JOINTS}
    max_roll = max_pitch = 0.0
    first_hit = None
    swing_paths = {leg: [] for leg in LEGS}
    swing_start = {}

    steps = int(SECONDS / model.opt.timestep)
    start_x = data.xpos[body][0]

    for i in range(steps):
        t = i * model.opt.timestep
        walk.drive(data, act, t, walk.STEP_TIME)
        targets = {k: float(data.ctrl[v]) for k, v in act.items()}
        mujoco.mj_step(model, data)

        roll, pitch = rpy(data.xquat[body])
        max_roll = max(max_roll, abs(roll))
        max_pitch = max(max_pitch, abs(pitch))

        for leg in LEGS:
            for joint in JOINTS:
                key = f"{leg}_{joint}"
                q = math.degrees(data.qpos[qadr[key]])
                qd = math.degrees(data.qvel[dadr[key]])
                tau = float(data.actuator_force[act[key]])
                s = stats[joint]
                s["err"] = max(s["err"], abs(q - targets[key]))
                s["vel"] = max(s["vel"], abs(qd))
                s["tau"] = max(s["tau"], abs(tau))
                s["n"] += 1
                if abs(tau) >= 0.95 * limits[key]:
                    s["sat"] += 1

        # foot travel during each swing
        for leg, _side, offset, _scale in walk.LEGS:
            phase = ((t / walk.STEP_TIME) + offset) % 1.0
            airborne = phase < walk.SWING_SHARE
            pos = data.geom_xpos[tip[leg]].copy()
            if airborne and leg not in swing_start:
                swing_start[leg] = pos
            elif not airborne and leg in swing_start:
                swing_start.pop(leg)
            elif airborne:
                d = pos - swing_start[leg]
                swing_paths[leg].append(d)

        if first_hit is None:
            for c in range(data.ncon):
                con = data.contact[c]
                a = part_of(gname.get(con.geom1))
                b = part_of(gname.get(con.geom2))
                if a == "ground" or b == "ground" or a == b:
                    continue
                first_hit = {
                    "t": t, "a": gname.get(con.geom1),
                    "b": gname.get(con.geom2),
                    "roll": roll, "pitch": pitch,
                    "height": float(data.xpos[body][2]),
                    "depth": -float(con.dist) * 1000,
                }
                break

    travelled = data.xpos[body][0] - start_x

    print()
    print("PROBE REPORT")
    print("=" * 68)
    print(f"  walked {travelled:+.3f} m in {SECONDS:.0f} s")
    print(f"  worst roll {max_roll:.1f} deg, worst pitch {max_pitch:.1f} deg")
    print()
    print("  1. ARE THE ACTUATORS FOLLOWING ORDERS?")
    print(f"     {'joint':<8} {'worst error':>12} {'peak speed':>12} "
          f"{'peak torque':>12} {'saturated':>10}")
    print("     " + "-" * 58)
    for joint in JOINTS:
        s = stats[joint]
        share = 100.0 * s["sat"] / max(s["n"], 1)
        print(f"     {joint:<8} {s['err']:9.1f} deg {s['vel']:8.0f} deg/s "
              f"{s['tau']:9.2f} Nm {share:9.1f}%")
    worst_sat = max(100.0 * stats[j]["sat"] / max(stats[j]["n"], 1)
                    for j in JOINTS)
    worst_err = max(stats[j]["err"] for j in JOINTS)
    print()
    if worst_sat > 20:
        print(f"     SATURATED {worst_sat:.0f}% of the time. The motors cannot")
        print("     deliver what the gait asks. Raise the torque limits in")
        print("     spider.py, or lower kp so it asks for less.")
    elif worst_err > 15:
        print(f"     Tracking is poor - up to {worst_err:.0f} deg behind the")
        print("     target. The legs are not where the gait thinks they are.")
        print("     Raise kp, or raise damping to stop overshoot.")
    else:
        print("     Tracking is good. The legs are going where they are told.")

    print()
    print("  2. WHEN A FOOT LIFTS, WHERE DOES IT GO?")
    print(f"     {'leg':<5} {'up':>9} {'sideways':>10} {'forward':>10}"
          f"   verdict")
    print("     " + "-" * 52)
    bad_lift = False
    for leg in LEGS:
        if not swing_paths[leg]:
            continue
        arr = np.array(swing_paths[leg])
        up = float(arr[:, 2].max())
        side = float(np.abs(arr[:, 1]).max())
        fwd = float(np.abs(arr[:, 0]).max())
        note = ""
        if side > up * 1.2:
            note = "sideways more than up"
            bad_lift = True
        print(f"     {leg:<5} {up * 1000:6.0f} mm {side * 1000:7.0f} mm "
              f"{fwd * 1000:7.0f} mm   {note}")
    print()
    if bad_lift:
        print("     THE LIFT IS SWINGING FEET SIDEWAYS more than upward.")
        print("     Raising the knee on an outward-angled tibia pushes the")
        print("     foot out as well as up. That is what puts a swinging")
        print("     leg into its neighbour.")
    else:
        print("     Feet lift mostly upward. The swing path is sensible.")

    print()
    print("  3. THE FIRST LEG-TO-LEG CONTACT")
    if first_hit is None:
        print("     Never happened. No legs touched each other at all.")
    else:
        h = first_hit
        print(f"     at {h['t']:.3f} s   {h['a']} hit {h['b']}")
        print(f"     body was rolled {h['roll']:+.1f} deg, "
              f"pitched {h['pitch']:+.1f} deg, at {h['height']:.4f} m")
        print(f"     overlap {h['depth']:.1f} mm")
        print()
        if abs(h["roll"]) > 10 or abs(h["pitch"]) > 10:
            print("     The body was already tipping when they first touched,")
            print("     so balance is the cause and the collision follows.")
        else:
            print("     The body was still level when they first touched, so")
            print("     the legs were driven together, not shaken together.")

    # ---------------------------------------------------------------
    # PART B - no gravity, body pinned. Pure kinematics under dynamics.
    # ---------------------------------------------------------------
    print()
    print("  4. SAME GAIT, GRAVITY OFF, BODY HELD STILL")
    model.opt.gravity[:] = 0.0
    data2 = mujoco.MjData(model)
    hits = 0
    for i in range(int(4.0 / model.opt.timestep)):
        walk.drive(data2, act, i * model.opt.timestep, walk.STEP_TIME)
        data2.qpos[0:3] = [0.0, 0.0, 0.5]
        data2.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        data2.qvel[0:6] = 0.0
        mujoco.mj_step(model, data2)
        for c in range(data2.ncon):
            con = data2.contact[c]
            a = part_of(gname.get(con.geom1))
            b = part_of(gname.get(con.geom2))
            if a == "ground" or b == "ground" or a == b:
                continue
            hits += 1
    print(f"     leg-to-leg contacts with no gravity and no falling: {hits}")
    print()
    if hits == 0:
        print("     Clean. So the legs only collide once the robot has to")
        print("     hold itself up. It is a balance problem, not a gait one.")
    else:
        print("     Still colliding even with nothing to fall over from.")
        print("     So the trajectory itself drives them together - this is")
        print("     the swing path, not balance.")

    print()
    print("=" * 68)
    print("Paste this whole report back.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
