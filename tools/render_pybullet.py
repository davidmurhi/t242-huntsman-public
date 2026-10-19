#!/usr/bin/env python3
"""
render_pybullet.py

Runs Kai's PyBullet spider (simulation/pybullet/spider8_v6.py) headless,
with the keyboard replaced by a scripted drive, and saves screenshots and
a short walking GIF. The gait code is Kai's, executed unchanged apart
from the window, keyboard and camera lines listed in PATCHES.

    pip install pybullet numpy pillow
    python tools/render_pybullet.py --out renders/pybullet
"""

from __future__ import annotations

import argparse
import math
import os

import numpy as np
import pybullet as p
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "simulation", "pybullet", "spider8_v6.py")

# (seconds, keys held) - forward, then turn left, then stop
DRIVE = [(1.0, ""), (7.0, "w"), (4.0, "wa"), (2.0, "")]

PATCHES = [
    ("p.connect(p.GUI)", "p.connect(p.DIRECT)"),
    ("while p.isConnected():", "while _tick(robot_id):"),
    ("keys = p.getKeyboardEvents()", "keys = _keys()"),
    ("FOLLOW_CAMERA = True", "FOLLOW_CAMERA = False"),
    ("time.sleep(DT)", "pass"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="renders/pybullet")
    ap.add_argument("--size", type=int, nargs=2, default=(960, 600))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    W, H = args.size

    with open(SCRIPT, encoding="utf-8") as fh:
        src = fh.read()
    for old, new in PATCHES:
        if old not in src:
            raise SystemExit(f"spider8_v6.py changed: cannot find {old!r}")
        src = src.replace(old, new)

    dt = 1.0 / 240.0
    total = sum(s for s, _k in DRIVE)
    state = {"step": 0, "frames": [], "track": []}

    def held_keys(t):
        for secs, keys in DRIVE:
            if t < secs:
                return keys
            t -= secs
        return ""

    def _keys():
        t = state["step"] * dt
        return {ord(k): p.KEY_IS_DOWN for k in held_keys(t)}

    def snap(robot_id, yaw, pitch, dist, w=W, h=H):
        pos, _ = p.getBasePositionAndOrientation(robot_id)
        view = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[pos[0], pos[1], 0.6 * pos[2]], distance=dist,
            yaw=yaw, pitch=pitch, roll=0, upAxisIndex=2)
        proj = p.computeProjectionMatrixFOV(fov=45, aspect=w / h,
                                            nearVal=0.02, farVal=20)
        img = p.getCameraImage(w, h, view, proj,
                               renderer=p.ER_TINY_RENDERER,
                               lightDirection=[1, 1, 2], shadow=1)
        rgb = np.reshape(img[2], (h, w, 4))[:, :, :3].astype(np.uint8)
        return Image.fromarray(rgb)

    def _tick(robot_id):
        k = state["step"]
        t = k * dt
        if t >= total:
            return False
        pos, orn = p.getBasePositionAndOrientation(robot_id)
        state["track"].append((t, *pos, p.getEulerFromQuaternion(orn)[2]))
        if k % 12 == 0:                                   # 20 frames/s
            state["frames"].append(snap(robot_id, 45 + 20 * t / total, -22,
                                        2.0, 480, 300))
        if k == int(4.0 / dt):
            for name, yaw, pitch, dist in (("iso", 45, -25, 1.9),
                                           ("side", 0, -8, 2.0),
                                           ("top", 0, -80, 2.2),
                                           ("front", 90, -15, 1.9)):
                snap(robot_id, yaw, pitch, dist).save(
                    os.path.join(args.out, f"pybullet_{name}.png"))
        state["step"] += 1
        return True

    env = {"__name__": "__main__", "__file__": SCRIPT, "_keys": _keys,
           "_tick": _tick}
    exec(compile(src, SCRIPT, "exec"), env)     # noqa: S102 - Kai's script

    frames = state["frames"]
    frames[0].save(os.path.join(args.out, "pybullet_walk.gif"),
                   save_all=True, append_images=frames[1:], duration=50,
                   loop=0, optimize=True)
    tr = np.array(state["track"])
    walked = math.hypot(tr[-1, 1] - tr[0, 1], tr[-1, 2] - tr[0, 2])
    turned = math.degrees(tr[-1, 4] - tr[0, 4])
    with open(os.path.join(args.out, "pybullet_run.txt"), "w") as fh:
        fh.write(f"Kai's v6 gait, gear 3, {total:.0f} s scripted drive\n"
                 f"moved {walked:.2f} m, turned {turned:+.0f} deg, "
                 f"final height {tr[-1, 3]:.3f} m\n")
    print(open(os.path.join(args.out, "pybullet_run.txt")).read())


if __name__ == "__main__":
    main()
