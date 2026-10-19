#!/usr/bin/env python3
"""
watch_policy.py

Watch the robot on a terrain, driven by a trained policy or by the
hand-written gait alone, and see how far each gets.

    mjpython watch_policy.py --terrain rubble              hand gait only
    mjpython watch_policy.py runs/<name>/best_model.zip --terrain rubble
    python watch_policy.py runs/<name>/best_model.zip --terrain steps --score
                                                       (no window, 5 runs)
(macOS needs mjpython for the window; plain python elsewhere.)
"""

from __future__ import annotations

import argparse
import os
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def load_policy(path):
    if not path:
        return None, None
    from stable_baselines3 import PPO
    import pickle
    model = PPO.load(path, device="cpu")
    norm = os.path.join(os.path.dirname(path), "vecnormalize.pkl")
    with open(norm, "rb") as fh:
        vn = pickle.load(fh)
    return model, vn


def act(model, vn, obs):
    if model is None:
        return np.zeros(24)
    o = np.clip((obs - vn.obs_rms.mean) / np.sqrt(vn.obs_rms.var + vn.epsilon),
                -vn.clip_obs, vn.clip_obs)
    a, _ = model.predict(o, deterministic=True)
    return a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("policy", nargs="?", default="",
                    help="best_model.zip from a training run (omit for the "
                         "hand gait alone)")
    ap.add_argument("--terrain", default="rubble",
                    choices=("flat", "rubble", "slope", "steps", "mixed"))
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--difficulty", type=float, default=1.0)
    ap.add_argument("--score", action="store_true",
                    help="no window: 5 runs, report distance and falls")
    args = ap.parse_args()

    from huntsman_env import HuntsmanEnv, CONTROL_HZ
    env = HuntsmanEnv(terrains=(args.terrain,), difficulty=args.difficulty,
                      randomise=False, seed=args.seed)
    model, vn = load_policy(args.policy)
    who = "trained policy" if model else "hand gait only"

    if args.score:
        dist = []
        for k in range(5):
            obs, _ = env.reset(seed=args.seed + k)
            done = False
            while not done:
                obs, _r, term, trunc, info = env.step(act(model, vn, obs))
                done = term or trunc
            dist.append(info["x"])
            print(f"  run {k + 1}: {info['x']:+.2f} m{'  FELL' if term else ''}")
        print(f"{who} on {args.terrain}: {np.mean(dist):+.2f} m average "
              f"in {env.t:.0f} s")
        return 0

    import mujoco.viewer
    obs, _ = env.reset(seed=args.seed)
    print(f"{who} on {args.terrain} - close the window to stop")
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        viewer.cam.distance, viewer.cam.elevation = 1.2, -22.0
        viewer.cam.azimuth = 135.0
        body = env.model.body("body").id
        while viewer.is_running():
            t0 = time.time()
            with viewer.lock():
                obs, _r, term, trunc, info = env.step(act(model, vn, obs))
                viewer.cam.lookat[:] = env.data.xpos[body]
            if term or trunc:
                print(f"  walked {info['x']:+.2f} m"
                      f"{' - fell' if term else ''}")
                break
            viewer.sync()
            time.sleep(max(0.0, 1.0 / CONTROL_HZ - (time.time() - t0)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
