"""
Project Huntsman — Step 2: Body + L1 leg
T242 | IFB398 26se2 | V Source Development

What you're looking at:
  The body shell with one leg — front-left (L1).
  This is the geometry template for all 8 legs.

  Coxa  (light grey capsule): 0.08m, 30° forward from lateral — reach
  Femur (mid grey capsule):   0.14m, nearly vertical, 10° outward lean
  Tibia (darker capsule):     0.16m, angles 15.8° inward — foot under CoM
  Foot  (dark sphere):        r=0.030m, high friction

  The leg hangs under gravity. Use the controls below to
  drive each joint and confirm the geometry makes sense
  before we mirror it across all 8 legs in step 3.

Controls:
  SPACE         Pause / resume
  R             Reset to home pose
  H / K         Hip left / right   (yaw)
  J / U         Knee down / up     (lift)
  N / M         Ankle down / up    (angle)
  Q / ESC       Quit

Run:
  mjpython huntsman_l1.py    (Mac)
  python   huntsman_l1.py    (Windows / Linux)
"""

import mujoco
import mujoco.viewer
import numpy as np
import time, os

XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mjcf", "huntsman_l1.xml")

model = mujoco.MjModel.from_xml_path(XML)
data  = mujoco.MjData(model)

# Actuator indices
HIP   = 0   # l1_hip
KNEE  = 1   # l1_knee
ANKLE = 2   # l1_ankle

STEP = 0.05   # rad per keypress

paused = False

def reset():
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    data.qpos[:] = model.key_qpos[key_id]
    data.qvel[:] = 0
    data.ctrl[HIP]   = 0.0
    data.ctrl[KNEE]  = 0.45
    data.ctrl[ANKLE] = 0.0
    mujoco.mj_forward(model, data)
    print("\nReset to home pose  (hip=0.0  knee=0.45  ankle=0.0)")

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def on_key(key):
    global paused
    if key == 32:        # SPACE
        paused = not paused
        print(f"\n{'PAUSED' if paused else 'RESUMED'}")
    elif key == 82:      # R
        reset()
    elif key == 72:      # H — hip left
        data.ctrl[HIP] = clamp(data.ctrl[HIP] - STEP, -0.6, 0.6)
        print(f"\r  hip={data.ctrl[HIP]:+.2f}  knee={data.ctrl[KNEE]:.2f}  ankle={data.ctrl[ANKLE]:+.2f}   ", end="")
    elif key == 75:      # K — hip right
        data.ctrl[HIP] = clamp(data.ctrl[HIP] + STEP, -0.6, 0.6)
        print(f"\r  hip={data.ctrl[HIP]:+.2f}  knee={data.ctrl[KNEE]:.2f}  ankle={data.ctrl[ANKLE]:+.2f}   ", end="")
    elif key == 74:      # J — knee down (extend)
        data.ctrl[KNEE] = clamp(data.ctrl[KNEE] - STEP, -0.3, 1.4)
        print(f"\r  hip={data.ctrl[HIP]:+.2f}  knee={data.ctrl[KNEE]:.2f}  ankle={data.ctrl[ANKLE]:+.2f}   ", end="")
    elif key == 85:      # U — knee up (lift)
        data.ctrl[KNEE] = clamp(data.ctrl[KNEE] + STEP, -0.3, 1.4)
        print(f"\r  hip={data.ctrl[HIP]:+.2f}  knee={data.ctrl[KNEE]:.2f}  ankle={data.ctrl[ANKLE]:+.2f}   ", end="")
    elif key == 78:      # N — ankle down
        data.ctrl[ANKLE] = clamp(data.ctrl[ANKLE] - STEP, -0.6, 0.6)
        print(f"\r  hip={data.ctrl[HIP]:+.2f}  knee={data.ctrl[KNEE]:.2f}  ankle={data.ctrl[ANKLE]:+.2f}   ", end="")
    elif key == 77:      # M — ankle up
        data.ctrl[ANKLE] = clamp(data.ctrl[ANKLE] + STEP, -0.6, 0.6)
        print(f"\r  hip={data.ctrl[HIP]:+.2f}  knee={data.ctrl[KNEE]:.2f}  ankle={data.ctrl[ANKLE]:+.2f}   ", end="")
    elif key in (81, 256):
        raise SystemExit

print("Project Huntsman — Step 2: L1 leg")
print(f"  Bodies: {model.nbody}  |  Joints: {model.njnt-1}  |  Actuators: {model.nu}")
print(f"  Coxa 0.08m → Femur 0.14m → Tibia 0.16m → Foot r=0.03m")
print()
print("  H/K = hip swing    J/U = knee lift    N/M = ankle angle")
print("  SPACE = pause      R = reset          Q = quit")
print()

reset()

with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as v:
    # Angle from front-left to show the leg clearly
    v.cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
    v.cam.distance  = 1.6
    v.cam.elevation = -22
    v.cam.azimuth   = 160
    v.cam.lookat[:] = [0.05, 0.1, 0.0]

    while v.is_running():
        if not paused:
            mujoco.mj_step(model, data)
        v.sync()
        time.sleep(1/240)
