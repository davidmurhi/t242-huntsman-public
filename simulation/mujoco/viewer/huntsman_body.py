"""
Project Huntsman — Step 1: Body only
T242 | IFB398 26se2 | V Source Development

What you're looking at:
  The body shell sitting flat on the floor.
  Grey box = main shell (0.38 x 0.22 x 0.07m)
  Dark front panel = sensor bay (camera / depth sensor will mount here)
  Small raised pads along sides = where the 8 legs will attach (step 2)

Controls:
  Mouse drag   — rotate view
  Scroll       — zoom
  Right drag   — pan
  SPACE        — pause / resume physics
  R            — reset
  Q / ESC      — quit

Run:
  mjpython huntsman_body.py    (Mac)
  python   huntsman_body.py    (Windows / Linux)
"""

import mujoco
import mujoco.viewer
import time, os

XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mjcf", "huntsman_body.xml")

model = mujoco.MjModel.from_xml_path(XML)
data  = mujoco.MjData(model)

print("Project Huntsman — Step 1: Body")
print(f"  Body: 0.38 x 0.22 x 0.07m  |  mass: 0.447kg")
print(f"  Geoms: {model.ngeom}  (shell + sensor bay + 8 leg pads)")
print("  SPACE=pause  R=reset  Q=quit\n")

paused = False

def on_key(key):
    global paused
    if key == 32:  paused = not paused
    if key == 82:  reset()
    if key in (81, 256): raise SystemExit

def reset():
    data.qpos[:] = 0
    data.qpos[2] = 0.035
    data.qpos[3] = 1.0
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)

reset()

with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as v:
    # Isometric-ish view from front-right-above
    v.cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
    v.cam.distance  = 1.4
    v.cam.elevation = -28
    v.cam.azimuth   = 220
    v.cam.lookat[:] = [0, 0, 0.035]

    while v.is_running():
        if not paused:
            mujoco.mj_step(model, data)
        v.sync()
        time.sleep(1/240)
