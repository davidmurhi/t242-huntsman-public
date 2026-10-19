"""
Project Huntsman — Spider8 MuJoCo Simulation v2
T242 | IFB398 26se2 | V Source Development

Craig Schneider spec:
  - Square body (~0.34x0.34m footprint)
  - 8 legs, 3 joints each = 24 servos total
  - 8 horizontal joints (hip/yaw) + 16 vertical joints (knee+ankle/pitch)
  - Purpose: search and rescue on rough terrain

Research applied (He et al. 2024 — Georgia Tech, arxiv 2409.09473):
  - Foot touch sensors per leg → contact ratio beta = feet_grounded / 8
  - Contact-ratio feedback controller:
      when beta drops (feet losing ground on obstacles), knee lift increases
      proportional gain Kp=0.30, target beta=0.875 (7/8 feet grounded)
      This is the same principle that gave 30-60% speed gain on rugged terrain
  - Staggered obstacle field with asymmetric block heights to test mixed
    foot-contact states (same design as He et al. lab terrain)
  - He et al. reward structure (for Kai's Gymnasium wrapper):
      R = v_forward - 0.6 * |v_lateral|  (speed minus yaw drift penalty)
      + foot_contact_bonus               (reward for maintaining beta >= 0.75)
      - 0.001 * sum(ctrl^2)              (energy penalty)

Prab's pipeline:
  sim policy -> joint angles (rad) -> Pi5 -> ESP32 PWM -> 24 servos
  Actuator order: l1_hip, l1_knee, l1_ankle, r1_hip... (see XML)

Kai's Gymnasium integration:
  Observation (50-dim):
    - qpos[7:]     : 24 joint positions
    - qvel[6:]     : 24 joint velocities
    - qpos[3:7]    : body orientation quaternion
    - sensordata   : 8 foot contact readings (touch sensors)
  Action: 24 actuator targets (same order as ACT dict)
  Reward: R = v_forward - 0.6*|v_lateral| + contact_bonus - energy_penalty
  Reset: keyframe "home" (qpos defined in XML)
  Episode termination: torso height < 0.12m (fallen) OR 30s elapsed

Controls:
  SPACE   Pause / resume
  R       Reset to home pose
  T       Toggle top-down camera
  UP      Speed up gait
  DOWN    Slow down gait
  Q/ESC   Quit

Run:
  mjpython spider_mujoco.py    (Mac)
  python   spider_mujoco.py    (Windows / Linux)

Requirements:
  pip install mujoco numpy
"""

import mujoco
import mujoco.viewer
import numpy as np
import time, os, csv
from datetime import datetime

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
XML_PATH   = os.path.join(SCRIPT_DIR, "spider8_v5_mujoco.xml")
LOG_PATH   = os.path.join(SCRIPT_DIR,
             f"gait_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

# ── Gait parameters ───────────────────────────────────────────────────────────
HIP_AMP      = 0.22   # hip swing amplitude (rad) — slightly wider than v1
KNEE_STANCE  = 0.45   # knee angle while foot is planted
KNEE_SWING   = 0.60   # knee lift during swing phase — raised from 0.55 for obstacle clearance
ANKLE_STANCE = 0.02
ANKLE_SWING  = 0.28   # more ankle lift for obstacle clearance (was 0.22)
SWING_FRAC   = 0.18   # 18% swing, 82% stance (was 15%) — longer swing = higher step
CYCLE_PERIOD = 3.2    # slightly slower default to give feedback controller time to react
SPEED_STEP   = 0.3

# He et al. contact-ratio feedback controller parameters
# Paper: R = v_f - 0.6|v_l|; Kp proportional to contact discrepancy
BETA_TARGET  = 0.875  # 7/8 feet grounded = ideal for wave gait
Kp_CONTACT   = 0.30   # proportional gain for knee lift adjustment
MAX_LIFT_ADD = 0.20   # max extra knee lift from contact feedback (rad)

# Individual leg order — interleaved L/R so opposite legs never lift together
ALL_LEGS = ["l1", "r2", "l3", "r4", "r1", "l2", "r3", "l4"]
FOOT_SENSORS = ["l1_sensor","r1_sensor","l2_sensor","r2_sensor",
                "l3_sensor","r3_sensor","l4_sensor","r4_sensor"]

# ── Load model ────────────────────────────────────────────────────────────────
print(f"Loading: {XML_PATH}")
model = mujoco.MjModel.from_xml_path(XML_PATH)
data  = mujoco.MjData(model)
print(f"Model: {model.nu} actuators | {model.nbody} bodies | "
      f"{model.nv} DOF | {model.nsensor} sensors")

key_id    = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
INIT_QPOS = model.key_qpos[key_id].copy() if key_id >= 0 else model.qpos0.copy()
INIT_QVEL = np.zeros(model.nv)
TORSO_ID  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso")

# ── Actuator + sensor lookup ──────────────────────────────────────────────────
def act_id(name):
    for i in range(model.nu):
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) == name:
            return i
    return None

def sensor_id(name):
    for i in range(model.nsensor):
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i) == name:
            return i
    return None

LEG_PAIRS = [("l1","r1"), ("l2","r2"), ("l3","r3"), ("l4","r4")]
ACT = {f"{leg}_{jnt}": act_id(f"{leg}_{jnt}")
       for pair in LEG_PAIRS for leg in pair for jnt in ("hip","knee","ankle")}
assert all(v is not None for v in ACT.values()), "Missing actuator — check XML"

SENSOR_IDS = {}
for s in FOOT_SENSORS:
    sid = sensor_id(s)
    assert sid is not None, f"Missing sensor: {s}"
    SENSOR_IDS[s] = sid
print(f"All {len(SENSOR_IDS)} foot sensors found.")

# ── State ─────────────────────────────────────────────────────────────────────
paused       = False
top_view     = False
cycle_period = CYCLE_PERIOD
t0           = time.time()
start_x      = None
step_n       = 0
log_rows     = []
beta_history = []   # contact ratio history for display


def reset():
    global t0, start_x, step_n, beta_history
    data.qpos[:] = INIT_QPOS
    data.qvel[:] = INIT_QVEL
    data.ctrl[:]  = 0.0
    for pair in LEG_PAIRS:
        for leg in pair:
            data.ctrl[ACT[f"{leg}_knee"]] = KNEE_STANCE
    mujoco.mj_forward(model, data)
    t0           = time.time()
    start_x      = None
    step_n       = 0
    beta_history = []
    print("\nReset.")


def torso_pos():
    return data.xpos[TORSO_ID].copy()


def get_roll_pitch():
    """Body roll and pitch from freejoint quaternion."""
    w, x, y, z = data.qpos[3:7]
    roll  = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    return roll, pitch


def get_contact_ratio():
    """
    He et al. 2024 contact ratio beta = fraction of feet currently in contact.
    Uses MuJoCo touch sensors — returns 0.0 (no contact) to 1.0 (all 8 grounded).
    A touch sensor reads > 0 when the site is in contact with any geom.
    """
    # sensordata is updated automatically by mj_step — no separate call needed
    contacts = sum(
        1 for s in FOOT_SENSORS if data.sensordata[SENSOR_IDS[s]] > 0.1
    )
    return contacts / 8.0


def apply_gait(t):
    """
    Individual-leg wave gait with two feedback layers:

    1. Roll+pitch stabilisation (v1 — geometric correction from body orientation)
    2. Contact-ratio feedback (new — He et al. 2024):
         beta = fraction of feet grounded (touch sensors)
         delta_beta = BETA_TARGET - beta   (positive = fewer feet than ideal)
         extra_lift = Kp_CONTACT * delta_beta  (boost knee lift for all swing legs)
       When feet lose contact on an obstacle, beta drops below 0.875,
       the controller automatically increases knee lift to step over it.
       This is the core mechanism from He et al. that gave 30-60% speed gain.

    Hip (yaw): swings leg forward/back in horizontal plane.
    Knee+ankle (pitch): lifts foot during swing.
    """
    roll, pitch = get_roll_pitch()
    beta        = get_contact_ratio()
    beta_history.append(beta)

    # Contact-ratio feedback: how much extra lift to add (He et al. Eq.4 adapted)
    delta_beta = BETA_TARGET - beta
    extra_lift = np.clip(Kp_CONTACT * delta_beta, 0.0, MAX_LIFT_ADD)

    base = (t / cycle_period) % 1.0

    for i, leg in enumerate(ALL_LEGS):
        phase = (base + i / 8.0) % 1.0
        side  = 1 if leg.startswith("l") else -1
        leg_n = int(leg[1])

        if phase < SWING_FRAC:
            sc    = 0.5 - 0.5 * np.cos((phase / SWING_FRAC) * np.pi)
            fwd   = -HIP_AMP + 2 * HIP_AMP * sc
            # Apply extra_lift during swing to help clear obstacles
            knee  = KNEE_STANCE + (KNEE_SWING + extra_lift) * np.sin(sc * np.pi)
            ankle = ANKLE_STANCE + (ANKLE_SWING + extra_lift * 0.5) * np.sin(sc * np.pi)
        else:
            sc    = 0.5 - 0.5 * np.cos(((phase - SWING_FRAC) / (1 - SWING_FRAC)) * np.pi)
            fwd   = HIP_AMP - 2 * HIP_AMP * sc
            knee  = KNEE_STANCE
            ankle = ANKLE_STANCE

        # Geometric stabilisation (v1 — unchanged, works well)
        pitch_sign = -1 if leg_n <= 2 else 1
        knee_pitch = pitch_sign * np.clip(pitch * 2.0, -0.30, 0.30)
        knee_roll  = -side * np.clip(roll * 1.5, -0.25, 0.25)

        data.ctrl[ACT[f"{leg}_hip"]]   = -side * fwd
        data.ctrl[ACT[f"{leg}_knee"]]  = np.clip(knee + knee_pitch + knee_roll, 0.05, 1.50)
        data.ctrl[ACT[f"{leg}_ankle"]] = np.clip(ankle, -0.40, 0.40)


def print_status(t, pos, dist):
    roll, pitch = get_roll_pitch()
    beta        = beta_history[-1] if beta_history else 0.0
    spd         = dist / max(t, 0.001)
    print(f"\r  t={t:5.1f}s  x={pos[0]:5.2f}  z={pos[2]:.3f}  "
          f"dist={dist:4.2f}m  spd={spd:.3f}m/s  "
          f"roll={np.degrees(roll):+4.1f}°  pitch={np.degrees(pitch):+4.1f}°  "
          f"beta={beta:.2f}  cycle={cycle_period:.1f}s  ", end="", flush=True)


# ── Key handler ───────────────────────────────────────────────────────────────
def on_key(keycode):
    global paused, top_view, cycle_period
    if   keycode == 32:        paused = not paused; print(f"\n{'PAUSED' if paused else 'RESUMED'}")
    elif keycode == 82:        reset()
    elif keycode == 84:        top_view = not top_view
    elif keycode == 265:       cycle_period = max(1.0, cycle_period - SPEED_STEP); print(f"\nSpeed UP — cycle={cycle_period:.1f}s")
    elif keycode == 264:       cycle_period = min(8.0, cycle_period + SPEED_STEP); print(f"\nSpeed DOWN — cycle={cycle_period:.1f}s")
    elif keycode in (81, 256): print("\nQuit."); raise SystemExit


# ── Main loop ─────────────────────────────────────────────────────────────────
print("\nProject Huntsman — Spider8 v2")
print("Research: He et al. 2024 contact-ratio feedback controller")
print("SPACE=pause  R=reset  T=top  UP/DOWN=speed  Q=quit\n")

with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:

    viewer.cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.distance  = 4.5
    viewer.cam.elevation = -18
    viewer.cam.azimuth   = 200

    reset()

    while viewer.is_running():
        if paused:
            time.sleep(0.05)
            viewer.sync()
            continue

        t   = time.time() - t0
        pos = torso_pos()
        if start_x is None:
            start_x = pos[0]
        dist = pos[0] - start_x

        apply_gait(t)
        mujoco.mj_step(model, data)

        if not top_view:
            viewer.cam.lookat[:]  = [pos[0], pos[1], pos[2]]
            viewer.cam.distance   = 4.5
            viewer.cam.elevation  = -18
            viewer.cam.azimuth    = 200
        else:
            viewer.cam.lookat[:]  = [pos[0], pos[1], 0.0]
            viewer.cam.distance   = 5.5
            viewer.cam.elevation  = -90
            viewer.cam.azimuth    = 0

        step_n += 1
        if step_n % 60  == 0: print_status(t, pos, dist)
        if step_n % 120 == 0:
            roll, pitch = get_roll_pitch()
            beta        = beta_history[-1] if beta_history else 0.0
            log_rows.append([
                round(t, 2),
                round(pos[0], 4), round(pos[1], 4), round(pos[2], 4),
                round(dist, 4),
                round(dist / max(t, 0.001), 4),
                round(np.degrees(roll), 2),
                round(np.degrees(pitch), 2),
                round(beta, 3)
            ])

        viewer.sync()
        time.sleep(1.0 / 240.0)

# ── Save log ──────────────────────────────────────────────────────────────────
if log_rows:
    with open(LOG_PATH, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s","x","y","z","distance_m","speed_ms",
                    "roll_deg","pitch_deg","contact_beta"])
        w.writerows(log_rows)
    print(f"\nGait log: {LOG_PATH}")
