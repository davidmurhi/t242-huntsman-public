"""
huntsman_env.py

The reinforcement-learning environment for the Huntsman: V Source's CAD
robot (spider.xml) on the terrains in terrain.py, wrapped as a
Gymnasium environment so Stable-Baselines3 (or any RL library) can
train on it.

HOW THE ROBOT LEARNS (the short version)
    The policy is a small neural network. Fifty times a second it sees
    the robot's state (below) and outputs 24 numbers, one per servo.
    Those numbers are *corrections* added on top of our hand-written
    gait (gait.py): the gait already walks on flat ground, so the network
    only has to learn what to change when the ground is not flat - lift
    a foot higher over a rock, lean into a slope, shorten a step at a
    stair edge. This is called residual learning; it trains far faster
    than learning to walk from nothing, and the robot never starts from
    flailing.

    After every step it gets a reward: points for moving at the speed it
    was asked to, holding its heading and staying level; small penalties
    for wasting effort or jerking the servos. An episode ends after 20 s,
    or early if it tips past 50 degrees. PPO, the training algorithm,
    nudges the network towards whatever earned more reward. Millions of
    steps later it has learned what works.

WHAT THE POLICY SEES (observation, 83 numbers)
    24 joint angles and 24 joint speeds, which way is down (3), how fast
    the body is turning (3) and moving (3), the drive command (3), where
    the gait is in its cycle (2), and its own last action (24).
    Everything is something the real robot can measure: servo positions
    from the servos, the rest from the IMU.

WHAT IT CONTROLS (action, 24 numbers in -1..1)
    A correction of up to +/- 0.35 rad (20 deg) per servo, added to the
    gait's target for that servo.

MAKING IT TRANSFER TO THE REAL ROBOT (domain randomisation)
    Each episode draws a new terrain and seed, and varies floor
    friction (0.6-1.6), payload mass (+/- 25 %), servo strength (85-110 %)
    and adds sensor noise, so the policy cannot rely on the simulation
    being exactly right.

    from huntsman_env import HuntsmanEnv
    env = HuntsmanEnv(terrains=("flat", "rubble", "slope", "steps"))
"""

from __future__ import annotations

import copy
import math
import os

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:                       # lets the file import without it
    gym, spaces = None, None

import mujoco

import gait as gaitlib
import terrain as terrainlib

HERE = os.path.dirname(os.path.abspath(__file__))

CONTROL_HZ = 50
EPISODE_S = 20.0
ACTION_SCALE = 0.35          # rad of correction at action = +/-1
TILT_LIMIT = 50.0            # deg: tipped further than this ends the episode

# Reward weights. Tracking the command is what matters; the rest shape
# how it gets there.
W_SPEED = 2.0
W_TURN = 0.6
W_UPRIGHT = 0.5
W_ALIVE = 0.2
W_EFFORT = 0.002
W_SMOOTH = 0.05
W_FALL = 10.0

_Base = gym.Env if gym else object


class HuntsmanEnv(_Base):
    metadata = {"render_modes": []}

    def __init__(self, terrains=("flat", "rubble", "slope", "steps", "mixed"),
                 difficulty=1.0, command=(0.12, 0.0, 0.0), gait="rubble",
                 randomise=True, seed=None):
        if gym is None:
            raise ImportError("pip install gymnasium")
        self.terrains = tuple(terrains)
        self.difficulty = difficulty
        self.command = np.array(command, float)       # vx m/s, vy m/s, wz deg/s
        self.gait_name = gait
        self.randomise = randomise
        self.rng = np.random.default_rng(seed)
        self.models = {}
        self._load("flat", 0)
        self._gait_proto = gaitlib.from_spider()   # leg geometry, built once

        self.nu = self.model.nu
        self.action_space = spaces.Box(-1.0, 1.0, (self.nu,), np.float32)
        n_obs = 24 + 24 + 3 + 3 + 3 + 3 + 2 + 24
        self.observation_space = spaces.Box(-np.inf, np.inf, (n_obs,),
                                            np.float32)
        self.per = max(1, round(1.0 / CONTROL_HZ / self.model.opt.timestep))

    # ------------------------------------------------------------ model
    def _load(self, kind, seed):
        """Load (and cache) the model for a terrain."""
        key = (kind, seed if kind != "flat" else 0, self.difficulty)
        if key not in self.models:
            xml = terrainlib.model_xml(kind, seed, difficulty=self.difficulty)
            path = os.path.join(HERE, f".env_{os.getpid()}_{kind}_{seed}.xml")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(xml)              # file paths (textures) resolve here
            m = mujoco.MjModel.from_xml_path(path)
            os.remove(path)
            # Keep the untouched values, so randomising never compounds.
            self.models[key] = (m, m.actuator_forcerange.copy(),
                                m.body_mass.copy(), m.geom_friction.copy())
            if len(self.models) > 12:      # keep memory bounded
                self.models.pop(next(iter(self.models)))
        (self.model, self.base_force, self.base_mass,
         self.base_friction) = self.models[key]
        self.data = mujoco.MjData(self.model)
        self.order = [mujoco.mj_name2id(self.model,
                                        mujoco.mjtObj.mjOBJ_ACTUATOR,
                                        f"{leg}_{j}")
                      for leg in gaitlib.LEGS for j in gaitlib.JOINTS]
        self.qadr = [self.model.jnt_qposadr[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{leg}_{j}")]
            for leg in gaitlib.LEGS for j in gaitlib.JOINTS]
        self.dadr = [self.model.jnt_dofadr[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{leg}_{j}")]
            for leg in gaitlib.LEGS for j in gaitlib.JOINTS]

    def _randomise(self):
        m, r = self.model, self.rng
        m.actuator_forcerange[:] = self.base_force * r.uniform(0.85, 1.10)
        m.body_mass[:] = self.base_mass
        m.body_mass[1] = self.base_mass[1] * r.uniform(0.75, 1.25)
        m.geom_friction[:] = self.base_friction
        m.geom_friction[:, 0] = self.base_friction[:, 0] * r.uniform(0.6, 1.1)

    # ------------------------------------------------------------ gym API
    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        kind = str(self.rng.choice(self.terrains))
        self._load(kind, int(self.rng.integers(0, 1000)))
        if self.randomise:
            self._randomise()
        mujoco.mj_resetData(self.model, self.data)
        for _ in range(200):                      # settle onto its feet
            mujoco.mj_step(self.model, self.data)
        self.gait = copy.deepcopy(self._gait_proto)
        self.gait.set_gait(self.gait_name)
        self.gait.heading_hold = False            # the policy must hold it
        self.gait.drive(*self.command)
        self.last_action = np.zeros(self.nu)
        self.t = 0.0
        self.start_yaw = self._yaw()
        self.terrain = kind
        return self._obs(), {"terrain": kind}

    def step(self, action):
        action = np.clip(np.asarray(action, float), -1.0, 1.0)
        base = self.gait.step(1.0 / CONTROL_HZ)
        target = base + ACTION_SCALE * action
        for k, a in enumerate(self.order):
            lo, hi = self.model.actuator_ctrlrange[a]
            self.data.ctrl[a] = np.clip(target[k], lo, hi)
        for _ in range(self.per):
            mujoco.mj_step(self.model, self.data)
        self.t += 1.0 / CONTROL_HZ

        reward, fell = self._reward(action)
        self.last_action = action
        terminated = fell
        truncated = self.t >= EPISODE_S
        info = {"terrain": self.terrain, "x": float(self.data.qpos[0])}
        return self._obs(), reward, terminated, truncated, info

    # ------------------------------------------------------------ pieces
    def _yaw(self):
        w, x, y, z = self.data.qpos[3:7]
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def _body_frame(self):
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, self.data.qpos[3:7])
        return R.reshape(3, 3)

    def _obs(self):
        d = self.data
        R = self._body_frame()
        q = d.qpos[self.qadr]
        dq = d.qvel[self.dadr]
        down = R.T @ np.array([0.0, 0.0, -1.0])
        lin = R.T @ d.qvel[0:3]
        ang = d.qvel[3:6]                        # freejoint: body frame
        cmd = self.command * np.array([1.0, 1.0, math.pi / 180])
        ph = 2 * math.pi * self.gait.phase
        obs = np.concatenate([q, 0.1 * dq, down, 0.25 * ang, lin, cmd,
                              [math.sin(ph), math.cos(ph)], self.last_action])
        if self.randomise:                       # sensor noise
            obs[:48] += self.rng.normal(0, 0.01, 48)
            obs[48:54] += self.rng.normal(0, 0.02, 6)
        return obs.astype(np.float32)

    def _reward(self, action):
        d = self.data
        R = self._body_frame()
        lin = R.T @ d.qvel[0:3]
        want_v = self.command[:2]
        want_w = math.radians(self.command[2])
        speed_err = float(np.sum((lin[:2] - want_v) ** 2))
        turn_err = (d.qvel[5] - want_w) ** 2
        up = R[2, 2]                              # 1 when level
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up))))
        effort = float(np.sum(np.square(d.actuator_force)))
        jerk = float(np.sum(np.square(action - self.last_action)))
        fell = tilt > TILT_LIMIT or d.qpos[2] < 0.02

        r = (W_SPEED * math.exp(-speed_err / 0.01)
             + W_TURN * math.exp(-turn_err / 0.1)
             + W_UPRIGHT * up
             + W_ALIVE
             - W_EFFORT * effort
             - W_SMOOTH * jerk)
        if fell:
            r -= W_FALL
        return float(r), fell


def make(terrains=("flat", "rubble", "slope", "steps", "mixed"), **kw):
    """A factory, for vectorised environments."""
    return lambda: HuntsmanEnv(terrains=terrains, **kw)
