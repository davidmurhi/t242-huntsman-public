# 0001: Simulation and RL stack for Phase 1

**Status:** Accepted in Sprint 1 (team meetings 5 and 8 Sep 2026).
Partly superseded by [0004](0004-cad-calibrated-mujoco.md), which moved
the robot model to MuJoCo for Phase 2.
**Owner:** Kai (ML and simulation), with the whole team.

## Context

At the first meeting with V Source (2 Sep), Craig asked us to come back
knowing how we would *"put a 3D block spider into a space and throw
obstacles at it"*. He also needed to know which simulation package and ML
libraries we would use. There was no CAD yet, only a verbal description:
a block body, eight legs, one horizontal and two vertical joints per leg,
and 24 servos. Training would eventually run on V Source's compute, not
on laptops.

## Options

| Option | For | Against |
|---|---|---|
| PyBullet | free, Python-first, URDF loading, GUI and headless modes, many locomotion examples | upstream release ageing |
| MuJoCo | accurate contact, fast, standard in current locomotion research | team less familiar; model format (MJCF) new to us |
| Isaac Gym / Isaac Lab | massive GPU parallelism | needs an NVIDIA GPU; heavy set-up for a first prototype |

The RL side was not in doubt. Gymnasium is the standard environment
interface, so any algorithm can plug in. Stable-Baselines3 PPO is the
standard, stable starting point for continuous control.

## Decision

Use **PyBullet** for the first robot model, wrapped in a **Gymnasium**
environment, and train with **Stable-Baselines3 PPO**.

## Consequences

* Kai had a walking 24-joint spider (v1-v5) within a week, and Prab had
  the environment skeleton around it.
* We later learned the cost of the ageing PyBullet release: it would not
  build on Apple Silicon with Python 3.13 ([0002](0002-pybullet-arm64.md)).
* Gymnasium and SB3 carried over unchanged to the MuJoCo environment, so
  none of the RL work was lost when the robot model moved.
