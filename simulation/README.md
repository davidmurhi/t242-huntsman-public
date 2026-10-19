# Simulation

Two simulations of the Huntsman live here. They are the two stages of
Phase 1.

| | [`pybullet/`](pybullet/README.md): Phase 1 prototype | [`mujoco/`](mujoco/README.md): the platform for Phase 2 |
|---|---|---|
| | <img src="../docs/media/pybullet/pybullet_walk.gif" width="100%" alt="PyBullet spider walking"> | <img src="mujoco/media/huntsman_walk.gif" width="100%" alt="CAD-built spider walking"> |
| Built by | Kai (robot, gait, PPO), Prab (Gymnasium env, telemetry, PWM) | David (CAD import, model, gait, controller, terrains, RL env) |
| Robot model | hand-written URDF from Craig's description (Sep) | generated from V Source's Inventor CAD (Oct): every part, joint axis and mass |
| Walking | wave gait on joint angles; keyboard driving, 7 gears | foot-path gait with inverse kinematics; 4 gaits, any direction, heading hold |
| Learning | PPO from scratch: first moving policy at 2 M steps | residual PPO on top of the gait; terrains, randomisation, curriculum |
| Engine | PyBullet (`pybullet-arm64` on Apple Silicon) | MuJoCo 3 |
| Run it | `python spider8_v6.py` | `python spider.py --save && python controller.py` |

The prototype proved the pipeline end to end (robot, gait, environment,
PPO, dashboard telemetry) before any CAD existed. When V Source's CAD
arrived, the team rebuilt the robot from it so a trained policy can
transfer to the real hardware
([ADR 0004](../docs/decisions/0004-cad-calibrated-mujoco.md)). Both share
the same leg names, joint order and dashboard protocol.

[`mujoco-evaluation/`](mujoco-evaluation/README.md) is the Sprint 3
framework evaluation and diagnostic harness that led there
([ADR 0002](../docs/decisions/0002-pybullet-arm64.md)).
