# Simulation Stack — Project Huntsman

**T242 | IFB398 26se2 | Updated: 9 Sep 2026**

## What we are using and why

### PyBullet
Physics simulation engine. Simulates rigid body dynamics — gravity, joints, friction — closely enough for sim-to-real transfer. We use it to build and test the spider robot model before touching the physical hardware.

- Two modes: `p.GUI` (live visual window, good for debugging) and `p.DIRECT` (headless, no rendering, used for RL training)
- Install: `pip install pybullet`
- Docs: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA

### Gymnasium
Standard Python library for building RL training environments. Provides a standardised API — `env.reset()`, `env.step(action)` — so any environment we build can plug into any training algorithm.

- Install: `pip install gymnasium`
- Docs: https://gymnasium.farama.org

### Stable Baselines 3 (PPO)
RL training library with PPO built in. PPO (Proximal Policy Optimization) is the standard algorithm for robot locomotion — makes stable policy updates through a clipping mechanism.

- Install: `pip install stable-baselines3`
- Docs: https://stable-baselines3.readthedocs.io

## The spider model

- Square body in the centre
- 8 legs — 4 pairs (l1/r1, l2/r2, l3/r3, l4/r4)
- 3 joints per leg: hip (horizontal, forward/back), knee (vertical, up/down), ankle (vertical, up/down)
- 24 motors total
- Built from Craig's shape description — URDF files in `simulation/pybullet/` (history in `simulation/pybullet/versions/`)

## Current status (Week 9)

| Component | Status |
|-----------|--------|
| PyBullet setup | Done — tested in GUI and DIRECT modes |
| Spider URDF model | Done — v1 through v5 in /simulation |
| Wave gait in PyBullet | Done — spider walks straight in v5 |
| Gymnasium integration | Not started — next step |
| PPO training loop | Not started — pending Gymnasium integration |
| Robot dimensions from Craig | Pending — awaiting Craig's details |
| Cloud compute access | Pending — awaiting Craig's details |

## Next steps

1. Wire PyBullet spider environment into Gymnasium (env.reset / env.step)
2. Get actual robot dimensions from Craig
3. Implement PPO training loop with Stable Baselines 3
4. Run first training on Craig's cloud compute
