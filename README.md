# Project Huntsman: learned gait control for a search-and-rescue spider robot

[![CI](https://github.com/davidmurhi/T242-Project-Huntsman/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/davidmurhi/T242-Project-Huntsman/actions/workflows/ci.yml)

**Team T242 | QUT IFB398 IT Capstone, Phase 1 (2026 Semester 2) |
Industry partner: [V Source Development](https://www.vsource.dev)**

V Source is building **Huntsman**, eight-legged robots that walk into
disaster zones where wheels and tracks cannot go. They search for
survivors and keep rescuers informed. Their current controller walks
on flat ground only. Our project is the **walking gait control
system**: teach the robot to cross rubble, slopes and steps with
machine learning, and run that controller on the robot's Raspberry Pi 5
and ESP32 stack.

<p align="center">
<img src="docs/media/mujoco/huntsman_hero.jpg" width="49%" alt="The Huntsman simulated from V Source's CAD">
<img src="hardware/reference/photos/robot_1.jpg" width="49%" alt="The real Huntsman in V Source's lab">
<br><em>Left: our simulation, generated from V Source's CAD. Right: the robot in V Source's lab.</em>
</p>

## The two spiders

<table>
<tr>
<th width="50%">Phase 1 prototype: PyBullet (Kai)</th>
<th width="50%">CAD-calibrated platform: MuJoCo (David)</th>
</tr>
<tr>
<td><img src="docs/media/pybullet/pybullet_walk.gif" width="100%" alt="PyBullet spider walking and turning"></td>
<td><img src="simulation/mujoco/media/huntsman_walk.gif" width="100%" alt="CAD-built spider walking and turning"></td>
</tr>
<tr>
<td>Built from Craig's description before any CAD existed. It proved
the pipeline: 24-joint URDF, keyboard-driven wave gait, Prab's
Gymnasium environment, and a PPO policy that moves after 2 M steps.</td>
<td>Generated from V Source's Inventor CAD: every part, joint axis and
mass (4.326 kg). Foot-path gait with four modes. Drivable from keyboard,
drive pad or dashboard. Rough terrains and a residual-RL training
environment for Phase 2.</td>
</tr>
</table>

### Quick start: CAD-calibrated simulation

```bash
git clone https://github.com/davidmurhi/T242-Project-Huntsman.git
cd T242-Project-Huntsman/simulation/mujoco
python3 -m venv .venv && source .venv/bin/activate     # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python spider.py --save          # build the robot from the CAD; prints its checks
python controller.py             # drive it: arrows/WASD, Q/E sideways, 1-4 gaits, X e-stop
python controller.py --terrain rubble     # or slope, steps, mixed
```

The same window serves a drive pad at http://localhost:8765, and the
team's dashboard can connect on `ws://localhost:8765/ws`.

### Quick start: PyBullet prototype

```bash
cd T242-Project-Huntsman/simulation/pybullet
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt  # Apple Silicon + Python 3.13: pip install pybullet-arm64
python spider8_v6.py             # arrows/WASD to drive, E/Q gears, Space to stop
```

More in [`simulation/README.md`](simulation/README.md).

## Where it stands (end of Phase 1)

| | |
|---|---|
| Simulation | built from the partner's CAD, reproducible in CI, matches the weight sheet |
| Baseline gait | flat ground: straight to about 1 mm over 3 m, no leg contact; 15 deg slopes: yes; 50 mm steps and 80 mm rubble: stalls. This gap is the target for the learned controller |
| Learning | PPO pipeline proven on the prototype; CAD-model training environment with terrains, randomisation and curriculum ready |
| Hardware | Pi 5 to ESP32 serial link proven; servo PWM mapping written; robot photos, video and CAD received |
| Operator interface | SvelteKit console (mission control, map, diagnostics); live telemetry and drive protocol defined |
| Quality | 64 automated tests; CI on every push |

<p align="center"><img src="docs/media/mujoco/terrains.jpg" width="100%" alt="The robot on slope, steps and rubble"></p>

## Repository

```
simulation/
  mujoco/              CAD-built simulation, gait, controller, terrains, RL environment, tests
  pybullet/            Kai's prototype, Prab's Gymnasium env, telemetry bridge, PWM mapping, tests
  mujoco-evaluation/   Sprint 3 framework evaluation and diagnostic harness
dashboard/             SvelteKit operator console (Tom)
firmware/              Pi 5 and ESP32 integration notes (Prab)
hardware/              V Source's CAD, battery files, part weights, photos and video
docs/                  project definition, sprints, backlog, decisions, meetings, testing, diagrams
research/              research links and notes
tools/                 diagram and render generators, CI helpers
```

## Documentation

* **Definition:** [project definition](docs/project/definition.md),
  [risks, ethics and governance](docs/project/risks_and_ethics.md)
* **Plans:** [Phase 1 sprint record](docs/sprints/phase1.md),
  [Phase 2 plan](docs/sprints/phase2_plan.md),
  [backlog](docs/agile/backlog.md)
* **Outputs:** [architecture](docs/architecture.md),
  [decision records](docs/decisions/README.md),
  [testing](docs/testing.md), [hardware](hardware/README.md)
* **Operation:** [how we work](docs/agile/way_of_working.md),
  [meetings](docs/meetings/README.md),
  [contributing](CONTRIBUTING.md)

![System architecture](docs/media/diagrams/system_architecture.svg)

## Team T242

| | Role | Owns |
|---|---|---|
| David Murhi ([@davidmurhi](https://github.com/davidmurhi)) | Team lead, project manager | partner and tutor communication; CAD-calibrated simulation, gait and controller; repository and CI |
| Kai Langley ([@kai12345689](https://github.com/kai12345689)) | ML and simulation | PyBullet prototype, PPO training |
| Prabhanjan Muthukannan ([@Prab-QUT](https://github.com/Prab-QUT)) | Hardware and embedded | Pi 5 and ESP32, Gymnasium environment, telemetry bridge, servo mapping |
| Tom Huynh ([@tomotthyy](https://github.com/tomotthyy)) | Operator interface | SvelteKit dashboard, telemetry protocol |

**Industry partner:** Craig Schneider, Founder and CTO, V Source
Development (craig@vsource.dev). CAD and part data by V Source
engineer Ronan Donahoe.

The hardware files in `hardware/` belong to V Source Development and
are shared for this project only.
