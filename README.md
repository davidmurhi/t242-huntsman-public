# T242 — Project Huntsman
**IFB398 IT Capstone Phase 1 | QUT 2026 Semester 2**

![Tests](https://github.com/davidmurhi/T242-Project-Huntsman/actions/workflows/ci.yml/badge.svg)

## Project Overview

Project Huntsman is an 8-legged spider robot developed by V Source Development, designed for deployment in post-disaster search and rescue environments. The robot has 24 servo motors (3 per leg) controlled by an ESP32, with a Raspberry Pi 5 as the onboard compute and ML inference platform.

Our Phase 1 work covers the ML-based gait control system — teaching the robot to walk across unpredictable terrain using reinforcement learning — along with the operator interface and hardware integration layer.

## Team T242

| Name | Role | GitHub |
|------|------|--------|
| David Murhi | Team Lead / Project Manager / Simulation | @davidmurhi |
| Tom Huynh | Frontend / Operator Dashboard | @tomotthyy |
| Kai Langley | ML / Simulation | @kai12345689 |
| Prabhanjan Muthukannan | Hardware / Embedded Systems | @Prab-QUT |

## Industry Partner

Craig Schneider — Founder and CTO, V Source Development  
craig@vsource.dev | www.vsource.dev

---

## Tech Stack

| Layer | Technology |
|---|---|
| Physics simulation | MuJoCo 3.x (MJCF XML) |
| RL environment | Gymnasium |
| RL algorithm | PPO via Stable Baselines 3 |
| Onboard compute | Raspberry Pi 5 |
| Motor control | ESP32 + PWM (24 servos) |
| Operator interface | SvelteKit |
| Embedded firmware | C/C++ (ESP32) |

---

## Repository Structure

```
/simulation
  /capstone spider robot   — PyBullet URDF prototypes (v1–v6, Sprints 2–4, Kai)
  /current spider robot    — current PyBullet environment
  /mujoco                  — MuJoCo rebuild from hardware first principles (Sprint 5, David)
    /mjcf                  — MJCF model files
      huntsman_body.xml    ✅ Step 1: body geometry + physics (validated)
      huntsman_l1.xml      ✅ Step 2: body + L1 leg (validated)
    /viewer                — interactive viewer scripts
      huntsman_body.py     — view body (SPACE/R/Q)
      huntsman_l1.py       — view + control L1 leg (H/K J/U N/M)
    /tests                 — unit tests (pytest)
      test_simulation.py   — 12 tests: mass, geometry, physics stability
  /rl                      — RL training scripts (updated paths)
    train_ppo.py           — PPO training (Stable Baselines 3)
    run_policy.py          — load + run trained policy
  spider_env.py            — Gymnasium wrapper (Kai)

/.github/workflows/ci.yml  — GitHub Actions: auto-runs tests on push

/docs
  /sprints                 — Sprint 1–6 plans and retrospectives
  /hardware                — Hardware spec (confirmed with Craig)
  /design                  — Simulation design decisions and rationale

/dashboard                 — SvelteKit operator interface (Tom)
/firmware                  — Pi 5 + ESP32 firmware (Prab)
/research                  — Research notes and references
/frontend                  — SvelteKit project root
```

---

## Hardware Summary

- **Robot mass:** ~1.207 kg total
- **Legs:** 8 × 3 DOF (hip yaw + knee pitch + ankle pitch) = 24 servos
- **Control chain:** Pi 5 → UART → ESP32 → 24× PWM servo channels
- **Battery:** 3S LiPo 2200mAh (~20 min runtime at full load)

Full spec: [`docs/hardware/hardware_spec.md`](docs/hardware/hardware_spec.md)

---

## Simulation Build Plan (MuJoCo)

The MuJoCo simulation is built one validated step at a time from Craig's confirmed hardware dimensions.

| Step | File | Status |
|---|---|---|
| 1 | `huntsman_body.xml` | ✅ Body geometry, mass, inertia validated |
| 2 | `huntsman_l1.xml` | ✅ Body + L1 leg, joint geometry, foot sensor validated |
| 3 | `huntsman_full.xml` | 🔄 All 8 legs — in progress |
| 4 | Gait controller | ⬜ Wave gait (1 swing, 7 stance) |
| 5 | Gymnasium wrapper | ⬜ Full RL interface for new model |
| 6 | PPO training | ⬜ Phase 2 |

Design rationale: [`docs/design/simulation_design.md`](docs/design/simulation_design.md)

---

## Quick Start

```bash
pip install mujoco numpy pytest

# View body (Step 1)
mjpython simulation/mujoco/viewer/huntsman_body.py

# View + control L1 leg (Step 2)
# Controls: H/K=hip  J/U=knee  N/M=ankle  SPACE=pause  R=reset  Q=quit
mjpython simulation/mujoco/viewer/huntsman_l1.py

# Run tests
python -m pytest simulation/mujoco/tests/ -v
```

---

## Sprint History

See [`docs/sprints/sprint_plan.md`](docs/sprints/sprint_plan.md) for full sprint plans and retrospectives.

| Sprint | Weeks | Key output |
|---|---|---|
| 1 | Wk 1–2 | Project setup, Craig kickoff, roles confirmed |
| 2 | Wk 3–4 | First PyBullet prototype, hardware scoped |
| 3 | Wk 5–6 | Gymnasium wrapper, PPO pipeline (Kai) |
| 4 | Wk 7–8 | He et al. contact-ratio feedback, obstacle testing |
| 5 | Wk 9–10 | MuJoCo rebuild from hardware first principles (David) |
| 6 | Wk 11–12 | Full 8-leg model, A2 presentation (Week 12) |

---

## Branch Strategy

- `main` — stable only
- `dev` — active development
- `simulation` — RL and simulation work
- `dashboard` — SvelteKit dashboard
- `firmware` — Pi 5 and ESP32 firmware
