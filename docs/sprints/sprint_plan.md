# Project Huntsman — Sprint Plan
**T242 | IFB398 26se2 | V Source Development**

Kanban board: https://github.com/users/davidmurhi/projects/3

---

## Sprint Overview

2-week sprints aligned with fortnightly tutor check-ins.

| Sprint | Weeks | Goal | Status |
|---|---|---|---|
| Sprint 1 | Wk 1–2 | Project setup, Craig kickoff meeting, roles confirmed | Done |
| Sprint 2 | Wk 3–4 | Hardware scoping, first PyBullet prototype | Done |
| Sprint 3 | Wk 5–6 | Gymnasium wrapper, PPO pipeline, first training run | Done |
| Sprint 4 | Wk 7–8 | Contact-ratio feedback, obstacle testing, A1.2 Week 7 | Done |
| Sprint 5 | Wk 9–10 | MuJoCo simulation rebuild from hardware first principles | Done |
| Sprint 6 | Wk 11–12 | Full 8-leg model, gait controller, A2 presentation | In Progress |

---

## Sprint 1 — Wk 1–2: Setup

**Goal:** Align team, confirm scope with Craig, set up tooling.

- [x] Kickoff meeting with Craig — confirmed hardware stack (Pi 5 → ESP32 → 24 servos)
- [x] GitHub repo and Kanban board created
- [x] Roles assigned: David (PM), Kai (RL/Sim), Tom (Dashboard), Prab (Hardware)
- [x] Tech stack agreed: PyBullet + Gymnasium + SB3 for RL; SvelteKit dashboard
- [x] A1.1 submitted

**Retro:** Craig confirmed the hardware constraints that drive all our simulation geometry: 3 DOF per leg, MG90S servos, 3S LiPo power, total mass ~1.2kg.

---

## Sprint 2 — Wk 3–4: First Prototype

**Goal:** First working simulation; document hardware precisely.

- [x] Hardware component list and mass budget documented
- [x] First PyBullet URDF prototype — spider v1/v2
- [x] Basic leg structure from Craig's CAD sketches
- [x] `spider_env.py` Gymnasium wrapper skeleton (Kai)
- [x] SvelteKit dashboard project initialised (Tom)

**Retro:** Prototype worked structurally but geometry proportions were eyeballed — not derived from real hardware. Flagged for Sprint 5 rework.

---

## Sprint 3 — Wk 5–6: RL Pipeline

**Goal:** End-to-end PPO training working.

**Owner:** Kai

- [x] Complete Gymnasium wrapper with observation and action spaces
- [x] Observation: 24 joint positions + 24 velocities + 8 touch sensors + 6D pose
- [x] Action: 24 normalised servo targets
- [x] `train_ppo.py` — PPO via Stable Baselines 3
- [x] `run_policy.py` — load and display trained policy in viewer
- [x] First 500k-step training run — basic forward motion achieved

**Retro:** Policy converges but gait is unstable — frequent tipping. Missing: contact-ratio reward shaping.

---

## Sprint 4 — Wk 7–8: Contact-Ratio Feedback + A1.2 Wk7

**Goal:** Improve locomotion stability using published research.

- [x] He et al. 2024 (arxiv:2409.09473) integrated — contact-ratio reward
- [x] Reward: `R = v_forward - 0.6*|v_lateral|`, bonus `0.1*(feet_grounded/8)`
- [x] 8 foot touch sensors added to MJCF
- [x] Staggered obstacle course added to environment
- [x] 1M-step retraining — improved straight-line stability
- [x] A1.2 Week 7 submission (team report)
- [x] Week 7 tutor meeting

**Retro:** Obstacle traversal still unreliable. Root cause identified: leg geometry not grounded in real hardware specs → foot placement off-centre → lateral instability that RL can't overcome. Decision: rebuild simulation from scratch in Sprint 5.

---

## Sprint 5 — Wk 9–10: MuJoCo Rebuild

**Goal:** Rebuild simulation from Craig's confirmed hardware specs, one validated step at a time.

**Owner:** David

- [x] Switch to MuJoCo (better contact physics, matches SOTA RL papers)
- [x] Leg geometry derived mathematically from hardware dimensions
  - Coxa 80mm at 30° forward — front-leg reach without body collision
  - Femur 140mm at 10° outward lean — natural wide stable stance
  - Tibia 160mm at ~16° inward — foot under CoM for lateral stability
- [x] Step 1: `huntsman_body.xml` — body only, mass=0.447kg, validated
  - Settles at z=0.035m, zero tilt, no interpenetration
- [x] Step 2: `huntsman_l1.xml` — body + L1 leg, foot sensor
  - Joint geometry correct, touch sensor working, physics stable 500+ steps
- [x] Unit tests (`test_simulation.py`) — 12 tests, all pass
- [x] GitHub Actions CI — auto-runs tests on push
- [x] A1.2 Week 9 submission

**Retro:** New model validates correctly. This is the authoritative simulation going into Phase 2. Kai's PyBullet/Gymnasium work remains the semester's main RL output and will be featured in the A2 presentation.

---

## Sprint 6 — Wk 11–12: Full Model + A2

**Goal:** Complete full 8-leg model; deliver A2 presentation (Week 12).

- [ ] Step 3: `huntsman_full.xml` — all 8 legs, symmetric layout
- [ ] Static stability test — robot stands without falling
- [ ] Step 4: Wave gait controller — 1 leg swing, 7 in stance
- [ ] A2 presentation slides (Definition / Plans / Outputs / Operation)
- [ ] Synopsis PDF — due night before Week 12 presentation
- [ ] Team rehearsal (recorded practice run)
- [ ] Final repo cleanup — tests passing, README current

---

## GitHub Labels

| Label | Meaning |
|---|---|
| `simulation` | MuJoCo model work |
| `rl` | Reinforcement learning |
| `hardware` | Physical robot / firmware |
| `dashboard` | SvelteKit operator interface |
| `docs` | Documentation |
| `testing` | Test coverage |
| `presentation` | A2 artefacts |
