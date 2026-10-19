# Risks, ethics and governance

Reviewed each sprint. Likelihood (L) and impact (I) are rated 1-3; their
product sets priority.

## Risk register

| # | Risk | L | I | Mitigation | Owner | Status |
|---|---|---|---|---|---|---|
| R1 | Simulation does not match the real robot, so a trained policy fails on hardware (sim-to-real gap) | 3 | 3 | model built from V Source's CAD and weight sheet; domain randomisation (friction, mass, servo strength, sensor noise); P2-S2 calibration against the real robot | David, Kai | Open: main Phase 2 focus |
| R2 | Partner information arrives late | 3 | 2 | placeholders clearly marked; work structured to swap in real values; regular Discord follow-ups | David | Realised in Phase 1 (CAD came 15 Oct); now low |
| R3 | Toolchain blocks team members (e.g. PyBullet on Apple Silicon) | 2 | 2 | pinned requirements; CI runs on Linux; `pybullet-arm64`; MuJoCo installs everywhere | David | Closed ([ADR 0002](../decisions/0002-pybullet-arm64.md)) |
| R4 | Training needs more compute than laptops | 3 | 2 | V Source compute confirmed; requirements sent (GPU machine, memory for headless simulation, storage for checkpoints); residual learning cuts the steps needed | David | Open: access set-up in P2-S1 |
| R5 | Servo command order on hardware differs from the simulation, so the robot moves wrongly with no error | 2 | 3 | single leg/joint order shared by `gait.py`, the model and the action space ([leg map](../media/diagrams/leg_map.svg)); P2-S1 one-servo-at-a-time check | Prab | Open |
| R6 | Robot damages itself or hurts someone during hardware tests | 2 | 3 | joint limits enforced in code; e-stop and dead-man; first tests with the robot on a stand; low gear and speed limits | Prab, David | Open |
| R7 | Integration drift between dashboard, simulation and hardware | 2 | 2 | one documented WebSocket protocol; CI builds the dashboard; telemetry field names fixed | Tom | Mitigated |
| R8 | Single points of knowledge (one person understands a component) | 2 | 2 | code owners plus a reviewer from another area on every PR; READMEs per component; pairing in Phase 2 | David | Open |
| R9 | Scope creep into V Source's wider platform (mesh, swarm, sensing) | 2 | 2 | scope table in the [definition](definition.md); changes agreed with Craig at sprint reviews | David | Mitigated |
| R10 | Team availability over the break between phases | 2 | 2 | Phase 2 backlog ready before the break; Sprint P2-S1 is hardware bring-up, which needs little ramp-up | David | Open |

## Ethics

* **Dual use.** "Defence grade" was the first question we raised with
  Craig. It means robustness ("built not to fail"); the robot is not a
  weapon, and the primary use is search and rescue. Craig acknowledged
  that once designed, the technology can be copied by others. We
  recorded the issue, discussed it with our tutor (Week 7), and are
  comfortable proceeding. Our work is locomotion only, with no
  targeting or weapon-related capability.
* **Privacy.** Later payloads (cameras, microphones, phone connectivity)
  will capture data about people in distress. That is out of our scope,
  but the dashboard and telemetry are designed to carry robot state
  only. Any personal data would need consent, retention and access rules
  agreed with V Source before it is added.
* **Safety of people near the robot.** A 4.3 kg robot with 55 kg-cm
  servos can pinch and strike. See R6: e-stop, dead-man, limits and
  supervised tests.
* **Honest reporting.** Simulation results are reported as simulation
  results, with their limits stated (for example, the baseline stalls on
  steps), and are not presented as hardware performance.

## Governance and legal

* **Work Integrated Learning.** This is a WIL unit with an industry
  partner. Meetings with Craig are minuted (`docs/meetings/`), and
  decisions are recorded as ADRs.
* **Intellectual property and confidentiality.** CAD, photos and videos
  in `hardware/` are V Source's and are used for this project only. The
  repository is private; Craig has access. Team-written code and
  documentation are shared with V Source as the project deliverable.
* **Software licences.** The stack is open source (MuJoCo Apache-2.0,
  PyBullet zlib, Gymnasium MIT, Stable-Baselines3 MIT, SvelteKit MIT).
  No paid tools are used.
* **Remote work and on-premises requirement.** Confirmed by Craig as 100%
  remote, with one visit to see and collect the robot.
* **External compute.** Training on V Source infrastructure. We asked
  the teaching team whether QUT needs to be aware of external compute
  (Week 7 and Week 11 summaries).
