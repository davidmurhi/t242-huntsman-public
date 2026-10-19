# Project definition

**P647 - Project Huntsman** | Industry partner: V Source Development
(Craig Schneider, Founder and CTO) | Team T242, IFB398 Phase 1, 2026
Semester 2

## Context

After a natural disaster the first 72 hours decide who is found alive.
They are also when rubble, collapsed structures and lost communications
make it most dangerous for rescuers. V Source is building **Huntsman**, a
platform of eight-legged robots for human-denied search and rescue. In
Craig's description, a swarm of 20-30 robots walks into a disaster zone,
searches for survivors, settles beside one, sets off an EPIRB, and
provides phone connectivity through a self-sustaining mesh network.

Wheels and tracks struggle on rubble, mud and broken ground. Legs do
not, but only if the gait adapts to the ground. V Source's current
controller replays simple leg sequences that work on flat ground and
fail on slopes, steps and rocks.

## The problem this project solves

> Teach the Huntsman to walk reliably across unpredictable terrain with a
> learned (machine-learning) gait controller instead of hand-tuned
> movement, and integrate it with the robot's onboard control stack.

## Stakeholders and users

| Who | Interest | How we engage |
|---|---|---|
| V Source Development (Craig; engineer Ronan) | a working ML gait on their platform; reusable code; findings about their design | Discord (daily), fortnightly meetings, GitHub access (`@Virologyst`) |
| Rescue operators (end users) | a robot that goes where it is pointed and reports what it sees | represented by the operator dashboard requirements (e-stop, status, manual control) |
| People trapped in disasters (beneficiaries) | faster location, connectivity | safety and reliability requirements ("defence grade" = built not to fail) |
| QUT teaching team (Sarah; David Lovell) | evidence of the unit learning outcomes | fortnightly tutor meetings and written summaries |
| Team T242 | a meaningful capstone; skills in ML, embedded and web | roles and processes in [way of working](../agile/way_of_working.md) |

## Objectives

1. **Simulation** that matches the real robot closely enough for
   learned behaviour to transfer.
2. **A walking controller**: a robust hand-written baseline and a
   learned policy that beats it on rough terrain.
3. **Integration path** to hardware: Raspberry Pi 5 payload, then ESP32,
   then PWM servos.
4. **Operator interface** to drive the robot and monitor it.
5. **Reproducible, documented code** and a technical report for V
   Source.

## Scope

| In scope | Out of scope (V Source's or later) |
|---|---|
| gait control (simulation, RL training, deployment to the Pi 5) | mechanical design and electronics (V Source) |
| the simulation and its terrains | Arduino to ESP32 port and the 6 V to 8 V rail change (Craig) |
| Pi 5 to ESP32 command interface and servo mapping | mesh networking and swarm coordination (later phase) |
| operator dashboard | survivor detection, microphone-array localisation (V Source research) |

## Requirements

Requirements are tracked as user stories in the [product
backlog](../agile/backlog.md). The key ones:

**Functional**

* F1: the robot walks forward, backward, sideways and turns on command.
* F2: it holds its heading without operator correction.
* F3: it crosses slopes, steps and rubble typical of disaster sites.
* F4: an operator can drive it, change gait and trigger an e-stop from
  the dashboard; a dropped connection stops it.
* F5: telemetry (orientation, speed, feet down, status) reaches the
  dashboard live.
* F6: the trained policy runs on the Pi 5 and commands the ESP32.

**Non-functional**

* N1 (transfer): the simulation is calibrated to the real robot's
  geometry, masses and servo limits, and the policy is trained with
  randomisation for the differences that remain.
* N2 (safety): servo targets stay within joint limits; an e-stop
  freezes every servo; a dead-man stops the robot if commands stop.
* N3 (reliability, "defence grade"): repeatable results, tested in CI;
  a hand-written gait fallback.
* N4 (reproducibility): one command builds the model from V Source's CAD;
  training and evaluation are scripted.
* N5 (maintainability): documented decisions, tests, code ownership.

## Deliverables (from the brief)

1. Trained ML model(s) for gait generation and adaptation across varied
   terrain.
2. Reproducible training and evaluation code (Python).
3. Firmware and control integration: ESP32 PWM control through the Pi 5.
4. A working prototype demonstrated on at least one real platform.
5. A short technical report on the approach, results and limitations,
   and a final demonstration to V Source.

Progress against each is tracked in the [Phase 1 sprint
record](../sprints/phase1.md) and planned in the [Phase 2
plan](../sprints/phase2_plan.md).

## Success measures

These are measured in the simulation now and on the robot in Phase 2,
using the same scripts.

| Measure | Baseline gait (Oct 2026) | Phase 2 target |
|---|---|---|
| Flat ground: straight-line error over 3 m | about 1 mm | keep |
| Leg-to-leg contacts while walking | 0 | 0 |
| 15 deg slope, 30 s | 3.5 m (full course) | keep |
| 50 mm steps, 30 s | 0.8 m (stalls at first step) | over 3 m |
| 80 mm rubble, 30 s | 0.6 m (snags, veers 27 deg) | over 3 m, under 10 deg heading drift |
| Same behaviour on the real robot | n/a | within 20% of simulation on flat and slope |
