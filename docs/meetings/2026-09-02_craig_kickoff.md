# First meeting with V Source Development

| | |
|---|---|
| Date | 2 September 2026, 11:30 am, Microsoft Teams (recorded) |
| Attendees | Craig Schneider (V Source), David Murhi, Tom Huynh, Kai Langley, Prab Muthukannan |
| Purpose | introductions and project overview |
| Minutes | David Murhi |

## About Craig

Founder and CTO of V Source Development, a QUT computer science
graduate, now finishing engineering (computer and software systems,
robotics). He is also researching tetrahedral localisation of drones
with microphone arrays and time of arrival. These arrays are planned for
the Huntsman later.

## The robot: corrections to the brief

* It is an **8-legged spider**, not a 4-legged quadruped as we had
  assumed from the brief.
* **24 servos**: one horizontal joint per leg (forward/back only) and two
  vertical joints per leg (up/down).
* It currently runs on Arduino with two banks of PWM controllers. Craig
  is moving it to ESP32 (same C code) and raising the voltage rail from
  6 V to 8 V. Both are Craig's tasks, not ours.

## How the system works

Two layers:

* **ESP32 (chassis):** moves the legs. It accepts high-level commands
  (move forward, turn left, climb). Craig's existing code only works on
  flat ground: it has no sensing, and it fails on slopes, steps and
  rocks.
* **Raspberry Pi 5 (payload):** connects to the ESP32 through an API,
  runs the ML policy and commands the chassis. This is where our work
  lives.

> "The Pi tells the API 'move from A to B, make it happen', and the ESP32
> figures out the legs."

## What we are building

* the ML gait controller: a policy trained in simulation to walk over
  any terrain
* the 3D simulation, from scratch (Craig can describe the shape: a block
  with eight legs and joints)
* new code throughout (the existing code just replays leg sequences)
* the software stack is our choice (Craig suggests Python for the ML)
* the operator interface (SvelteKit)

## Other points

* **On premises:** 100% remote. One visit to see and pick up the robot
  (Craig may bring it to QUT).
* **Hardware and compute:** Craig will give access to powerful hardware
  and cloud compute ("don't train on laptops"). Pi 5, ESP32 and PWM
  confirmed; mesh networking for the swarm.
* **"Defence grade":** means built not to fail, to the robustness
  standards defence uses. It does not mean weapons. The primary use is
  search and rescue: a swarm of 20-30 robots searches a disaster zone,
  sits beside survivors it finds, sets off an EPIRB, and gives people
  trapped underground phone connectivity. The ADF is interested
  logistically. Craig acknowledged that once designed, the technology
  can be copied by others.
* **Communication:** Discord (Craig always has it open). Fortnightly
  meetings in the off week from tutor meetings, after 6 pm, kept short
  with an agenda and minutes.

## What Craig wants by the next meeting

A plan for the ML environment:

* which simulation package or framework
* which ML libraries
* how we will put a 3D spider into a simulated environment and train it

It does not need to be implemented.

## Actions

| Action | Owner | Due |
|---|---|---|
| Discord server; invite Craig | Tom | done that day |
| Research ML simulation frameworks for an 8-legged spider (PyBullet, MuJoCo, Isaac Gym) | Kai, all | before next meeting |
| Update work logs and research log | all, David | this week |
| Confirm the fortnightly meeting time with Craig | David | this week |

**Outcome:** the stack decision in [ADR 0001](../decisions/0001-simulation-and-rl-stack.md).
