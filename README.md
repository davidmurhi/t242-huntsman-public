# T242 — Project Huntsman
**IFB398 IT Capstone Phase 1 | QUT 2026 Semester 2**

## Project Overview
Project Huntsman is an 8-legged spider robot developed by V Source Development, designed for deployment in post-disaster search and rescue environments. The robot has 24 servo motors controlled by ESP32 microcontrollers, with a Raspberry Pi 5 as the onboard payload controller.

Our work is to build the ML-based gait control system — teaching the robot to walk across unpredictable terrain using reinforcement learning — along with the operator interface and hardware integration layer.

## Team T242
| Name | Role | GitHub |
|------|------|--------|
| David Murhi | Team Lead / Project Manager | @davidmurhi |
| Tom Huynh | Frontend / Operator Interface | @tomotthyy |
| Kai Langley | ML / Simulation | @kai12345689 |
| Prabhanjan Muthukannan | Hardware / Embedded Systems | @Prab-QUT |

## Industry Partner
Craig Schneider — Founder and CTO, V Source Development  
craig@vsource.dev | www.vsource.dev

## Tech Stack
- **Python** — ML training, gait control policy, Pi 5 control layer
- **PyBullet + Gymnasium** — physics simulation and RL training environment
- **PPO (Proximal Policy Optimization)** — RL training algorithm
- **Raspberry Pi 5** — onboard payload controller, runs trained ML policy
- **ESP32 + PWM** — motor control, 24 servo motors across 8 legs
- **SvelteKit** — operator dashboard and monitoring interface
- **C/C++** — ESP32 embedded firmware

## Repository Structure
```
/simulation    — RL environment, training scripts, gait policy models
/firmware      — Pi 5 Python control scripts, ESP32 C/C++ firmware
/dashboard     — SvelteKit operator interface
/docs          — architecture decisions, meeting notes, research summaries
/research      — notes, references, links to papers and resources
```

## Branch Strategy
- `main` — stable code only, no direct pushes
- `dev` — active development, all feature branches merge here
- `simulation` — Kai's RL and simulation work
- `dashboard` — Tom's SvelteKit dashboard work
- `firmware` — Prab's Pi 5 and ESP32 work

## Status
**Phase 1 — Week 9 | Active Development**

Phase 1 focus: simulation environment, basic gait training, dashboard prototype, hardware communication proof of concept.  
Phase 2 (Semester 1 2027): deploy trained policy to physical robot, full swarm functionality.
