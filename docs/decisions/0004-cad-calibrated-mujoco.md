# 0004: Build the simulation from V Source's CAD in MuJoCo

**Status:** Accepted in Sprint 4 (team decision, 19 Oct 2026).
**Owner:** David (simulation), with Kai and Prab.

## Context

Until mid-October the robot's dimensions were placeholders. We told our
tutor that no trained policy would transfer meaningfully until Craig's
measurements arrived (Week 11 summary). On 15 Oct V Source sent the
Inventor CAD and part-weight sheet. The real robot differs from the
prototype in ways that matter for a learned controller:

* it weighs 4.33 kg, with a heavy 18 V battery at one end
* it uses 55 kg-cm servos with real torque limits
* its legs are fanned at different angles, with very little clearance
  between neighbours

A policy only transfers if it is trained on a model that matches the
hardware.

## Options

1. Hand-edit Kai's URDF to the new dimensions in PyBullet.
2. Generate the model from the CAD files themselves, in MuJoCo, reusing
   the Sprint 3 evaluation tooling.

## Decision

Option 2. `simulation/mujoco` reads V Source's Inventor files directly
(no Inventor licence), builds every part's real shape, joint axis and
mass, and checks itself against the weight sheet. It is the team's
simulation for Phase 2. Kai's PyBullet spider stays in the repository as
the Phase 1 prototype and as the record of how the RL pipeline was
proven.

## Why MuJoCo for this

* It is the standard engine in current legged-locomotion research, with
  accurate contact and fast headless stepping for training.
* The Sprint 3 harness (stability, contact, servo load) was already
  built for it.
* It runs everywhere from one `pip install`, so it avoids the toolchain
  problems in [0002](0002-pybullet-arm64.md).

## Consequences

* The simulation now produces findings V Source can act on. For
  example, the two middle legs on each side have about 6 mm of
  clearance in the CAD pose.
* CI re-extracts the CAD on every push, so the model always traces back
  to the partner's files.
* Kai's training lessons (episode length, parallel environments,
  normalisation) carried straight into `huntsman_env.py`.
