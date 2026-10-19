# Architecture

## System

![System architecture](media/diagrams/system_architecture.svg)

The robot has two layers, as V Source designed it. The **ESP32 chassis**
drives 24 servos through PWM. The **Raspberry Pi 5 payload** runs the
gait and the learned policy and serves the operator. The simulation
stands in for both layers: `controller.py` runs the same gait code and
serves the same WebSocket protocol on the same port. The dashboard,
the drive pad and any test client therefore cannot tell the simulation
from the robot. In Phase 2 the gait and policy move onto the Pi 5
unchanged.

## Simulation

![Simulation pipeline](media/diagrams/simulation_pipeline.svg)

* `cad/extract_cad.py` reads Inventor 2025 files directly. Each data
  segment is Zstandard-compressed; part shapes and circular edges come
  from the ACIS solid inside; assembly placements are sparse 4x4
  matrices. It writes `cad/huntsman_cad.json`.
* `spider.py` builds the MuJoCo model. Each part gets a convex shape for
  contact and a textured "skin" (its real outline and holes) for looks.
  Joint axes come from the servo horn frames, masses from the weight
  sheet, and servo torque limits from the DSC55MG rating.
* `cad/step.py` turns STEP files (the battery) into meshes, including
  their curved surfaces.

## Gait

![Gait timing](media/diagrams/gait_timing.svg)

`gait.py` moves feet, not joints. A planted foot travels backwards at
exactly the body's speed (so it does not slip). A lifted foot follows a
smooth arc to half a stride ahead of its home position. Damped
least-squares inverse kinematics on the CAD leg geometry gives the three
joint angles. The ripple always runs from each side's leading end, so
neighbouring legs move the same way and never meet.

![Leg map](media/diagrams/leg_map.svg)

## Learning

![Residual RL](media/diagrams/residual_rl.svg)

See [ADR 0005](decisions/0005-residual-learning.md). The observation
contains only what the real robot can measure (joint angles and speeds,
IMU, command, gait phase, last action), so the trained policy can run
on the Pi 5.
