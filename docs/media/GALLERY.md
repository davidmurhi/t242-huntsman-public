# Image gallery

Images the team produced in Phase 1. V Source's photos of the physical robot
and renders of its CAD are on our [Canvas group page](https://canvas.qut.edu.au/groups/225191/pages/project-huntsman-partner-evidence-teaching-team-only)
(teaching team only).

## The CAD-built MuJoCo model

| | |
|---|---|
| ![Huntsman model, three-quarter view](mujoco/huntsman_hero.jpg) | ![Huntsman model from the front](mujoco/huntsman_front.jpg) |
| Generated from V Source's Inventor CAD: every part, joint axis and mass (4.326 kg in total). | Front view. |
| ![Huntsman model from above](mujoco/huntsman_top.jpg) | ![Huntsman model from below](mujoco/huntsman_under.jpg) |
| From above: eight legs, three servos each. | From below. |
| ![Huntsman model from the side](mujoco/huntsman_leg.jpg) | ![The battery at the rear](mujoco/huntsman_battery.jpg) |
| Side view, showing the three servos on each leg. | Rear view with the Makita 18 V battery, which we identified from V Source's photos and modelled before the CAD arrived. |

## The test terrains

![Slope, steps and rubble](mujoco/terrains.jpg)

The terrains used to measure the hand-written baseline gait, with a 30 s
recording of each run:

| Terrain | Baseline gait, distance in 30 s | Recording |
|---|---|---|
| ![15 degree slope](mujoco/terrain_slope.jpg) 15° slope | 3.5 m | [video](video/terrain_slope.mp4) |
| Mixed ground (slope, steps and rubble) | 2.9 m | [video](video/terrain_mixed.mp4) |
| ![50 mm steps](mujoco/terrain_steps.jpg) 50 mm steps | 0.8 m | [video](video/terrain_steps.mp4) |
| ![80 mm rubble](mujoco/terrain_rubble.jpg) 80 mm rubble | 0.6 m | [video](video/terrain_rubble.mp4) |

The Phase 2 target is more than 3 m in 30 s on steps and rubble.

## The PyBullet prototype (Kai)

| | |
|---|---|
| ![PyBullet spider, three-quarter view](pybullet/pybullet_iso.png) | ![PyBullet spider from the side](pybullet/pybullet_side.png) |
| The placeholder spider built from Craig's description before the CAD existed. | Side view. |
| ![PyBullet spider from the front](pybullet/pybullet_front.png) | ![PyBullet spider from above](pybullet/pybullet_top.png) |
| Front view. | From above. |

## The operator console (Tom)

| | |
|---|---|
| ![Mission control](dashboard/mission.png) | ![Map and route](dashboard/map.png) |
| Mission control: gait mode, drive pad, emergency stop, telemetry. | Map and route: waypoints and site map. |
| ![Diagnostics](dashboard/diagnostics.png) | |
| Diagnostics: joint temperature and torque, subsystem health. | |

All three screens run on sample data in Phase 1; wiring them to the live
drive protocol is the first Phase 2 task.

## Diagrams

| | |
|---|---|
| ![System architecture](diagrams/system_architecture.svg) | ![Simulation pipeline](diagrams/simulation_pipeline.svg) |
| System architecture. | From V Source's CAD to the simulation. |
| ![Leg map](diagrams/leg_map.svg) | ![Gait timing](diagrams/gait_timing.svg) |
| Leg and servo numbering shared by the simulation and the hardware. | Gait timing. |
| ![Residual learning](diagrams/residual_rl.svg) | ![Phase 2 roadmap](diagrams/roadmap.svg) |
| Residual reinforcement learning: the learned controller corrects the hand-written gait. | Phase 2 roadmap. |
