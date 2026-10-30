# Product backlog

Epics come from the brief's deliverables. Stories are prioritised with
MoSCoW (Must, Should, Could) and estimated in story points. Status
shows the sprint where an item was done, or its planned Phase 2 sprint.
The live board is the GitHub Project; this page is the snapshot at the
end of Phase 1.

## E1: Simulation that matches the robot

| ID | Story | Acceptance criteria | Pri | Pts | Status |
|---|---|---|---|---|---|
| S1.1 | As the team, I want a spider robot in a physics simulator so that we can develop a gait before hardware | 8 legs x 3 joints load and stand | M | 5 | Done S2 (Kai, PyBullet v1-v5) |
| S1.2 | As the team, I want the simulation to run on every member's machine so that everyone can review it | runs on macOS (Apple Silicon), Windows and Linux with Python 3.11-3.13 | M | 3 | Done S3 (ADR 0002) |
| S1.3 | As V Source, I want the simulated robot built from our CAD so that results apply to the real robot | geometry, joint axes and masses from the CAD; mass matches the weight sheet (4.326 kg) | M | 13 | Done S4 |
| S1.4 | As the team, I want the CAD import to be reproducible so that design changes flow into the simulation | one command re-extracts; CI compares with the committed geometry | S | 3 | Done S4 |
| S1.5 | As the team, I want rough terrains (rubble, slope, steps) so that we can test where the current controller fails | seeded and repeatable; difficulty 0-1; full-size: 80 mm rocks, 15 deg, 50 mm steps | M | 5 | Done S4 |
| S1.6 | As V Source, I want design findings from the simulation | clearances, servo loads and collisions reported | C | 2 | Done S4 (simulation README) |
| S1.7 | As the team, I want the payload case and rail from the real part files | `case2` and `rear rail v2` from CAD, not renders | S | 2 | P2-S1 (waiting on V Source) |

## E2: Walking controller

| ID | Story | Acceptance criteria | Pri | Pts | Status |
|---|---|---|---|---|---|
| S2.1 | As an operator, I want the robot to walk straight so that it goes where I send it | 3 m on flat ground, under 1 cm off line, no leg contact | M | 8 | Done S4 |
| S2.2 | As an operator, I want to drive in any direction and turn on the spot | forward, back, sideways, spin and arcs, no leg contact | M | 5 | Done S4 |
| S2.3 | As an operator, I want gait modes for different ground | walk, trot, rubble, crouch; switch while moving | S | 3 | Done S4 |
| S2.4 | As an operator, I want the robot to hold its heading without correction | under 2 deg drift over 3 m on flat | S | 2 | Done S4 |
| S2.5 | As the team, I want a learned policy trained end to end so that we know the pipeline works | PPO trains and the policy produces movement | M | 8 | Done S3 (Kai, PyBullet, 2 M steps) |
| S2.6 | As V Source, I want a learned controller that crosses steps and rubble | over 3 m in 30 s on full-difficulty steps and rubble, tilt under 25 deg | M | 13 | P2-S3 |
| S2.7 | As the team, I want the RL environment on the CAD model | Gymnasium API; terrains; randomisation; residual actions; `train.py --check` passes | M | 5 | Done S4 |

## E3: Hardware integration

| ID | Story | Acceptance criteria | Pri | Pts | Status |
|---|---|---|---|---|---|
| S3.1 | As the team, I want the Pi 5 to talk to an ESP32 | serial link proven | M | 3 | Done S2 (Prab) |
| S3.2 | As the team, I want joint angles converted to servo pulses | per-joint mapping using real limits | M | 2 | Done S4 (Prab, placeholder range) |
| S3.3 | As the team, I want servo order verified on the robot | each of the 24 indices moves the expected joint | M | 3 | P2-S1 |
| S3.4 | As the team, I want the gait running on the Pi 5 driving the real servos | walks on flat ground, e-stop works | M | 8 | P2-S4 |
| S3.5 | As the team, I want measured servo response to calibrate the simulation | step response, speed and torque per joint type | S | 5 | P2-S2 |

## E4: Operator interface

| ID | Story | Acceptance criteria | Pri | Pts | Status |
|---|---|---|---|---|---|
| S4.1 | As an operator, I want a mission-control screen | gait mode, drive pad, e-stop, telemetry, map and diagnostics screens | M | 5 | Done S3 (Tom, mock data) |
| S4.2 | As an operator, I want live orientation from the simulation | roll, pitch and yaw over WebSocket at 30 Hz or more | M | 3 | Done S4 (Prab bridge; controller.py) |
| S4.3 | As an operator, I want to drive the simulation from the dashboard | drive, hold, gait and e-stop commands; dead-man on dropped connection | M | 3 | Done S4 (protocol); console wiring before P2-S1 |
| S4.4 | As V Source, I want the command channel secured | authentication before commands reach hardware | M | 3 | P2-S4 |

## E5: Project and quality

| ID | Story | Acceptance criteria | Pri | Pts | Status |
|---|---|---|---|---|---|
| S5.1 | As the team, I want a shared repository and board | branches, folders, board | M | 2 | Done S1 |
| S5.2 | As the team, I want automated tests and CI | every push runs tests, the model build, a walk check and the dashboard build | M | 5 | Done S4 |
| S5.3 | As V Source, I want documentation to pick the work up | READMEs, ADRs, hardware notes | M | 3 | Done S4 |
| S5.4 | As the teaching team, I want Phase 1 outputs presented | A2 presentation and synopsis; A3 report | M | 8 | S4 / Week 14 |
