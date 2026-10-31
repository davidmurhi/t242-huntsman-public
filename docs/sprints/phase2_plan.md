# Phase 2 plan (IFB399, 2027 Semester 1)

**Phase goal:** a learned gait controller running on V Source's Huntsman.
It should cross terrain the current controller cannot, and be driven
from the operator dashboard.

Six sprints of two to three weeks (the handover sprint runs weeks 11-13). Each sprint has a
goal, the backlog items it pulls, acceptance criteria, owners and its
dependencies on V Source.

![Roadmap](../media/diagrams/roadmap.svg)

## Milestones

| | Milestone | Sprint | How we will know |
|---|---|---|---|
| M1 | Robot under our control | P2-S1 | the gait drives all 24 servos in the right order, e-stop tested |
| M2 | Simulation calibrated to the robot | P2-S2 | simulated and real joint step responses agree within 20% |
| M3 | Terrain policy in simulation | P2-S3 | steps and rubble: over 3 m in 30 s (baseline 0.8 m and 0.6 m) |
| M4 | Policy on the robot | P2-S4 | walks on flat ground under policy on the Pi 5, within 20% of simulated speed |
| M5 | Field trials | P2-S5 | slope, steps and rubble course on the real robot, recorded |
| M6 | Handover | P2-S6 | report, demo to V Source, repository handed over |

## Sprints

### Before semester (break)

* **Goal:** everything that does not need the robot is ready, so the
  first robot session is spent on the robot.
* **Items:**
  * book robot sessions with V Source and obtain the servo datasheet
    (David)
  * one-servo bench rig with the Pi 5 and ESP32 (Prab)
  * S4.3 console wired to the live drive commands (Tom)
  * training dry run on V Source compute (Kai)
* **Accept when:** sessions are booked and the bench rig moves a servo.

### P2-S1: Hardware bring-up

* **Goal:** the robot moves under our software, safely.
* **Items:**
  * S3.3 servo order check: one servo at a time, against the [leg
    map](../media/diagrams/leg_map.svg)
  * S3.2 replace the PWM placeholder with datasheet values
  * the drive protocol on the Pi 5 (`controller.py` without the
    simulation)
  * training set up on V Source compute
  * stand test of the e-stop and dead-man
  * S1.7 `case2` and rail parts into the model
* **Owners:** Prab (servo order, signal range), David (gait on the Pi 5),
  Kai (training set up on V Source compute); all (stand test).
* **Accept when:**
  * all 24 joints move the expected way
  * the e-stop and dead-man are tested on a stand
  * the robot stands from rest
* **Depends on:** robot access (visit or loan); servo datasheet; ESP32
  firmware status from Craig.

### P2-S2: Sim-to-real calibration

* **Goal:** the simulation behaves like this robot.
* **Items:**
  * S3.5 measure servo step response, speed and stall per joint type;
    fit actuator gains and limits
  * measure friction on test surfaces
  * compare standing pose and walking speed with the simulation
  * set the domain-randomisation ranges from the measured spread
* **Owners:** Prab and David (servo measurements), Kai (friction, stance
  and speed), David and Kai (servo model, randomisation ranges), Tom
  (logging robot telemetry).
* **Accept when:** joint responses and flat-ground walking speed agree
  within 20%.
* **Depends on:** P2-S1.

### P2-S3: Rough-ground policy

* **Goal:** a policy that beats the baseline gait on steps and rubble
  in simulation.
* **Items:**
  * S2.6 residual PPO on V Source compute
  * curriculum and reward tuning
  * evaluation suite: fixed seeds, all terrains, five runs each, with
    the gait-only baseline for comparison
  * map and diagnostics screens on live data
* **Owners:** Kai (training), Kai and David (reward and curriculum),
  David (evaluation suite), Prab (compute monitoring), Tom (console
  screens).
* **Accept when:** steps and rubble over 3 m in 30 s, tilt under 25 deg,
  no leg contact, no regression on flat or slope.
* **Depends on:** compute access (requirements already sent).

### P2-S4: Policy on the robot

* **Goal:** the learned controller runs on the Pi 5 and drives the real
  servos.
* **Items:**
  * export the policy (ONNX or TorchScript)
  * inference loop on the Pi 5 at 50 Hz with IMU input
  * S3.4 ESP32 command API
  * S4.4 authentication on the command channel
  * fallback to the hand gait on fault
* **Owners:** Kai and Prab (policy on the Pi 5), Prab (ESP32 command
  API), Tom (command-channel authentication), David (fallback to the
  hand-written gait).
* **Accept when:**
  * walks on flat ground under the policy within 20% of simulated speed
  * policy inference under 10 ms
  * the fallback is triggered and tested
* **Depends on:** IMU fitted (confirm with V Source).

### P2-S5: Field trials

* **Goal:** show the controller on real rough ground.
* **Items:**
  * a test course (ramp, 50 mm steps, rubble)
  * recorded runs with the policy and the baseline
  * live telemetry on the dashboard
  * usability check of the console with Craig as the operator (Tom)
  * fix what the trials expose
* **Owners:** all.
* **Accept when:** recorded comparison on each terrain; a list of
  failure cases with causes.

### P2-S6: Evaluation and handover

* **Goal:** V Source can run, retrain and extend everything without us.
* **Items:**
  * technical report (approach, results, limitations)
  * final demonstration to V Source
  * documentation pass
  * retraining guide
* **Owners:** all; David coordinates.
* **Accept when:** a V Source engineer reproduces training and
  deployment from the README.

## Capacity and working agreements

* Four members at about 15 hours a week each; two days per sprint
  reserved for reviews, documentation and buffer.
* Hardware time is the scarcest resource: hardware sessions are booked
  with V Source a sprint ahead.
* Everything that can be tested in the simulation is tested there first;
  hardware sessions run a written test plan.

## Risks specific to Phase 2

See the full [risk register](../project/risks_and_ethics.md). The ones
that shape this plan are:

* R1 (sim-to-real), which is why P2-S2 sits before training
* R4 (compute), set up in P2-S1
* R5 (servo order), the first item in P2-S1
* R6 (safety), stand tests before floor tests
