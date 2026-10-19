# Week 11 fortnightly summary (A1.4)

*P647: Project Huntsman w/ V Source Development*

## Past fortnight

- Simulation milestone reached. Kai has the spider walking forward in simulation with the full 24-joint configuration, matching Craig's specification of 3 joints per leg across 8 legs. Versions 1 to 5 are committed to the simulation branch. Craig has seen it and responded positively.
- Prab built the Gymnasium environment skeleton (spider_env.py) with a 24-value action space and 28-value observation space, and resolved a joint-count discrepancy between an earlier 16-joint URDF and the current 24-joint model.
- Tom has a first SvelteKit frontend running with placeholder data to establish the structure, and is coordinating with Kai and Prab on what the ML and hardware layers will actually expose.
- David resolved a toolchain blocker stopping team members running the simulation locally: the upstream PyBullet release is two years old and will not build on Apple Silicon under Python 3.13. He evaluated MuJoCo as a replacement, building a working comparison model and diagnostic harness, then found pybullet-arm64, a maintained drop-in fork that fixes the problem with no code changes. The MuJoCo evaluation is retained as documentation for the Phase 1 report.
- Repository structure formalised: five branches, five folders, and a Kanban board with tasks across three sprints. Documentation added to /docs and /research.
- Craig added to the GitHub repository (@Virologyst) so he can review code and progress directly.
- Craig confirmed cloud compute will be hosted on V Source infrastructure and asked us to specify what we need. Requirements drafted and sent.
- Prab requested the physical robot specifications from Craig — segment dimensions, weight, servo torque and angle range, planned sensors, and CAD files. Craig confirmed CAD is still in progress but specifications will follow this week.

## Fortnight ahead

- Apply Craig's real measurements to the simulation model once they arrive. Until then the dimensions are placeholders, so no trained policy would transfer meaningfully to hardware.
- Kai: connect the working gait model to the Gymnasium environment, replacing the placeholder reset() and step() with real physics calls, then begin PPO training with Stable Baselines 3.
- Prab: build the minimal end-to-end hardware test (Pi 5 to ESP32 to PWM to motor) and confirm the servo command ordering matches the simulation actuator ordering.
- Tom: validate the dashboard feature list against the actual data the hardware and ML layers expose, and replace placeholder values with the real telemetry schema.
- David: finalise cloud compute requirements with Craig, set up access, and begin the Phase 1 report structure so documentation is not left to the final week.
- Roll out the pybullet-arm64 fix across the team so every member can run and review the simulation locally, not just Kai.

## To discuss

- We have a working simulated gait but the robot dimensions are still placeholders until Craig's measurements arrive. Is it reasonable to begin RL training against approximate dimensions to prove the pipeline, or should we wait for the real values?
- We evaluated switching simulation frameworks, then found a fix that removed the need. How much weight should the Phase 1 report give to an evaluation we investigated thoroughly but ultimately did not adopt?
- With the presentation in Week 13 and the report in Week 14, how much of the coming fortnight should we be shifting from build work to writing?
- Craig is providing cloud compute on V Source infrastructure. Is there anything from QUT's side we should be aware of before we set that up?

## Tom

**My work**

- Built a first working SvelteKit frontend for the operator interface using placeholder values and names, to get a concrete structure in place rather than continuing to design on paper.
- Established the dashboard layout and component structure that the real telemetry will plug into.
- Began coordinating with Kai and Prab on the data contract between the frontend, the ML layer and the hardware layer, so the interface is not built against assumptions that later turn out to be wrong.
- Continued research into the real-time telemetry channel decision (WebSockets against Server-Sent Events) in the context of what the Pi 5 will actually stream.

**My plans**

- Consult with Kai and the others on what needs adding or removing so the frontend and backend line up without discrepancies.
- Replace placeholder values with the real telemetry schema once the hardware and ML outputs are defined.
- Finalise the WebSockets against SSE decision now that the data volume and direction are clearer.
- Build out the must-have dashboard features: connection status, emergency stop, autonomous and manual toggle, battery and fault indicators.

## Kai

**My work**

- Researched Stable Baselines 3, PPO, and how to properly wire a PyBullet simulation into a Gymnasium environment for RL training.  - Uploaded to git under research /Stable Baselines 3, PPO, and how to properly wire a PyBullet simulation into a Gymnasium environment for RL training.md
- Wrote train_ppo.py and watch_trained_policy.py to train and test a PPO policy via Stable Baselines 3
- Ran multiple training iterations, debugging and tuning along the way and found out that TOTAL_TIMESTEPS = 200,000 was nowhere near enough for the policy to learn real walking, it just stood still. As standing still scored almost as well as attempting to move under the original reward.
- Changed TOTAL_TIMESTEPS = 500,000 but was still not enough
- Sped up training significantly by parallelizing across CPU cores (SubprocVecEnv) as it was taking a very long time to run
- Added VecNormalize after discovering raw PPO wasn't learning effectively without observation/reward normalization
- Got the first policy that actually produced movement when TOTAL_TIMESTEPS = 2M
- The versions of trainppo.py and watch_trained _policy.py were uploaded to git hub under simulation /capstone spider robot

**My plans**

- Continue tuning the reward function based on results likely more iteration needed before gait quality is solid
- Need to Identify and fix the issue where the trained policy moves very fast and veering off to the left
- Once a stable walking policy is achieved, begin looking at turning controls (currently out of scope — straight-line walking only)

## David

**My work**

- Formalised the repository structure — five branches (main, dev, simulation, dashboard, firmware), five folders, and a Kanban board populated with tasks across three sprints. Updated the README with the corrected project scope and tech stack.
- Added simulation stack documentation and a research reference index to /docs and /research so technical decisions and sources are recorded as we go rather than reconstructed for the report.
- Managed communication with Craig throughout the fortnight — added him to the GitHub repository (@Virologyst), confirmed Discord as the ongoing update channel, and established the cloud compute arrangement.
- Drafted the cloud compute requirements for V Source infrastructure: remote access to a GPU machine for RL training, Python version, memory for headless simulation, and storage for model checkpoints.
- Diagnosed and resolved a toolchain blocker preventing team members from running the simulation locally. The upstream PyBullet release is two years old and fails to compile on Apple Silicon under Python 3.13, so the simulation could not be run or reviewed outside Kai's machine.
- Evaluated MuJoCo as a possible replacement and built a full comparison: a parametric model generator taking the robot dimensions as inputs, a static MJCF schema validator, and a headless diagnostic harness reporting stance stability, ground contact, servo load and gait performance without requiring a display.
- Used that harness to identify three geometry faults in the model under evaluation: adjacent legs overlapping in their sweep range and therefore colliding, leg-to-body proportions well outside the range for a real arachnid, and a velocity discontinuity at the swing-to-stance transition causing visible jerkiness. All three were corrected and verified numerically.
- Continued investigating rather than accepting the workaround, and found pybullet-arm64, a maintained fork publishing prebuilt wheels for Python 3.13 on Apple Silicon. It is a drop-in replacement requiring no code changes, which unblocks the team on the existing PyBullet codebase and makes a framework migration unnecessary.
- Maintained the team research log and individual work log throughout.

**My plans**

- Finalise cloud compute requirements with Craig and get access set up so Kai is not blocked when training begins.
- Share the pybullet-arm64 fix with the team so everyone can run the simulation locally, and write up the MuJoCo comparison as a documented technical evaluation for the Phase 1 report.
- Start the Phase 1 report structure and confirm what evidence needs capturing now, so the final fortnight is writing rather than reconstruction.
- Coordinate applying Craig's measurements across the simulation and hardware layers once they arrive, since both depend on the same numbers.
- Keep the Kanban board and project documentation current as work moves between sprints.

## Prabhanjan

**My work**

- Fixed spider8's URDF to properly match Craig's spec — added the missing third joint per leg (ankle), bringing the model from 16 to the correct 24 joints (8 legs × 3: hip/horizontal + knee/ankle/vertical)
- Built the Gymnasium environment skeleton (spider_env.py) — a 24-value action space mapping one value per motor, and a 28-value observation space covering joint angles, body orientation and body height.
- Completed the simulation environment setup locally with PyBullet and Gymnasium working, and loaded Kai's URDF to verify the model and environment agree.
- Documented the Pi 5 to ESP32 communication approach and committed it to the firmware branch.
- Requested the physical robot specifications from Craig: body and leg segment dimensions, total weight, servo model, torque, speed and angle range, and any planned sensors such as an IMU. Also asked whether CAD files exist in STL or STEP format, which would give exact geometry for the simulation. However craig said “Yeah we dont have the complete cad design yet. One of our engineers is currently working on it tho but we can get all the specs to you this week”

**My plans**

- Apply Craig's measurements to the hardware plan once they arrive this week.
- Build the minimal end-to-end test: Pi 5 sends a command, ESP32 receives it, PWM signal drives a motor.
- Confirm the servo command ordering matches the simulation actuator ordering exactly — a mismatch there produces no error but makes the robot move incorrectly.
- Scope what sensor data the hardware must expose for both the RL policy and the operator dashboard.
- Help kai with RL TRAINTING PPO - Continue tuning the reward function based on results
