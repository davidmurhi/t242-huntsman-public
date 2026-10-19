# Week 7 fortnightly summary (A1.2)

*P647: Project Huntsman w/ V Source Development*

## Past fortnight

- Had first meeting with Craig Schneider (V Source Development) on 2 Sep via Teams — introductions, project scope, hardware confirmation, on-premises setup and comms preferences. Meeting recorded and notes documented.
- Ran a 15 min pre-meeting prep session the morning of the Craig meeting — confirmed roles, questions, talking points and mic checks.
- Tom set up the project Discord server and sent Craig the invite immediately after the meeting.
- Ran a 1 hour post-meeting debrief — walked through Craig's answers, divided tasks, aligned on what needs to be done going into this week.
- Key outcomes: robot is an 8-legged spider (not quadruped as assumed from brief), 100% remote work confirmed, writing ML system from scratch, Craig providing cloud compute for simulations. Full notes in OneDrive 02 Meetings.
- Defence grade technology clarified by Craig — means robustness and reliability, built not to fail. Primary use is search and rescue. We are comfortable proceeding and wanted to keep you in the loop.
- All team members updating individual work logs and research log — see OneDrive 03 Project.

## Fortnight ahead

- Friday 5 Sep team meeting (9am) — share Craig meeting notes, decide on ML simulation framework, build Phase 1 plan of action, divide hands-on work.
- Kai: decide on and set up ML simulation environment — PyBullet confirmed, define observation space, action space and reward function for the 8-legged gait task. Get robot URDF/CAD model from Craig.
- Tom: decide on WebSockets vs SSE for telemetry channel, validate UI feature list against hardware outputs, begin dashboard wireframe.
- Prab: confirm hardware details with Craig via Discord (servo count per leg, ESP32 distribution, power budget), build minimal Pi 5 → ESP32 → PWM → motor end-to-end test.
- David: draft Phase 1 sprint plan, confirm fortnightly meeting schedule with Craig, keep all logs and documents updated.
- Ongoing questions to Craig via Discord as they come up.

## To discuss

- Craig confirmed the defence application question — defence grade means robustness standard for search and rescue, not military technology. Wanted to keep you in the loop that this has been resolved.
- Craig wants us to have the simulation framework chosen and understood by our next meeting with him — what level of technical progress would you expect to see from us by Week 9 to reflect that?
- We are moving into hands-on work this week — with the Phase 1 report (A3) due at the end of semester, what should we be capturing and documenting now so we are not scrambling at Week 14?
- What should our Phase 1 deliverable actually look like at the end of semester — working simulation, report, prototype, or all of the above?

## Tom

**My work**

- Drafted a full UI feature list for the operator interface — must-have (connection status, e-stop, autonomous/manual toggle, manual controls, battery, fault alerts, gait mode, orientation feedback), ideal (camera feed, terrain classification, mission log, latency indicators, position tracking, command queue feedback), and stretch (multi-operator view, configurable layout, replay, field-visibility theming).
- Began SvelteKit research — confirmed its role as routing between dashboard views and handling server-side auth for the Pi connection. Reference: SvelteKit docs.
- Researched SvelteKit core concepts: file-based routing, load functions, form actions, and API endpoints.
- Identified core frontend challenge: the interface needs continuous telemetry, not request/response — compared WebSockets, Server-Sent Events, and polling as options.
- Researched handling loss of connection gracefully in real-time interfaces.
- Remade the team summary template to match the required A1 structure.

**My plans**

- Decide on WebSockets vs SSE for the telemetry channel.
- Validate the UI feature list against what data the ML/gait system and hardware will actually output.
- Look into state management for live data in Svelte 5 (runes vs stores).
- Research existing teleoperation UI patterns (ROS rqt, QGroundControl) for layout reference.
- Start sketching a rough wireframe of the dashboard layout.

## Kai

**My work**

- Researched reinforcement learning fundamentals — core loop (state → action → reward → next state), exploration vs exploitation, why reward design is the defining factor in RL project success. Reference: RL for robot locomotion.
- Researched Gymnasium — standardised interface (reset, step, action/observation space) so any environment can plug into any training algorithm. Reference: Gymnasium docs.
- Researched PyBullet as physics engine — models rigid body dynamics (gravity, joints, friction) for sim-to-real transfer. Confirmed two modes: P.GUI (live rendering, good for debugging, slow) and P.DIRECT (headless, fast, used for actual RL training). Reference: PyBullet quickstart guide.
- Tested PyBullet — ran in both GUI and DIRECT modes to confirm setup works and get a feel for the simulated environment before building the actual robot model.

**My plans**

- Research PPO and how it applies to continuous control tasks like legged gait. Reference: Making Quadrupeds Learn to Walk.
- Define the observation space, action space and reward function for the gait task based on the robot's actual sensors and joint configuration — 8 legs, 24 motors, horizontal and vertical joints.
- Research multi-legged gait patterns (tripod, wave gait) as a possible baseline or reward shaping reference.
- Get the robot URDF/CAD model from Craig to build an accurate simulation environment.
- Coordinate with Prab on what format the trained policy needs to be exported in for deployment on the Pi 5.

## David

**My work**

- Revised partner intro letter V3 — rewrote AI-flagged sections in team's own voice, corrected T242 header throughout, resubmitted via Canvas.
- Email back and forth with Craig Schneider to arrange the first meeting — sent initial email 28 Aug, Craig confirmed Wednesday 2 Sep 11:30am, replied confirming same day.
- Read Canvas modules and reviewed A1.2 requirements and marking scheme ahead of this submission.
- Read further into 8-legged robot locomotion and how spider gait differs from quadruped gait — context for the first client meeting.
- Checked V Source Development LinkedIn and additional public information about Craig and the company ahead of the meeting.
- Read further into PyBullet simulation for 8-legged robots ahead of the Craig meeting. Reference: PyBullet quickstart guide.
- Compiled and finalised the questions list for the first Craig meeting — scope, hardware confirmation, on-premises, comms, defence application, next steps.
- Team coordination — checked everyone had read the meeting prep doc and confirmed availability for Wednesday. Reminded group of the meeting time on Monday 31 Aug.
- Prepared and led 15 min pre-meeting prep session on 2 Sep — confirmed roles, questions, talking points and mic checks.
- Attended and led first meeting with Craig — introduced the team, confirmed hardware and software stack, asked all prepared questions, recorded the meeting.
- Wrote up full Craig meeting notes from the transcript. Led 1 hour post-meeting debrief — walked team through key outcomes and divided tasks.
- Updated team research log with Craig meeting findings and corrections. Updated individual work log with all contributions.

**My plans**

- Draft Phase 1 sprint plan at Friday team meeting (5 Sep, 9am).
- Confirm fortnightly meeting schedule with Craig via Discord.
- Continue managing all communications with Sarah and Craig.
- Keep work log, research log and team documents updated as Phase 1 hands-on work begins.
- Research what documentation and evidence needs to be captured now for the Phase 1 report (A3).
- Support all team members across areas as needed.

## Prabhanjan

**My work**

- Prepped for and attended intro meeting with Craig — covered scope, hardware confirmation, on-premises setup and comms preferences.
- Wrote up post-meeting notes and pulled out hardware follow-ups: motors/servos per leg, one ESP32 per leg vs per motor, whether a power/battery budget already exists.
- Compared Pi 5 → ESP32 communication options (serial vs WiFi vs I2C), weighing reliability and latency for a legged platform. Reference: Raspberry Pi 5 specs.
- Sketched rough idea of how many ESP32s the robot might need and how they would be split across legs.
- Set up Pi 5 with Python and basic GPIO/serial libraries, tested basic serial link with spare ESP32 as early proof-of-concept. Reference: ESP32 MCPWM — Espressif docs.
- Synced with Kai and Tom on integration points — how trained policy output maps to PWM commands, and what hardware/status data the dashboard needs from the Pi 5.
- Read up on ESP32 mesh networking and MQTT — current thinking is Phase 1 only needs a simple data link, not a full mesh.

**My plans**

- Confirm with Craig via Discord: motor/servo count per leg, ESP32 distribution, any existing power/battery constraints.
- Decide on comms protocol and build minimal end-to-end test: Pi 5 → ESP32 → PWM → motor.
- Feed hardware timeline and constraints into team sprint plan at Friday meeting.
- Scope what sensor data (joint angles, orientation) hardware needs to expose for both the RL policy and dashboard.
