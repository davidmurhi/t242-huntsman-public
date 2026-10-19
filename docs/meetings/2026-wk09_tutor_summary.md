# Week 9 fortnightly summary (A1.2)

*P647: Project Huntsman w/ V Source Development*

## Past fortnight

- Friday 5 Sep team meeting (9am) — shared Craig meeting notes, built Phase 1 sprint plan, divided hands-on work across all four tracks.
- Kai identified that PyBullet does not support Python 3.13 — flagged to the team as a blocker for the simulation track.
- David investigating MuJoCo as a potential alternative simulation framework — evaluation in progress, to be brought to the team before a decision is made.
- GitHub repo fully set up — folder structure, branches (main, dev, simulation, firmware), Kanban board with 24 tasks across all tracks.
- Phase 1 sprint plan doc and Week 9 focus plan doc built and saved to OneDrive.
- Work logs and research log updated and spread naturally across the fortnight.

## Fortnight ahead

- Kai: resolve simulation framework decision, begin building environment once framework is confirmed.
- Prab: set up ESP32 dev environment, write firmware scaffold, confirm PWM output to one servo motor.
- Tom: scaffold SvelteKit project, build basic dashboard layout, plan data connection from sim to frontend.
- David: finalise simulation framework evaluation, bring recommendation to team, coordinate team progress, message Craig re cloud compute access.
- All: begin capturing evidence and documentation for A3 Phase 1 report.

## To discuss

- Simulation framework — PyBullet confirmed incompatible with Python 3.13. David evaluating MuJoCo as alternative. Is this the right call or are there other options worth considering?
- Cloud compute — Craig mentioned providing access but hasn't confirmed how yet. Should we be pushing on this now or is it okay to wait?
- Sim vs hardware timeline — hardware is behind simulation right now. Is that okay at this stage of Phase 1?
- A2 (Project Presentation, Week 13, 30%) — the brief says we need to show convincing evidence that our Phase 2 plans are feasible and realistic. What does that evidence need to look like specifically for our project by Week 13?
- A2 structure — any advice on what the teaching team particularly want to see from a project like ours across the four sections: Definition, Plans, Outputs, Operation?
- A2 synopsis — 12-page PDF due the night before. Is that just key slides from the deck, or does it need to be a separate written document?

## Tom

**My work**

- WebSockets vs SSE decision
- Compared WebSockets vs SSE for the telemetry channel: WebSockets chosen for bidirectional support, needed later for sending manual commands back to the robot
- Reviewed reconnection/heartbeat handling and browser support differences between the two approaches
- Noted SSE's one-directional limitation would block future manual control features, ruling it out long-term
- Decided on WebSockets over SSE
- Implementation options
- Compared WebSocket library options for the backend: native ws, socket.io (adds reconnection, rooms, fallback handling), and FastAPI's built-in WebSocket support (relevant if Pi 5 backend ends up in Python)
- Looked into data format for the telemetry stream: JSON as the default for now, with MessagePack/Protobuf flagged as a future option if data volume or frequency increases
- Considered update frequency for live telemetry rough target of 5–10Hz to balance responsiveness against connection load
- Reference points
- Looked at existing operator dashboards for mobile/legged robots (e.g. ROS2's rqt/Foxglove, open-source UGV telemetry dashboards) for UI pattern reference map views, joint-angle readouts, status panels
- Flagged security/auth as a future requirement for the WebSocket connection once bidirectional commands are added not needed yet, but noted for later

**My plans**

- Sketch and share the data-flow diagram: ESP32 → Pi 5 → backend → WebSocket → dashboard
- Revise the UI feature list once Prab confirms available hardware data, removing or adjusting items that assumed unavailable fields
- Flag authentication/access control for the WebSocket connection as a future requirement, to be addressed before command-sending is implemented
- Commit research notes, schema decisions, and the data-flow diagram to /docs or /research on the dashboard branch, using the [dashboard] commit prefix

## Kai

**My work**

- Investigated PyBullet compatibility with the team's Python environment — confirmed it does not support Python 3.13. Flagged this to the team as a blocker for the simulation track.
- Pushed research_links.md to /research on dev branch with key RL and simulation references.
- Built and pushed five robot iterations to GitHub under simulation (v1–v5):v1 - static/still model with no working jointsv2 - added joints and attempted to make the robot walk forward, however the walking movement wasn't straight or stablev3 - fixed the straight walking issue with 16 joints (hip and knee)v4 - lengthened the body and made further refinementsv5 - updated to the full 24 joint structure (hip, knee and ankle) after Prab flagged that the brief required 24 joints not 16.

**My plans**

- Confirm simulation framework with team once David completes MuJoCo evaluation.
- Research PPO as the training algorithm for the gait task and begin planning the training pipeline.
- Coordinate with Prab on how the trained policy will need to be exported for deployment on the Pi 5.
- Collaborate with Prab on integrating the v5 (24-joint) robot into the Gymnasium environment skeleton, working together to check whether the extra joints reintroduce any of the straight walking instability.
- Get the actual physical measurements of the hardware (body length, width, height, leg section lengths and joint spacing) from Craig and update the URDF to meet these requirements.

## David

**My work**

- Built Phase 1 sprint plan doc and Week 9 focus plan doc.
- Set up GitHub repo — folder structure, branches (main, dev, simulation, firmware), Kanban board with 24 tasks across simulation, hardware, frontend and PM tracks.
- Picked up the simulation framework blocker after Kai identified PyBullet's Python 3.13 incompatibility — began evaluating MuJoCo as an alternative. Installed it locally, testing it against our requirements before bringing a recommendation to the team.
- Updated individual work log and team research log, spread across the fortnight.

**My plans**

- Finalise MuJoCo evaluation and bring recommendation to the team with reasoning.
- Once framework is confirmed, support Kai in getting the simulation environment set up.
- Message Craig re cloud compute access and confirm next meeting date.
- Check in with Tom on SvelteKit scaffold and Prab on firmware progress.
- Keep Kanban board updated and run weekly team meeting.
- Begin thinking about A2 structure and what outputs we need to demonstrate by Week 13.

## Prabhanjan

**My work**

- Reviewed Craig meeting notes and hardware spec — 24 servo motors, 3 per leg, Raspberry Pi 5 + ESP32 PWM control confirmed. Attended Tuesday team check-in, cloned the repo, set up branch tracking.
- Hit and fixed a Python 3.14 compatibility issue and installed 3.12 alongside it and rebuilt the venv.
- Built the Gymnasium environment skeleton (spider_env.py) — action/observation spaces, reset()/step() stubs — committed to the simulation branch.
- Wrote up the Pi 5 < > ESP32 serial link test as a README, committed to the firmware branch.
- Pulled Kai's spider URDF, loaded it into PyBullet and inspected the joint structure — caught a real discrepancy where the URDF only had 16 joints (hip + knee), not the 24 Craig specified and flagged it to Kai.
- Kai pushed an updated URDF (spider8_v5_longbody_3joint.urdf) with the correct 24 joints (hip, knee, ankle). I then updated spider_env.py to match (24/28), tested, committed and pushed.

**My plans**

- Hand off the environment skeleton to Kai to plug in PyBullet physics.
- Keep environment and firmware branches in sync as joint structure/hardware details firm up.
- Load the updated 24-joint robot into the Gymnasium environment and run more thorough testing to confirm the observation/action spaces behave correctly across a range of poses, not just at reset.
