# 0003: WebSockets for telemetry and commands

**Status:** Accepted in Sprint 1 (team meeting 8 Sep 2026).
**Owner:** Tom (operator interface).

## Context

The operator dashboard needs a continuous stream of robot state
(orientation, battery, status, gait mode), not request/response. Later
it must also send commands: drive, e-stop and gait changes.

## Options

* **Server-Sent Events:** simple, but one-directional. It would block
  manual control later.
* **Polling:** simple, but wasteful and laggy.
* **WebSockets:** bidirectional and persistent. Slightly more work.

## Decision

WebSockets with JSON messages. JSON is the default format; MessagePack or
Protobuf are noted as options if volume grows. The rough target is 5-10 Hz
telemetry, now 30 Hz in the simulation. Authentication is flagged as
required before any command channel reaches a real robot.

## Consequences

* Prab's `telemetry_bridge.py` streams `rollDeg`/`pitchDeg`/`yawDeg` from
  the PyBullet environment on port 8765.
* The MuJoCo `controller.py` serves the same port and field names, and
  adds `drive`, `hold`, `gait` and `estop` commands with a 0.5 s dead-man.
  The same dashboard drives either simulation now, and the robot in
  Phase 2.
* Open item for Phase 2: authentication and access control on the
  command channel (see the [risk register](../project/risks_and_ethics.md)).
