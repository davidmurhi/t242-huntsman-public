# Huntsman Operator Console

SvelteKit frontend prototype for Project Huntsman (QUT IFB398). Three screens:

- `/` Mission Control: gait mode, drive pad, e-stop, camera feed placeholder, telemetry
- `/map` Map and Route: waypoint list, site map, mission controls
- `/diagnostics` Diagnostics: joint temperatures and torque, subsystem health, gait policy

All numbers are sample data from `src/lib/mock.js` until the app is connected to the simulation or robot.

## Run it

Needs Node.js 20 or newer.

```sh
npm install
npm run dev -- --open
```

Then open http://localhost:5173.

Other commands: `npm run check` (type check), `npm run build` (production build).

## Project layout

```
src/
  app.css                       colours, fonts and shared classes
  lib/
    mock.js                     sample data for every screen
    state.svelte.js             shared UI state (gait, e-stop, selected waypoint)
    telemetry.svelte.js         live telemetry store + WebSocket connect/send
    components/Header.svelte    top bar with navigation and e-stop
  routes/
    +layout.svelte              page shell
    +page.svelte                Mission Control
    map/+page.svelte            Map and Route
    diagnostics/+page.svelte    Diagnostics
```

## Connecting to the simulation

`src/lib/telemetry.svelte.js` holds the values the UI displays. Call
`connect('ws://<host>:<port>/telemetry')` (for example in `onMount` in a layout) and send JSON
messages containing any of the fields in the `Telemetry` class; they are merged into the store.
Use `send({...})` to push operator commands back. Agree the field names, units and update
rate with the simulation team before building against them.

A page served over HTTPS cannot open a plain `ws://` connection. Use `http://` during local
development, or `wss://` if the robot or sim server has TLS.

## Adding this to the team repository

See the steps in the chat message that came with this project, or in short: copy this folder
into the repo as `frontend/` (without `node_modules`), create a branch, commit and push.
