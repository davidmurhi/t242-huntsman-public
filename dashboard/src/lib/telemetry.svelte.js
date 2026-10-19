// Live telemetry store. Starts with sample values so the UI works on its own.
//
// To connect to the simulation or robot, call connect('ws://<host>:<port>/telemetry')
// from a component (for example in onMount). Each WebSocket message should be a
// JSON object containing any subset of the fields below; they are merged in.
//
// AGREE THIS SCHEMA WITH THE SIM TEAM: field names, units and update rate.
class Telemetry {
	connected = $state(false);
	batteryPct = $state(72);
	runtimeMin = $state(41);
	rollDeg = $state(-6.0);
	pitchDeg = $state(2.4);
	yawDeg = $state(42);
	latencyMs = $state(48);
	rssiDbm = $state(-61);
	cpuTempC = $state(58);
	espHz = $state(50);
	policyHz = $state(48);
}

export const telemetry = new Telemetry();

/** @type {WebSocket | null} */
let socket = null;

/** @param {string} url */
export function connect(url) {
	disconnect();
	socket = new WebSocket(url);
	socket.onopen = () => (telemetry.connected = true);
	socket.onclose = () => (telemetry.connected = false);
	socket.onmessage = (event) => {
		try {
			Object.assign(telemetry, JSON.parse(event.data));
		} catch {
			// Ignore malformed messages rather than crashing the console.
		}
	};
}

export function disconnect() {
	socket?.close();
	socket = null;
	telemetry.connected = false;
}

/** Send an operator command to the robot / simulation. @param {object} command */
export function send(command) {
	if (socket && socket.readyState === WebSocket.OPEN) {
		socket.send(JSON.stringify(command));
	}
}
