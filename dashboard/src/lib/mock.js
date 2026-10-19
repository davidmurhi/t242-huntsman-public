// Sample data used by the prototype screens. Replace with live data
// from the telemetry stream once the simulation / robot publishes it.

export const gaits = [
	{ id: 'walk', label: 'Walk', sub: 'STABLE', speed: 0.4 },
	{ id: 'trot', label: 'Trot', sub: 'DEFAULT', speed: 0.9 },
	{ id: 'rubble', label: 'Rubble', sub: 'ROUGH TERRAIN', speed: 0.25 },
	{ id: 'crouch', label: 'Crouch', sub: 'LOW PROFILE', speed: 0.15 }
];

export const events = [
	{ time: '13:52:08', level: 'INFO', text: 'Gait policy loaded, switched to trot' },
	{ time: '13:51:40', level: 'INFO', text: 'Waypoint 2 reached, holding position' },
	{ time: '13:50:12', level: 'WARN', text: 'Link latency above threshold for 3 s' },
	{ time: '13:48:55', level: 'INFO', text: 'Motor controller heartbeat restored' },
	{ time: '13:47:03', level: 'INFO', text: 'Mission started from staging point' },
	{ time: '13:46:30', level: 'INFO', text: 'Operator console connected' }
];

export const waypoints = [
	{ n: 1, name: 'Staging point', terrain: 'Flat, paved', gait: 'Trot', task: 'Depart', x: 90, y: 700 },
	{ n: 2, name: 'Wall crossing', terrain: 'Rubble', gait: 'Rubble', task: 'Traverse and hold', x: 240, y: 540 },
	{ n: 3, name: 'Search zone A', terrain: 'Debris field', gait: 'Walk', task: 'Camera sweep', x: 420, y: 380 },
	{ n: 4, name: 'Supply drop point', terrain: 'Open ground', gait: 'Trot', task: 'Deliver payload', x: 580, y: 180 }
];

// [leg, joint, temperature C, torque %]
/** @type {[string, string, number, number][]} */
const jointRows = [
	['FL', 'Hip', 41, 18], ['FL', 'Thigh', 47, 42], ['FL', 'Calf', 52, 55],
	['FR', 'Hip', 40, 17], ['FR', 'Thigh', 46, 40], ['FR', 'Calf', 50, 51],
	['RL', 'Hip', 42, 20], ['RL', 'Thigh', 49, 44], ['RL', 'Calf', 58, 63],
	['RR', 'Hip', 41, 19], ['RR', 'Thigh', 48, 43], ['RR', 'Calf', 54, 57]
];

export const joints = jointRows.map(([leg, joint, temp, torque]) => ({ leg, joint, temp, torque }));

export const subsystems = [
	{ name: 'Raspberry Pi 5', detail: 'Compute, camera stream, gait policy', status: 'ONLINE', ok: true },
	{ name: 'ESP32 motor controller', detail: 'Heartbeat 50 Hz', status: 'ONLINE', ok: true },
	{ name: 'IMU', detail: 'Attitude estimate', status: 'ONLINE', ok: true },
	{ name: 'Front camera', detail: 'Placeholder stream', status: 'ONLINE', ok: true },
	{ name: 'Radio link', detail: 'Latency above target', status: 'DEGRADED', ok: false },
	{ name: 'Battery pack', detail: 'Pack status', status: 'ONLINE', ok: true }
];
