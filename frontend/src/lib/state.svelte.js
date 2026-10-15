// Shared UI state. Lives in a module so it persists as the operator
// moves between routes (e.g. an active e-stop stays active on every screen).
class UiState {
	/** Gait id, one of the ids in mock.js (walk, trot, rubble, crouch). */
	gait = $state('trot');
	estop = $state(false);
	selectedWaypoint = $state(1);
}

export const ui = new UiState();
