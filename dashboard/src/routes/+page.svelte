<script>
	import { ui } from '$lib/state.svelte.js';
	import { telemetry } from '$lib/telemetry.svelte.js';
	import { gaits, events } from '$lib/mock.js';

	const current = $derived(gaits.find((g) => g.id === ui.gait) ?? gaits[1]);
	const speed = $derived(ui.estop ? 0 : current.speed);

	const stats = $derived([
		{ label: 'Link latency', value: `${telemetry.latencyMs} ms`, ok: telemetry.latencyMs < 100 },
		{ label: 'Signal', value: `${telemetry.rssiDbm} dBm`, ok: true },
		{ label: 'RPi5 CPU temp', value: `${telemetry.cpuTempC} C`, ok: telemetry.cpuTempC < 75 },
		{ label: 'ESP32 heartbeat', value: `${telemetry.espHz} Hz`, ok: true },
		{ label: 'Gait policy rate', value: `${telemetry.policyHz} Hz`, ok: telemetry.policyHz >= 50 }
	]);
</script>

<!-- Left column: controls -->
<div class="col left">
	<section class="panel">
		<div class="panel-title">Gait mode</div>
		<div class="gaits">
			{#each gaits as g}
				<button
					type="button"
					class="gait"
					class:on={g.id === ui.gait}
					onclick={() => (ui.gait = g.id)}
				>
					<span class="g-label">{g.label}</span>
					<span class="g-sub mono">{g.sub}</span>
				</button>
			{/each}
		</div>
	</section>

	<section class="panel">
		<div class="panel-title">Drive</div>
		<div class="pad">
			<button type="button" aria-label="Rotate left"><svg viewBox="0 0 24 24"><path d="M3 12a9 9 0 1 0 3-6.7" /><path d="M3 4v5h5" /></svg></button>
			<button type="button" aria-label="Forward"><svg viewBox="0 0 24 24"><path d="M12 19V5" /><path d="M5 12l7-7 7 7" /></svg></button>
			<button type="button" aria-label="Rotate right"><svg viewBox="0 0 24 24"><path d="M21 12a9 9 0 1 1-3-6.7" /><path d="M21 4v5h-5" /></svg></button>
			<button type="button" aria-label="Strafe left"><svg viewBox="0 0 24 24"><path d="M19 12H5" /><path d="M12 5l-7 7 7 7" /></svg></button>
			<button type="button" class="hold" aria-label="Hold position">HOLD</button>
			<button type="button" aria-label="Strafe right"><svg viewBox="0 0 24 24"><path d="M5 12h14" /><path d="M12 5l7 7-7 7" /></svg></button>
			<span></span>
			<button type="button" aria-label="Backward"><svg viewBox="0 0 24 24"><path d="M12 5v14" /><path d="M19 12l-7 7-7-7" /></svg></button>
			<span></span>
		</div>
	</section>

	<section class="panel grow">
		<div class="panel-title">Speed limit</div>
		<div class="speed-row">
			<span class="speed mono">{speed.toFixed(2)} m/s</span>
			<span class="mono max">of 1.00 m/s</span>
		</div>
		<div class="bar"><span style:width="{speed * 100}%" style:background="var(--amber)"></span></div>
		<p class="note">
			The limit follows the selected gait. Placeholder values, to be replaced by the robot's real limits.
		</p>
	</section>
</div>

<!-- Centre column: camera and log -->
<div class="col centre">
	<div class="feed">
		<svg viewBox="0 0 736 414" preserveAspectRatio="xMidYMid slice" aria-hidden="true">
			<g stroke="#1f251f" stroke-width="1">
				<line x1="368" y1="207" x2="0" y2="414" /><line x1="368" y1="207" x2="184" y2="414" />
				<line x1="368" y1="207" x2="368" y2="414" /><line x1="368" y1="207" x2="552" y2="414" />
				<line x1="368" y1="207" x2="736" y2="414" />
				<line x1="0" y1="250" x2="736" y2="250" /><line x1="0" y1="300" x2="736" y2="300" />
				<line x1="0" y1="365" x2="736" y2="365" /><line x1="0" y1="207" x2="736" y2="207" />
			</g>
			<g stroke="#f2a541" stroke-width="2" fill="none">
				<path d="M348 207h40M368 187v40" /><circle cx="368" cy="207" r="26" />
			</g>
		</svg>
		<span class="tag tl mono">FRONT CAM - placeholder stream</span>
		<span class="tag tr mono"><span class="dot" style:background="var(--danger)"></span>REC</span>
		<span class="tag bl mono">HDG {String(Math.round(telemetry.yawDeg)).padStart(3, '0')} | MODE {current.label.toUpperCase()}</span>
		<span class="tag br mono dim">Live video will come from the RPi5</span>
		{#if ui.estop}
			<div class="estop-overlay">
				<div class="estop-banner">EMERGENCY STOP ACTIVE - ACTUATORS DISABLED</div>
			</div>
		{/if}
	</div>

	<section class="panel grow log">
		<div class="panel-title">Event log</div>
		{#each events as ev}
			<div class="ev">
				<span class="mono t">{ev.time}</span>
				<span class="mono lvl" class:warn={ev.level === 'WARN'}>{ev.level}</span>
				<span>{ev.text}</span>
			</div>
		{/each}
	</section>
</div>

<!-- Right column: telemetry -->
<div class="col right">
	<section class="panel">
		<div class="panel-title">Power</div>
		<div class="speed-row">
			<span class="big mono">{telemetry.batteryPct}%</span>
			<span class="mono max">EST. RUNTIME {telemetry.runtimeMin} MIN</span>
		</div>
		<div class="bar"><span style:width="{telemetry.batteryPct}%" style:background="var(--ok)"></span></div>
	</section>

	<section class="panel">
		<div class="panel-title">Attitude (IMU)</div>
		<div class="att">
			<svg viewBox="0 0 132 132" width="132" height="132" role="img" aria-label="Artificial horizon">
				<defs><clipPath id="hz"><circle cx="66" cy="66" r="60" /></clipPath></defs>
				<g clip-path="url(#hz)">
					<rect width="132" height="132" fill="#1d2b33" />
					<g transform="rotate({telemetry.rollDeg} 66 66)">
						<rect x="-20" y={72 + telemetry.pitchDeg} width="172" height="80" fill="#3a2f22" />
						<line x1="-20" y1={72 + telemetry.pitchDeg} x2="152" y2={72 + telemetry.pitchDeg} stroke="#e8eae4" stroke-width="2" />
					</g>
				</g>
				<circle cx="66" cy="66" r="60" fill="none" stroke="#3a403a" stroke-width="2" />
				<path d="M36 66h20M76 66h20" stroke="#f2a541" stroke-width="3" stroke-linecap="round" />
			</svg>
			<div class="angles mono">
				<div><span class="dim k">ROLL</span>{telemetry.rollDeg.toFixed(1)} deg</div>
				<div><span class="dim k">PITCH</span>{telemetry.pitchDeg.toFixed(1)} deg</div>
				<div><span class="dim k">YAW</span>{String(Math.round(telemetry.yawDeg)).padStart(3, '0')} deg</div>
			</div>
		</div>
	</section>

	<section class="panel grow">
		<div class="panel-title">Link and compute</div>
		{#each stats as s}
			<div class="row">
				<span class="label">{s.label}</span>
				<span class="mono val"><span class="dot" style:background={s.ok ? 'var(--ok)' : 'var(--amber)'}></span>{s.value}</span>
			</div>
		{/each}
		<div class="note small">All values on this screen are sample data.</div>
	</section>
</div>

<style>
	.col {
		display: flex;
		flex-direction: column;
		gap: 16px;
		min-width: 0;
	}
	.left {
		width: 300px;
		flex-shrink: 0;
	}
	.right {
		width: 340px;
		flex-shrink: 0;
	}
	.centre {
		flex-grow: 1;
	}
	.grow {
		flex-grow: 1;
	}
	.dim {
		color: var(--dim);
	}
	.note {
		margin: 0;
		font-size: 13px;
		line-height: 1.4;
		color: var(--dim);
	}
	.note.small {
		font-size: 12px;
	}

	.gaits {
		display: grid;
		grid-template-columns: repeat(2, minmax(0, 1fr));
		gap: 8px;
	}
	.gait {
		height: 60px;
		padding: 0 12px;
		border-radius: 6px;
		text-align: left;
		display: flex;
		flex-direction: column;
		justify-content: center;
		gap: 2px;
		border: 1px solid var(--edge);
		background: var(--panel-2);
		color: var(--text);
	}
	.gait.on {
		background: var(--amber);
		border-color: var(--amber);
		color: var(--bg);
	}
	.g-label {
		font-weight: 600;
		font-size: 16px;
	}
	.g-sub {
		font-size: 11px;
		opacity: 0.8;
	}

	.pad {
		display: grid;
		grid-template-columns: repeat(3, minmax(0, 1fr));
		gap: 8px;
	}
	.pad button {
		height: 56px;
		border-radius: 6px;
		border: 1px solid var(--edge);
		background: var(--panel-2);
		color: var(--text);
		display: flex;
		align-items: center;
		justify-content: center;
	}
	.pad svg {
		width: 24px;
		height: 24px;
		fill: none;
		stroke: currentColor;
		stroke-width: 2;
		stroke-linecap: round;
		stroke-linejoin: round;
	}
	.pad .hold {
		border-color: var(--amber);
		color: var(--amber);
		font-family: var(--font-display);
		font-weight: 700;
		font-size: 16px;
		letter-spacing: 0.08em;
	}

	.speed-row {
		display: flex;
		align-items: baseline;
		justify-content: space-between;
	}
	.speed {
		font-size: 28px;
		font-weight: 500;
	}
	.big {
		font-size: 32px;
		font-weight: 500;
	}
	.max {
		font-size: 12px;
		color: var(--dim);
	}

	.feed {
		position: relative;
		width: 100%;
		aspect-ratio: 16 / 9;
		border-radius: 8px;
		overflow: hidden;
		background: #0b0d0b;
		border: 1px solid var(--line);
	}
	.feed svg {
		position: absolute;
		inset: 0;
		width: 100%;
		height: 100%;
	}
	.tag {
		position: absolute;
		display: flex;
		align-items: center;
		gap: 6px;
		font-size: 12px;
		padding: 4px 8px;
		border-radius: 4px;
		background: rgba(11, 13, 11, 0.8);
	}
	.tl { left: 16px; top: 14px; }
	.tr { right: 16px; top: 14px; }
	.bl { left: 16px; bottom: 14px; }
	.br { right: 16px; bottom: 14px; }

	.estop-overlay {
		position: absolute;
		inset: 0;
		background: rgba(255, 90, 77, 0.16);
		display: flex;
		align-items: center;
		justify-content: center;
	}
	.estop-banner {
		background: var(--danger);
		color: var(--bg);
		font-family: var(--font-display);
		font-weight: 700;
		font-size: 28px;
		letter-spacing: 0.1em;
		padding: 12px 28px;
		border-radius: 6px;
	}

	.log {
		gap: 0;
	}
	.log .panel-title {
		margin-bottom: 8px;
	}
	.ev {
		display: flex;
		gap: 16px;
		align-items: baseline;
		padding: 6px 0;
		border-top: 1px solid var(--line-soft);
		font-size: 14px;
	}
	.ev .t {
		font-size: 12px;
		color: var(--dim);
		width: 64px;
	}
	.ev .lvl {
		font-size: 12px;
		width: 56px;
		color: var(--ok);
	}
	.ev .lvl.warn {
		color: var(--amber);
	}

	.att {
		display: flex;
		gap: 16px;
		align-items: center;
	}
	.angles {
		display: flex;
		flex-direction: column;
		gap: 8px;
		font-size: 14px;
	}
	.angles .k {
		display: inline-block;
		width: 60px;
	}
	.val {
		display: flex;
		align-items: center;
		gap: 8px;
		font-size: 14px;
	}
</style>
