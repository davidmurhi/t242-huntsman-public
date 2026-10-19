<script>
	import { ui } from '$lib/state.svelte.js';
	import { waypoints } from '$lib/mock.js';

	const sel = $derived(waypoints[ui.selectedWaypoint]);
	const route = waypoints.map((w) => `${w.x},${w.y}`).join(' ');
</script>

<!-- Waypoint list -->
<section class="panel list">
	<div class="panel-title">Route - {waypoints.length} waypoints</div>
	{#each waypoints as w, i}
		<button
			type="button"
			class="wp"
			class:on={i === ui.selectedWaypoint}
			onclick={() => (ui.selectedWaypoint = i)}
		>
			<span class="badge mono">{w.n}</span>
			<span class="txt">
				<span class="nm">{w.name}</span>
				<span class="tr mono">{w.terrain}</span>
			</span>
		</button>
	{/each}
	<div class="spacer"></div>
	<button type="button" class="btn add">+ Add waypoint</button>
</section>

<!-- Map -->
<div class="map">
	<svg viewBox="0 0 696 812" preserveAspectRatio="xMidYMid meet" role="img" aria-label="Site map with planned route">
		<defs>
			<pattern id="grid" width="48" height="48" patternUnits="userSpaceOnUse">
				<path d="M48 0H0V48" fill="none" stroke="#1a201a" stroke-width="1" />
			</pattern>
			<pattern id="hatch" width="10" height="10" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">
				<line x1="0" y1="0" x2="0" y2="10" stroke="#f2a541" stroke-width="2" opacity="0.5" />
			</pattern>
		</defs>
		<rect width="696" height="812" fill="url(#grid)" />
		<g fill="#161b16" stroke="#2f342f" stroke-width="2">
			<rect x="40" y="90" width="150" height="110" />
			<rect x="60" y="300" width="110" height="130" />
			<rect x="440" y="520" width="190" height="120" />
			<rect x="500" y="290" width="130" height="100" />
		</g>
		<path d="M190 610 L310 560 L330 470 L250 440 Z" fill="url(#hatch)" stroke="#f2a541" stroke-width="1.5" stroke-dasharray="6 4" />
		<text x="222" y="522" font-family="IBM Plex Mono, monospace" font-size="12" fill="#f2a541">RUBBLE</text>
		<polyline points={route} fill="none" stroke="#5fc9a1" stroke-width="3" stroke-dasharray="10 6" />
		{#each waypoints as w, i}
			<circle cx={w.x} cy={w.y} r={i === ui.selectedWaypoint ? 20 : 16} fill={i === ui.selectedWaypoint ? '#f2a541' : '#e8eae4'} stroke="#121412" stroke-width="3" />
			<text x={w.x} y={w.y + 5} text-anchor="middle" font-family="IBM Plex Mono, monospace" font-size="14" font-weight="500" fill="#121412">{w.n}</text>
		{/each}
		<g transform="translate(206 560)">
			<circle r="14" fill="#121412" stroke="#e8eae4" stroke-width="2" />
			<path d="M0 -8 L6 6 L0 3 L-6 6 Z" fill="#e8eae4" />
		</g>
	</svg>
	<span class="tag tl mono">SITE MAP - placeholder layout</span>
	<div class="tag bl mono legend">
		<span><svg width="22" height="4"><line x1="0" y1="2" x2="22" y2="2" stroke="#5fc9a1" stroke-width="3" stroke-dasharray="6 3" /></svg>Planned route</span>
		<span><svg width="14" height="14"><rect width="14" height="14" fill="#2b2418" stroke="#f2a541" /></svg>Rubble zone</span>
		<span><svg width="14" height="14"><circle cx="7" cy="7" r="6" fill="#121412" stroke="#e8eae4" stroke-width="2" /></svg>Robot</span>
	</div>
</div>

<!-- Detail -->
<div class="detail">
	<section class="panel">
		<div class="panel-title">Selected waypoint</div>
		<div class="sel-name">{sel.n}. {sel.name}</div>
		<div class="row"><span class="label">Terrain</span><span>{sel.terrain}</span></div>
		<div class="row"><span class="label">Gait</span><span>{sel.gait}</span></div>
		<div class="row"><span class="label">Task</span><span>{sel.task}</span></div>
		<div class="row"><span class="label">Position</span><span class="mono pos">X {sel.x}, Y {sel.y}</span></div>
	</section>

	<section class="panel grow">
		<div class="panel-title">Mission</div>
		<p class="note">
			Route and terrain data here are placeholders. Later this panel would take the plan from the mission
			planner and show progress as the robot advances.
		</p>
		<div class="spacer"></div>
		<button type="button" class="btn btn-primary">SEND ROUTE TO ROBOT</button>
		<button type="button" class="btn">Pause mission</button>
	</section>
</div>

<style>
	.list {
		width: 340px;
		flex-shrink: 0;
	}
	.spacer {
		flex-grow: 1;
	}
	.grow {
		flex-grow: 1;
	}
	.wp {
		display: flex;
		align-items: center;
		gap: 12px;
		padding: 12px;
		min-height: 64px;
		border-radius: 6px;
		text-align: left;
		border: 1px solid var(--edge);
		background: var(--panel-2);
		color: var(--text);
	}
	.wp.on {
		background: var(--amber);
		border-color: var(--amber);
		color: var(--bg);
	}
	.badge {
		width: 32px;
		height: 32px;
		border-radius: 50%;
		flex-shrink: 0;
		display: flex;
		align-items: center;
		justify-content: center;
		font-weight: 500;
		font-size: 14px;
		background: var(--text);
		color: var(--bg);
	}
	.wp.on .badge {
		background: var(--bg);
		color: var(--amber);
	}
	.txt {
		display: flex;
		flex-direction: column;
		gap: 2px;
	}
	.nm {
		font-weight: 600;
		font-size: 16px;
	}
	.tr {
		font-size: 12px;
		opacity: 0.8;
	}
	.add {
		border-style: dashed;
		background: transparent;
		color: var(--dim);
	}

	.map {
		position: relative;
		flex-grow: 1;
		min-width: 0;
		border-radius: 8px;
		overflow: hidden;
		border: 1px solid var(--line);
		background: #0f120f;
	}
	.map > svg {
		width: 100%;
		height: 100%;
		display: block;
	}
	.tag {
		position: absolute;
		font-size: 12px;
		padding: 4px 8px;
		border-radius: 4px;
		background: rgba(15, 18, 15, 0.85);
	}
	.tl { left: 16px; top: 14px; }
	.bl { left: 16px; bottom: 14px; }
	.legend {
		display: flex;
		gap: 16px;
		padding: 6px 10px;
	}
	.legend span {
		display: flex;
		align-items: center;
		gap: 6px;
	}

	.detail {
		width: 340px;
		flex-shrink: 0;
		display: flex;
		flex-direction: column;
		gap: 16px;
	}
	.sel-name {
		font-family: var(--font-display);
		font-weight: 600;
		font-size: 28px;
		color: var(--amber);
	}
	.pos {
		font-size: 13px;
	}
	.note {
		margin: 0;
		font-size: 14px;
		line-height: 1.5;
		color: var(--dim);
	}
</style>
