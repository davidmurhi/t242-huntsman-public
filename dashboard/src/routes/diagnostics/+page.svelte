<script>
	import { joints, subsystems } from '$lib/mock.js';
	import { telemetry } from '$lib/telemetry.svelte.js';

	const WARM_C = 55;
</script>

<!-- Joints -->
<section class="panel joints">
	<div class="head">
		<span class="panel-title">Joints - {joints.length} actuators</span>
		<span class="mono sample">Sample data</span>
	</div>
	<div class="jrow hdr mono">
		<span class="c-leg">LEG</span>
		<span class="c-joint">JOINT</span>
		<span class="c-bar">TEMPERATURE</span>
		<span class="c-bar">TORQUE</span>
		<span class="c-state">STATE</span>
	</div>
	{#each joints as j}
		{@const warm = j.temp >= WARM_C}
		<div class="jrow">
			<span class="c-leg mono">{j.leg}</span>
			<span class="c-joint dim">{j.joint}</span>
			<span class="c-bar cell">
				<span class="bar small"><span style:width="{(j.temp / 80) * 100}%" style:background={warm ? 'var(--amber)' : 'var(--ok)'}></span></span>
				<span class="mono num">{j.temp} C</span>
			</span>
			<span class="c-bar cell">
				<span class="bar small"><span style:width="{j.torque}%" style:background="var(--dim)"></span></span>
				<span class="mono num">{j.torque}%</span>
			</span>
			<span class="c-state mono" style:color={warm ? 'var(--amber)' : 'var(--ok)'}>{warm ? 'WARM' : 'OK'}</span>
		</div>
	{/each}
</section>

<!-- Subsystems -->
<div class="right">
	<section class="panel">
		<div class="panel-title">Subsystems</div>
		{#each subsystems as s}
			<div class="sub">
				<span class="dot big" style:background={s.ok ? 'var(--ok)' : 'var(--amber)'}></span>
				<span class="sname">
					<span class="n">{s.name}</span>
					<span class="d mono">{s.detail}</span>
				</span>
				<span class="mono st" style:color={s.ok ? 'var(--ok)' : 'var(--amber)'}>{s.status}</span>
			</div>
		{/each}
	</section>

	<section class="panel grow">
		<div class="panel-title">Gait policy (ML)</div>
		<div class="row"><span class="label">Policy</span><span class="mono v">loaded</span></div>
		<div class="row"><span class="label">Inference rate</span><span class="mono v">{telemetry.policyHz} Hz</span></div>
		<div class="row"><span class="label">Fallback controller</span><span class="mono v">standby</span></div>
		<div class="spacer"></div>
		<button type="button" class="btn">Run self-test</button>
	</section>
</div>

<style>
	.joints {
		width: 820px;
		flex-shrink: 0;
		gap: 8px;
	}
	.head {
		display: flex;
		align-items: baseline;
		justify-content: space-between;
	}
	.sample {
		font-size: 12px;
		color: var(--dim);
	}
	.jrow {
		display: flex;
		align-items: center;
		gap: 16px;
		height: 40px;
		border-top: 1px solid var(--line-soft);
		font-size: 14px;
	}
	.jrow.hdr {
		height: auto;
		padding: 8px 0;
		border-top: 0;
		font-size: 12px;
		color: var(--dim);
	}
	.c-leg { width: 40px; }
	.c-joint { width: 72px; }
	.c-bar { width: 216px; }
	.c-state { width: 64px; font-size: 12px; }
	.dim { color: var(--dim); }
	.cell {
		display: flex;
		align-items: center;
		gap: 12px;
	}
	.bar.small {
		width: 140px;
		height: 8px;
		border-radius: 4px;
		flex-shrink: 0;
	}
	.num {
		font-size: 13px;
	}

	.right {
		flex-grow: 1;
		min-width: 0;
		display: flex;
		flex-direction: column;
		gap: 16px;
	}
	.grow {
		flex-grow: 1;
	}
	.spacer {
		flex-grow: 1;
	}
	.sub {
		display: flex;
		align-items: center;
		gap: 12px;
		min-height: 52px;
		border-top: 1px solid var(--line-soft);
	}
	.dot.big {
		width: 10px;
		height: 10px;
	}
	.sname {
		display: flex;
		flex-direction: column;
		flex-grow: 1;
	}
	.sname .n {
		font-weight: 600;
		font-size: 15px;
	}
	.sname .d {
		font-size: 12px;
		color: var(--dim);
	}
	.st {
		font-size: 12px;
	}
	.v {
		font-size: 13px;
	}
</style>
