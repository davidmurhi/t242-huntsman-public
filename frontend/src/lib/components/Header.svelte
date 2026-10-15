<script>
	import { page } from '$app/state';
	import { ui } from '$lib/state.svelte.js';
	import { telemetry } from '$lib/telemetry.svelte.js';

	const links = [
		{ href: '/', label: 'Mission Control' },
		{ href: '/map', label: 'Map and Route' },
		{ href: '/diagnostics', label: 'Diagnostics' }
	];
</script>

<header>
	<div class="brand">
		<span class="name">HUNTSMAN</span>
		<span class="sub mono">OPERATOR CONSOLE</span>
	</div>

	<nav>
		{#each links as link}
			<a
				href={link.href}
				class:active={page.url.pathname === link.href}
				aria-current={page.url.pathname === link.href ? 'page' : undefined}
			>
				{link.label}
			</a>
		{/each}
	</nav>

	<div class="link mono">
		<span class="dot" style:background={telemetry.connected ? 'var(--ok)' : 'var(--amber)'}></span>
		<span>{telemetry.connected ? 'LINK OK' : 'SAMPLE DATA'}</span>
		<span class="robot">HUNTSMAN-01</span>
	</div>

	<button
		type="button"
		class="estop"
		class:engaged={ui.estop}
		aria-label="Emergency stop"
		onclick={() => (ui.estop = !ui.estop)}
	>
		{ui.estop ? 'RELEASE E-STOP' : 'E-STOP'}
	</button>
</header>

<style>
	header {
		height: 56px;
		flex-shrink: 0;
		display: flex;
		align-items: center;
		gap: 32px;
		padding: 0 16px;
		background: var(--panel);
		border-bottom: 1px solid var(--line);
	}

	.brand {
		display: flex;
		align-items: baseline;
		gap: 12px;
	}

	.name {
		font-family: var(--font-display);
		font-weight: 700;
		font-size: 26px;
		letter-spacing: 0.08em;
		color: var(--amber);
	}

	.sub {
		font-size: 12px;
		color: var(--dim);
	}

	nav {
		display: flex;
		gap: 4px;
		flex-grow: 1;
	}

	nav a {
		height: 56px;
		padding: 0 16px;
		display: flex;
		align-items: center;
		font-weight: 500;
		font-size: 15px;
		color: var(--dim);
		border-bottom: 3px solid transparent;
	}

	nav a.active {
		font-weight: 600;
		color: var(--amber);
		border-bottom-color: var(--amber);
	}

	.link {
		display: flex;
		align-items: center;
		gap: 8px;
		font-size: 13px;
	}

	.link .dot {
		width: 10px;
		height: 10px;
	}

	.robot {
		color: var(--dim);
	}

	.estop {
		height: 44px;
		min-width: 168px;
		padding: 0 20px;
		border-radius: 6px;
		border: 2px solid var(--danger);
		background: var(--danger);
		color: var(--bg);
		font-family: var(--font-display);
		font-weight: 700;
		font-size: 18px;
		letter-spacing: 0.1em;
	}

	.estop.engaged {
		background: var(--bg);
		color: var(--danger);
	}
</style>
