<script>
	import '../app.css';
	import favicon from '$lib/assets/favicon.svg';
	import Header from '$lib/components/Header.svelte';

	let { children } = $props();
</script>

<svelte:head>
	<title>Huntsman Operator Console</title>
	<link rel="icon" href={favicon} />
</svelte:head>

<div class="app">
	<Header />
	<main>
		{@render children()}
	</main>
</div>

<style>
	.app {
		height: 100vh;
		min-width: 1280px;
		display: flex;
		flex-direction: column;
	}

	main {
		flex-grow: 1;
		min-height: 0;
		display: flex;
		gap: 16px;
		padding: 16px;
	}
</style>
