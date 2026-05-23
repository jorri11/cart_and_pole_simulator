<script lang="ts">
	import data from '$lib/assets/data.csv?raw';
	//@ts-ignore
	import Papa from 'papaparse';
	let result = $state<any>([]);
	let angle = $state(0);
	let cart_position = $state(0);
	const dt = 0.01;
	const sleep = (ms: number) => new Promise((resolve) => setTimeout(resolve, ms));
	async function doTheThing() {
		result = Papa.parse(data, { header: true }).data;
		for (let i = 0; i < result.length; i++) {
			cart_position = result[i].cart_position * 40;
			angle = (result[i].angle * 180) / 3.1415;
			//console.log(result[i].cart_position, result[i].angle);
			await sleep(dt * 1000);
		}
	}
</script>

<button type="button" onclick={doTheThing}>Test</button>
<main>
	<div class="cart" style={'transform:translatex(' + cart_position + 'cm);'}>
		<div class="pole" style={'transform:rotate(' + (angle + 180) + 'deg);'}></div>
	</div>
</main>

<style>
	main {
		height: 100dvh;
		display: flex;
		align-items: center;
		justify-content: center;
	}
	.cart {
		width: 100px;
		height: 50px;
		background-color: gray;
		position: relative;
	}
	.pole {
		width: 10px;
		height: 150px;
		background-color: brown;
		position: absolute;
		top: 50%;
		left: 50%;
		transform-origin: 50% 0%;
		transform: translatex(-50%);
	}
</style>
