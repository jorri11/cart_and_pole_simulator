# Cart and Pole Simulator

A small cart-and-pole simulation project with:

- a C++ simulator/controller that writes simulation data to `data.csv`
- a SvelteKit frontend that visualizes the CSV data

## Requirements

- C++ compiler, for example `g++`
- Node.js
- pnpm

## Generate simulation data

Compile the C++ simulator from the repository root:

```bash
g++ controller.cpp -o controller
```

Run it to generate `data.csv`:

```bash
./controller
```

By default, the simulator runs for 60 seconds. You can pass the number of seconds as an argument:

```bash
./controller 10
```

This creates a CSV file named:

```text
data.csv
```

## Move the CSV to the frontend

The frontend imports the CSV from SvelteKit's `$lib/assets` folder, so copy the generated file here:

```text
frontend/src/lib/assets/data.csv
```

Example:

```bash
cp data.csv frontend/src/lib/assets/data.csv
```

## Run the frontend

```bash
cd frontend
pnpm install
pnpm run dev
```

Open the local URL shown in the terminal, then click **Test** to start the animation.

## Data flow

```text
controller.cpp -> data.csv -> frontend/src/lib/assets/data.csv -> SvelteKit visualizer
```
