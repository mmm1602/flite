# FLITE – Mission Control (React + Vite)

FLITE is a mission-control style UI with an animated drone path over a MapLibre basemap, built with React, Vite, and deck.gl. It supports theme switching and can overlay solver-generated routes from the NextEra dataset.

## Features

- MapLibre GL basemap (light/dark)
- deck.gl PathLayer animation (no geo-layers)
- Telemetry HUD (speed, battery, heading, signal)
- Follow camera and keyboard shortcut (f)
- Optional overlay of solver routes from `public/routes.json`

## Getting Started

Development

1. Install deps: `npm install`
2. Run the dev server: `npm run dev`

Build

1. `npm run build`
2. Preview locally: `npm run preview`

## Solver overlay (routes.json)

You can generate a `public/routes.json` file from the provided NextEra dataset and have it overlay in the UI.

Python prerequisites

- Python 3.10+
- Create/activate a virtualenv (recommended)
- Install requirements: `pip install -r scripts/requirements.txt`

Generate routes

Run the solver scaffold:

```
python scripts/solve_routes.py \
	--data-dir "../NEE UCF Hackathon Challenge Data & Guide" \
	--out "public/routes.json" \
	--max-meters 11500
```

Notes

- Defaults assume depot index `0` and use the "photo" index range from the dataset.
- The script builds a global TSP, greedily splits into battery-constrained legs, reconstructs detailed waypoints with the provided predecessor matrix, and writes `routes.json`.
- In the app, you can toggle the overlay with the “Show/Hide Routes” button.

## Tech Stack

- React 19, Vite 7
- deck.gl core layers (PathLayer, ScatterplotLayer, PolygonLayer)
- MapLibre GL via `react-map-gl`

## License

For hackathon/demo use.
