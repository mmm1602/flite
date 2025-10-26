# Scripts

This folder contains the Makefile and related scripts for setting up, verifying, solving, and running the demo for the project.

## Commands

- `make setup`  
  Sets up the Python virtual environment and installs the required dependencies.

- `make check`  
  Verifies shapes and targets.

- `make solve`  
  Writes the following files:
  - `routes.json`
  - `targets.json`
  - `metrics.json`
  - `routes.geojson`

- `make demo`  
  Builds and serves the UI.