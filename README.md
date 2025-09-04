# paper_gorbani_elobaid_2025_lcss_df_mpc-ironcub

## Dependency

This project uses [pixi](https://prefix.dev/docs/pixi) for environment and dependency management. You can install pixi by following [this installation guide](https://pixi.sh/latest/installation/).

In addition, the [ironcub-models](https://github.com/ami-iit/ironcub-models) repository is required. Please clone and install it following the instructions in its README before running any simulation.

## Usage

To run the simulation:

1. (If not already done) Install the `ironcub-models` repository as described above.

2. Run the simulation:
  ```bash
  pixi run run-simulation
  ```

This will execute the python script to run the Mujoco simulation.

If you want to run the toy problem, run:
```bash
pixi run simple-example
```
