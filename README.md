# Welcome to the 2026 IEEE SMC AI-Powered Collaborative Autonomy (AICA) Challenge Competition Files Repository

This repository contains the required libraries, setup files, and competition files for the SMC AICA 2026 competition.

## Contents

| Folder | Description |
|---|---|
| `0_libraries/` | Required Quanser libraries and support files |
| `1_setup/` | Setup and configuration files |
| `2_SMC_AICA_2026_Competition_Files/` | Main SMC AICA 2026 competition files |

For system and setup details, see the [System and Software Setup](https://utadnclab.github.io/AICA-Competition-Documentation-2026/03_Setup/System_and_Software_setup.html)

### On-Site Finals files

These are added under `2_SMC_AICA_2026_Competition_Files/python/` for the
On-Site Finals.

| File | Description |
|---|---|
| `game_finals.py` | Core simulation file for the finals scenario. Do not modify. |
| `setup_env_finals.py` | Builds the finals environment and spawns the vehicles. Run before `game_finals.py`. |
| `drivable_grid.npz` | Fixed map of the drivable road surface, used for the curb penalty. Identical for every team. |
| `plot_trace.py` | Draws the vehicle's path over the road surface with each recorded curb hit marked. Run after a run. |

Keep all four in the same folder as your navigator files. `game_finals.py`
loads `drivable_grid.npz` from beside itself; if it is missing the run still
works, but no curb penalty is applied.

`game_finals.py` writes `curb_trace.csv` at the end of every run. It is
generated, not supplied, and is overwritten each time. `plot_trace.py` needs
both `curb_trace.csv` and `drivable_grid.npz` to draw the path and the hits.
