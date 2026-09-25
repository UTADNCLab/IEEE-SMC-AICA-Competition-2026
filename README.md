# Welcome to the 2026 IEEE SMC AI-Powered Collaborative Autonomy (AICA) Challenge Competition Files Repository

This repository contains the required libraries, setup files, and competition files for the SMC AICA 2026 competition.

## Contents

| Folder | Description |
|---|---|
| `0_libraries/` | Required Quanser libraries and support files |
| `1_setup/` | Setup and configuration files |
| `2_SMC_AICA_2026_Competition_Files/` | Main SMC AICA 2026 competition files |

For system and setup details, see the [System and Software Setup](https://utadnclab.github.io/AICA-Competition-Documentation-2026/03_Setup/System_and_Software_setup.html)

### On-Site Finals Files

Download these files for On-Site Finals.

| File | Description |
|---|---|
| `game_finals.py` | Core simulation file for the finals scenario. Do not modify. |
| `setup_env_finals.py` | Builds the finals environment and spawns the vehicles. Run before `game_finals.py`.  Do not modify. |
| `drivable_grid.npz` | Fixed map of the drivable road surface, used for the curb penalty. Do not modify.  |
| `plot_trace.py` | Draws the vehicle's path over the road surface with each recorded curb hit marked.  Do not modify. |
|`run_finals.bat` or `run_finals.m` | Run entire simulation 

