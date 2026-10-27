```markdown
# Waypoint Converter (NPY to CSV)

A Python utility script for the ROAR-26-Simulation environment. It batch converts waypoint data stored as NumPy array files (`.npy`) into Comma-Separated Values (`.csv`) format for navigation mission setup.

## Prerequisites

- Python 3.x
- NumPy (`pip install numpy` or `sudo apt install python3-numpy`)

## Directory Structure

The script is located in the `navMission_setup` package and expects the following structure:

```text
~/Simulation_ws/src/ROAR-26-Simulation/navMission_setup/waypoints_setup/waypoint_converter/
├── convert_waypoints.py     # The conversion script
├── inputs/                  # Place your source wpXX.npy files here
└── outputs/                 # The script generates the wpXX.csv files here

```

## How to Use

1. **Navigate to the workspace directory:**
```bash
cd ~/Simulation_ws/src/ROAR-26-Simulation/navMission_setup/waypoints_setup/waypoint_converter/

```


2. **Add your waypoint files:**
Ensure your `.npy` files (e.g., `wp00.npy` to `wp75.npy`) are located inside the `inputs` folder.
3. **Run the script:**
```bash
python3 convert_waypoints.py

```


4. **Retrieve Outputs:**
The script will process the files and output standard comma-delimited `.csv` files into the `outputs` directory.

## Formatting Details

* **Decimals vs. Scientific Notation:** The script is configured to format floats as standard decimal numbers (using `fmt='%f'`) to prevent scientific notation (e.g., `1.23e-01`).
* **Delimiters:** Output files use a standard comma `,` delimiter.

```

```
