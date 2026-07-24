# ROAR Dev Environment

A **standalone** development environment for testing the rover across multiple maps and scenarios.

## Structure

```
dev_environment/
├── src/
│   └── rover/                    # Standalone rover packages
│       ├── roar_rover_clean/     # Clean simulation (no noise)
│       ├── roar_rover_noise/     # Noisy simulation (sensor degradation)
│       └── roar_simulation_full/ # Full simulation (arm + noise)
│
├── worlds/                       # Generated worlds for testing
│   ├── world1/                   # Each world contains:
│   │   ├── world1.world          #   - SDF world file
│   │   ├── world1.launch.py      #   - Launch file
│   │   ├── obstacle_data.npy     #   - Obstacle positions
│   │   ├── costmap.npz           #   - Cost map
│   │   └── heightmap.npz         #   - Height map
│   ├── world2/
│   └── world3/
│
├── test_configs/                 # Test scenario configurations
│
├── results/                      # Test run logs and results
│
└── launch_test.sh                # Main launch script
```

## Usage

### Build the workspace
```bash
cd dev_environment
colcon build
source install/setup.bash
```

### Launch a test
```bash
bash launch_test.sh <rover_config> <world_name>
# Example:
bash launch_test.sh clean world1
bash launch_test.sh noise world2
bash launch_test.sh full world3
```

### Adding new worlds
Copy generated worlds from `navMission_setup/outputs/` into the `worlds/` directory:
```bash
cp -r ../navMission_setup/outputs/world4 worlds/
```

## Notes
- This environment is **separate** from the main source code in `src/`
- Changes made here do NOT affect the main rover packages
- To sync rover updates back, copy the modified files to the main `rover/` directory
