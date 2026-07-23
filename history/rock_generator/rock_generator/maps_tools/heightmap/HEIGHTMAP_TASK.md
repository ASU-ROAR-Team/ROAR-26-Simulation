# Automatic height map per generated world

The world fuser now generates a metric height map immediately after saving each
new `.world` file. The map includes the Mars Yard terrain and every generated
rock model in that world.

Default outputs:

```text
<worlds_directory>/heightmaps/<world_name>_heightmap.npz
<worlds_directory>/heightmaps/<world_name>_heightmap.png
```

Normal generation (height map is automatic):

```bash
ros2 run rock_generator generate_world \
  --input /path/to/obstacle_data.npy \
  --world-name marsyard.world
```

Change resolution:

```bash
ros2 run rock_generator generate_world \
  --input /path/to/obstacle_data.npy \
  --world-name marsyard.world \
  --heightmap-resolution 0.10
```

Generate a height map for an existing final world:

```bash
ros2 run rock_generator generate_heightmap \
  /path/to/final.world \
  -o /path/to/final_heightmap.npz \
  --preview /path/to/final_heightmap.png
```
