# Obstacle Data Generator (`obsData_gen`)

## Overview

`obsData_gen` procedurally places rock obstacles across a Mars Yard world using the terrain's baked heightmap. Each rock is dropped at its true ground Z (no physics air-drop / settle needed), spaced apart by a minimum distance, and tagged as collidable or non-collidable based on the **actual measured size of its mesh** in `rocks_ws`. The result is saved as a `.npy` obstacle array plus a human-readable summary, ready to be consumed by world-building and mission-scoring packages.

---

## Directory Structure

```text
obsData_gen/
├── generator.py     # Core generation logic (heightmap sampling, mesh sizing, placement)
├── script.py        # CLI entry point -- run this one
├── inputs/           # Optional: drop a heightmap .npz here to override the default
└── outputs/
    ├── obstacle_data.npy                    # Latest run (always overwritten)
    ├── obstacle_data_<timestamp>.npy        # Timestamped copy of every run (never overwritten)
    └── obstacle_data_info.txt               # Human-readable summary of the latest run
```

Rock meshes live one level up, in `world_setup/rocks_ws/`:

```text
rocks_ws/
├── rock_1/meshes/rock_1.obj
├── rock_2/meshes/rock_2.obj
├── ...
└── rock_9/meshes/rock_9.obj
```

Any folder matching `rock_N/meshes/rock_N.obj` is picked up automatically -- add, remove, or replace rocks here and the generator adapts without code changes.

---

## Two ID Systems -- Read This Before Wiring Up Other Packages

Every placed rock carries **two different IDs**, and they mean different things:

| Field | What it is | Who should use it |
|---|---|---|
| `mesh_id` | The `rocks_ws/rock_N` folder number this instance's geometry came from | World-building / spawning code, to know which `.obj`/`model.sdf` to instantiate |
| `rock_id` | A canonical placement ID, **independent** of `mesh_id` | The mission-scoring / blacklist package, to decide collidability |

**`rock_id` follows a strict parity convention: ODD = collidable, EVEN = non-collidable.** This means a consumer never needs a lookup table to check collidability -- `rock_id % 2 == 1` is the whole check. (The explicit `is_collidable` boolean is still included on every entry too, as the authoritative source of truth -- the parity is a convenience layered on top, not a replacement.)

`rock_id` is **not** the same number as `mesh_id`. Two rocks with the same `rock_id` parity can come from completely different meshes, and the same mesh (`mesh_id`) can appear under multiple `rock_id`s if pool balancing reused it (see below). If you ever add a script that spawns geometry, it must key off `mesh_id`, never `rock_id`.

### How `rock_id` is assigned

1. Every discovered mesh in `rocks_ws` is classified as collidable or non-collidable by its **measured height** (mesh bounding-box Z extent) against `--min-collidable-height` (default `0.15` m).
2. Collidable meshes get `rock_id`s `1, 3, 5, 7, ...` in discovery order.
3. Non-collidable meshes get `rock_id`s `2, 4, 6, 8, ...` in discovery order.
4. If the two pools are different sizes, the smaller pool's meshes are cycled/reused until both pools have equal length -- so the odd and even `rock_id` ranges come out the same count. Disable this with `--no-balance-model-pools` if you'd rather each mesh get exactly one `rock_id`.

Both the console output and `outputs/obstacle_data_info.txt` print the full `rock_id -> mesh_id` map at the top of every run, so you can check exactly what's collidable before anything gets exported.

---

## Placement Logic

- Rocks are sampled at random `(x, y)` within `x_range` / `y_range`, rejected if the terrain there is flat/void (`min_terrain_height`, `min_roughness`), and rejected if too close to an already-placed rock (`spacing`).
- `--collidable-ratio` controls the probability of drawing the next placed rock from the collidable `rock_id` pool vs. the non-collidable pool -- it does **not** set collidability directly (that's fully determined by which pool/`rock_id` gets chosen).
- With `--deadends`, a fixed barrier line of 5 rocks is placed across the course center first, always drawn from the collidable pool (a barrier that can't be hit isn't a barrier).
- Rock density scales with map area: `num_rocks = density * (x_range width * y_range height)`.

---

## Usage

Run from inside `obsData_gen/`:

```bash
python3 script.py
```

This auto-resolves:
- the heightmap: `inputs/*.npz`, falling back to `world_setup/initial_inputs/i_heightmap/`
- `rocks_ws`: `world_setup/rocks_ws/`

### Common flags

| Flag | Default | Meaning |
|---|---|---|
| `--world-name` | `marsyard.world` | World name tag written into each obstacle entry |
| `--density` | `0.012` | Rocks per m² |
| `-c, --collidable-ratio` | `0.5` | Probability of drawing from the collidable pool per placement |
| `--min-collidable-height` | `0.15` | Mesh height (m) at/above which a rock_ws mesh is classified collidable |
| `-s, --spacing` | `1.0` | Minimum center-to-center distance between placed rocks (m) |
| `--min-roughness` | `0.02` | Minimum local terrain Z std-dev to accept a placement cell |
| `--min-terrain-height` | `0.15` | Minimum terrain Z to accept a placement cell |
| `--deadends` | off | Place a 5-rock barrier line across the course center |
| `--heightmap` | auto-detected | Override path to a specific heightmap `.npz` |
| `--rocks-dir` | `world_setup/rocks_ws` | Override path to the rocks mesh directory |
| `--no-balance-model-pools` | balancing on | Disable reusing meshes to equalize collidable/non-collidable `rock_id` counts |
| `-o, --output` | `outputs/obstacle_data.npy` | Override output path |

Example -- denser field, stricter collidability cutoff:

```bash
python3 script.py --density 0.02 --min-collidable-height 0.20 --spacing 0.75
```

---

## Output Schema

Each entry in the exported `.npy` object array (and each line in `obstacle_data_info.txt`) has:

```text
id             sequential placement index (1, 2, 3, ...) -- just a label, not a mesh reference
name           "Rock_<id>" -- display name only
x, y, z        world position (m), z read directly from the heightmap
roll, pitch    always 0.0
yaw            random heading
rock_id        canonical id -- ODD = collidable, EVEN = non-collidable (see above)
mesh_id        rocks_ws/rock_N folder this instance's geometry came from
is_collidable  boolean, authoritative collidability flag
is_barrier     True only for --deadends barrier rocks
world_name     world name tag
length/width/height   mesh bounding-box size (m), looked up via mesh_id
frame_id       always "world"
```

---

## Files Produced Per Run

- `outputs/obstacle_data.npy` -- always overwritten, "latest" pointer for downstream tooling
- `outputs/obstacle_data_<timestamp>.npy` -- one snapshot per run, never overwritten
- `outputs/obstacle_data_info.txt` -- run config, the `rock_id -> mesh_id` map, and a full per-rock listing
