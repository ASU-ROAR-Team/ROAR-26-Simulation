#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

# Ensure local imports work cleanly when script is executed directly
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from generator import generate_obstacle_data

DEFAULT_INPUT_DIR = SCRIPT_DIR / "inputs"
DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "outputs"
INITIAL_INPUT_DIR = SCRIPT_DIR.parent / "initial_inputs" / "i_heightmap"
DEFAULT_ROCKS_DIR = SCRIPT_DIR.parent / "rocks_ws"


def find_heightmap_file(custom_path: Path | None) -> Path:
    if custom_path:
        resolved = custom_path.resolve()
        if not resolved.is_file():
            raise FileNotFoundError(f"Specified heightmap file not found: {resolved}")
        return resolved

    # Search in obsData_gen/inputs first, then fallback to initial_inputs/i_heightmap
    for folder in [DEFAULT_INPUT_DIR, INITIAL_INPUT_DIR]:
        files = sorted(folder.glob("*.npz"))
        if len(files) == 1:
            return files[0].resolve()
        elif len(files) > 1:
            return files[0].resolve()

    raise FileNotFoundError(
        f"No heightmap NPZ file was found inside:\n{DEFAULT_INPUT_DIR} or {INITIAL_INPUT_DIR}"
    )


def find_rocks_dir(custom_path: Path | None) -> Path:
    if custom_path:
        resolved = custom_path.resolve()
        if not resolved.is_dir():
            raise FileNotFoundError(f"Specified rocks directory not found: {resolved}")
        return resolved

    if DEFAULT_ROCKS_DIR.is_dir():
        return DEFAULT_ROCKS_DIR.resolve()

    raise FileNotFoundError(
        f"No rocks_ws directory was found at:\n{DEFAULT_ROCKS_DIR}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate obstacle data for one Mars Yard world (Standalone mode)."
    )

    parser.add_argument("--world-name", default="marsyard.world")
    parser.add_argument("--heightmap", type=Path)
    parser.add_argument("--rocks-dir", type=Path)
    parser.add_argument("--density", type=float, default=0.012)
    parser.add_argument("--collidable-ratio", "-c", type=float, default=0.5)
    parser.add_argument("--min-collidable-size", type=float, default=0.15)
    parser.add_argument("--spacing", "-s", type=float, default=1.0)
    parser.add_argument("--min-roughness", type=float, default=0.02)
    parser.add_argument("--min-terrain-height", type=float, default=-1.3)
    parser.add_argument("--deadends", action="store_true")
    parser.add_argument("--no-balance-model-pools", dest="balance_model_pools",
                         action="store_false", default=True)
    parser.add_argument("--no-clean-outputs", dest="clean_previous_outputs",
                         action="store_false", default=True)
    parser.add_argument("--output", "-o", type=Path)

    args = parser.parse_args()

    heightmap = find_heightmap_file(args.heightmap)
    rocks_dir = find_rocks_dir(args.rocks_dir)

    output = (
        args.output.resolve()
        if args.output
        else (DEFAULT_OUTPUT_DIR / "obstacle_data.npy").resolve()
    )

    output.parent.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("Obstacle Data Generation (Standalone)")
    print(f"Heightmap : {heightmap}")
    print(f"Rocks Dir : {rocks_dir}")
    print(f"Output    : {output}")
    print("=" * 70)

    generate_obstacle_data(
        world_name=args.world_name,
        density=args.density,
        collidable_ratio=args.collidable_ratio,
        spacing=args.spacing,
        min_terrain_height=args.min_terrain_height,
        min_roughness=args.min_roughness,
        deadends=args.deadends,
        output_file=str(output),
        heightmap_path=str(heightmap),
        rocks_dir=str(rocks_dir),
        min_collidable_size_m=args.min_collidable_size,
        balance_model_pools=args.balance_model_pools,
        clean_previous_outputs=args.clean_previous_outputs,
    )


if __name__ == "__main__":
    main()