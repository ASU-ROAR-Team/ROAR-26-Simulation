#!/usr/bin/env python3

import argparse
import shlex
import subprocess
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = Path(__file__).resolve().parents[3]

DEFAULT_INPUT_DIR = SCRIPT_DIR / "inputs"
DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "outputs"


def find_single_file(folder: Path, pattern: str, description: str) -> Path:
    files = sorted(folder.glob(pattern))

    if not files:
        raise FileNotFoundError(
            f"No {description} was found inside:\n{folder}\n"
            f"Expected pattern: {pattern}"
        )

    if len(files) > 1:
        raise RuntimeError(
            f"More than one {description} was found inside:\n{folder}\n"
            "Pass the required file explicitly through the command line."
        )

    return files[0].resolve()


def run_ros_command(arguments: list[str]) -> None:
    setup_file = WORKSPACE_ROOT / "install" / "setup.bash"

    if not setup_file.exists():
        raise FileNotFoundError(
            f"Workspace setup file was not found:\n{setup_file}\n"
            "Build the rock_generator package first."
        )

    command = (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(setup_file))} && "
        + shlex.join(arguments)
    )

    subprocess.run(
        ["bash", "-lc", command],
        cwd=WORKSPACE_ROOT,
        check=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate obstacle data for one Mars Yard world."
    )

    parser.add_argument("--world-name", default="marsyard.world")
    parser.add_argument("--heightmap", type=Path)
    parser.add_argument("--density", type=float, default=0.012)
    parser.add_argument("--collidable-ratio", "-c", type=float, default=0.5)
    parser.add_argument("--spacing", "-s", type=float, default=1.0)
    parser.add_argument("--min-roughness", type=float, default=0.02)
    parser.add_argument("--min-terrain-height", type=float, default=0.15)
    parser.add_argument("--deadends", action="store_true")
    parser.add_argument("--output", "-o", type=Path)

    args = parser.parse_args()

    heightmap = (
        args.heightmap.resolve()
        if args.heightmap
        else find_single_file(
            DEFAULT_INPUT_DIR,
            "*.npz",
            "heightmap NPZ file",
        )
    )

    output = (
        args.output.resolve()
        if args.output
        else (DEFAULT_OUTPUT_DIR / "obstacle_data.npy").resolve()
    )

    output.parent.mkdir(parents=True, exist_ok=True)

    command = [
        "ros2",
        "run",
        "rock_generator",
        "generate_obs",
        "--world-name",
        args.world_name,
        "--heightmap",
        str(heightmap),
        "--density",
        str(args.density),
        "--collidable-ratio",
        str(args.collidable_ratio),
        "--spacing",
        str(args.spacing),
        "--min-roughness",
        str(args.min_roughness),
        "--min-terrain-height",
        str(args.min_terrain_height),
        "-o",
        str(output),
    ]

    if args.deadends:
        command.append("--deadends")

    print("=" * 70)
    print("Obstacle Data Generation")
    print(f"Heightmap : {heightmap}")
    print(f"Output    : {output}")
    print("=" * 70)

    run_ros_command(command)


if __name__ == "__main__":
    main()
