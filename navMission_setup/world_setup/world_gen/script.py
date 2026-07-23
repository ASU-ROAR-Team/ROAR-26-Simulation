#!/usr/bin/env python3

import argparse
import shlex
import subprocess
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = Path(__file__).resolve().parents[4]

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
            f"Workspace setup file was not found:\n{setup_file}"
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
        description="Fuse obstacle data into a standalone Gazebo world."
    )

    parser.add_argument("--input-data", "-i", type=Path)
    parser.add_argument("--base-world", "-w", type=Path)
    parser.add_argument("--output-world", "-o", type=Path)

    args = parser.parse_args()

    obstacle_data = (
        args.input_data.resolve()
        if args.input_data
        else find_single_file(
            DEFAULT_INPUT_DIR,
            "*.npy",
            "obstacle-data NPY file",
        )
    )

    base_world = (
        args.base_world.resolve()
        if args.base_world
        else find_single_file(
            DEFAULT_INPUT_DIR,
            "*.world",
            "base world file",
        )
    )

    output_world = (
        args.output_world.resolve()
        if args.output_world
        else (DEFAULT_OUTPUT_DIR / "generated.world").resolve()
    )

    output_world.parent.mkdir(parents=True, exist_ok=True)

    command = [
        "python3",
        str(SCRIPT_DIR / "world_generator.py"),
        "-i",
        str(obstacle_data),
        "-w",
        str(base_world),
        "-o",
        str(output_world),
    ]

    print("=" * 70)
    print("World Generation")
    print(f"Base world    : {base_world}")
    print(f"Obstacle data : {obstacle_data}")
    print(f"Output world  : {output_world}")
    print("=" * 70)

    subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
