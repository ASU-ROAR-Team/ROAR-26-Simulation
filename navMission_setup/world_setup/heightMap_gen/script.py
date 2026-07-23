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
            f"No {description} was found inside:\n{folder}"
        )

    if len(files) > 1:
        raise RuntimeError(
            f"More than one {description} was found inside:\n{folder}"
        )

    return files[0].resolve()


def run_ros_command(arguments: list[str]) -> None:
    setup_file = WORKSPACE_ROOT / "install" / "setup.bash"

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
        description="Generate a metric heightmap from a Gazebo world."
    )

    parser.add_argument("--input-world", "-i", type=Path)
    parser.add_argument("--output", "-o", type=Path)
    parser.add_argument("--preview", type=Path)
    parser.add_argument("--resolution", type=float, default=0.25)

    args = parser.parse_args()

    input_world = (
        args.input_world.resolve()
        if args.input_world
        else find_single_file(
            DEFAULT_INPUT_DIR,
            "*.world",
            "generated world file",
        )
    )

    output = (
        args.output.resolve()
        if args.output
        else (DEFAULT_OUTPUT_DIR / "heightmap.npz").resolve()
    )

    preview = (
        args.preview.resolve()
        if args.preview
        else (DEFAULT_OUTPUT_DIR / "heightmap.png").resolve()
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    preview.parent.mkdir(parents=True, exist_ok=True)

    command = [
        "ros2",
        "run",
        "rock_generator",
        "generate_heightmap",
        str(input_world),
        "-o",
        str(output),
        "--preview",
        str(preview),
        "--resolution",
        str(args.resolution),
    ]

    print("=" * 70)
    print("Heightmap Generation")
    print(f"Input world : {input_world}")
    print(f"Output NPZ  : {output}")
    print(f"Preview PNG : {preview}")
    print("=" * 70)

    run_ros_command(command)


if __name__ == "__main__":
    main()
