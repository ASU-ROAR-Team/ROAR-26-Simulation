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
        description="Generate a terrain costmap from a metric heightmap."
    )

    parser.add_argument("--input-heightmap", "-i", type=Path)
    parser.add_argument("--output", "-o", type=Path)
    parser.add_argument("--preview", type=Path)
    parser.add_argument("--csv-dir", type=Path)
    parser.add_argument("--gradient-scale", type=float, default=150.0)
    parser.add_argument("--stability-scale", type=float, default=90.0)

    args = parser.parse_args()

    input_heightmap = (
        args.input_heightmap.resolve()
        if args.input_heightmap
        else find_single_file(
            DEFAULT_INPUT_DIR,
            "*.npz",
            "heightmap NPZ file",
        )
    )

    output = (
        args.output.resolve()
        if args.output
        else (DEFAULT_OUTPUT_DIR / "costmap.npz").resolve()
    )

    preview = (
        args.preview.resolve()
        if args.preview
        else (DEFAULT_OUTPUT_DIR / "costmap.png").resolve()
    )

    csv_dir = (
        args.csv_dir.resolve()
        if args.csv_dir
        else (DEFAULT_OUTPUT_DIR / "csv").resolve()
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    preview.parent.mkdir(parents=True, exist_ok=True)
    csv_dir.mkdir(parents=True, exist_ok=True)

    command = [
        "python3",
        str(SCRIPT_DIR / "costmap_generator.py"),
        str(input_heightmap),
        "-o",
        str(output),
        "--preview",
        str(preview),
        "--csv-dir",
        str(csv_dir),
        "--gradient-scale",
        str(args.gradient_scale),
        "--stability-scale",
        str(args.stability_scale),
    ]

    print("=" * 70)
    print("Costmap Generation")
    print(f"Input heightmap : {input_heightmap}")
    print(f"Output NPZ      : {output}")
    print(f"Preview PNG     : {preview}")
    print(f"CSV directory   : {csv_dir}")
    print("=" * 70)

    subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
