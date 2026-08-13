#!/usr/bin/env python3

import argparse
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


SETUP_DIR = Path(__file__).resolve().parent
WORLD_SETUP_DIR = SETUP_DIR / "world_setup"

INITIAL_WORLD_DIR = WORLD_SETUP_DIR / "initial_inputs" / "i_world"
INITIAL_HEIGHTMAP_DIR = WORLD_SETUP_DIR / "initial_inputs" / "i_heightmap"

MASTER_OUTPUTS_DIR = SETUP_DIR / "outputs"

OBS_SCRIPT = WORLD_SETUP_DIR / "obsData_gen" / "script.py"
WORLD_SCRIPT = WORLD_SETUP_DIR / "world_gen" / "script.py"
HEIGHTMAP_SCRIPT = WORLD_SETUP_DIR / "heightMap_gen" / "script.py"
COSTMAP_SCRIPT = WORLD_SETUP_DIR / "costMap_gen" / "script.py"

NEW_MARSYARD_URI = "model://erc_marsyard_2026"
NEW_MARSYARD_COLLISION_MESH = "marsyard_collision.obj"


def validate_base_terrain(base_world: Path, base_heightmap: Path) -> None:
    """Reject stale / mismatched Mars Yard inputs before placing any rocks."""
    root = ET.parse(base_world).getroot()
    world = root.find("world") if root.tag != "world" else root
    if world is None:
        raise ValueError(f"Base world has no <world> element: {base_world}")

    terrain_includes = [
        include
        for include in world.findall("include")
        if (include.findtext("uri") or "").strip() == NEW_MARSYARD_URI
    ]
    if len(terrain_includes) != 1:
        found = sorted(
            (include.findtext("uri") or "").strip()
            for include in world.findall("include")
        )
        raise ValueError(
            f"Base world must include exactly one {NEW_MARSYARD_URI}; found {found}"
        )

    pose = [float(value) for value in (terrain_includes[0].findtext("pose") or "0 0 0 0 0 0").split()]
    if len(pose) != 6 or any(abs(value) > 1e-9 for value in pose):
        raise ValueError(
            "The new Mars Yard must use the survey frame at pose '0 0 0 0 0 0'; "
            f"found {pose}"
        )

    with np.load(base_heightmap, allow_pickle=False) as heightmap:
        required = {"xs", "ys", "grid", "resolution", "world_path", "terrain_mesh_path"}
        missing = required.difference(heightmap.files)
        if missing:
            raise ValueError(f"Base heightmap is missing fields: {sorted(missing)}")
        xs = np.asarray(heightmap["xs"], dtype=np.float64)
        ys = np.asarray(heightmap["ys"], dtype=np.float64)
        grid = np.asarray(heightmap["grid"], dtype=np.float64)
        source = str(np.asarray(heightmap["terrain_mesh_path"]).item())

    if Path(source).name != NEW_MARSYARD_COLLISION_MESH:
        raise ValueError(
            "Base heightmap was not generated from the new Mars Yard collision mesh: "
            f"{source}"
        )
    if grid.shape != (len(ys), len(xs)) or len(xs) < 2 or len(ys) < 2:
        raise ValueError("Base heightmap axes/grid dimensions are inconsistent")
    if not (np.all(np.diff(xs) > 0.0) and np.all(np.diff(ys) > 0.0)):
        raise ValueError("Base heightmap X/Y axes must increase in the Gazebo world frame")

    print("Validated terrain pipeline:")
    print(f"  World model : {NEW_MARSYARD_URI}")
    print("  Model pose  : 0 0 0 0 0 0 (survey frame)")
    print(f"  Height mesh : {source}")
    print(f"  Map bounds  : X=[{xs[0]:.3f}, {xs[-1]:.3f}] Y=[{ys[0]:.3f}, {ys[-1]:.3f}]")


def find_single_file(
    folder: Path,
    pattern: str,
    description: str,
) -> Path:
    files = sorted(folder.glob(pattern))

    if not files:
        raise FileNotFoundError(
            f"No {description} was found inside:\n"
            f"{folder}\n"
            f"Expected pattern: {pattern}"
        )

    if len(files) > 1:
        raise RuntimeError(
            f"More than one {description} was found inside:\n"
            f"{folder}\n"
            "Pass the required path explicitly."
        )

    return files[0].resolve()


def sanitize_name(value: str, description: str) -> str:
    safe_value = "".join(
        character
        if character.isalnum() or character in ("_", "-")
        else "_"
        for character in value.strip()
    ).strip("_")

    if not safe_value:
        raise ValueError(
            f"{description} must contain at least one letter or number."
        )

    return safe_value


def request_dataset_name(
    cli_index: str | None,
    cli_name: str | None,
) -> tuple[str, str, str]:
    """
    Returns (dataset_name, naming_mode, tag).

    tag is the short piece used to suffix every individual output file
    (e.g. "02" for --index 02, or the full custom name for --name mode).
    """
    if cli_index is not None and cli_name is not None:
        raise ValueError(
            "Use either --index or --name, not both."
        )

    if cli_index is not None:
        safe_index = sanitize_name(
            cli_index,
            "Dataset index",
        )
        return f"world_Data_{safe_index}", "index", safe_index

    if cli_name is not None:
        safe_name = sanitize_name(
            cli_name,
            "Dataset name",
        )
        return safe_name, "custom", safe_name

    while True:
        print()
        print("Choose dataset naming mode:")
        print("1) Enter an index after world_Data_")
        print("2) Enter a complete custom name")

        choice = input("Select [1/2]: ").strip()

        if choice == "1":
            while True:
                index = input(
                    "Enter dataset index: "
                ).strip()

                if index:
                    safe_index = sanitize_name(
                        index,
                        "Dataset index",
                    )
                    return (
                        f"world_Data_{safe_index}",
                        "index",
                        safe_index,
                    )

                print("Dataset index cannot be empty.")

        elif choice == "2":
            while True:
                custom_name = input(
                    "Enter complete dataset name: "
                ).strip()

                if custom_name:
                    safe_name = sanitize_name(
                        custom_name,
                        "Dataset name",
                    )
                    return safe_name, "custom", safe_name

                print("Dataset name cannot be empty.")

        else:
            print("Invalid selection. Please enter 1 or 2.")


def run_stage(
    command: list[str],
    title: str,
) -> None:
    print()
    print("=" * 78)
    print(title)
    print("=" * 78)
    print(" ".join(str(item) for item in command))
    print("=" * 78)

    subprocess.run(command, check=True)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Run the complete Mars Yard world-generation pipeline "
            "and store the final dataset inside the selected output folder."
        )
    )

    parser.add_argument(
        "--index",
        type=str,
        help=(
            "Index appended after world_Data_. "
            "Example: --index 02 creates world_Data_02."
        ),
    )

    parser.add_argument(
        "--name",
        type=str,
        help=(
            "Complete custom dataset name without adding World_Data_."
        ),
    )

    parser.add_argument(
        "--base-world",
        type=Path,
    )

    parser.add_argument(
        "--base-heightmap",
        type=Path,
    )

    parser.add_argument(
        "--world-name",
        default="marsyard.world",
    )

    parser.add_argument(
        "--density",
        type=float,
        default=0.012,
    )

    parser.add_argument(
        "--collidable-ratio",
        "-c",
        type=float,
        default=0.5,
    )

    parser.add_argument(
        "--spacing",
        "-s",
        type=float,
        default=1.0,
    )

    parser.add_argument(
        "--min-roughness",
        type=float,
        default=0.02,
    )

    parser.add_argument(
        "--min-terrain-height",
        type=float,
        default=-1.5,
    )

    parser.add_argument(
        "--deadends",
        action="store_true",
    )

    parser.add_argument(
        "--heightmap-resolution",
        type=float,
        default=0.25,
    )

    parser.add_argument(
        "--gradient-scale",
        type=float,
        default=150.0,
    )

    parser.add_argument(
        "--stability-scale",
        type=float,
        default=90.0,
    )

    args = parser.parse_args()

    dataset_name, naming_mode, tag = request_dataset_name(
        args.index,
        args.name,
    )

    base_world = (
        args.base_world.resolve()
        if args.base_world
        else find_single_file(
            INITIAL_WORLD_DIR,
            "*.world",
            "initial Gazebo world",
        )
    )

    base_heightmap = (
        args.base_heightmap.resolve()
        if args.base_heightmap
        else find_single_file(
            INITIAL_HEIGHTMAP_DIR,
            "*.npz",
            "initial terrain heightmap",
        )
    )

    validate_base_terrain(base_world, base_heightmap)

    run_dir = MASTER_OUTPUTS_DIR / dataset_name

    if run_dir.exists():
        raise FileExistsError(
            f"Output dataset already exists:\n"
            f"{run_dir}\n"
            "Choose another index or custom name."
        )

    obstacle_dir = run_dir / "obstacle_data"
    world_dir = run_dir / "world"
    heightmap_dir = run_dir / "heightmap"
    costmap_dir = run_dir / "costmap"
    csv_dir = costmap_dir / "csv"

    for directory in (
        obstacle_dir,
        world_dir,
        heightmap_dir,
        costmap_dir,
        csv_dir,
    ):
        directory.mkdir(
            parents=True,
            exist_ok=True,
        )

    # Every output file is tagged with just the dataset index/name --
    # no density or collidable-ratio baked into filenames anymore.
    obstacle_output = (
        obstacle_dir / f"obstacle_data_{tag}.npy"
    )

    world_output = (
        world_dir / f"{dataset_name}.world"
    )

    heightmap_output = (
        heightmap_dir
        / f"heightmap_{tag}.npz"
    )

    heightmap_preview = (
        heightmap_dir
        / f"heightmap_{tag}.png"
    )

    costmap_output = (
        costmap_dir
        / f"costmap_{tag}.npz"
    )

    costmap_preview = (
        costmap_dir
        / f"costmap_{tag}.png"
    )

    try:
        obs_command = [
            sys.executable,
            str(OBS_SCRIPT),
            "--world-name",
            args.world_name,
            "--heightmap",
            str(base_heightmap),
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
            "--output",
            str(obstacle_output),
        ]

        if args.deadends:
            obs_command.append("--deadends")

        run_stage(
            obs_command,
            "Stage 1/4 — Obstacle Data Generation",
        )

        run_stage(
            [
                sys.executable,
                str(WORLD_SCRIPT),
                "--input-data",
                str(obstacle_output),
                "--base-world",
                str(base_world),
                "--output-world",
                str(world_output),
            ],
            "Stage 2/4 — World Generation",
        )

        run_stage(
            [
                sys.executable,
                str(HEIGHTMAP_SCRIPT),
                "--input-world",
                str(world_output),
                "--output",
                str(heightmap_output),
                "--preview",
                str(heightmap_preview),
                "--resolution",
                str(args.heightmap_resolution),
            ],
            "Stage 3/4 — Heightmap Generation",
        )

        run_stage(
            [
                sys.executable,
                str(COSTMAP_SCRIPT),
                "--input-heightmap",
                str(heightmap_output),
                "--output",
                str(costmap_output),
                "--preview",
                str(costmap_preview),
                "--csv-dir",
                str(csv_dir),
                "--gradient-scale",
                str(args.gradient_scale),
                "--stability-scale",
                str(args.stability_scale),
            ],
            "Stage 4/4 — Costmap Generation",
        )

        info_candidates = sorted(
            obstacle_output.parent.glob(
                "*_info.txt"
            )
        )

        if info_candidates:
            expected_info = (
                obstacle_dir
                / f"obstacle_data_{tag}_info.txt"
            )

            if info_candidates[0] != expected_info:
                shutil.copy2(
                    info_candidates[0],
                    expected_info,
                )

        metadata = run_dir / "metadata.txt"

        metadata.write_text(
            "\n".join(
                [
                    "ROAR World Dataset",
                    "=" * 50,
                    f"Dataset name            : {dataset_name}",
                    f"Naming mode             : {naming_mode}",
                    f"Base world              : {base_world}",
                    f"Base heightmap          : {base_heightmap}",
                    f"Requested density       : {args.density}",
                    f"Requested collidable    : {args.collidable_ratio}",
                    f"Rock spacing            : {args.spacing}",
                    f"Minimum roughness       : {args.min_roughness}",
                    f"Minimum terrain height  : {args.min_terrain_height}",
                    f"Dead-end generation     : {args.deadends}",
                    f"Heightmap resolution    : {args.heightmap_resolution}",
                    f"Gradient scale          : {args.gradient_scale}",
                    f"Stability scale         : {args.stability_scale}",
                    f"Obstacle data           : {obstacle_output}",
                    f"Generated world         : {world_output}",
                    f"Generated heightmap     : {heightmap_output}",
                    f"Generated costmap       : {costmap_output}",
                    "",
                ]
            ),
            encoding="utf-8",
        )

    except Exception:
        print()
        print("Pipeline failed.")
        print(
            f"Partial output was left at: "
            f"{run_dir}"
        )
        raise

    print()
    print("=" * 78)
    print("WORLD PIPELINE COMPLETED SUCCESSFULLY")
    print("=" * 78)
    print(f"Dataset name    : {dataset_name}")
    print(f"Naming mode     : {naming_mode}")
    print(f"Dataset folder  : {run_dir}")
    print(f"World           : {world_output}")
    print(f"Heightmap       : {heightmap_output}")
    print(f"Costmap         : {costmap_output}")
    print(f"Metadata        : {metadata}")
    print("=" * 78)


if __name__ == "__main__":
    main()