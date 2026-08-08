#!/usr/bin/env python3

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


SETUP_DIR = Path(__file__).resolve().parent
WORLD_SETUP_DIR = SETUP_DIR / "world_setup"

INITIAL_WORLD_DIR = WORLD_SETUP_DIR / "initial_inputs" / "i_world"
INITIAL_HEIGHTMAP_DIR = WORLD_SETUP_DIR / "initial_inputs" / "i_heightmap"

MASTER_OUTPUTS_DIR = SETUP_DIR / "outputs"

OBS_SCRIPT = WORLD_SETUP_DIR / "obsData_gen" / "script.py"
WORLD_SCRIPT = WORLD_SETUP_DIR / "world_gen" / "script.py"
HEIGHTMAP_SCRIPT = WORLD_SETUP_DIR / "heightMap_gen" / "script.py"
COSTMAP_SCRIPT = WORLD_SETUP_DIR / "costMap_gen" / "script.py"
ARUCO_SCRIPT = WORLD_SETUP_DIR / "aruco_gen" / "script.py"
ARUCO_MODELS_DIR = WORLD_SETUP_DIR / "aruco_gen" / "models"


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
) -> tuple[str, str]:
    if cli_index is not None and cli_name is not None:
        raise ValueError(
            "Use either --index or --name, not both."
        )

    if cli_index is not None:
        safe_index = sanitize_name(
            cli_index,
            "Dataset index",
        )
        return f"worldData_{safe_index}", "index"

    if cli_name is not None:
        safe_name = sanitize_name(
            cli_name,
            "Dataset name",
        )
        return safe_name, "custom"

    while True:
        print()
        print("Choose dataset naming mode:")
        print("1) Enter an index after world Data_")
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
                        f"worldData_{safe_index}",
                        "index",
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
                    return safe_name, "custom"

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
            "Index appended after world Data_. "
            "Example: --index 007 creates world Data_007."
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
        default=0.15,
    )

    parser.add_argument(
        "--deadends",
        action="store_true",
    )

    parser.add_argument(
        "--target-rock-count",
        type=int,
        default=None,
    )

    parser.add_argument(
        "--rock-seed",
        type=int,
        default=None,
    )

    parser.add_argument(
        "--corridor-width",
        type=float,
        default=0.0,
    )

    parser.add_argument(
        "--corridor-center-x",
        type=float,
        default=0.0,
    )

    parser.add_argument(
        "--deadend-count",
        type=int,
        default=0,
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

    parser.add_argument(
        "--aruco-count",
        type=int,
        default=15,
    )

    parser.add_argument(
        "--aruco-id-start",
        type=int,
        default=1,
    )

    parser.add_argument(
        "--aruco-seed",
        type=int,
        default=None,
    )

    parser.add_argument(
        "--aruco-marker-size",
        type=float,
        default=0.21,
    )

    parser.add_argument(
        "--aruco-min-spacing",
        type=float,
        default=3.0,
    )

    parser.add_argument(
        "--aruco-obstacle-clearance",
        type=float,
        default=1.5,
    )

    args = parser.parse_args()

    dataset_name, naming_mode = request_dataset_name(
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

    run_dir = MASTER_OUTPUTS_DIR / dataset_name

    if run_dir.exists():
        raise FileExistsError(
            f"Output dataset already exists:\n"
            f"{run_dir}\n"
            "Choose another index or custom name."
        )

    aruco_models_dir = run_dir / "models"
    csv_dir = run_dir / "csv"

    for directory in (
        run_dir,
        aruco_models_dir,
        csv_dir,
    ):
        directory.mkdir(
            parents=True,
            exist_ok=True,
        )

    file_prefix = (
        f"{dataset_name}_"
        f"d{args.density:.3f}_"
        f"c{args.collidable_ratio:.2f}"
    )

    obstacle_output = (
        run_dir / "obstacle_data.npy"
    )

    world_output = (
        run_dir / f"{dataset_name}.world"
    )

    aruco_yaml_output = (
        run_dir / "aruco_data.yaml"
    )

    aruco_npy_output = (
        run_dir / "aruco_data.npy"
    )

    heightmap_output = (
        run_dir / "heightmap.npz"
    )

    heightmap_preview = (
        run_dir / "heightmap.png"
    )

    costmap_output = (
        run_dir / "costmap.npz"
    )

    costmap_preview = (
        run_dir / "costmap.png"
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

            "--corridor-width",
            str(args.corridor_width),

            "--corridor-center-x",
            str(args.corridor_center_x),

            "--deadend-count",
            str(args.deadend_count),
            "--output",
            str(obstacle_output),
        ]

        if args.target_rock_count is not None:
            obs_command.extend([
                "--target-rock-count",
                str(args.target_rock_count),
            ])

        if args.rock_seed is not None:
            obs_command.extend([
                "--seed",
                str(args.rock_seed),
            ])

        if args.deadends:
            obs_command.append("--deadends")

        run_stage(
            obs_command,
            "Stage 1/5 — Obstacle Data Generation",
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
            "Stage 2/5 — World Generation",
        )

        aruco_seed = (
            args.aruco_seed
            if args.aruco_seed is not None
            else abs(hash(dataset_name)) % (2**31)
        )

        run_stage(
            [
                sys.executable,
                str(ARUCO_SCRIPT),

                "--input-world",
                str(world_output),

                "--output-world",
                str(world_output),

                "--obstacle-data",
                str(obstacle_output),

                "--terrain-heightmap",
                str(base_heightmap),

                "--output-yaml",
                str(aruco_yaml_output),

                "--output-npy",
                str(aruco_npy_output),

                "--models-source",
                str(ARUCO_MODELS_DIR),

                "--models-output",
                str(aruco_models_dir),

                "--count",
                str(args.aruco_count),

                "--marker-id-start",
                str(args.aruco_id_start),

                "--seed",
                str(aruco_seed),

                "--marker-size",
                str(args.aruco_marker_size),

                "--min-spacing",
                str(args.aruco_min_spacing),

                "--obstacle-clearance",
                str(args.aruco_obstacle_clearance),
            ],
            "Stage 3/5 — ArUco + Ground Truth + World Plugins",
        )

        # ---------------------------------------------------------
        # Stage 4/5 — Heightmap Generation
        # Zero-rock worlds reuse the original Marsyard heightmap.
        # ---------------------------------------------------------
        if args.target_rock_count == 0:
            print("\n" + "=" * 72)
            print("Stage 4/5 — Heightmap Generation")
            print("=" * 72)
            print("Zero-rock world detected.")
            print(f"Using base Marsyard heightmap: {base_heightmap}")

            shutil.copy2(
                base_heightmap,
                heightmap_output,
            )

            base_preview = base_heightmap.with_suffix('.png')
            if base_preview.exists():
                shutil.copy2(
                    base_preview,
                    heightmap_preview,
                )

            print(f"Heightmap copied to: {heightmap_output}")

        else:
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
                "Stage 4/5 — Heightmap Generation",
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
            "Stage 5/5 — Costmap Generation",
        )

        info_candidates = sorted(
            obstacle_output.parent.glob(
                "*_info.txt"
            )
        )

        if info_candidates:
            expected_info = (
                run_dir
                / "obstacle_data_info.txt"
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
                    f"Target rock count       : {args.target_rock_count}",
                    f"Rock seed               : {args.rock_seed}",
                    f"Corridor width          : {args.corridor_width}",
                    f"Corridor center X       : {args.corridor_center_x}",
                    f"Dead-end count          : {args.deadend_count}",
                    f"Heightmap resolution    : {args.heightmap_resolution}",
                    f"Gradient scale          : {args.gradient_scale}",
                    f"Stability scale         : {args.stability_scale}",
                    f"ArUco count             : {args.aruco_count}",
                    f"ArUco ID start          : {args.aruco_id_start}",
                    f"ArUco seed              : {aruco_seed}",
                    f"ArUco marker size       : {args.aruco_marker_size}",
                    f"Obstacle data           : {obstacle_output}",
                    f"ArUco GT YAML           : {aruco_yaml_output}",
                    f"ArUco GT NPY            : {aruco_npy_output}",
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
