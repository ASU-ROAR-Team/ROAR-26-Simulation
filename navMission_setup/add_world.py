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
            "Pass the required path explicitly."
        )

    return files[0].resolve()


def clean_pipeline() -> None:
    print("=" * 78)
    print("Cleaning Pipeline Temporary Folders (excluding initial_inputs)")
    print("=" * 78)
    stages = ["obsData_gen", "world_gen", "heightMap_gen", "costMap_gen"]
    for stage in stages:
        for folder_name in ["inputs", "outputs"]:
            folder = WORLD_SETUP_DIR / stage / folder_name
            if folder.exists():
                print(f"Cleaning: {folder.relative_to(SETUP_DIR)}")
                for item in folder.iterdir():
                    if item.name == ".gitkeep":
                        continue
                    try:
                        if item.is_dir():
                            shutil.rmtree(item)
                        else:
                            item.unlink()
                    except Exception as e:
                        print(f"  Error removing {item}: {e}")
    print("Pipeline cleaned successfully.")
    print("=" * 78)


def run_stage(command: list[str], title: str) -> None:
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
            "Run the complete Mars Yard world-generation pipeline and store "
            "the final dataset inside outputs/world{index}."
        )
    )

    # Positional Arguments (required)
    parser.add_argument(
        "density",
        type=float,
        help="Rock density in rocks per square meter (e.g. 0.012)"
    )
    parser.add_argument(
        "percentage",
        type=float,
        help="Ratio (0.0 to 1.0) or percentage (0 to 100) of solid/collidable vs ghost/non-collidable rocks (e.g. 50)"
    )
    parser.add_argument(
        "index",
        type=int,
        help="Index of the world (e.g. 1)"
    )

    # Optional Arguments
    parser.add_argument(
        "--name",
        type=str,
        help="Generated world name. If omitted, defaults to world{index}.",
    )
    parser.add_argument("--base-world", type=Path)
    parser.add_argument("--base-heightmap", type=Path)

    parser.add_argument("--world-name", default="marsyard.world")
    parser.add_argument("--spacing", "-s", type=float, default=1.0)
    parser.add_argument("--min-roughness", type=float, default=0.02)
    parser.add_argument("--min-terrain-height", type=float, default=0.15)
    parser.add_argument("--deadends", action="store_true")

    parser.add_argument("--heightmap-resolution", type=float, default=0.25)
    parser.add_argument("--gradient-scale", type=float, default=150.0)
    parser.add_argument("--stability-scale", type=float, default=90.0)
    parser.add_argument(
        "--run",
        action="store_true",
        help="Launch the generated world dataset immediately after generation."
    )

    args = parser.parse_args()

    # Clean the pipeline folders
    clean_pipeline()

    # Determine world name and safe world name
    world_name = args.name if args.name else f"world{args.index}"
    safe_world_name = "".join(
        character if character.isalnum() or character in ("_", "-") else "_"
        for character in world_name
    ).strip("_")

    if not safe_world_name:
        raise ValueError(
            "World name must contain at least one letter or number."
        )

    # Resolve initial inputs
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

    # Parse and convert percentage to ratio if necessary
    collidable_ratio = args.percentage
    if collidable_ratio > 1.0:
        collidable_ratio /= 100.0

    # Ensure output folders exist
    run_id = f"world{args.index}"
    run_dir = MASTER_OUTPUTS_DIR / run_id

    try:
        # -------------------------------------------------------------
        # Stage 1: Obstacle Data Generation (obsData_gen)
        # -------------------------------------------------------------
        obs_inputs_dir = WORLD_SETUP_DIR / "obsData_gen" / "inputs"
        obs_outputs_dir = WORLD_SETUP_DIR / "obsData_gen" / "outputs"
        obs_inputs_dir.mkdir(parents=True, exist_ok=True)
        obs_outputs_dir.mkdir(parents=True, exist_ok=True)

        # Copy base world and base heightmap to inputs folder of obs gen
        shutil.copy2(base_world, obs_inputs_dir / base_world.name)
        shutil.copy2(base_heightmap, obs_inputs_dir / base_heightmap.name)

        obs_output_file = obs_outputs_dir / "obstacle_data.npy"

        obs_command = [
            sys.executable,
            str(OBS_SCRIPT),
            "--world-name",
            args.world_name,
            "--heightmap",
            str(obs_inputs_dir / base_heightmap.name),
            "--density",
            str(args.density),
            "--collidable-ratio",
            str(collidable_ratio),
            "--spacing",
            str(args.spacing),
            "--min-roughness",
            str(args.min_roughness),
            "--min-terrain-height",
            str(args.min_terrain_height),
            "--output",
            str(obs_output_file),
        ]

        if args.deadends:
            obs_command.append("--deadends")

        run_stage(obs_command, "Stage 1/4 — Obstacle Data Generation")

        # -------------------------------------------------------------
        # Stage 2: World Generation (world_gen)
        # -------------------------------------------------------------
        world_inputs_dir = WORLD_SETUP_DIR / "world_gen" / "inputs"
        world_outputs_dir = WORLD_SETUP_DIR / "world_gen" / "outputs"
        world_inputs_dir.mkdir(parents=True, exist_ok=True)
        world_outputs_dir.mkdir(parents=True, exist_ok=True)

        # Copy generated obstacle data and plain world to world gen inputs
        shutil.copy2(obs_output_file, world_inputs_dir / obs_output_file.name)
        for info_file in obs_outputs_dir.glob("*_info.txt"):
            shutil.copy2(info_file, world_inputs_dir / info_file.name)
        shutil.copy2(base_world, world_inputs_dir / base_world.name)

        world_output_file = world_outputs_dir / f"world{args.index}.world"

        world_command = [
            sys.executable,
            str(WORLD_SCRIPT),
            "--input-data",
            str(world_inputs_dir / obs_output_file.name),
            "--base-world",
            str(world_inputs_dir / base_world.name),
            "--output-world",
            str(world_output_file),
        ]

        run_stage(world_command, "Stage 2/4 — World Generation")

        # -------------------------------------------------------------
        # Stage 3: Heightmap Generation (heightMap_gen)
        # -------------------------------------------------------------
        hm_inputs_dir = WORLD_SETUP_DIR / "heightMap_gen" / "inputs"
        hm_outputs_dir = WORLD_SETUP_DIR / "heightMap_gen" / "outputs"
        hm_inputs_dir.mkdir(parents=True, exist_ok=True)
        hm_outputs_dir.mkdir(parents=True, exist_ok=True)

        # Copy fused world to heightmap gen inputs
        shutil.copy2(world_output_file, hm_inputs_dir / world_output_file.name)

        hm_output_file = hm_outputs_dir / "heightmap.npz"
        hm_preview_file = hm_outputs_dir / "heightmap.png"

        hm_command = [
            sys.executable,
            str(HEIGHTMAP_SCRIPT),
            "--input-world",
            str(hm_inputs_dir / world_output_file.name),
            "--output",
            str(hm_output_file),
            "--preview",
            str(hm_preview_file),
            "--resolution",
            str(args.heightmap_resolution),
        ]

        run_stage(hm_command, "Stage 3/4 — Heightmap Generation")

        # -------------------------------------------------------------
        # Stage 4: Costmap Generation (costMap_gen)
        # -------------------------------------------------------------
        cm_inputs_dir = WORLD_SETUP_DIR / "costMap_gen" / "inputs"
        cm_outputs_dir = WORLD_SETUP_DIR / "costMap_gen" / "outputs"
        cm_inputs_dir.mkdir(parents=True, exist_ok=True)
        cm_outputs_dir.mkdir(parents=True, exist_ok=True)

        # Copy generated heightmap to costmap gen inputs
        shutil.copy2(hm_output_file, cm_inputs_dir / hm_output_file.name)

        cm_output_file = cm_outputs_dir / "costmap.npz"
        cm_preview_file = cm_outputs_dir / "costmap.png"
        cm_csv_dir = cm_outputs_dir / "csv"

        cm_command = [
            sys.executable,
            str(COSTMAP_SCRIPT),
            "--input-heightmap",
            str(cm_inputs_dir / hm_output_file.name),
            "--output",
            str(cm_output_file),
            "--preview",
            str(cm_preview_file),
            "--csv-dir",
            str(cm_csv_dir),
            "--gradient-scale",
            str(args.gradient_scale),
            "--stability-scale",
            str(args.stability_scale),
        ]

        run_stage(cm_command, "Stage 4/4 — Costmap Generation")

        # -------------------------------------------------------------
        # Copy to Master Output Folder
        # -------------------------------------------------------------
        if run_dir.exists():
            print(f"Removing pre-existing master output folder: {run_dir}")
            shutil.rmtree(run_dir)
        run_dir.mkdir(parents=True, exist_ok=True)

        # Copied names to master folder
        master_obs = run_dir / "obstacle_data.npy"
        master_world = run_dir / f"world{args.index}.world"
        master_heightmap = run_dir / "heightmap.npz"
        master_heightmap_preview = run_dir / "heightmap.png"
        master_costmap = run_dir / "costmap.npz"
        master_costmap_preview = run_dir / "costmap.png"
        master_csv_dir = run_dir / "csv"

        shutil.copy2(obs_output_file, master_obs)
        for info_file in obs_outputs_dir.glob("*_info.txt"):
            shutil.copy2(info_file, run_dir / "obstacle_data_info.txt")

        shutil.copy2(world_output_file, master_world)
        shutil.copy2(hm_output_file, master_heightmap)
        if hm_preview_file.exists():
            shutil.copy2(hm_preview_file, master_heightmap_preview)

        shutil.copy2(cm_output_file, master_costmap)
        if cm_preview_file.exists():
            shutil.copy2(cm_preview_file, master_costmap_preview)
        if cm_csv_dir.exists():
            shutil.copytree(cm_csv_dir, master_csv_dir)

        metadata = run_dir / "metadata.txt"
        metadata.write_text(
            "\n".join(
                [
                    "ROAR World Dataset",
                    "=" * 50,
                    f"Run ID                  : {run_id}",
                    f"Base world              : {base_world}",
                    f"Base heightmap          : {base_heightmap}",
                    f"Requested density       : {args.density}",
                    f"Requested percentage    : {args.percentage}",
                    f"Mapped collidable ratio : {collidable_ratio}",
                    f"Rock spacing            : {args.spacing}",
                    f"Minimum roughness       : {args.min_roughness}",
                    f"Minimum terrain height  : {args.min_terrain_height}",
                    f"Dead-end generation     : {args.deadends}",
                    f"Heightmap resolution    : {args.heightmap_resolution}",
                    f"Gradient scale          : {args.gradient_scale}",
                    f"Stability scale         : {args.stability_scale}",
                    f"Obstacle data           : {master_obs}",
                    f"Generated world         : {master_world}",
                    f"Generated heightmap     : {master_heightmap}",
                    f"Generated costmap       : {master_costmap}",
                    "",
                ]
            ),
            encoding="utf-8",
        )

        # Generate launch file inside outputs/world{index}
        launch_file = run_dir / f"{run_id}.launch.py"
        launch_content = f"""import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_worlds = get_package_share_directory('worlds')
    world_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '{run_id}.world'
    )
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_worlds, 'launch', 'launch_map.launch.py')
            ),
            launch_arguments={{'world': world_path}}.items()
        )
    ])
"""
        launch_file.write_text(launch_content, encoding="utf-8")
        print(f"-> Generated local launch file: {launch_file}")

    except Exception:
        print()
        print("Pipeline failed.")
        print(f"Partial output was left at: {run_dir}")
        raise

    print()
    print("=" * 78)
    print("WORLD PIPELINE COMPLETED SUCCESSFULLY")
    print("=" * 78)
    print(f"Run ID          : {run_id}")
    print(f"Dataset folder  : {run_dir}")
    print(f"World           : {master_world}")
    print(f"Heightmap       : {master_heightmap}")
    print(f"Costmap         : {master_costmap}")
    print(f"Metadata        : {metadata}")
    print("=" * 78)

    if args.run:
        import shlex
        print()
        print("=" * 78)
        print(f"Launching Generated World Dataset: {run_id}")
        print("=" * 78)
        launch_file = run_dir / f"{run_id}.launch.py"
        workspace_root = Path(__file__).resolve().parents[2]
        setup_file = workspace_root / "install" / "setup.bash"
        if setup_file.exists():
            launch_command = (
                "source /opt/ros/humble/setup.bash && "
                f"source {shlex.quote(str(setup_file))} && "
                f"ros2 launch {shlex.quote(str(launch_file))}"
            )
            subprocess.run(["bash", "-lc", launch_command], check=True)
        else:
            print("Workspace setup file was not found, cannot launch.")


if __name__ == "__main__":
    main()
