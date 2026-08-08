#!/usr/bin/env python3

import argparse
import math
import random
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml


REQUIRED_WORLD_PLUGINS = [
    (
        "ignition-gazebo-physics-system",
        "ignition::gazebo::systems::Physics",
    ),
    (
        "ignition-gazebo-user-commands-system",
        "ignition::gazebo::systems::UserCommands",
    ),
    (
        "ignition-gazebo-scene-broadcaster-system",
        "ignition::gazebo::systems::SceneBroadcaster",
    ),
    (
        "ignition-gazebo-contact-system",
        "ignition::gazebo::systems::Contact",
    ),
    (
        "ignition-gazebo-imu-system",
        "ignition::gazebo::systems::Imu",
    ),
]


def ensure_plugins(world_elem):
    existing = {
        plugin.attrib.get("filename")
        for plugin in world_elem.findall("plugin")
    }

    for filename, name in REQUIRED_WORLD_PLUGINS:
        if filename in existing:
            continue

        ET.SubElement(
            world_elem,
            "plugin",
            attrib={
                "filename": filename,
                "name": name,
            },
        )


def load_obstacles(path):
    if path is None or not path.exists():
        return []

    data = np.load(path, allow_pickle=True)

    obstacles = []

    for item in data:
        if isinstance(item, dict):
            r = item
        elif hasattr(item, "item"):
            r = item.item()
        else:
            r = dict(item)

        obstacles.append(
            (
                float(r.get("x", 0.0)),
                float(r.get("y", 0.0)),
            )
        )

    return obstacles


def far_from_obstacles(x, y, obstacles, clearance):
    for ox, oy in obstacles:
        if math.hypot(x - ox, y - oy) < clearance:
            return False

    return True



def load_terrain_heightmap(path):
    """
    Load ROAR terrain heightmap NPZ.

    Expected fields:
        xs
        ys
        grid
        resolution
        origin_x
        origin_y
    """
    data = np.load(path, allow_pickle=False)

    required = {
        "xs",
        "ys",
        "grid",
        "resolution",
        "origin_x",
        "origin_y",
    }

    missing = required.difference(data.files)

    if missing:
        raise RuntimeError(
            "Terrain heightmap is missing fields: "
            + ", ".join(sorted(missing))
        )

    xs = np.asarray(
        data["xs"],
        dtype=np.float64,
    )

    ys = np.asarray(
        data["ys"],
        dtype=np.float64,
    )

    grid = np.asarray(
        data["grid"],
        dtype=np.float64,
    )

    resolution = float(data["resolution"])
    origin_x = float(data["origin_x"])
    origin_y = float(data["origin_y"])

    if grid.shape != (len(ys), len(xs)):
        raise RuntimeError(
            f"Unexpected heightmap shape {grid.shape}; "
            f"expected {(len(ys), len(xs))}."
        )

    return {
        "xs": xs,
        "ys": ys,
        "grid": grid,
        "resolution": resolution,
        "origin_x": origin_x,
        "origin_y": origin_y,
    }


def terrain_height_at(x, y, terrain):
    """
    Bilinear terrain-height lookup.

    Returns None when:
      - (x, y) is outside map bounds
      - surrounding terrain data is invalid / NaN
    """
    grid = terrain["grid"]
    resolution = terrain["resolution"]
    origin_x = terrain["origin_x"]
    origin_y = terrain["origin_y"]

    col_f = (x - origin_x) / resolution
    row_f = (y - origin_y) / resolution

    if (
        col_f < 0.0
        or row_f < 0.0
        or col_f > grid.shape[1] - 1
        or row_f > grid.shape[0] - 1
    ):
        return None

    c0 = int(math.floor(col_f))
    r0 = int(math.floor(row_f))

    c1 = min(c0 + 1, grid.shape[1] - 1)
    r1 = min(r0 + 1, grid.shape[0] - 1)

    q00 = grid[r0, c0]
    q10 = grid[r0, c1]
    q01 = grid[r1, c0]
    q11 = grid[r1, c1]

    values = np.asarray(
        [q00, q10, q01, q11],
        dtype=np.float64,
    )

    if np.any(np.isnan(values)):
        return None

    tx = col_f - c0
    ty = row_f - r0

    z0 = q00 * (1.0 - tx) + q10 * tx
    z1 = q01 * (1.0 - tx) + q11 * tx

    z = z0 * (1.0 - ty) + z1 * ty

    return float(z)


def generate_marker_positions(
    count,
    obstacles,
    terrain,
    seed,
    x_min,
    x_max,
    y_min,
    y_max,
    min_spacing,
    obstacle_clearance,
):
    rng = random.Random(seed)

    markers = []

    attempts = 0
    max_attempts = 10000

    while len(markers) < count and attempts < max_attempts:
        attempts += 1

        x = rng.uniform(x_min, x_max)
        y = rng.uniform(y_min, y_max)

        if not far_from_obstacles(
            x,
            y,
            obstacles,
            obstacle_clearance,
        ):
            continue

        valid = True

        for marker in markers:
            if math.hypot(
                x - marker["x"],
                y - marker["y"],
            ) < min_spacing:
                valid = False
                break

        if not valid:
            continue

        # Terrain height at this exact world XY location.
        z = terrain_height_at(
            x,
            y,
            terrain,
        )

        # Reject positions outside the valid terrain raster
        # or over undefined / NaN terrain cells.
        if z is None:
            continue

        yaw = rng.uniform(-math.pi, math.pi)

        markers.append(
            {
                "x": x,
                "y": y,
                "z": z,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": yaw,
            }
        )

    if len(markers) != count:
        raise RuntimeError(
            f"Could only place {len(markers)} "
            f"ArUco markers out of requested {count}."
        )

    return markers


def yaw_to_quaternion(yaw):
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw / 2.0),
        "w": math.cos(yaw / 2.0),
    }


def inject_markers(
    input_world,
    output_world,
    obstacle_data,
    terrain_heightmap,
    output_yaml,
    output_npy,
    models_source,
    models_output,
    count,
    marker_id_start,
    seed,
    x_min,
    x_max,
    y_min,
    y_max,
    marker_size,
    min_spacing,
    obstacle_clearance,
):
    tree = ET.parse(input_world)

    root = tree.getroot()
    world = root.find("world")

    if world is None:
        raise RuntimeError(
            "Invalid SDF: <world> element not found."
        )

    ensure_plugins(world)

    obstacles = load_obstacles(obstacle_data)

    terrain = load_terrain_heightmap(
        terrain_heightmap
    )

    marker_positions = generate_marker_positions(
        count=count,
        obstacles=obstacles,
        terrain=terrain,
        seed=seed,
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        min_spacing=min_spacing,
        obstacle_clearance=obstacle_clearance,
    )

    gt_markers = []

    npy_markers = []

    models_output.mkdir(
        parents=True,
        exist_ok=True,
    )

    for index, pose in enumerate(
        marker_positions,
        start=1,
    ):
        marker_id = marker_id_start + index - 1

        model_name = f"aruco_{index}"

        source_model = (
            models_source / model_name
        )

        if not source_model.exists():
            raise FileNotFoundError(
                f"Missing ArUco model:\n"
                f"{source_model}"
            )

        destination_model = (
            models_output / model_name
        )

        if destination_model.exists():
            shutil.rmtree(destination_model)

        shutil.copytree(
            source_model,
            destination_model,
        )

        model_elem = ET.SubElement(
            world,
            "model",
            attrib={
                "name": f"ArUco_{marker_id}",
            },
        )

        pose_elem = ET.SubElement(
            model_elem,
            "pose",
        )

        pose_elem.text = (
            f'{pose["x"]:.6f} '
            f'{pose["y"]:.6f} '
            f'{pose["z"]:.6f} '
            f'{pose["roll"]:.6f} '
            f'{pose["pitch"]:.6f} '
            f'{pose["yaw"]:.6f}'
        )

        include_elem = ET.SubElement(
            model_elem,
            "include",
        )

        uri_elem = ET.SubElement(
            include_elem,
            "uri",
        )

        uri_elem.text = (
            f"model://{model_name}"
        )

        quaternion = yaw_to_quaternion(
            pose["yaw"]
        )

        # In the current ArUco model.sdf, the textured
        # marker head is centred 0.496250 m above the model origin.
        #
        # Gazebo model pose:
        #       z = terrain surface
        #
        # Benchmark GT:
        #       z = actual ArUco marker centre
        marker_center_offset_z = 0.496250

        marker_center_z = (
            float(pose["z"])
            + marker_center_offset_z
        )

        marker_gt = {
            "id": marker_id,

            "model": model_name,

            "terrain_z": float(pose["z"]),
            "model_origin_z": float(pose["z"]),
            "marker_center_offset_z": marker_center_offset_z,

            "position": {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "z": marker_center_z,
            },

            "orientation": quaternion,

            "size_m": float(marker_size),

            "frame_id": "map",
        }

        gt_markers.append(marker_gt)

        npy_markers.append(
            {
                "id": marker_id,

                "model": model_name,

                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "z": marker_center_z,

                "terrain_z": float(pose["z"]),
                "model_origin_z": float(pose["z"]),
                "marker_center_offset_z": marker_center_offset_z,

                "roll": float(pose["roll"]),
                "pitch": float(pose["pitch"]),
                "yaw": float(pose["yaw"]),

                "size_m": float(marker_size),

                "frame_id": "map",
            }
        )

    ET.indent(
        tree,
        space="  ",
        level=0,
    )

    tree.write(
        output_world,
        encoding="utf-8",
        xml_declaration=True,
    )

    yaml_data = {
        "frame_id": "map",
        "marker_count": len(gt_markers),
        "markers": gt_markers,
    }

    output_yaml.write_text(
        yaml.safe_dump(
            yaml_data,
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    np.save(
        output_npy,
        np.array(
            npy_markers,
            dtype=object,
        ),
        allow_pickle=True,
    )

    print()
    print("=" * 70)
    print("ARUCO GENERATION COMPLETE")
    print("=" * 70)
    print(f"Markers      : {len(gt_markers)}")
    print(f"World        : {output_world}")
    print(f"GT YAML      : {output_yaml}")
    print(f"GT NPY       : {output_npy}")
    print(f"Models       : {models_output}")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--input-world",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--output-world",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--obstacle-data",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--terrain-heightmap",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--output-yaml",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--output-npy",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--models-source",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--models-output",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--count",
        type=int,
        default=15,
    )

    parser.add_argument(
        "--marker-id-start",
        type=int,
        default=1,
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=1,
    )

    parser.add_argument(
        "--x-min",
        type=float,
        default=-12.0,
    )

    parser.add_argument(
        "--x-max",
        type=float,
        default=12.0,
    )

    parser.add_argument(
        "--y-min",
        type=float,
        default=-2.0,
    )

    parser.add_argument(
        "--y-max",
        type=float,
        default=28.0,
    )

    parser.add_argument(
        "--marker-size",
        type=float,
        default=0.21,
    )

    parser.add_argument(
        "--min-spacing",
        type=float,
        default=3.0,
    )

    parser.add_argument(
        "--obstacle-clearance",
        type=float,
        default=1.5,
    )

    args = parser.parse_args()

    args.output_world.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    args.output_yaml.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    args.output_npy.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    inject_markers(
        input_world=args.input_world.resolve(),
        output_world=args.output_world.resolve(),
        obstacle_data=args.obstacle_data.resolve(),
        terrain_heightmap=args.terrain_heightmap.resolve(),
        output_yaml=args.output_yaml.resolve(),
        output_npy=args.output_npy.resolve(),
        models_source=args.models_source.resolve(),
        models_output=args.models_output.resolve(),
        count=args.count,
        marker_id_start=args.marker_id_start,
        seed=args.seed,
        x_min=args.x_min,
        x_max=args.x_max,
        y_min=args.y_min,
        y_max=args.y_max,
        marker_size=args.marker_size,
        min_spacing=args.min_spacing,
        obstacle_clearance=args.obstacle_clearance,
    )


if __name__ == "__main__":
    main()
