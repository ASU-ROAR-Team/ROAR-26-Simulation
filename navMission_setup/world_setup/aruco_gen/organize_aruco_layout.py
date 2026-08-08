#!/usr/bin/env python3

import argparse
import math
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


HEAD_CENTER_OFFSET = 0.49625   # shorter ArUco head center
ROWS = 3
COLS = 5


def load_object_array(path):
    arr = np.load(path, allow_pickle=True)

    if getattr(arr, "ndim", 1) == 0:
        obj = arr.item()
        if isinstance(obj, list):
            return obj
        if isinstance(obj, dict):
            return [obj]

    result = []

    for item in arr:
        if hasattr(item, "item"):
            try:
                item = item.item()
            except Exception:
                pass

        result.append(item)

    return result


def marker_id(item):
    if not isinstance(item, dict):
        return None

    for key in ("id", "marker_id", "aruco_id"):
        if key in item:
            return int(item[key])

    return None


def update_position(item, x, y, z):
    if "position" in item and isinstance(item["position"], dict):
        item["position"]["x"] = float(x)
        item["position"]["y"] = float(y)
        item["position"]["z"] = float(z)
    else:
        item["x"] = float(x)
        item["y"] = float(y)
        item["z"] = float(z)


def obstacle_list(path):
    if not path.exists():
        return []

    raw = load_object_array(path)
    result = []

    for obs in raw:
        if not isinstance(obs, dict):
            continue

        try:
            x = float(obs["x"])
            y = float(obs["y"])

            length = float(obs.get("length", 0.7))
            width = float(obs.get("width", 0.7))

            radius = 0.5 * math.hypot(length, width)

            result.append((x, y, radius))

        except Exception:
            continue

    return result


def erode_valid_mask(valid, radius_cells):
    """
    Remove terrain cells near:
      - map boundary
      - invalid / NaN terrain
      - void regions
    """

    if radius_cells <= 0:
        return valid.copy()

    h, w = valid.shape
    safe = valid.copy()

    for di in range(-radius_cells, radius_cells + 1):
        for dj in range(-radius_cells, radius_cells + 1):

            shifted = np.zeros_like(valid, dtype=bool)

            src_i0 = max(0, -di)
            src_i1 = min(h, h - di)
            src_j0 = max(0, -dj)
            src_j1 = min(w, w - dj)

            dst_i0 = max(0, di)
            dst_i1 = min(h, h + di)
            dst_j0 = max(0, dj)
            dst_j1 = min(w, w + dj)

            shifted[
                dst_i0:dst_i1,
                dst_j0:dst_j1
            ] = valid[
                src_i0:src_i1,
                src_j0:src_j1
            ]

            safe &= shifted

    return safe


def obstacle_ok(x, y, obstacles, clearance):
    for ox, oy, radius in obstacles:
        if math.hypot(x - ox, y - oy) < radius + clearance:
            return False

    return True


def marker_spacing_ok(x, y, selected, min_spacing):
    for old in selected:
        if math.hypot(x - old["x"], y - old["y"]) < min_spacing:
            return False

    return True



def compute_roughness(grid):
    """
    Same 3x3 local-Z standard deviation concept used by
    the rock generator.
    """
    rows, cols = grid.shape
    roughness = np.full_like(grid, np.nan, dtype=float)

    for r in range(rows):
        for c in range(cols):

            if not np.isfinite(grid[r, c]):
                continue

            r0 = max(0, r - 1)
            r1 = min(rows - 1, r + 1)

            c0 = max(0, c - 1)
            c1 = min(cols - 1, c + 1)

            patch = grid[
                r0:r1 + 1,
                c0:c1 + 1
            ]

            values = patch[np.isfinite(patch)]

            if len(values) >= 2:
                roughness[r, c] = float(np.std(values))
            else:
                roughness[r, c] = 0.0

    return roughness


def world_to_grid(x, y, xs, ys):
    """
    Return nearest heightmap cell.
    """
    col = int(np.argmin(np.abs(xs - x)))
    row = int(np.argmin(np.abs(ys - y)))

    return row, col


def terrain_point_valid(
    x,
    y,
    xs,
    ys,
    grid,
    roughness,
    min_height=0.15,
    min_roughness=0.02,
):
    # Heightmap extent
    if (
        x < float(xs[0])
        or x > float(xs[-1])
        or y < float(ys[0])
        or y > float(ys[-1])
    ):
        return False

    # --------------------------------------------------------
    # Same exclusion region used by the rock generator.
    #
    # Collision mesh exists here, but no real visual Marsyard
    # terrain exists.
    # --------------------------------------------------------
    if (
        7.0 <= x <= 14.0
        and
        -11.0 <= y <= 1.0
    ):
        return False

    row, col = world_to_grid(
        x,
        y,
        xs,
        ys,
    )

    z = float(grid[row, col])
    r = float(roughness[row, col])

    if not np.isfinite(z):
        return False

    if z < min_height:
        return False

    if not np.isfinite(r):
        return False

    if r < min_roughness:
        return False

    return True


def marker_footprint_valid(
    x,
    y,
    xs,
    ys,
    grid,
    roughness,
    radius=0.35,
):
    """
    Validate not only the marker centre but its surrounding
    footprint.

         o---o---o
         |       |
         o   X   o
         |       |
         o---o---o

    This prevents markers from sitting on terrain edges / voids.
    """

    offsets = [
        (0.0, 0.0),

        (-radius, 0.0),
        (+radius, 0.0),

        (0.0, -radius),
        (0.0, +radius),

        (-radius, -radius),
        (-radius, +radius),
        (+radius, -radius),
        (+radius, +radius),
    ]

    for dx, dy in offsets:

        if not terrain_point_valid(
            x + dx,
            y + dy,
            xs,
            ys,
            grid,
            roughness,
        ):
            return False

    return True


def choose_layout(
    xs,
    ys,
    grid,
    obstacles,
    edge_margin,
    obstacle_clearance,
    min_spacing,
):
    valid = np.isfinite(grid)

    # Same terrain-quality logic used by the rock generator.
    roughness = compute_roughness(grid)

    valid &= (
        (grid >= 0.15)
        & np.isfinite(roughness)
        & (roughness >= 0.02)
    )

    resolution = float(
        min(
            np.median(np.diff(xs)),
            np.median(np.diff(ys)),
        )
    )

    radius_cells = max(
        1,
        int(math.ceil(edge_margin / resolution)),
    )

    safe = erode_valid_mask(
        valid,
        radius_cells,
    )

    # ---------------------------------------------------------
    # BENCHMARK PLAYABLE AREA
    #
    # These are the same XY bounds used by the rock generator.
    # The Marsyard mesh / heightmap extends farther than the
    # actual benchmark area, so ArUco markers must stay here.
    # ---------------------------------------------------------
    PLAY_X_MIN = -19.0
    PLAY_X_MAX = 13.0
    PLAY_Y_MIN = -11.0
    PLAY_Y_MAX = 11.0

    # Keep markers slightly away from the playable-area border.
    PLAY_MARGIN = 0.75

    usable_x_min = PLAY_X_MIN + PLAY_MARGIN
    usable_x_max = PLAY_X_MAX - PLAY_MARGIN
    usable_y_min = PLAY_Y_MIN + PLAY_MARGIN
    usable_y_max = PLAY_Y_MAX - PLAY_MARGIN

    xx, yy = np.meshgrid(xs, ys)

    playable_mask = (
        (xx >= usable_x_min)
        & (xx <= usable_x_max)
        & (yy >= usable_y_min)
        & (yy <= usable_y_max)
    )

    safe &= playable_mask

    # ---------------------------------------------------------
    # Remove hidden / white platform region.
    # Same exclusion used by the rock generator.
    # ---------------------------------------------------------
    xx2, yy2 = np.meshgrid(xs, ys)

    hidden_platform = (
        (xx2 >= 7.0)
        & (xx2 <= 14.0)
        & (yy2 >= -11.0)
        & (yy2 <= 1.0)
    )

    safe &= ~hidden_platform

    safe_indices = np.argwhere(safe)

    if len(safe_indices) < 15:
        raise RuntimeError(
            "Not enough safe terrain cells after edge/void filtering."
        )

    safe_xs = xs[safe_indices[:, 1]]
    safe_ys = ys[safe_indices[:, 0]]

    xmin = float(safe_xs.min())
    xmax = float(safe_xs.max())
    ymin = float(safe_ys.min())
    ymax = float(safe_ys.max())

    print()
    print("Safe usable terrain:")
    print(f"  X: {xmin:.3f} -> {xmax:.3f}")
    print(f"  Y: {ymin:.3f} -> {ymax:.3f}")
    print(f"  Safe cells: {len(safe_indices)}")
    print()

    x_edges = np.linspace(
        xmin,
        xmax,
        COLS + 1,
    )

    y_edges = np.linspace(
        ymin,
        ymax,
        ROWS + 1,
    )

    selected = []

    marker_number = 1

    # top row -> bottom row visually
    for row in reversed(range(ROWS)):
        for col in range(COLS):

            rx0 = x_edges[col]
            rx1 = x_edges[col + 1]

            ry0 = y_edges[row]
            ry1 = y_edges[row + 1]

            center_x = (rx0 + rx1) / 2.0
            center_y = (ry0 + ry1) / 2.0

            candidates = []

            for i, j in safe_indices:

                x = float(xs[j])
                y = float(ys[i])

                if not (
                    rx0 <= x <= rx1
                    and ry0 <= y <= ry1
                ):
                    continue

                if not marker_footprint_valid(
                    x,
                    y,
                    xs,
                    ys,
                    grid,
                    roughness,
                    radius=0.35,
                ):
                    continue

                if not obstacle_ok(
                    x,
                    y,
                    obstacles,
                    obstacle_clearance,
                ):
                    continue

                if not marker_spacing_ok(
                    x,
                    y,
                    selected,
                    min_spacing,
                ):
                    continue

                distance_to_center = (
                    (x - center_x) ** 2
                    + (y - center_y) ** 2
                )

                candidates.append(
                    (
                        distance_to_center,
                        i,
                        j,
                    )
                )

            # If this region is obstructed, search nearest safe
            # point around the desired region center.
            if not candidates:

                for i, j in safe_indices:

                    x = float(xs[j])
                    y = float(ys[i])

                    if not obstacle_ok(
                        x,
                        y,
                        obstacles,
                        obstacle_clearance,
                    ):
                        continue

                    if not marker_spacing_ok(
                        x,
                        y,
                        selected,
                        min_spacing,
                    ):
                        continue

                    distance_to_center = (
                        (x - center_x) ** 2
                        + (y - center_y) ** 2
                    )

                    candidates.append(
                        (
                            distance_to_center,
                            i,
                            j,
                        )
                    )

            if not candidates:
                raise RuntimeError(
                    f"Cannot find safe position for ArUco {marker_number}"
                )

            candidates.sort(
                key=lambda v: v[0]
            )

            _, i, j = candidates[0]

            x = float(xs[j])
            y = float(ys[i])
            terrain_z = float(grid[i, j])

            selected.append(
                {
                    "id": marker_number,
                    "x": x,
                    "y": y,
                    "terrain_z": terrain_z,
                    "gt_z": terrain_z + HEAD_CENTER_OFFSET,
                    "row": ROWS - row,
                    "col": col + 1,
                }
            )

            marker_number += 1

    return selected, safe


def update_world(world_path, positions):
    tree = ET.parse(world_path)
    root = tree.getroot()

    by_id = {
        p["id"]: p
        for p in positions
    }

    updated = 0

    for model in root.findall(".//model"):

        name = model.attrib.get(
            "name",
            "",
        )

        if not name.startswith("ArUco_"):
            continue

        try:
            mid = int(
                name.split("_")[-1]
            )
        except Exception:
            continue

        if mid not in by_id:
            continue

        p = by_id[mid]

        pose = model.find("pose")

        if pose is None:
            pose = ET.Element("pose")
            model.insert(0, pose)

        values = (
            pose.text.split()
            if pose.text
            else ["0", "0", "0", "0", "0", "0"]
        )

        while len(values) < 6:
            values.append("0")

        values[0] = f'{p["x"]:.6f}'
        values[1] = f'{p["y"]:.6f}'

        # model base sits on terrain
        values[2] = f'{p["terrain_z"]:.6f}'

        pose.text = " ".join(values)

        updated += 1

    if updated != 15:
        raise RuntimeError(
            f"Expected 15 ArUco models, updated {updated}"
        )

    ET.indent(
        tree,
        space="  ",
    )

    tree.write(
        world_path,
        encoding="unicode",
    )


def update_yaml(path, positions):
    data = yaml.safe_load(
        path.read_text()
    )

    markers = data.get(
        "markers",
        [],
    )

    pos_by_id = {
        p["id"]: p
        for p in positions
    }

    for marker in markers:
        mid = marker_id(marker)

        if mid in pos_by_id:
            p = pos_by_id[mid]

            update_position(
                marker,
                p["x"],
                p["y"],
                p["gt_z"],
            )

    data["marker_count"] = 15

    path.write_text(
        yaml.safe_dump(
            data,
            sort_keys=False,
        )
    )


def update_npy(path, positions):
    items = load_object_array(path)

    pos_by_id = {
        p["id"]: p
        for p in positions
    }

    for item in items:

        if not isinstance(item, dict):
            continue

        mid = marker_id(item)

        if mid in pos_by_id:
            p = pos_by_id[mid]

            update_position(
                item,
                p["x"],
                p["y"],
                p["gt_z"],
            )

    np.save(
        path,
        np.array(
            items,
            dtype=object,
        ),
        allow_pickle=True,
    )


def backup(path):
    backup_path = Path(
        str(path)
        + ".before_organized_aruco"
    )

    if not backup_path.exists():
        shutil.copy2(
            path,
            backup_path,
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--world-dir",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--terrain-heightmap",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--edge-margin",
        type=float,
        default=1.0,
    )

    parser.add_argument(
        "--obstacle-clearance",
        type=float,
        default=0.8,
    )

    parser.add_argument(
        "--min-spacing",
        type=float,
        default=2.0,
    )

    args = parser.parse_args()

    world_dir = args.world_dir.resolve()
    name = world_dir.name

    world_path = world_dir / f"{name}.world"
    obstacle_path = world_dir / "obstacle_data.npy"
    yaml_path = world_dir / "aruco_data.yaml"
    npy_path = world_dir / "aruco_data.npy"

    for path in (
        world_path,
        yaml_path,
        npy_path,
    ):
        if not path.exists():
            raise FileNotFoundError(path)

        backup(path)

    hm = np.load(
        args.terrain_heightmap,
        allow_pickle=True,
    )

    xs = hm["xs"].astype(float)
    ys = hm["ys"].astype(float)
    grid = hm["grid"].astype(float)

    obstacles = obstacle_list(
        obstacle_path
    )

    positions, safe = choose_layout(
        xs,
        ys,
        grid,
        obstacles,
        args.edge_margin,
        args.obstacle_clearance,
        args.min_spacing,
    )

    update_world(
        world_path,
        positions,
    )

    update_yaml(
        yaml_path,
        positions,
    )

    update_npy(
        npy_path,
        positions,
    )

    print("=" * 72)
    print(f"ORGANIZED ARUCO LAYOUT — {name}")
    print("=" * 72)

    for p in positions:
        print(
            f'ArUco_{p["id"]:02d} '
            f'row={p["row"]} col={p["col"]} '
            f'x={p["x"]:7.3f} '
            f'y={p["y"]:7.3f} '
            f'z={p["gt_z"]:6.3f}'
        )

    print()
    print("15/15 organized ArUco markers ✅")
    print("All markers are on valid terrain ✅")
    print("Void / edge margin checked ✅")
    print("Obstacle clearance checked ✅")


if __name__ == "__main__":
    main()
