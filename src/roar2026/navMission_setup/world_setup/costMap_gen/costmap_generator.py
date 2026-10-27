#!/usr/bin/env python3
"""Convert a metric height-map NPZ into directional and total terrain cost maps.

This fused module combines heightmap gradient / Laplacian terrain scoring with
lethal obstacle detection (cliff height diffs, steep slopes) and distance-based
inflation layers (inscribed radius & exponential cost decay).
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np

try:
    from PIL import Image
except ImportError:  # pragma: no cover
    Image = None

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

try:
    from scipy import ndimage
except ImportError:  # pragma: no cover
    ndimage = None


@dataclass(frozen=True)
class GradientData:
    x: np.ndarray
    y: np.ndarray
    magnitude: np.ndarray
    laplacian: np.ndarray
    height_diff_3x3: np.ndarray


@dataclass(frozen=True)
class CostData:
    total: np.ndarray
    x: np.ndarray
    y: np.ndarray


def _fill_unknown_nearest(grid: np.ndarray) -> np.ndarray:
    """Fill NaNs for derivative calculation while preserving the original mask."""
    filled = np.asarray(grid, dtype=np.float64).copy()
    missing = np.isnan(filled)
    if not np.any(missing):
        return filled
    if np.all(missing):
        raise ValueError("height map contains no valid cells")

    while np.any(np.isnan(filled)):
        valid = ~np.isnan(filled)
        sums = np.zeros_like(filled)
        counts = np.zeros_like(filled, dtype=np.int16)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                shifted = np.roll(np.roll(filled, dy, axis=0), dx, axis=1)
                shifted_valid = np.roll(np.roll(valid, dy, axis=0), dx, axis=1)
                if dy > 0:
                    shifted_valid[:dy, :] = False
                elif dy < 0:
                    shifted_valid[dy:, :] = False
                if dx > 0:
                    shifted_valid[:, :dx] = False
                elif dx < 0:
                    shifted_valid[:, dx:] = False
                sums[shifted_valid] += shifted[shifted_valid]
                counts[shifted_valid] += 1
        update = np.isnan(filled) & (counts > 0)
        if not np.any(update):
            break
        filled[update] = sums[update] / counts[update]
    return filled


def calculate_gradients(height_grid: np.ndarray, resolution: float) -> GradientData:
    if resolution <= 0:
        raise ValueError("resolution must be positive")
    filled = _fill_unknown_nearest(height_grid)

    if cv2 is not None:
        h_blur = cv2.GaussianBlur(filled.astype(np.float32), (3, 3), 0)
        sobel_x = cv2.Sobel(h_blur, cv2.CV_32F, 1, 0, ksize=3) / (8.0 * resolution)
        sobel_y = cv2.Sobel(h_blur, cv2.CV_32F, 0, 1, ksize=3) / (8.0 * resolution)
        grad_x = sobel_x.astype(np.float64)
        grad_y = sobel_y.astype(np.float64)
        magnitude = np.hypot(grad_x, grad_y)
        laplacian = cv2.Laplacian(h_blur, cv2.CV_32F).astype(np.float64) / (resolution ** 2)

        kernel_3x3 = np.ones((3, 3), np.uint8)
        max_h = cv2.dilate(h_blur, kernel_3x3).astype(np.float64)
        min_h = cv2.erode(h_blur, kernel_3x3).astype(np.float64)
        height_diff_3x3 = max_h - min_h
    else:
        grad_y, grad_x = np.gradient(filled, resolution, resolution)
        magnitude = np.hypot(grad_x, grad_y)
        d2x = np.gradient(grad_x, resolution, axis=1)
        d2y = np.gradient(grad_y, resolution, axis=0)
        laplacian = d2x + d2y
        if ndimage is not None:
            max_h = ndimage.maximum_filter(filled, size=3)
            min_h = ndimage.minimum_filter(filled, size=3)
            height_diff_3x3 = max_h - min_h
        else:
            height_diff_3x3 = magnitude * resolution

    return GradientData(grad_x, grad_y, magnitude, laplacian, height_diff_3x3)


def calculate_distance_transform(lethal_binary: np.ndarray, resolution: float) -> np.ndarray:
    """Calculate Euclidean distance map in meters from lethal obstacle cells."""
    if cv2 is not None:
        # cv2.distanceTransform needs 0 for obstacle cells, 1 for free cells
        free_binary = (1 - lethal_binary).astype(np.uint8)
        dist = cv2.distanceTransform(free_binary, cv2.DIST_L2, 5) * resolution
        return dist.astype(np.float64)
    elif ndimage is not None:
        free_binary = (1 - lethal_binary).astype(bool)
        dist = ndimage.distance_transform_edt(free_binary) * resolution
        return dist.astype(np.float64)
    else:
        # Fallback simple distance transform
        return np.where(lethal_binary > 0, 0.0, 999.0)


def calculate_costs(
    gradients: GradientData,
    valid_mask: np.ndarray,
    resolution: float,
    *,
    gradient_scale: float = 150.0,
    stability_scale: float = 90.0,
    gradient_reference: float = 1.0,
    stability_reference: float = 1.0,
    max_safe_slope: float = 0.5,
    max_height_diff: float = 0.30,
    inscribed_radius: float = 0.25,
    inflation_radius: float = 0.80,
    inflation_scaling: float = 3.0,
) -> CostData:
    if gradient_reference <= 0 or stability_reference <= 0:
        raise ValueError("gradient and stability reference values must be positive")

    stability = np.abs(gradients.laplacian)
    gradient_cost = (gradients.magnitude / gradient_reference) * gradient_scale
    stability_cost = (stability / stability_reference) * stability_scale

    base_terrain_cost = np.clip(gradient_cost + stability_cost, 0.0, 99.0)

    # 1. Detect lethal cells (steep slope OR cliff height difference)
    steep_lethal = gradients.magnitude > max_safe_slope
    diff_lethal = gradients.height_diff_3x3 > max_height_diff
    lethal_mask = (steep_lethal | diff_lethal) & valid_mask

    # 2. Inflation layer calculation
    lethal_binary = lethal_mask.astype(np.uint8)
    dist_map = calculate_distance_transform(lethal_binary, resolution)

    inscribed_mask = (dist_map < inscribed_radius) & valid_mask
    all_lethal = lethal_mask | inscribed_mask

    inflation_cost = np.zeros_like(dist_map, dtype=np.float64)
    inflation_band = (dist_map >= inscribed_radius) & (dist_map < inflation_radius) & valid_mask
    if inflation_radius > inscribed_radius:
        norm_dist = (dist_map[inflation_band] - inscribed_radius) / (inflation_radius - inscribed_radius)
        inflation_cost[inflation_band] = 99.0 * np.exp(-inflation_scaling * norm_dist)

    total = np.maximum(base_terrain_cost, inflation_cost)
    total[all_lethal] = 100.0

    cost_x = np.clip((np.abs(gradients.x) / gradient_reference) * gradient_scale + stability_cost, 0.0, 99.0)
    cost_x[all_lethal] = 100.0

    cost_y = np.clip((np.abs(gradients.y) / gradient_reference) * gradient_scale + stability_cost, 0.0, 99.0)
    cost_y[all_lethal] = 100.0

    for grid in (total, cost_x, cost_y):
        grid[~valid_mask] = -1.0

    return CostData(
        np.rint(total).astype(np.int16),
        np.rint(cost_x).astype(np.int16),
        np.rint(cost_y).astype(np.int16),
    )


def _write_csv(path: str, grid: np.ndarray) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([""] + [str(index) for index in range(grid.shape[1])])
        for row_index, row in enumerate(grid):
            writer.writerow([str(row_index)] + [str(int(value)) for value in row])


def _write_preview(path: str, costs: CostData) -> None:
    if Image is None:
        return
    panels = []
    for grid in (costs.total, costs.x, costs.y):
        image = np.full(grid.shape, 128, dtype=np.uint8)  # unknown = grey
        valid = grid >= 0
        image[valid] = np.clip(np.rint(grid[valid] * 2.55), 0, 255).astype(np.uint8)
        panels.append(np.flipud(image))
    combined = np.concatenate(panels, axis=1)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    Image.fromarray(combined, mode="L").save(path)


def generate_costmap_from_heightmap(
    heightmap_path: str,
    output_path: str,
    *,
    preview_path: Optional[str] = None,
    csv_directory: Optional[str] = None,
    gradient_scale: float = 150.0,
    stability_scale: float = 90.0,
    gradient_reference: float = 1.0,
    stability_reference: float = 1.0,
    max_safe_slope: float = 0.5,
    max_height_diff: float = 0.30,
    inscribed_radius: float = 0.25,
    inflation_radius: float = 0.80,
    inflation_scaling: float = 3.0,
) -> str:
    heightmap_path = os.path.abspath(os.path.expanduser(heightmap_path))
    output_path = os.path.abspath(os.path.expanduser(output_path))
    if not os.path.isfile(heightmap_path):
        raise FileNotFoundError(f"Height map not found: {heightmap_path}")

    source = np.load(heightmap_path, allow_pickle=False)
    required = {"grid", "resolution", "origin_x", "origin_y"}
    missing = required.difference(source.files)
    if missing:
        raise ValueError(f"Height map is missing required arrays: {sorted(missing)}")

    height_grid = np.asarray(source["grid"], dtype=np.float64)
    resolution = float(np.asarray(source["resolution"]).item())
    valid_mask = ~np.isnan(height_grid)
    gradients = calculate_gradients(height_grid, resolution)
    costs = calculate_costs(
        gradients,
        valid_mask,
        resolution,
        gradient_scale=gradient_scale,
        stability_scale=stability_scale,
        gradient_reference=gradient_reference,
        stability_reference=stability_reference,
        max_safe_slope=max_safe_slope,
        max_height_diff=max_height_diff,
        inscribed_radius=inscribed_radius,
        inflation_radius=inflation_radius,
        inflation_scaling=inflation_scaling,
    )

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    np.savez_compressed(
        output_path,
        total=costs.total,
        cost_x=costs.x,
        cost_y=costs.y,
        gradient_x=gradients.x.astype(np.float32),
        gradient_y=gradients.y.astype(np.float32),
        gradient_magnitude=gradients.magnitude.astype(np.float32),
        laplacian=gradients.laplacian.astype(np.float32),
        resolution=np.float64(resolution),
        origin_x=np.float64(np.asarray(source["origin_x"]).item()),
        origin_y=np.float64(np.asarray(source["origin_y"]).item()),
        heightmap_path=np.asarray(heightmap_path),
        gradient_scale=np.float64(gradient_scale),
        stability_scale=np.float64(stability_scale),
        gradient_reference=np.float64(gradient_reference),
        stability_reference=np.float64(stability_reference),
    )

    if csv_directory:
        csv_directory = os.path.abspath(os.path.expanduser(csv_directory))
        _write_csv(os.path.join(csv_directory, "cost_x.csv"), costs.x)
        _write_csv(os.path.join(csv_directory, "cost_y.csv"), costs.y)
        _write_csv(os.path.join(csv_directory, "total_cost.csv"), costs.total)
    if preview_path:
        preview_path = os.path.abspath(os.path.expanduser(preview_path))
        _write_preview(preview_path, costs)

    valid_costs = costs.total[costs.total >= 0]
    lethal_count = int((costs.total == 100).sum())
    print("=" * 60)
    print("Generated terrain cost map from height map")
    print(f"Heightmap:   {heightmap_path}")
    print(f"Output:      {output_path}")
    if preview_path:
        print(f"Preview:     {preview_path}")
    if csv_directory:
        print(f"CSV folder:  {csv_directory}")
    print(f"Resolution:  {resolution:.3f} m/cell")
    print(f"Grid:        {height_grid.shape[1]} x {height_grid.shape[0]}")
    if valid_costs.size:
        print(f"Cost range:  {int(valid_costs.min())} .. {int(valid_costs.max())}")
    print(f"Lethal obstacles (100): {lethal_count}")
    print(f"Unknown (-1):           {int((costs.total < 0).sum())}")
    print("=" * 60)
    return output_path


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert a metric height-map NPZ into terrain cost maps with lethal cliff & inflation layers."
    )
    parser.add_argument("heightmap", help="input height-map .npz")
    parser.add_argument("-o", "--output", required=True, help="output cost-map .npz")
    parser.add_argument("--preview", help="optional preview PNG (total, X, Y panels)")
    parser.add_argument("--csv-dir", help="optional folder for cost_x.csv, cost_y.csv and total_cost.csv")
    parser.add_argument("--gradient-scale", type=float, default=150.0)
    parser.add_argument("--stability-scale", type=float, default=90.0)
    parser.add_argument(
        "--gradient-reference",
        type=float,
        default=1.0,
        help="gradient magnitude that receives gradient_scale cost (default: 1.0 m/m)",
    )
    parser.add_argument(
        "--stability-reference",
        type=float,
        default=1.0,
        help="absolute Laplacian that receives stability_scale cost (default: 1.0 m/m²)",
    )
    parser.add_argument("--max-safe-slope", type=float, default=0.5, help="maximum safe slope gradient magnitude before lethal (default: 0.5)")
    parser.add_argument("--max-height-diff", type=float, default=0.30, help="maximum safe 3x3 height difference in meters before lethal cliff (default: 0.30m)")
    parser.add_argument("--inscribed-radius", type=float, default=0.25, help="inscribed robot radius in meters for lethal inflation (default: 0.25m)")
    parser.add_argument("--inflation-radius", type=float, default=0.80, help="inflation decay radius in meters (default: 0.80m)")
    parser.add_argument("--inflation-scaling", type=float, default=3.0, help="exponential inflation decay factor (default: 3.0)")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    if not args.output.lower().endswith(".npz"):
        parser.error("the output file must use the .npz extension")
    generate_costmap_from_heightmap(
        args.heightmap,
        args.output,
        preview_path=args.preview,
        csv_directory=args.csv_dir,
        gradient_scale=args.gradient_scale,
        stability_scale=args.stability_scale,
        gradient_reference=args.gradient_reference,
        stability_reference=args.stability_reference,
        max_safe_slope=args.max_safe_slope,
        max_height_diff=args.max_height_diff,
        inscribed_radius=args.inscribed_radius,
        inflation_radius=args.inflation_radius,
        inflation_scaling=args.inflation_scaling,
    )


if __name__ == "__main__":
    main()
