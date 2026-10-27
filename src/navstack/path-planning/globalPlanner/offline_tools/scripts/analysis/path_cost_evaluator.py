import pathlib
#!/usr/bin/env python3
"""
path_cost_evaluator.py

Computes the cost of a path (given in real-world coordinates) against a
costmap (given in pixel/cell coordinates, raw OccupancyGrid style values:
-1 = unknown, 0-100 = occupancy probability), using the SAME edge-cost
logic as the D* planner in dstar_planner.cpp.

--------------------------------------------------------------------------
Why this replicates the D* logic
--------------------------------------------------------------------------
In dstar_planner.cpp, occupancy values are first remapped to an internal
0-255 "cost byte" (mapToCostmap / the /map callback):

    raw == -1        -> NO_INFORMATION   (255)
    raw == 100       -> LETHAL_OBSTACLE  (254)
    otherwise        -> (raw / 100.0) * 253.0      (truncated to a byte)

The actual edge cost between two grid cells (edgeCost()) is:

    dist = pixel distance between the two cells (hypot(dx, dy))
    if to_cell cost == NO_INFORMATION:
        edge_cost = dist * unknown_traversal_cost
    else:
        normalized = clamp(cost / 252.0, 0, 1)
        corridor_term        = corridor_penalty_scale * normalized
        clearance_term       = clearance_cost_weight * normalized^2      (only if enabled)
        clearance_field_term = clearance_field_weight * exp(-clearance_m/0.30)  (only if enabled)
        edge_cost = dist * (1 + corridor_term + clearance_term + clearance_field_term)

Notice the costmap weight (corridor_term) is ALREADY folded into edge_cost
via the "* (1 + normalized)" multiplier -- D* does NOT ignore the costmap.
This script therefore does not add a second, separate costmap-weight term
(that would double count it); it simply reproduces edgeCost() exactly, so
whatever weighting D* applies is exactly what gets summed here. The two
extra terms (clearance_cost / clearance_field) are OFF by default, exactly
matching the planner's default parameters, and can be switched on with the
flags below if your D* run had them enabled.

--------------------------------------------------------------------------
Coordinate systems
--------------------------------------------------------------------------
The path CSV (x,y) is in REAL-WORLD (meters) coordinates. The costmap CSV
is in PIXEL/CELL coordinates. To compare them we need the same
resolution/origin the planner used (worldToMapSafe in the C++):

    mx = floor((wx - origin_x) / resolution)
    my = floor((wy - origin_y) / resolution)

These three values (resolution, origin_x, origin_y) are NOT stored in
either CSV, so they must be supplied. Defaults below match the planner's
own hardcoded defaults (resolution=0.05, origin=(0,0)) -- but if the map
that produced your costmap.csv had a different resolution/origin, you
MUST pass --resolution/--origin-x/--origin-y to get a meaningful answer.
"""

import argparse
import csv
import math
import sys
import heapq


NO_INFORMATION = 255
LETHAL_OBSTACLE = 254


# --------------------------------------------------------------------------
# CSV loading
# --------------------------------------------------------------------------

def load_costmap(path):
    """Loads costmap.csv into a 2D list [y][x] of RAW occupancy values
    (-1..100), reading width/height from the file itself (never assumed
    to be square)."""
    lines = []
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                lines.append(line)
    reader = csv.reader(lines)
    header = next(reader)
    width = len(header) - 1  # first column is the row-index label
    if width <= 0:
        raise ValueError("costmap.csv header does not look right "
                          "(expected an index column followed by pixel columns).")

    rows = []
    for row_num, row in enumerate(reader):
        if not row:
            continue
        values = row[1:]  # drop the leading row-index column
        if len(values) != width:
            raise ValueError(
                f"costmap.csv row {row_num} has {len(values)} data columns, "
                f"expected {width} (from header). File may be malformed."
            )
        rows.append([int(v) for v in values])

    height = len(rows)
    if height == 0:
        raise ValueError("costmap.csv contains no data rows.")

    return rows, width, height


def load_path(path):
    """Loads combined_offline_path.csv into a list of (x, y) world-frame
    floats, in file order."""
    pts = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None or "x" not in reader.fieldnames or "y" not in reader.fieldnames:
            raise ValueError("combined_offline_path.csv must have 'x' and 'y' columns.")
        for row in reader:
            pts.append((float(row["x"]), float(row["y"])))
    if len(pts) < 2:
        raise ValueError("Path file needs at least 2 points to compute a cost.")
    return pts


# --------------------------------------------------------------------------
# Costmap raw-value -> internal cost byte (mirrors the /map callback)
# --------------------------------------------------------------------------

def raw_to_cost_byte(raw):
    if raw == -1:
        return NO_INFORMATION
    if raw == 100:
        return LETHAL_OBSTACLE
    # static_cast<unsigned char>((val / 100.0) * 253.0) -> truncation, not rounding
    return int((raw / 100.0) * 253.0)


def build_cost_grid(raw_rows, width, height):
    return [[raw_to_cost_byte(raw_rows[y][x]) for x in range(width)] for y in range(height)]


# --------------------------------------------------------------------------
# Optional clearance field (multi-source Dijkstra to nearest lethal cell),
# mirroring computeClearanceField() in the C++. Only computed if requested.
# --------------------------------------------------------------------------

def compute_clearance_field(cost_grid, width, height, resolution):
    INF = float("inf")
    clearance = [[INF] * width for _ in range(height)]
    pq = []
    for y in range(height):
        for x in range(width):
            if cost_grid[y][x] >= LETHAL_OBSTACLE:
                clearance[y][x] = 0.0
                heapq.heappush(pq, (0.0, x, y))

    if not pq:
        return [[1e6] * width for _ in range(height)]

    while pq:
        d, x, y = heapq.heappop(pq)
        if d > clearance[y][x] + 1e-6:
            continue
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                step = math.hypot(dx, dy) * resolution
                nd = d + step
                if nd + 1e-6 < clearance[ny][nx]:
                    clearance[ny][nx] = nd
                    heapq.heappush(pq, (nd, nx, ny))
    return clearance


# --------------------------------------------------------------------------
# Edge cost (exact port of DStarPlanner::edgeCost)
# --------------------------------------------------------------------------

def edge_cost(fx, fy, tx, ty, cost_grid, clearance_field, params):
    dist = math.hypot(fx - tx, fy - ty)
    to_cost = cost_grid[ty][tx]

    if to_cost == NO_INFORMATION:
        return dist * params.unknown_traversal_cost, dist, 0.0

    normalized = min(1.0, max(0.0, to_cost / 252.0))
    corridor_term = params.corridor_penalty_scale * normalized

    clearance_term = 0.0
    if params.enable_clearance_cost:
        clearance_term = params.clearance_cost_weight * normalized * normalized

    clearance_field_term = 0.0
    if params.enable_clearance_field and params.clearance_field_weight > 0.0 and clearance_field is not None:
        c_m = max(0.0, clearance_field[ty][tx])
        k_decay_m = 0.30
        clearance_field_term = params.clearance_field_weight * math.exp(-c_m / k_decay_m)

    weight_multiplier = corridor_term + clearance_term + clearance_field_term
    total = dist * (1.0 + weight_multiplier)
    return total, dist, dist * weight_multiplier


def is_cell_valid(x, y, cost_grid, width, height, params):
    if x < 0 or y < 0 or x >= width or y >= height:
        return False, "out-of-bounds"
    cost = cost_grid[y][x]
    if cost == NO_INFORMATION:
        return params.allow_unknown, "unknown" if not params.allow_unknown else None
    if cost >= LETHAL_OBSTACLE:
        return False, "lethal-obstacle"
    if cost >= params.lethal_cost_threshold:
        return False, "above-lethal-threshold"
    return True, None


# --------------------------------------------------------------------------
# World -> map conversion (mirrors worldToMapSafe / mapToWorldPose)
# --------------------------------------------------------------------------

def world_to_map(wx, wy, origin_x, origin_y, resolution, width, height):
    if wx < origin_x or wy < origin_y:
        return None
    mx = int((wx - origin_x) / resolution)
    my = int((wy - origin_y) / resolution)
    if mx >= width or my >= height:
        return None
    return mx, my


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Compute the cost of an offline path using the D* planner's edge-cost logic."
    )
    ap.add_argument("--costmap", default=str(pathlib.Path(__file__).resolve().parent.parent.parent / "data" / "costmap.csv"), help="Path to costmap CSV (pixel coordinates).")
    ap.add_argument("--path", default=str(pathlib.Path(__file__).resolve().parent.parent.parent / "data" / "combined_offline_path.csv"), help="Path to path CSV (world/real coordinates).")

    ap.add_argument("--resolution", type=float, default=None,
                     help="Meters per pixel, same as the planner's resolution_ (default: inferred from costmap CSV comments or 0.1).")
    ap.add_argument("--origin-x", type=float, default=None, help="Costmap origin X in world frame (meters) (default: inferred from comments or 0.0).")
    ap.add_argument("--origin-y", type=float, default=None, help="Costmap origin Y in world frame (meters) (default: inferred from comments or 0.0).")

    ap.add_argument("--lethal-cost-threshold", type=int, default=100,
                     help="Cells with internal cost >= this are treated as blocked (default: 100, matching the costmap generator).")
    ap.add_argument("--unknown-traversal-cost", type=float, default=1.0,
                     help="Multiplier applied to dist when the target cell is unknown (default: 1.0).")
    ap.add_argument("--disallow-unknown", action="store_true",
                     help="Treat unknown (-1) cells as invalid/blocked (planner default: unknown IS allowed).")
    ap.add_argument("--corridor-penalty-scale", type=float, default=1.0,
                     help="Weight applied to the normalized costmap value in edge cost (default: 1.0).")
    ap.add_argument("--enable-clearance-cost", action="store_true",
                     help="Enable the normalized^2 clearance cost term (planner default: off).")
    ap.add_argument("--clearance-cost-weight", type=float, default=1.0)
    ap.add_argument("--enable-clearance-field", action="store_true",
                     help="Enable the distance-to-obstacle clearance field term (planner default: off). "
                          "Computing this requires an extra Dijkstra pass over the whole costmap.")
    ap.add_argument("--clearance-field-weight", type=float, default=1.0)

    args = ap.parse_args()
    args.allow_unknown = not args.disallow_unknown

    # ---- Load data -------------------------------------------------------
    raw_rows, width, height = load_costmap(args.costmap)
    cost_grid = build_cost_grid(raw_rows, width, height)
    world_pts = load_path(args.path)

    # Parse resolution and origin from comments in costmap file
    parsed_res = None
    parsed_origin_x = None
    parsed_origin_y = None
    with open(args.costmap, "r") as f:
        for line in f:
            if line.startswith("#"):
                if "resolution=" in line:
                    parsed_res = float(line.split("resolution=")[1].split()[0])
                if "origin_x=" in line:
                    parts = line.split()
                    for p in parts:
                        if p.startswith("origin_x="):
                            parsed_origin_x = float(p.split("=")[1])
                        elif p.startswith("origin_y="):
                            parsed_origin_y = float(p.split("=")[1])
            else:
                break

    # Apply command-line overrides or fallback to parsed values or defaults
    if args.resolution is None:
        args.resolution = parsed_res if parsed_res is not None else 0.1
    if args.origin_x is None:
        args.origin_x = parsed_origin_x if parsed_origin_x is not None else 0.0
    if args.origin_y is None:
        args.origin_y = parsed_origin_y if parsed_origin_y is not None else 0.0

    print(f"Loaded costmap: width={width}, height={height} (from {args.costmap})")
    print(f"Loaded path: {len(world_pts)} points (from {args.path})")
    print(f"Using resolution={args.resolution} m/px, origin=({args.origin_x}, {args.origin_y})")

    clearance_field = None
    if args.enable_clearance_field:
        print("Computing clearance field (this may take a moment)...")
        clearance_field = compute_clearance_field(cost_grid, width, height, args.resolution)

    # ---- Convert path to grid cells --------------------------------------
    cells = []
    out_of_bounds_pts = []
    for i, (wx, wy) in enumerate(world_pts):
        m = world_to_map(wx, wy, args.origin_x, args.origin_y, args.resolution, width, height)
        if m is None:
            out_of_bounds_pts.append(i)
            continue
        cells.append(m)

    if out_of_bounds_pts:
        print(f"\nWARNING: {len(out_of_bounds_pts)} path point(s) fell outside the costmap "
              f"bounds given resolution/origin (e.g. point indices {out_of_bounds_pts[:5]}...). "
              f"They were skipped. Double-check --resolution/--origin-x/--origin-y.")

    if len(cells) < 2:
        print("\nERROR: fewer than 2 path points map inside the costmap bounds -- cannot "
              "compute a cost. Check your resolution/origin values.")
        sys.exit(1)

    # ---- Walk the path, summing edge cost exactly like edgeCost() -------
    total_cost = 0.0
    total_dist_px = 0.0
    total_weight_contrib = 0.0
    invalid_cells_hit = []

    for (fx, fy), (tx, ty) in zip(cells[:-1], cells[1:]):
        valid, reason = is_cell_valid(tx, ty, cost_grid, width, height, args)
        if not valid:
            invalid_cells_hit.append(((tx, ty), reason))

        cost, dist, weight_contrib = edge_cost(fx, fy, tx, ty, cost_grid, clearance_field, args)
        total_cost += cost
        total_dist_px += dist
        total_weight_contrib += weight_contrib

    total_dist_m = total_dist_px * args.resolution

    # ---- Report ------------------------------------------------------------
    print("\n" + "=" * 60)
    print("PATH COST (D* edge-cost logic)")
    print("=" * 60)
    print(f"Total D* path cost         : {total_cost:.4f}  (cost units, unitless)")
    print(f"  - base grid-distance sum : {total_dist_px:.4f}  (pixels)")
    print(f"  - costmap-weight portion : {total_weight_contrib:.4f}  (already included above, "
          f"not double-counted)")
    print(f"Total path length          : {total_dist_m:.4f} m  ({total_dist_px:.4f} px x "
          f"{args.resolution} m/px)")
    print(f"Path points used           : {len(cells)} / {len(world_pts)}")

    if invalid_cells_hit:
        print(f"\nWARNING: path passes through {len(invalid_cells_hit)} invalid cell(s) "
              f"(lethal obstacle / above threshold / disallowed-unknown). This path would "
              f"NOT have been produced by D* itself under these settings. First few:")
        for (x, y), reason in invalid_cells_hit[:10]:
            print(f"  cell ({x}, {y}): {reason}")
    else:
        print("\nAll traversed cells are valid under the current planner settings.")


if __name__ == "__main__":
    main()
