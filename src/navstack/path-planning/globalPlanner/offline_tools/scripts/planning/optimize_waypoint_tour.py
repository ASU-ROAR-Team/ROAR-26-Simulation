#!/usr/bin/env python3
"""Optimal Multi-Goal Waypoint Tour Planner (TSP Optimizer).

Evaluates all 24 permutations of the 4 intermediate mission waypoints:
  Start (0,0) -> W_pi(1) -> W_pi(2) -> W_pi(3) -> W_pi(4) -> Return to Start (0,0)

Uses 8-connected Dijkstra search on the terrain costmap to compute exact
shortest path distances (m) and terrain traversal costs for all 20 directed pairs.
Identifies and saves the optimal waypoint sequence that minimizes total distance & cost.
"""

import os
import csv
import math
import itertools
import heapq
import numpy as np

def load_costmap(path):
    meta = {}
    lines = []
    with open(path) as f:
        for line in f:
            if line.startswith('#'):
                if 'resolution=' in line:
                    meta['resolution'] = float(line.split('resolution=')[1].split()[0])
                if 'origin_x=' in line:
                    for tok in line.split():
                        if tok.startswith('origin_x='):
                            meta['origin_x'] = float(tok.split('=')[1])
                        elif tok.startswith('origin_y='):
                            meta['origin_y'] = float(tok.split('=')[1])
            else:
                lines.append(line)

    reader = csv.reader(lines)
    header = next(reader)
    width = len([h for h in header[1:] if h.strip() != ''])
    rows = []
    for r in reader:
        if r:
            vals = [int(v.strip()) if v.strip() else 0 for v in r[1:width+1]]
            rows.append(vals)

    grid = np.array(rows, dtype=np.int16)
    res = meta.get('resolution', 0.1)
    ox = meta.get('origin_x', -4.552489)
    oy = meta.get('origin_y', -10.339172)
    return grid, width, len(rows), res, ox, oy

def load_waypoints(path):
    wps = []
    with open(path) as f:
        reader = csv.reader(f)
        for idx, row in enumerate(reader):
            row = [c.strip() for c in row]
            if len(row) >= 2:
                lbl = row[2].strip() if len(row) >= 3 and row[2].strip() else f"WP{idx}"
                wps.append((float(row[0]), float(row[1]), lbl))
    return wps

def dijkstra_pair(grid, W, H, res, ox, oy, start_wpt, end_wpt):
    """Computes exact 8-connected shortest path distance and traversal cost between two waypoints."""
    sx = int((start_wpt[0] - ox) / res)
    sy = int((start_wpt[1] - oy) / res)
    gx = int((end_wpt[0] - ox) / res)
    gy = int((end_wpt[1] - oy) / res)

    # Snap out-of-bounds or lethal start/goal to nearest valid cell
    def get_valid_cell(x, y):
        x = max(0, min(W - 1, x))
        y = max(0, min(H - 1, y))
        if grid[y, x] < 100 and grid[y, x] != -1:
            return x, y
        best_d = float('inf')
        best_c = (x, y)
        for r in range(1, 10):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < W and 0 <= ny < H:
                        if grid[ny, nx] < 100 and grid[ny, nx] != -1:
                            d = math.hypot(dx, dy)
                            if d < best_d:
                                best_d = d
                                best_c = (nx, ny)
        return best_c

    sx, sy = get_valid_cell(sx, sy)
    gx, gy = get_valid_cell(gx, gy)

    # 8-neighbor directions (dx, dy, step_length_m)
    neighbors = [
        (-1, 0, res), (1, 0, res), (0, -1, res), (0, 1, res),
        (-1, -1, res * math.sqrt(2)), (-1, 1, res * math.sqrt(2)),
        (1, -1, res * math.sqrt(2)), (1, 1, res * math.sqrt(2))
    ]

    dist_map = {}
    cost_map = {}
    pq = [(0.0, 0.0, sx, sy)] # (total_cost, distance_m, x, y)
    dist_map[(sx, sy)] = 0.0
    cost_map[(sx, sy)] = 0.0

    found_dist = float('inf')
    found_cost = float('inf')

    while pq:
        c_cost, c_dist, cx, cy = heapq.heappop(pq)

        if (cx, cy) == (gx, gy):
            found_dist = c_dist
            found_cost = c_cost
            break

        if c_cost > cost_map.get((cx, cy), float('inf')):
            continue

        for dx, dy, step_m in neighbors:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < W and 0 <= ny < H:
                val = grid[ny, nx]
                if val >= 100 or val == -1:
                    continue  # Lethal obstacle / unknown

                # Edge cost = distance + terrain penalty
                terrain_penalty = (val / 99.0) * 2.0 * step_m
                edge_cost = step_m + terrain_penalty

                new_cost = c_cost + edge_cost
                new_dist = c_dist + step_m

                if new_cost < cost_map.get((nx, ny), float('inf')):
                    cost_map[(nx, ny)] = new_cost
                    dist_map[(nx, ny)] = new_dist
                    heapq.heappush(pq, (new_cost, new_dist, nx, ny))

    return found_dist, found_cost

def main():
    _here = os.path.dirname(os.path.abspath(__file__))
    cwd = os.getcwd()

    def resolve_file(filename, subfolders=['data', 'reference', 'offline_tools', '']):
        candidates = []
        for sub in subfolders:
            candidates.extend([
                os.path.join(cwd, sub, filename),
                os.path.join(cwd, filename),
                os.path.join(_here, '..', '..', sub, filename),
                os.path.join(_here, '..', sub, filename),
                os.path.join(_here, filename),
            ])
        for path in candidates:
            if os.path.exists(path):
                return os.path.abspath(path)
        return os.path.join(cwd, filename)

    costmap_file = resolve_file('costmap.csv', ['data', 'reference', ''])
    waypoints_file = resolve_file('waypoints.csv', ['reference', 'data', ''])

    print(f"Loading costmap from: {costmap_file}")
    grid, W, H, res, ox, oy = load_costmap(costmap_file)
    print(f"Loaded costmap ({W}x{H}, res={res}m, origin=({ox:.2f}, {oy:.2f}))")

    print(f"Loading waypoints from: {waypoints_file}")
    wps = load_waypoints(waypoints_file)

    # Unique locations (ignoring duplicates / return to start)
    unique_wps = []
    seen = set()
    for w in wps:
        key = (round(w[0], 4), round(w[1], 4))
        if key not in seen:
            seen.add(key)
            unique_wps.append(w)

    print(f"Unique Waypoints: {len(unique_wps)} points -> {[w[2] for w in unique_wps]}")

    start_node = unique_wps[0] # S1 (0,0)
    intermediate_nodes = unique_wps[1:] # 4 waypoints

    print("\nComputing pairwise shortest D* paths between all waypoints...")
    pair_metrics = {}
    all_nodes = [start_node] + intermediate_nodes
    N = len(all_nodes)

    for i in range(N):
        for j in range(N):
            if i != j:
                src, dst = all_nodes[i], all_nodes[j]
                dist_m, cost_units = dijkstra_pair(grid, W, H, res, ox, oy, src, dst)
                pair_metrics[(i, j)] = (dist_m, cost_units)
                print(f"  Path {src[2]:4s} -> {dst[2]:4s}: Dist = {dist_m:6.2f}m, Cost = {cost_units:6.2f}")

    # Evaluate all 4! = 24 permutations
    print("\nEvaluating all 24 Waypoint Permutation Tours:")
    print("=" * 75)
    print(f"{'#':<3} | {'Waypoint Order':<35} | {'Distance (m)':<12} | {'Cost (units)':<12}")
    print("-" * 75)

    perm_results = []
    perm_indices = list(range(1, N)) # Indices 1..4 for intermediate nodes

    for idx, perm in enumerate(itertools.permutations(perm_indices), 1):
        tour_indices = [0] + list(perm) + [0] # Start (0) -> perm -> Start (0)
        tot_dist = 0.0
        tot_cost = 0.0
        valid = True

        for k in range(len(tour_indices) - 1):
            src_i, dst_i = tour_indices[k], tour_indices[k+1]
            d, c = pair_metrics[(src_i, dst_i)]
            if math.isinf(d):
                valid = False
                break
            tot_dist += d
            tot_cost += c

        tour_labels = " -> ".join([all_nodes[i][2] for i in tour_indices])
        if valid:
            perm_results.append((tot_cost, tot_dist, tour_indices, tour_labels))
            print(f"{idx:<3} | {tour_labels:<35} | {tot_dist:<12.2f} | {tot_cost:<12.2f}")
        else:
            print(f"{idx:<3} | {tour_labels:<35} | {'UNREACHABLE':<12} | {'UNREACHABLE':<12}")

    # Rank permutations
    perm_results_dist = sorted(perm_results, key=lambda x: x[1]) # Min distance
    perm_results_cost = sorted(perm_results, key=lambda x: x[0]) # Min cost

    best_dist_tour = perm_results_dist[0]
    best_cost_tour = perm_results_cost[0]

    print("=" * 75)
    print("\n🏆 OPTIMAL TOUR RESULTS:")
    print(f"🥇 Minimum Distance Tour: {best_dist_tour[3]}")
    print(f"   -> Total Distance: {best_dist_tour[1]:.2f} meters | Total Cost: {best_dist_tour[0]:.2f} units")

    print(f"\n🥇 Minimum Cost Tour    : {best_cost_tour[3]}")
    print(f"   -> Total Distance: {best_cost_tour[1]:.2f} meters | Total Cost: {best_cost_tour[0]:.2f} units")

    # Pick overall optimal tour (min distance)
    optimal_indices = best_dist_tour[2]
    optimal_wps = [all_nodes[i] for i in optimal_indices[:-1]] # Exclude duplicate return start for file

    # Save optimal waypoints to data/optimal_waypoints.csv and update waypoints.csv
    targets = set([
        os.path.normpath(os.path.join(_here, '..', '..', 'data', 'optimal_waypoints.csv')),
        os.path.normpath(os.path.join(_here, '..', '..', 'reference', 'waypoints.csv')),
        os.path.normpath(os.path.join(_here, '..', '..', 'waypoints.csv')),
        os.path.normpath(os.path.join(cwd, 'path-planning', 'globalPlanner', 'offline_tools', 'reference', 'waypoints.csv')),
        os.path.normpath(os.path.join(cwd, 'path-planning', 'globalPlanner', 'offline_tools', 'waypoints.csv')),
        os.path.normpath(os.path.join(cwd, 'waypoints.csv')),
    ])

    for target_path in targets:
        try:
            os.makedirs(os.path.dirname(target_path), exist_ok=True)
            with open(target_path, 'w', newline='') as f:
                writer = csv.writer(f)
                for w in optimal_wps:
                    writer.writerow([f"{w[0]:.4f}", f"{w[1]:.4f}", w[2]])
            print(f"Saved optimal tour waypoints to: {target_path}")
        except Exception as e:
            print(f"Failed to write target waypoints CSV {target_path}: {e}")

if __name__ == '__main__':
    main()
