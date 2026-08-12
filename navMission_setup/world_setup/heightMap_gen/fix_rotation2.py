import numpy as np
from heightmap_generator import load_mesh_triangles, apply_transform, rasterize_triangles, write_npz, write_preview

MESH = "/home/misara/nav_ws/assets/mars_yard/meshes/mars_yard_exact_collision.obj"
REF = "../initial_inputs/i_heightmap/heightmap.npz"
RESOLUTION = 0.1

ref = np.load(REF)
ref_xmin, ref_xmax = float(ref["xs"].min()), float(ref["xs"].max())
ref_ymin, ref_ymax = float(ref["ys"].min()), float(ref["ys"].max())

triangles = load_mesh_triangles(MESH)

best = None
for yaw_deg in (90, -90):
    rad = np.radians(yaw_deg)
    c, s = np.cos(rad), np.sin(rad)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    rotated = triangles @ R.T

    xmin, xmax = rotated[:,:,0].min(), rotated[:,:,0].max()
    ymin, ymax = rotated[:,:,1].min(), rotated[:,:,1].max()

    # translation needed to align this rotated mesh's bounds to the reference bounds
    tx = ref_xmin - xmin
    ty = ref_ymin - ymin

    final = rotated.copy()
    final[:,:,0] += tx
    final[:,:,1] += ty

    err = abs(final[:,:,0].max() - ref_xmax) + abs(final[:,:,1].max() - ref_ymax)
    print(f"yaw={yaw_deg}: tx={tx:.3f} ty={ty:.3f} alignment_error={err:.3f}")

    if best is None or err < best[0]:
        best = (err, yaw_deg, final)

err, yaw_deg, final_triangles = best
print(f"\nBest match: yaw={yaw_deg}")

xs, ys, grid = rasterize_triangles(final_triangles, RESOLUTION,
                                     bounds=(ref_xmin, ref_xmax, ref_ymin, ref_ymax))

write_npz("/tmp/aligned_heightmap.npz", xs=xs, ys=ys, grid=grid,
          resolution=RESOLUTION, world_path=MESH, geometry_count=1)
write_preview("/tmp/aligned_heightmap.png", grid)

empty = int(np.isnan(grid).sum())
print(f"Grid: {grid.shape}  Empty cells: {empty} / {grid.size}")
print("Saved /tmp/aligned_heightmap.npz and /tmp/aligned_heightmap.png")
