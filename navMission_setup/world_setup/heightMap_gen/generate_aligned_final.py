import numpy as np
from heightmap_generator import load_mesh_triangles, rasterize_triangles, write_npz, write_preview

MESH = "/home/misara/nav_ws/assets/mars_yard/meshes/mars_yard_exact_collision.obj"
REF = "../initial_inputs/i_heightmap/heightmap.npz"
RESOLUTION = 0.1
YAW_DEG = 88

ref = np.load(REF)
ref_bounds = (
    float(ref["xs"].min()), float(ref["xs"].max()),
    float(ref["ys"].min()), float(ref["ys"].max()),
)

triangles = load_mesh_triangles(MESH)
rad = np.radians(YAW_DEG)
c, s = np.cos(rad), np.sin(rad)
R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
rotated = triangles @ R.T

# align mins to reference mins (same translation logic as fix_rotation2.py)
xmin, ymin = rotated[:,:,0].min(), rotated[:,:,1].min()
tx = ref_bounds[0] - xmin
ty = ref_bounds[2] - ymin
rotated[:,:,0] += tx
rotated[:,:,1] += ty

xs, ys, grid = rasterize_triangles(rotated, RESOLUTION, bounds=ref_bounds)

write_npz("outputs/heightmap.npz", xs=xs, ys=ys, grid=grid,
          resolution=RESOLUTION, world_path=MESH, geometry_count=1)
write_preview("outputs/heightmap.png", grid)

empty = int(np.isnan(grid).sum())
print(f"Grid: {grid.shape}  Empty cells: {empty} / {grid.size}")
