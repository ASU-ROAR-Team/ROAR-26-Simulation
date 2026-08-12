import numpy as np
from heightmap_generator import load_mesh_triangles

MESH = "/home/misara/nav_ws/assets/mars_yard/meshes/mars_yard_exact_collision.obj"
REF = "../initial_inputs/i_heightmap/heightmap.npz"

ref = np.load(REF)
ref_w = float(ref["xs"].max() - ref["xs"].min())
ref_h = float(ref["ys"].max() - ref["ys"].min())
print(f"Target: w={ref_w:.3f} h={ref_h:.3f}")

triangles = load_mesh_triangles(MESH)

best = None
for yaw_deg in np.arange(0, 180, 0.5):
    rad = np.radians(yaw_deg)
    c, s = np.cos(rad), np.sin(rad)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    rotated = triangles @ R.T
    w = rotated[:,:,0].max() - rotated[:,:,0].min()
    h = rotated[:,:,1].max() - rotated[:,:,1].min()
    err = abs(w - ref_w) + abs(h - ref_h)
    if best is None or err < best[0]:
        best = (err, yaw_deg, w, h)

err, yaw_deg, w, h = best
print(f"Best yaw: {yaw_deg} deg  w={w:.3f} h={h:.3f}  err={err:.3f}")
