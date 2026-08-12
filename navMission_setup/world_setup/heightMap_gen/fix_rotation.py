import numpy as np
from heightmap_generator import load_mesh_triangles, rasterize_triangles, write_npz, write_preview

MESH = "/home/misara/nav_ws/assets/mars_yard/meshes/mars_yard_exact_collision.obj"
REF = "../initial_inputs/i_heightmap/heightmap.npz"
RESOLUTION = 0.1

ref = np.load(REF)
ref_xmin, ref_xmax = ref["xs"].min(), ref["xs"].max()
ref_ymin, ref_ymax = ref["ys"].min(), ref["ys"].max()
print(f"REFERENCE bounds: x[{ref_xmin:.2f},{ref_xmax:.2f}] y[{ref_ymin:.2f},{ref_ymax:.2f}]")

triangles = load_mesh_triangles(MESH)

for yaw_deg in (0, 90, -90, 180):
    rad = np.radians(yaw_deg)
    c, s = np.cos(rad), np.sin(rad)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    rotated = triangles @ R.T
    xmin, xmax = rotated[:,:,0].min(), rotated[:,:,0].max()
    ymin, ymax = rotated[:,:,1].min(), rotated[:,:,1].max()
    print(f"yaw={yaw_deg:>4}: x[{xmin:.2f},{xmax:.2f}] (w={xmax-xmin:.2f}) "
          f"y[{ymin:.2f},{ymax:.2f}] (h={ymax-ymin:.2f})")
