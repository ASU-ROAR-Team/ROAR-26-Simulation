import numpy as np
from heightmap_generator import load_mesh_triangles

MESH_PATH = "/home/misara/Simulation_ws/src/ROAR-26-Simulation/marsyards/marsyard/models/mars_yard/meshes/mars_yard_exact_collision.obj"

triangles = load_mesh_triangles(MESH_PATH)
print("triangle count:", len(triangles))

xy = triangles[:, :, :2].reshape(-1, 2)
xmin, ymin = xy.min(axis=0)
xmax, ymax = xy.max(axis=0)
footprint_area = (xmax - xmin) * (ymax - ymin)
print("footprint area (m^2):", footprint_area)

kept = min(len(triangles), 20000)
avg_area_per_tri = footprint_area / kept
print("avg edge after decimation (m):", avg_area_per_tri ** 0.5)
