import re

path = "heightmap_generator_slam.py"
with open(path) as f:
    content = f.read()

old_func = '''def load_mesh_triangles(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".stl":
        return read_stl(path)
    if trimesh is not None:
        loaded = trimesh.load(path, force="mesh")
        verts = loaded.vertices[loaded.faces]
        return np.asarray(verts, dtype=np.float64)
    raise RuntimeError(
        f"Cannot load mesh '{path}': only .stl is supported without the "
        f"optional 'trimesh' package. Install it with `pip install trimesh` "
        f"to support .dae/.obj/etc."
    )'''

new_func = '''def load_obj_manual(path):
    vertices = []
    faces = []
    with open(path, encoding="utf-8", errors="ignore") as f:
        for raw_line in f:
            line = raw_line.strip()
            if line.startswith("v "):
                parts = line.split()
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith("f "):
                indices = []
                for token in line.split()[1:]:
                    value = int(token.split("/")[0])
                    indices.append(value - 1 if value > 0 else len(vertices) + value)
                for i in range(1, len(indices) - 1):
                    faces.append([indices[0], indices[i], indices[i + 1]])
    if not vertices or not faces:
        raise RuntimeError(f"OBJ contains no triangle mesh data: {path}")
    verts = np.asarray(vertices, dtype=np.float64)
    return verts[np.asarray(faces, dtype=np.int64)]


def load_mesh_triangles(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".stl":
        return read_stl(path)
    if ext == ".obj":
        return load_obj_manual(path)
    if trimesh is not None:
        loaded = trimesh.load(path, force="mesh")
        verts = loaded.vertices[loaded.faces]
        return np.asarray(verts, dtype=np.float64)
    raise RuntimeError(
        f"Cannot load mesh '{path}': only .stl and .obj are supported without "
        f"the optional 'trimesh' package. Install it with `pip install trimesh` "
        f"to support .dae/etc."
    )'''

if old_func not in content:
    raise SystemExit("Could not find exact function text to replace — aborting, no changes made.")

content = content.replace(old_func, new_func)
with open(path, "w") as f:
    f.write(content)

print("Patched successfully.")
