#!/usr/bin/env python3
import argparse
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

try:
    from PIL import Image
except ImportError:
    Image = None

try:
    import trimesh
except ImportError:
    trimesh = None

def parse_pose(pose_str):
    if pose_str is None:
        return np.eye(4)
    vals = [float(v) for v in pose_str.split()]
    if len(vals) == 6:
        x, y, z, roll, pitch, yaw = vals
    elif len(vals) == 7:
        x, y, z, qw, qx, qy, qz = vals
        return quat_pose_matrix(x, y, z, qw, qx, qy, qz)
    else:
        return np.eye(4)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    R = Rz @ Ry @ Rx
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def quat_pose_matrix(x, y, z, qw, qx, qy, qz):
    n = qw * qw + qx * qx + qy * qy + qz * qz
    if n < 1e-12:
        R = np.eye(3)
    else:
        s = 2.0 / n
        R = np.array([
            [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
            [s * (qx * qy + qz * qw), 1 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
            [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1 - s * (qx * qx + qy * qy)],
        ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def apply_transform(T, points):
    pts_h = np.hstack([points, np.ones((points.shape[0], 1))])
    return (pts_h @ T.T)[:, :3]

def gather_search_dirs(world_path, extra_dirs):
    dirs = []
    if extra_dirs:
        dirs.extend(extra_dirs)

    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(world_path)))
    dirs.append(os.path.join(pkg_dir, "models"))

    src_root = pkg_dir
    for _ in range(3):
        parent = os.path.dirname(src_root)
        if parent == src_root:
            break
        src_root = parent
    for root, subdirs, _ in os.walk(src_root):
        if "install" in root.split(os.sep) or "build" in root.split(os.sep):
            subdirs[:] = []
            continue
        if os.path.basename(root) == "models":
            dirs.append(root)

    for env_var in ("GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH", "GAZEBO_MODEL_PATH"):
        val = os.environ.get(env_var)
        if val:
            dirs.extend(p for p in val.split(":") if p)

    dirs.append(os.path.expanduser("~/.ignition/fuel/models"))
    dirs.append(os.path.expanduser("~/.gazebo/models"))

    seen, uniq = set(), []
    for d in dirs:
        if d and d not in seen and os.path.isdir(d):
            seen.add(d)
            uniq.append(d)
    return uniq


def resolve_model_uri(uri, search_dirs):
    if uri.startswith("model://"):
        rest = uri[len("model://"):]
        name, _, sub = rest.partition("/")
        for base in search_dirs:
            candidate = os.path.join(base, name)
            if os.path.isdir(candidate):
                return os.path.join(candidate, sub) if sub else candidate
        return None
    if uri.startswith("file://"):
        return uri[len("file://"):]
    return uri


def find_world_includes(world_path):
    tree = ET.parse(world_path)
    root = tree.getroot()
    world_el = root.find("world")
    if world_el is None:
        world_el = root
    includes = []
    for inc in world_el.findall("include"):
        uri_el = inc.find("uri")
        if uri_el is None or not uri_el.text:
            continue
        pose_el = inc.find("pose")
        includes.append((uri_el.text.strip(), pose_el.text.strip() if pose_el is not None else None))
    return includes


def find_terrain_geometry(model_dir, prefer="collision"):
    config_path = os.path.join(model_dir, "model.config")
    sdf_file = "model.sdf"
    if os.path.isfile(config_path):
        cfg = ET.parse(config_path).getroot()
        sdf_el = cfg.find("sdf")
        if sdf_el is not None and sdf_el.text:
            sdf_file = sdf_el.text.strip()
    sdf_path = os.path.join(model_dir, sdf_file)
    if not os.path.isfile(sdf_path):
        return None

    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model_el = root.find("model")
    if model_el is None:
        return None

    model_pose = parse_pose(model_el.findtext("pose"))

    for link_el in model_el.findall("link"):
        link_pose = parse_pose(link_el.findtext("pose"))
        tags = ["collision", "visual"] if prefer == "collision" else ["visual", "collision"]
        for tag in tags:
            for geom_owner in link_el.findall(tag):
                geom_pose = parse_pose(geom_owner.findtext("pose"))
                geometry = geom_owner.find("geometry")
                if geometry is None:
                    continue
                total_pose = model_pose @ link_pose @ geom_pose

                heightmap = geometry.find("heightmap")
                if heightmap is not None:
                    return {
                        "type": "heightmap",
                        "element": heightmap,
                        "model_dir": model_dir,
                        "pose": total_pose,
                    }

                mesh = geometry.find("mesh")
                if mesh is not None:
                    uri = mesh.findtext("uri") or mesh.findtext("filename")
                    scale = mesh.findtext("scale")
                    return {
                        "type": "mesh",
                        "uri": uri,
                        "scale": [float(v) for v in scale.split()] if scale else [1.0, 1.0, 1.0],
                        "model_dir": model_dir,
                        "pose": total_pose,
                    }
    return None


def read_stl(path):
    with open(path, "rb") as f:
        header = f.read(80)
        rest = f.read()
    if header.strip().lower().startswith(b"solid") and b"facet normal" in rest[:2000]:
        return _read_stl_ascii(header + rest)
    return _read_stl_binary(rest)


def _read_stl_binary(rest):
    (n,) = struct.unpack_from("<I", rest, 0)
    offset = 4
    tri = np.zeros((n, 3, 3), dtype=np.float64)
    record = struct.Struct("<12fH")
    for i in range(n):
        vals = record.unpack_from(rest, offset)
        offset += record.size
        tri[i, 0] = vals[3:6]
        tri[i, 1] = vals[6:9]
        tri[i, 2] = vals[9:12]
    return tri


def _read_stl_ascii(data):
    text = data.decode("ascii", errors="ignore")
    verts = []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("vertex"):
            verts.append([float(v) for v in line.split()[1:4]])
    verts = np.array(verts, dtype=np.float64)
    return verts.reshape(-1, 3, 3)


def load_obj_triangles(path):
    """Native .obj loader — no trimesh required."""
    verts = []
    faces = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("v "):
                parts = line.split()
                verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith("f "):
                parts = line.split()[1:]
                idxs = [int(p.split("/")[0]) - 1 for p in parts]
                for i in range(1, len(idxs) - 1):
                    faces.append([idxs[0], idxs[i], idxs[i + 1]])
    verts = np.array(verts, dtype=np.float64)
    faces = np.array(faces, dtype=np.int64)
    return verts[faces]


def load_mesh_triangles(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".stl":
        return read_stl(path)
    if ext == ".obj":
        return load_obj_triangles(path)
    if trimesh is not None:
        loaded = trimesh.load(path, force="mesh")
        verts = loaded.vertices[loaded.faces]
        return np.asarray(verts, dtype=np.float64)
    raise RuntimeError(
        f"Cannot load mesh '{path}': only .stl and .obj are natively supported. "
        f"For other formats, install trimesh with `pip install trimesh`."
    )


def rasterize_triangles(triangles, resolution, bounds=None, margin=0.0):
    if bounds is None:
        xmin = triangles[:, :, 0].min() - margin
        xmax = triangles[:, :, 0].max() + margin
        ymin = triangles[:, :, 1].min() - margin
        ymax = triangles[:, :, 1].max() + margin
    else:
        xmin, xmax, ymin, ymax = bounds

    nx = max(2, int(np.ceil((xmax - xmin) / resolution)) + 1)
    ny = max(2, int(np.ceil((ymax - ymin) / resolution)) + 1)
    xs = xmin + np.arange(nx) * resolution
    ys = ymin + np.arange(ny) * resolution
    grid = np.full((ny, nx), np.nan, dtype=np.float64)

    for tri in triangles:
        (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = tri

        tx_min, tx_max = min(x0, x1, x2), max(x0, x1, x2)
        ty_min, ty_max = min(y0, y1, y2), max(y0, y1, y2)
        if tx_max < xmin or tx_min > xmax or ty_max < ymin or ty_min > ymax:
            continue

        i0 = max(0, int(np.floor((tx_min - xmin) / resolution)) - 1)
        i1 = min(nx - 1, int(np.ceil((tx_max - xmin) / resolution)) + 1)
        j0 = max(0, int(np.floor((ty_min - ymin) / resolution)) - 1)
        j1 = min(ny - 1, int(np.ceil((ty_max - ymin) / resolution)) + 1)
        if i1 < i0 or j1 < j0:
            continue

        sub_x = xs[i0:i1 + 1]
        sub_y = ys[j0:j1 + 1]
        gx, gy = np.meshgrid(sub_x, sub_y)

        denom = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(denom) < 1e-12:
            continue
        w0 = ((y1 - y2) * (gx - x2) + (x2 - x1) * (gy - y2)) / denom
        w1 = ((y2 - y0) * (gx - x2) + (x0 - x2) * (gy - y2)) / denom
        w2 = 1.0 - w0 - w1

        eps = -1e-6
        inside = (w0 >= eps) & (w1 >= eps) & (w2 >= eps)
        if not inside.any():
            continue

        z = w0 * z0 + w1 * z1 + w2 * z2
        sub_grid = grid[j0:j1 + 1, i0:i1 + 1]
        np.putmask(sub_grid, inside & ((sub_grid < z) | np.isnan(sub_grid)), z)
        grid[j0:j1 + 1, i0:i1 + 1] = sub_grid

    return xs, ys, grid


def rasterize_heightmap_image(heightmap_el, model_dir, search_dirs, resolution, pose):
    uri = heightmap_el.findtext("uri")
    size_str = heightmap_el.findtext("size") or "1 1 1"
    sx, sy, sz = (float(v) for v in size_str.split())
    pos_str = heightmap_el.findtext("pos") or "0 0 0"
    px, py, pz = (float(v) for v in pos_str.split())

    img_path = resolve_model_uri(uri, search_dirs + [model_dir])
    if img_path is None or not os.path.isfile(img_path):
        img_path = os.path.join(model_dir, uri) if uri else None
    if img_path is None or not os.path.isfile(img_path):
        raise RuntimeError(f"Could not resolve heightmap image '{uri}' under {model_dir}")
    if Image is None:
        raise RuntimeError("Reading heightmap images requires Pillow: pip install Pillow")

    img = Image.open(img_path).convert("L")
    nx = max(2, int(round(sx / resolution)) + 1)
    ny = max(2, int(round(sy / resolution)) + 1)
    img_resized = img.resize((nx, ny))
    arr = np.asarray(img_resized, dtype=np.float64) / 255.0
    z = pz + arr * sz

    xs = px - sx / 2.0 + np.arange(nx) * resolution
    ys = py - sy / 2.0 + np.arange(ny) * resolution

    xs = xs + pose[0, 3]
    ys = ys + pose[1, 3]
    return xs, ys, z


def fill_nan_nearest(xs, ys, grid):
    from scipy.interpolate import griddata
    gx, gy = np.meshgrid(xs, ys)
    mask = ~np.isnan(grid)
    if mask.sum() == 0:
        return grid
    filled = griddata(
        (gx[mask], gy[mask]), grid[mask], (gx, gy), method="nearest"
    )
    return filled


def write_csv(xs, ys, grid, out_path, resolution, long_format=False):
    if long_format:
        with open(out_path, "w") as f:
            f.write("x,y,z\n")
            for j, y in enumerate(ys):
                for i, x in enumerate(xs):
                    z = grid[j, i]
                    if not np.isnan(z):
                        f.write(f"{x:.4f},{y:.4f},{z:.6f}\n")
        return

    with open(out_path, "w") as f:
        f.write(f"# resolution={resolution}\n")
        f.write(f"# origin_x={xs[0]:.4f} origin_y={ys[0]:.4f}\n")
        f.write(f"# rows={grid.shape[0]} cols={grid.shape[1]}\n")
        f.write("# row 0 = min Y, col 0 = min X; value = NaN where no data\n")
        for row in grid:
            f.write(",".join("nan" if np.isnan(v) else f"{v:.6f}" for v in row))
            f.write("\n")


def write_npz(xs, ys, grid, out_path, resolution):
    np.savez(
        out_path,
        xs=xs,
        ys=ys,
        grid=grid,
        resolution=np.float64(resolution),
        origin_x=np.float64(xs[0]),
        origin_y=np.float64(ys[0]),
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", nargs="?", help="path to a .world SDF file")
    ap.add_argument("--mesh", help="skip world parsing, rasterize this mesh file directly")
    ap.add_argument("-o", "--output", required=True, help="output path; format is picked from the extension (.csv or .npz)")
    ap.add_argument("--resolution", type=float, default=0.25, help="grid cell size in meters (default: 0.25)")
    ap.add_argument("--prefer", choices=["collision", "visual"], default="collision",
                     help="which geometry to rasterize when a model has both (default: collision)")
    ap.add_argument("--model", help="if the world includes several models, pick this one by name/uri substring")
    ap.add_argument("--model-path", action="append", default=[], help="extra directory to search for model:// includes (repeatable)")
    ap.add_argument("--fill-nan", action="store_true", help="fill gaps in the grid via nearest-neighbor interpolation")
    ap.add_argument("--long-format", action="store_true", help="write long-format x,y,z rows instead of a grid matrix (CSV output only)")
    args = ap.parse_args()

    if not args.mesh and not args.world:
        ap.error("provide a .world file or --mesh")

    is_npz = args.output.lower().endswith(".npz")
    if is_npz and args.long_format:
        ap.error("--long-format is a CSV-only option")

    if args.mesh:
        triangles = load_mesh_triangles(args.mesh)
        xs, ys, grid = rasterize_triangles(triangles, args.resolution)
    else:
        search_dirs = gather_search_dirs(args.world, args.model_path)
        includes = find_world_includes(args.world)
        if not includes:
            sys.exit(f"No <include> models found in {args.world}")

        if args.model:
            includes = [inc for inc in includes if args.model in inc[0]] or includes

        geometry = None
        used_uri = None
        for uri, _pose in includes:
            model_dir = resolve_model_uri(uri, search_dirs)
            if model_dir is None or not os.path.isdir(model_dir):
                continue
            geometry = find_terrain_geometry(model_dir, prefer=args.prefer)
            if geometry is not None:
                used_uri = uri
                break

        if geometry is None:
            sys.exit(
                "Could not find a heightmap or mesh geometry in any included model.\n"
                f"Includes found: {[u for u, _ in includes]}\n"
                "Try --model-path to point at where the model:// resources live, "
                "or --mesh to rasterize a mesh file directly."
            )

        print(f"Using terrain from '{used_uri}' ({geometry['type']})", file=sys.stderr)

        if geometry["type"] == "heightmap":
            xs, ys, grid = rasterize_heightmap_image(
                geometry["element"], geometry["model_dir"], search_dirs,
                args.resolution, geometry["pose"],
            )
        else:
            mesh_path = resolve_model_uri(geometry["uri"], search_dirs + [geometry["model_dir"]])
            if mesh_path is None or not os.path.isfile(mesh_path):
                mesh_path = os.path.join(geometry["model_dir"], os.path.basename(geometry["uri"]))
            if not os.path.isfile(mesh_path):
                sys.exit(f"Could not resolve mesh uri '{geometry['uri']}'")
            triangles = load_mesh_triangles(mesh_path)
            triangles = triangles * np.array(geometry["scale"])
            n = triangles.shape[0]
            triangles = apply_transform(geometry["pose"], triangles.reshape(-1, 3)).reshape(n, 3, 3)
            xs, ys, grid = rasterize_triangles(triangles, args.resolution)

    if args.fill_nan:
        grid = fill_nan_nearest(xs, ys, grid)

    if is_npz:
        write_npz(xs, ys, grid, args.output, args.resolution)
    else:
        write_csv(xs, ys, grid, args.output, args.resolution, long_format=args.long_format)
    n_nan = int(np.isnan(grid).sum())
    print(
        f"Wrote {grid.shape[1]}x{grid.shape[0]} height map to {args.output} "
        f"(z range {np.nanmin(grid):.3f}..{np.nanmax(grid):.3f}, {n_nan} empty cells)",
        file=sys.stderr,
    )


if __name__ == "__main__":
    main()
