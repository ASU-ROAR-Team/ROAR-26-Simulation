#!/usr/bin/env python3
"""Generate a metric height map from a complete Gazebo / SDF world.

This module fuses base terrain (mesh or SDF heightmap tag) and all inline / included
rock meshes into one unified grid with SLAM coordinate alignment support (--yaw, --swap-xy, --fill-nan).
Collision geometry is preferred; visual geometry is used only when a model has no collision geometry,
so generated non-collidable rocks still appear in the world height map.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

try:
    from PIL import Image
except ImportError:  # pragma: no cover
    Image = None

try:
    import trimesh
except ImportError:  # pragma: no cover
    trimesh = None


@dataclass(frozen=True)
class GeometrySource:
    name: str
    mesh_path: str
    scale: Tuple[float, float, float]
    instance_transform: np.ndarray
    local_transform: np.ndarray
    geometry_kind: str


_MESH_CACHE: Dict[str, np.ndarray] = {}
_LOCAL_RASTER_CACHE: Dict[Tuple, Tuple[np.ndarray, np.ndarray, np.ndarray]] = {}


def parse_pose(pose_str: Optional[str]) -> np.ndarray:
    if not pose_str:
        return np.eye(4, dtype=np.float64)
    vals = [float(v) for v in pose_str.split()]
    if len(vals) == 6:
        x, y, z, roll, pitch, yaw = vals
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rz @ ry @ rx
        transform[:3, 3] = [x, y, z]
        return transform
    if len(vals) == 7:
        x, y, z, qw, qx, qy, qz = vals
        return quaternion_pose_matrix(x, y, z, qw, qx, qy, qz)
    raise ValueError(f"Unsupported pose with {len(vals)} values: {pose_str!r}")


def quaternion_pose_matrix(x, y, z, qw, qx, qy, qz) -> np.ndarray:
    norm = qw * qw + qx * qx + qy * qy + qz * qz
    if norm < 1e-12:
        rotation = np.eye(3)
    else:
        s = 2.0 / norm
        rotation = np.array(
            [
                [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
                [s * (qx * qy + qz * qw), 1 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
                [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1 - s * (qx * qx + qy * qy)],
            ],
            dtype=np.float64,
        )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = [x, y, z]
    return transform


def apply_transform(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    points_h = np.hstack([points, np.ones((points.shape[0], 1), dtype=np.float64)])
    return (points_h @ transform.T)[:, :3]


def _unique_existing_dirs(paths: Iterable[str]) -> List[str]:
    result: List[str] = []
    seen = set()
    for raw in paths:
        if not raw:
            continue
        path = os.path.abspath(os.path.expanduser(raw))
        if path not in seen and os.path.isdir(path):
            seen.add(path)
            result.append(path)
    return result


def gather_model_paths(world_path: str, extra_paths: Sequence[str]) -> List[str]:
    world_path = os.path.abspath(world_path)
    candidates: List[str] = list(extra_paths)

    current = os.path.dirname(world_path)
    ancestors = []
    for _ in range(7):
        ancestors.append(current)
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent

    for ancestor in ancestors:
        candidates.extend(
            [
                os.path.join(ancestor, "models"),
                os.path.join(ancestor, "marsyard", "models"),
                os.path.join(ancestor, "marsyards", "models"),
                os.path.join(ancestor, "marsyards", "marsyard", "models"),
                os.path.join(ancestor, "dev_environment", "models"),
                os.path.join(ancestor, "mars_yard", "models"),
            ]
        )

    for env_name in ("GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH", "GAZEBO_MODEL_PATH"):
        candidates.extend(p for p in os.environ.get(env_name, "").split(os.pathsep) if p)

    try:
        from ament_index_python.packages import get_package_share_directory

        for package_name in ("marsyard", "worlds", "rock_generator"):
            try:
                share = get_package_share_directory(package_name)
                candidates.extend([share, os.path.join(share, "models"), os.path.join(share, "rocks_ws")])
            except Exception:
                pass
    except Exception:
        pass

    candidates.extend(
        [
            os.path.expanduser("~/.gazebo/models"),
            os.path.expanduser("~/.ignition/fuel/models"),
        ]
    )
    return _unique_existing_dirs(candidates)


def parse_package_paths(values: Sequence[str]) -> Dict[str, str]:
    paths: Dict[str, str] = {}
    for value in values:
        if "=" not in value:
            raise ValueError("--package-path must be PACKAGE=/absolute/or/relative/path")
        package, path = value.split("=", 1)
        package, path = package.strip(), os.path.abspath(os.path.expanduser(path.strip()))
        if not package or not os.path.isdir(path):
            raise ValueError(f"Invalid package path: {value!r}")
        paths[package] = path
    return paths


def resolve_uri(
    uri: str,
    *,
    context_dir: str,
    model_paths: Sequence[str],
    package_paths: Dict[str, str],
) -> Optional[str]:
    if not uri:
        return None
    uri = uri.strip()

    if uri.startswith("package://"):
        remainder = uri[len("package://") :]
        package_name, _, subpath = remainder.partition("/")
        package_root = package_paths.get(package_name)
        if package_root:
            candidate = os.path.join(package_root, subpath)
            if os.path.exists(candidate):
                return os.path.abspath(candidate)
        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory(package_name)
            candidate = os.path.join(share, subpath)
            if os.path.exists(candidate):
                return os.path.abspath(candidate)
        except Exception:
            pass
        for base in model_paths:
            for candidate in (
                os.path.join(base, package_name, subpath),
                os.path.join(base, subpath) if os.path.basename(base) == package_name else "",
            ):
                if candidate and os.path.exists(candidate):
                    return os.path.abspath(candidate)
        return None

    if uri.startswith("model://"):
        remainder = uri[len("model://") :]
        model_name, _, subpath = remainder.partition("/")
        for base in model_paths:
            candidates = [os.path.join(base, model_name, subpath)]
            if os.path.basename(base) == model_name:
                candidates.append(os.path.join(base, subpath))
            for candidate in candidates:
                if os.path.exists(candidate):
                    return os.path.abspath(candidate)
        return None

    if uri.startswith("file://"):
        candidate = uri[len("file://") :]
        return os.path.abspath(os.path.expanduser(candidate)) if os.path.exists(candidate) else None

    candidate = uri if os.path.isabs(uri) else os.path.join(context_dir, uri)
    return os.path.abspath(candidate) if os.path.exists(candidate) else None


def _model_sdf_path(model_dir: str) -> Optional[str]:
    config_path = os.path.join(model_dir, "model.config")
    sdf_name = "model.sdf"
    if os.path.isfile(config_path):
        root = ET.parse(config_path).getroot()
        sdf_el = root.find("sdf")
        if sdf_el is not None and sdf_el.text:
            sdf_name = sdf_el.text.strip()
    candidate = os.path.join(model_dir, sdf_name)
    return candidate if os.path.isfile(candidate) else None


def _owners_for_link(link_el: ET.Element, prefer: str, visual_fallback: bool) -> List[Tuple[str, ET.Element]]:
    collisions = [("collision", element) for element in link_el.findall("collision")]
    visuals = [("visual", element) for element in link_el.findall("visual")]
    if prefer == "collision":
        if collisions:
            return collisions
        return visuals if visual_fallback else []
    if visuals:
        return visuals
    return collisions


def _collect_model_geometries(
    model_el: ET.Element,
    *,
    instance_transform: np.ndarray,
    model_context_dir: str,
    model_paths: Sequence[str],
    package_paths: Dict[str, str],
    prefer: str,
    visual_fallback: bool,
    source_prefix: str,
) -> List[GeometrySource]:
    sources: List[GeometrySource] = []
    model_local_pose = parse_pose(model_el.findtext("pose"))

    for link_index, link_el in enumerate(model_el.findall("link")):
        link_pose = parse_pose(link_el.findtext("pose"))
        for kind, owner in _owners_for_link(link_el, prefer, visual_fallback):
            owner_pose = parse_pose(owner.findtext("pose"))
            geometry_el = owner.find("geometry")
            if geometry_el is None:
                continue
            mesh_el = geometry_el.find("mesh")
            if mesh_el is None:
                continue
            uri = mesh_el.findtext("uri") or mesh_el.findtext("filename")
            mesh_path = resolve_uri(
                uri or "",
                context_dir=model_context_dir,
                model_paths=model_paths,
                package_paths=package_paths,
            )
            if not mesh_path or not os.path.isfile(mesh_path):
                print(f"[heightmap] Warning: could not resolve mesh URI {uri!r} for {source_prefix}", file=sys.stderr)
                continue
            scale_text = mesh_el.findtext("scale")
            scale = tuple(float(v) for v in scale_text.split()) if scale_text else (1.0, 1.0, 1.0)
            if len(scale) != 3:
                raise ValueError(f"Invalid mesh scale {scale_text!r} in {source_prefix}")
            local_transform = model_local_pose @ link_pose @ owner_pose
            sources.append(
                GeometrySource(
                    name=f"{source_prefix}:{link_index}:{kind}",
                    mesh_path=os.path.abspath(mesh_path),
                    scale=scale,
                    instance_transform=instance_transform,
                    local_transform=local_transform,
                    geometry_kind=kind,
                )
            )
    return sources


def collect_world_geometries(
    world_path: str,
    *,
    model_paths: Sequence[str],
    package_paths: Dict[str, str],
    prefer: str = "collision",
    visual_fallback: bool = True,
    model_filter: Optional[str] = None,
) -> List[GeometrySource]:
    tree = ET.parse(world_path)
    root = tree.getroot()
    world_el = root.find("world") if root.tag != "world" else root
    if world_el is None:
        raise ValueError(f"No <world> element found in {world_path}")

    sources: List[GeometrySource] = []
    world_dir = os.path.dirname(os.path.abspath(world_path))

    for include_index, include_el in enumerate(world_el.findall("include")):
        uri = (include_el.findtext("uri") or "").strip()
        name = (include_el.findtext("name") or uri or f"include_{include_index}").strip()
        if model_filter and model_filter not in name and model_filter not in uri:
            continue
        model_dir = resolve_uri(uri, context_dir=world_dir, model_paths=model_paths, package_paths=package_paths)
        if not model_dir or not os.path.isdir(model_dir):
            continue
        sdf_path = _model_sdf_path(model_dir)
        if not sdf_path:
            continue
        model_root = ET.parse(sdf_path).getroot()
        model_el = model_root.find("model") if model_root.tag != "model" else model_root
        if model_el is None:
            continue
        include_pose = parse_pose(include_el.findtext("pose"))
        sources.extend(
            _collect_model_geometries(
                model_el,
                instance_transform=include_pose,
                model_context_dir=model_dir,
                model_paths=model_paths,
                package_paths=package_paths,
                prefer=prefer,
                visual_fallback=visual_fallback,
                source_prefix=name,
            )
        )

    for model_index, model_el in enumerate(world_el.findall("model")):
        name = model_el.get("name", f"model_{model_index}")
        if model_filter and model_filter not in name:
            continue
        model_pose = parse_pose(model_el.findtext("pose"))
        model_clone = ET.fromstring(ET.tostring(model_el, encoding="unicode"))
        pose_el = model_clone.find("pose")
        if pose_el is not None:
            model_clone.remove(pose_el)
        sources.extend(
            _collect_model_geometries(
                model_clone,
                instance_transform=model_pose,
                model_context_dir=world_dir,
                model_paths=model_paths,
                package_paths=package_paths,
                prefer=prefer,
                visual_fallback=visual_fallback,
                source_prefix=name,
            )
        )

    return sources


def _read_stl(path: str) -> np.ndarray:
    with open(path, "rb") as file_obj:
        header = file_obj.read(80)
        rest = file_obj.read()
    if header.strip().lower().startswith(b"solid") and b"facet normal" in rest[:2000]:
        text = (header + rest).decode("ascii", errors="ignore")
        vertices = []
        for line in text.splitlines():
            line = line.strip()
            if line.startswith("vertex"):
                vertices.append([float(v) for v in line.split()[1:4]])
        return np.asarray(vertices, dtype=np.float64).reshape(-1, 3, 3)
    (count,) = struct.unpack_from("<I", rest, 0)
    triangles = np.zeros((count, 3, 3), dtype=np.float64)
    record = struct.Struct("<12fH")
    offset = 4
    for index in range(count):
        values = record.unpack_from(rest, offset)
        offset += record.size
        triangles[index, 0] = values[3:6]
        triangles[index, 1] = values[6:9]
        triangles[index, 2] = values[9:12]
    return triangles


def _load_obj(path: str) -> np.ndarray:
    vertices: List[List[float]] = []
    faces: List[List[int]] = []
    with open(path, encoding="utf-8", errors="ignore") as file_obj:
        for raw_line in file_obj:
            line = raw_line.strip()
            if line.startswith("v "):
                parts = line.split()
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith("f "):
                indices = []
                for token in line.split()[1:]:
                    value = int(token.split("/")[0])
                    indices.append(value - 1 if value > 0 else len(vertices) + value)
                for index in range(1, len(indices) - 1):
                    faces.append([indices[0], indices[index], indices[index + 1]])
    if not vertices or not faces:
        raise RuntimeError(f"OBJ contains no triangle mesh data: {path}")
    vertex_array = np.asarray(vertices, dtype=np.float64)
    return vertex_array[np.asarray(faces, dtype=np.int64)]


def load_mesh_triangles(path: str) -> np.ndarray:
    path = os.path.abspath(path)
    cached = _MESH_CACHE.get(path)
    if cached is not None:
        return cached
    extension = os.path.splitext(path)[1].lower()
    if extension == ".obj":
        triangles = _load_obj(path)
    elif extension == ".stl":
        triangles = _read_stl(path)
    elif trimesh is not None:
        loaded = trimesh.load(path, force="mesh")
        triangles = np.asarray(loaded.vertices[loaded.faces], dtype=np.float64)
    else:
        raise RuntimeError(f"Unsupported mesh format {extension!r}; install trimesh for additional formats")
    _MESH_CACHE[path] = triangles
    return triangles


def _decimate_triangles(triangles: np.ndarray, maximum: int = 9000) -> np.ndarray:
    if len(triangles) <= maximum:
        return triangles
    step = int(math.ceil(len(triangles) / maximum))
    return triangles[::step]


def rasterize_triangles(
    triangles: np.ndarray,
    resolution: float,
    *,
    bounds: Optional[Tuple[float, float, float, float]] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if resolution <= 0:
        raise ValueError("resolution must be positive")
    if bounds is None:
        xmin = float(np.min(triangles[:, :, 0]))
        xmax = float(np.max(triangles[:, :, 0]))
        ymin = float(np.min(triangles[:, :, 1]))
        ymax = float(np.max(triangles[:, :, 1]))
    else:
        xmin, xmax, ymin, ymax = bounds
    nx = max(2, int(math.ceil((xmax - xmin) / resolution)) + 1)
    ny = max(2, int(math.ceil((ymax - ymin) / resolution)) + 1)
    xs = xmin + np.arange(nx, dtype=np.float64) * resolution
    ys = ymin + np.arange(ny, dtype=np.float64) * resolution
    grid = np.full((ny, nx), np.nan, dtype=np.float64)

    for triangle in triangles:
        (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = triangle
        tx_min, tx_max = min(x0, x1, x2), max(x0, x1, x2)
        ty_min, ty_max = min(y0, y1, y2), max(y0, y1, y2)
        if tx_max < xmin or tx_min > xmax or ty_max < ymin or ty_min > ymax:
            continue
        i0 = max(0, int(math.floor((tx_min - xmin) / resolution)) - 1)
        i1 = min(nx - 1, int(math.ceil((tx_max - xmin) / resolution)) + 1)
        j0 = max(0, int(math.floor((ty_min - ymin) / resolution)) - 1)
        j1 = min(ny - 1, int(math.ceil((ty_max - ymin) / resolution)) + 1)
        if i1 < i0 or j1 < j0:
            continue
        gx, gy = np.meshgrid(xs[i0 : i1 + 1], ys[j0 : j1 + 1])
        denominator = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(denominator) < 1e-12:
            continue
        w0 = ((y1 - y2) * (gx - x2) + (x2 - x1) * (gy - y2)) / denominator
        w1 = ((y2 - y0) * (gx - x2) + (x0 - x2) * (gy - y2)) / denominator
        w2 = 1.0 - w0 - w1
        inside = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
        if not np.any(inside):
            continue
        z_values = w0 * z0 + w1 * z1 + w2 * z2
        subgrid = grid[j0 : j1 + 1, i0 : i1 + 1]
        np.putmask(subgrid, inside & (np.isnan(subgrid) | (z_values > subgrid)), z_values)
    return xs, ys, grid


def _source_world_triangles(source: GeometrySource, R_yaw: np.ndarray) -> np.ndarray:
    triangles = load_mesh_triangles(source.mesh_path) * np.asarray(source.scale, dtype=np.float64)
    triangles = apply_transform(source.local_transform, triangles.reshape(-1, 3)).reshape(-1, 3, 3)
    combined = R_yaw @ source.instance_transform
    triangles = apply_transform(combined, triangles.reshape(-1, 3)).reshape(-1, 3, 3)
    return triangles


def _source_bounds(source: GeometrySource, R_yaw: np.ndarray) -> Tuple[float, float, float, float, float, float]:
    triangles = _source_world_triangles(source, R_yaw)
    return (
        float(np.min(triangles[:, :, 0])),
        float(np.max(triangles[:, :, 0])),
        float(np.min(triangles[:, :, 1])),
        float(np.max(triangles[:, :, 1])),
        float(np.min(triangles[:, :, 2])),
        float(np.max(triangles[:, :, 2])),
    )


def _matrix_key(matrix: np.ndarray) -> Tuple[float, ...]:
    return tuple(np.round(matrix.reshape(-1), 8).tolist())


def _local_raster(source: GeometrySource, global_resolution: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    raw = load_mesh_triangles(source.mesh_path) * np.asarray(source.scale, dtype=np.float64)
    raw = apply_transform(source.local_transform, raw.reshape(-1, 3)).reshape(-1, 3, 3)
    extent = np.ptp(raw.reshape(-1, 3), axis=0)
    positive_xy = [value for value in extent[:2] if value > 1e-6]
    smallest_xy = min(positive_xy) if positive_xy else global_resolution
    local_resolution = min(global_resolution / 2.0, max(0.01, smallest_xy / 5.0))
    cache_key = (
        source.mesh_path,
        tuple(np.round(source.scale, 8)),
        _matrix_key(source.local_transform),
        round(local_resolution, 6),
    )
    cached = _LOCAL_RASTER_CACHE.get(cache_key)
    if cached is not None:
        return cached
    raster_triangles = _decimate_triangles(raw)
    result = rasterize_triangles(raster_triangles, local_resolution)
    _LOCAL_RASTER_CACHE[cache_key] = result
    return result


def _overlay_source(
    grid: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
    source: GeometrySource,
    resolution: float,
    R_yaw: np.ndarray,
) -> int:
    local_xs, local_ys, local_grid = _local_raster(source, resolution)
    valid_y, valid_x = np.where(~np.isnan(local_grid))
    if len(valid_x) == 0:
        return 0
    points = np.column_stack(
        [local_xs[valid_x], local_ys[valid_y], local_grid[valid_y, valid_x]]
    )
    combined_transform = R_yaw @ source.instance_transform
    world_points = apply_transform(combined_transform, points)

    col_float = (world_points[:, 0] - xs[0]) / resolution
    row_float = (world_points[:, 1] - ys[0]) / resolution
    columns = np.rint(col_float).astype(np.int64)
    rows = np.rint(row_float).astype(np.int64)
    valid = (
        (columns >= 0)
        & (columns < grid.shape[1])
        & (rows >= 0)
        & (rows < grid.shape[0])
    )
    columns, rows, heights = columns[valid], rows[valid], world_points[valid, 2]
    for row, column, height in zip(rows, columns, heights):
        current = grid[row, column]
        if np.isnan(current) or height > current:
            grid[row, column] = height
    return int(len(heights))


def write_npz(
    output_path: str,
    *,
    xs: np.ndarray,
    ys: np.ndarray,
    grid: np.ndarray,
    resolution: float,
    world_path: str,
    geometry_count: int,
) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    np.savez_compressed(
        output_path,
        xs=xs,
        ys=ys,
        grid=grid,
        resolution=np.float64(resolution),
        origin_x=np.float64(xs[0]),
        origin_y=np.float64(ys[0]),
        world_path=np.asarray(os.path.abspath(world_path)),
        geometry_count=np.int64(geometry_count),
    )


def write_preview(output_path: str, grid: np.ndarray) -> None:
    if Image is None:
        return
    valid = ~np.isnan(grid)
    image = np.full(grid.shape, 255, dtype=np.uint8)
    if np.any(valid):
        z_min = float(np.nanmin(grid))
        z_max = float(np.nanmax(grid))
        if z_max - z_min < 1e-9:
            image[valid] = 0
        else:
            normalized = (grid[valid] - z_min) / (z_max - z_min)
            image[valid] = np.clip(np.rint(normalized * 254.0), 0, 254).astype(np.uint8)
    # Flip vertically so positive Y appears upward in normal image viewers.
    Image.fromarray(np.flipud(image), mode="L").save(output_path)


def rasterize_heightmap_image(
    heightmap_el: ET.Element,
    model_dir: str,
    model_paths: Sequence[str],
    package_paths: Dict[str, str],
    resolution: float,
    pose: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    uri = heightmap_el.findtext("uri")
    size_str = heightmap_el.findtext("size") or "1 1 1"
    sx, sy, sz = (float(v) for v in size_str.split())
    pos_str = heightmap_el.findtext("pos") or "0 0 0"
    px, py, pz = (float(v) for v in pos_str.split())

    img_path = resolve_uri(
        uri or "",
        context_dir=model_dir,
        model_paths=model_paths,
        package_paths=package_paths,
    )
    if img_path is None or not os.path.isfile(img_path):
        raise RuntimeError(f"Could not resolve heightmap image '{uri}' under {model_dir}")
    if Image is None:
        raise RuntimeError("Reading heightmap images requires Pillow: pip install Pillow")

    img = Image.open(img_path).convert("L")
    nx = max(2, int(round(sx / resolution)) + 1)
    ny = max(2, int(round(sy / resolution)) + 1)
    img_resized = img.resize((nx, ny))
    arr = np.asarray(img_resized, dtype=np.float64) / 255.0
    z = pz + arr * sz  # 0..255 -> 0..sz, offset by pos.z

    xs = px - sx / 2.0 + np.arange(nx) * resolution
    ys = py - sy / 2.0 + np.arange(ny) * resolution

    # apply model/link/visual pose translation
    xs = xs + pose[0, 3]
    ys = ys + pose[1, 3]
    z = z + pose[2, 3]
    return xs, ys, z


def fill_nan_nearest(xs: np.ndarray, ys: np.ndarray, grid: np.ndarray) -> np.ndarray:
    from scipy.interpolate import griddata
    gx, gy = np.meshgrid(xs, ys)
    mask = ~np.isnan(grid)
    if mask.sum() == 0:
        return grid
    filled = griddata(
        (gx[mask], gy[mask]), grid[mask], (gx, gy), method="nearest"
    )
    return filled


def heightmap_grid_to_triangles(xs: np.ndarray, ys: np.ndarray, grid: np.ndarray) -> np.ndarray:
    ny, nx = grid.shape
    gx, gy = np.meshgrid(xs, ys)
    vertices = np.stack([gx, gy, grid], axis=-1)

    r = np.arange(ny - 1)[:, np.newaxis]
    c = np.arange(nx - 1)

    t1_v0 = vertices[r, c]
    t1_v1 = vertices[r + 1, c]
    t1_v2 = vertices[r, c + 1]

    t2_v0 = vertices[r + 1, c]
    t2_v1 = vertices[r + 1, c + 1]
    t2_v2 = vertices[r, c + 1]

    t1 = np.stack([t1_v0, t1_v1, t1_v2], axis=-2).reshape(-1, 3, 3)
    t2 = np.stack([t2_v0, t2_v1, t2_v2], axis=-2).reshape(-1, 3, 3)

    triangles = np.concatenate([t1, t2], axis=0)
    # Filter out triangles that contain NaN heights
    mask = ~np.isnan(triangles).any(axis=(1, 2))
    return triangles[mask]


def find_world_heightmap_geometry(
    world_path: str,
    model_paths: Sequence[str],
    package_paths: Dict[str, str],
) -> Optional[Tuple[ET.Element, str, np.ndarray]]:
    """
    Find a heightmap geometry element inside the world (either included or inline).
    Returns: (heightmap_element, model_dir, total_pose) or None.
    """
    tree = ET.parse(world_path)
    root = tree.getroot()
    world_el = root.find("world") if root.tag != "world" else root
    if world_el is None:
        return None

    world_dir = os.path.dirname(os.path.abspath(world_path))

    # 1. Search included models
    for include_index, include_el in enumerate(world_el.findall("include")):
        uri = (include_el.findtext("uri") or "").strip()
        model_dir = resolve_uri(uri, context_dir=world_dir, model_paths=model_paths, package_paths=package_paths)
        if not model_dir or not os.path.isdir(model_dir):
            continue
        sdf_path = _model_sdf_path(model_dir)
        if not sdf_path:
            continue
        model_root = ET.parse(sdf_path).getroot()
        model_el = model_root.find("model") if model_root.tag != "model" else model_root
        if model_el is None:
            continue

        include_pose = parse_pose(include_el.findtext("pose"))
        model_local_pose = parse_pose(model_el.findtext("pose"))

        for link_el in model_el.findall("link"):
            link_pose = parse_pose(link_el.findtext("pose"))
            # Search collision then visual
            for tag in ["collision", "visual"]:
                for geom_owner in link_el.findall(tag):
                    geom_pose = parse_pose(geom_owner.findtext("pose"))
                    geometry_el = geom_owner.find("geometry")
                    if geometry_el is None:
                        continue
                    heightmap_el = geometry_el.find("heightmap")
                    if heightmap_el is not None:
                        total_pose = include_pose @ model_local_pose @ link_pose @ geom_pose
                        return heightmap_el, model_dir, total_pose

    # 2. Search inline models
    for model_index, model_el in enumerate(world_el.findall("model")):
        model_pose = parse_pose(model_el.findtext("pose"))
        for link_el in model_el.findall("link"):
            link_pose = parse_pose(link_el.findtext("pose"))
            for tag in ["collision", "visual"]:
                for geom_owner in link_el.findall(tag):
                    geom_pose = parse_pose(geom_owner.findtext("pose"))
                    geometry_el = geom_owner.find("geometry")
                    if geometry_el is None:
                        continue
                    heightmap_el = geometry_el.find("heightmap")
                    if heightmap_el is not None:
                        total_pose = model_pose @ link_pose @ geom_pose
                        return heightmap_el, world_dir, total_pose

    return None


def generate_heightmap_for_world(
    world_path: str,
    output_path: str,
    *,
    resolution: float = 0.1,
    model_paths: Sequence[str] = (),
    package_paths: Optional[Dict[str, str]] = None,
    prefer: str = "collision",
    visual_fallback: bool = True,
    preview_path: Optional[str] = None,
    model_filter: Optional[str] = None,
    yaw: float = 0.0,
    swap_xy: bool = False,
    fill_nan: bool = False,
) -> str:
    world_path = os.path.abspath(os.path.expanduser(world_path))
    output_path = os.path.abspath(os.path.expanduser(output_path))
    if not os.path.isfile(world_path):
        raise FileNotFoundError(f"World file not found: {world_path}")
    if resolution <= 0:
        raise ValueError("resolution must be positive")

    package_paths = dict(package_paths or {})
    resolved_model_paths = gather_model_paths(world_path, model_paths)
    try:
        sources = collect_world_geometries(
            world_path,
            model_paths=resolved_model_paths,
            package_paths=package_paths,
            prefer=prefer,
            visual_fallback=visual_fallback,
            model_filter=model_filter,
        )
    except Exception:
        sources = []

    hm_src = find_world_heightmap_geometry(world_path, resolved_model_paths, package_paths)

    if not sources and hm_src is None:
        raise RuntimeError(f"No mesh geometry or heightmap terrain was found in {world_path}")

    # Define additional yaw rotation matrix
    R_yaw = np.eye(4, dtype=np.float64)
    if yaw != 0.0:
        yaw_rad = np.radians(yaw)
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        R_yaw[0, 0] = c
        R_yaw[0, 1] = -s
        R_yaw[1, 0] = s
        R_yaw[1, 1] = c

    if swap_xy:
        T_swap = np.array([
            [0.0, 1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=np.float64)
        R_yaw = R_yaw @ T_swap

    if hm_src is not None:
        heightmap_el, model_dir, total_pose = hm_src
        xs_gz, ys_gz, z_gz = rasterize_heightmap_image(
            heightmap_el, model_dir, resolved_model_paths, package_paths, resolution, total_pose
        )
        base_triangles = heightmap_grid_to_triangles(xs_gz, ys_gz, z_gz)
        base_triangles = apply_transform(R_yaw, base_triangles.reshape(-1, 3)).reshape(-1, 3, 3)
        base_source = None
    else:
        bounds_with_sources = [(source, _source_bounds(source, R_yaw)) for source in sources]
        # The largest projected mesh is the base terrain; its bounds define a stable map
        # frame so every generated world uses the same origin and dimensions.
        base_source, base_bounds_3d = max(
            bounds_with_sources,
            key=lambda item: (item[1][1] - item[1][0]) * (item[1][3] - item[1][2]),
        )
        base_triangles = _decimate_triangles(_source_world_triangles(base_source, R_yaw), maximum=90000)

    xmin = float(np.min(base_triangles[:, :, 0]))
    xmax = float(np.max(base_triangles[:, :, 0]))
    ymin = float(np.min(base_triangles[:, :, 1]))
    ymax = float(np.max(base_triangles[:, :, 1]))
    map_bounds = (xmin, xmax, ymin, ymax)

    xs, ys, grid = rasterize_triangles(base_triangles, resolution, bounds=map_bounds)

    overlay_count = 0
    for source in sources:
        if source is base_source:
            continue
        overlay_count += _overlay_source(grid, xs, ys, source, resolution, R_yaw)

    if fill_nan:
        grid = fill_nan_nearest(xs, ys, grid)

    write_npz(
        output_path,
        xs=xs,
        ys=ys,
        grid=grid,
        resolution=resolution,
        world_path=world_path,
        geometry_count=len(sources) + (1 if hm_src is not None else 0),
    )
    if preview_path:
        preview_path = os.path.abspath(os.path.expanduser(preview_path))
        os.makedirs(os.path.dirname(preview_path), exist_ok=True)
        write_preview(preview_path, grid)

    empty_cells = int(np.isnan(grid).sum())
    print("=" * 60)
    print("Generated height map for final world")
    print(f"World:       {world_path}")
    print(f"Output:      {output_path}")
    if preview_path:
        print(f"Preview:     {preview_path}")
    print(f"Resolution:  {resolution:.3f} m/cell")
    print(f"Grid:        {grid.shape[1]} x {grid.shape[0]}")
    print(f"Geometries:  {len(sources) + (1 if hm_src is not None else 0)} (terrain + world rocks)")
    print(f"Rock samples overlaid: {overlay_count}")
    print(f"Z range:     {np.nanmin(grid):.4f} .. {np.nanmax(grid):.4f} m")
    print(f"Empty cells: {empty_cells}")
    print("=" * 60)
    return output_path


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a metric height map from a complete Gazebo world, including rocks."
    )
    parser.add_argument("world", nargs="?", help="path to the generated .world file")
    parser.add_argument("-o", "--output", required=True, help="output .npz path")
    parser.add_argument("--preview", help="optional grayscale preview PNG path")
    parser.add_argument("--resolution", type=float, default=0.1, help="grid resolution in metres (default: 0.1)")
    parser.add_argument("--prefer", choices=("collision", "visual"), default="collision")
    parser.add_argument(
        "--no-visual-fallback",
        action="store_true",
        help="exclude models that have visual geometry but no collision geometry",
    )
    parser.add_argument("--model", help="optional model name filter")
    parser.add_argument("--model-path", action="append", default=[], help="extra model resource directory; repeatable")
    parser.add_argument(
        "--package-path",
        action="append",
        default=[],
        metavar="PACKAGE=PATH",
        help="explicit package directory for package:// URIs; repeatable",
    )
    parser.add_argument("--yaw", type=float, default=0.0, help="apply an additional yaw rotation in degrees to the mesh before rasterizing")
    parser.add_argument("--swap-xy", action="store_true", help="swap X and Y coordinates of the mesh before rasterizing")
    parser.add_argument("--fill-nan", action="store_true", help="fill gaps in the grid via nearest-neighbor interpolation")
    parser.add_argument("--mesh", help="skip world parsing, rasterize this mesh file directly")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    if not args.output.lower().endswith(".npz"):
        parser.error("the output file must use the .npz extension")

    is_mesh_file = False
    mesh_path = None
    if args.mesh:
        is_mesh_file = True
        mesh_path = args.mesh
    elif args.world and (args.world.lower().endswith(".stl") or args.world.lower().endswith(".obj")):
        is_mesh_file = True
        mesh_path = args.world

    if is_mesh_file:
        # Define additional yaw rotation matrix
        R_yaw = np.eye(4, dtype=np.float64)
        if args.yaw != 0.0:
            yaw_rad = np.radians(args.yaw)
            c, s = math.cos(yaw_rad), math.sin(yaw_rad)
            R_yaw[0, 0] = c
            R_yaw[0, 1] = -s
            R_yaw[1, 0] = s
            R_yaw[1, 1] = c
    
        if args.swap_xy:
            T_swap = np.array([
                [0.0, 1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ], dtype=np.float64)
            R_yaw = R_yaw @ T_swap
            
        triangles = load_mesh_triangles(mesh_path)
        triangles = apply_transform(R_yaw, triangles.reshape(-1, 3)).reshape(-1, 3, 3)
        xs, ys, grid = rasterize_triangles(triangles, args.resolution)
        if args.fill_nan:
            grid = fill_nan_nearest(xs, ys, grid)
            
        write_npz(
            args.output,
            xs=xs,
            ys=ys,
            grid=grid,
            resolution=args.resolution,
            world_path=mesh_path,
            geometry_count=1,
        )
        if args.preview:
            write_preview(args.preview, grid)
            
        empty_cells = int(np.isnan(grid).sum())
        print("=" * 60)
        print("Generated height map from direct mesh")
        print(f"Mesh:        {mesh_path}")
        print(f"Output:      {args.output}")
        if args.preview:
            print(f"Preview:     {args.preview}")
        print(f"Resolution:  {args.resolution:.3f} m/cell")
        print(f"Grid:        {grid.shape[1]} x {grid.shape[0]}")
        print(f"Z range:     {np.nanmin(grid):.4f} .. {np.nanmax(grid):.4f} m")
        print(f"Empty cells: {empty_cells}")
        print("=" * 60)
    else:
        package_paths = parse_package_paths(args.package_path)
        generate_heightmap_for_world(
            args.world,
            args.output,
            resolution=args.resolution,
            model_paths=args.model_path,
            package_paths=package_paths,
            prefer=args.prefer,
            visual_fallback=not args.no_visual_fallback,
            preview_path=args.preview,
            model_filter=args.model,
            yaw=args.yaw,
            swap_xy=args.swap_xy,
            fill_nan=args.fill_nan,
        )


if __name__ == "__main__":
    main()
