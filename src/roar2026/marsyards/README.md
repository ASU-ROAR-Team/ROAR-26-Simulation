# 🔴 ROAR 2026 — Marsyard Terrain & ArUco Models

[![Module](https://img.shields.io/badge/Module-roar2026-E10600?logo=gazebo&logoColor=white)](file://${ROAR_WS}/src/roar2026)
[![Terrain](https://img.shields.io/badge/Terrain-ERC_2026_Official-00D9FF)](file://${ROAR_WS}/src/roar2026/marsyards/marsyard/models/erc_marsyard_2026)

This directory hosts the official 3D digital twin assets for the **European Rover Challenge 2026 Marsyard** in Kielce, Poland, alongside the official competition ArUco navigation markers.

---

## 📁 Directory Structure

```text
src/roar2026/marsyards/
└── marsyard/
    └── models/
        ├── erc_marsyard_2026/        # Official 2026 ERC Marsyard Digital Twin
        │   ├── model.config          # Gazebo model metadata
        │   ├── model.sdf             # Model definition (visual & collision links)
        │   ├── materials/textures/
        │   │   └── marsyard_orthophoto.jpg # High-resolution aerial orthophoto
        │   └── meshes/
        │       ├── marsyard_visual.obj    # High-poly visual photogrammetry mesh
        │       └── marsyard_collision.obj # Optimized collision geometry (decimated)
        │
        ├── aruco_1/ .. aruco_15/     # Competition navigation ArUco markers 1..15
        │   ├── model.config          # Model metadata
        │   ├── model.sdf             # Heavy concrete pedestal + marker tag
        │   └── materials/textures/
        │       └── aruco_tag_*.png   # 4x4 ArUco dictionary marker patterns
```

---

## 🗺️ Terrain Specifications & Bounds

| Parameter | Value | Details |
| :--- | :--- | :--- |
| **Model URI** | `model://erc_marsyard_2026` | Resolves via `IGN_GAZEBO_RESOURCE_PATH` |
| **X Range** | $[-18.431\,\text{m}, +18.569\,\text{m}]$ | Total span: $37.0\,\text{m}$ |
| **Y Range** | $[-7.687\,\text{m}, +35.913\,\text{m}]$ | Total span: $43.6\,\text{m}$ |
| **Z Range** | $[-1.45\,\text{m}, +2.85\,\text{m}]$ | Peak crater rim to crater floor |
| **Visual Mesh** | `marsyard_visual.obj` | Textured with high-resolution orthophoto |
| **Collision Mesh**| `marsyard_collision.obj` | Clean manifold mesh for physics contacts |

---

## 🏷️ ArUco Tag Pedestals (`aruco_1` .. `aruco_15`)

Each marker represents an official ERC navigational landmark:
- **Marker Standard**: ArUco Dictionary 4x4 (50mm/100mm scale).
- **Physical Pedestal**: $0.35\,\text{m} \times 0.35\,\text{m} \times 0.05\,\text{m}$ concrete block ensuring rigid placement on steep crater slopes without tipping.
- **Elevation Adherence**: All tag models are raycast onto the exact terrain elevation at competition coordinates.

