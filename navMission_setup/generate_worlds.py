#!/usr/bin/env python3
"""
generate_worlds.py
------------------
Batch-generates 3 ERC Mars Yard 2026 world datasets using the navMission_setup
pipeline (add_world.py). Each world has a different obstacle density tier.

  world_1  →  LOW density      (~6  rocks / 704 m²)
  world_2  →  MODERATE density (~18 rocks / 704 m²)
  world_3  →  HIGH density     (~35 rocks / 704 m²)

Parameters follow the navMission_setup README conventions:
  --density         : rocks per square metre
  --collidable-ratio: fraction of rocks that are solid/collidable (0.0 – 1.0)

Usage:
    python3 generate_worlds.py

All outputs land in:
    navMission_setup/outputs/world_1/
    navMission_setup/outputs/world_2/
    navMission_setup/outputs/world_3/
"""

import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# ---------------------------------------------------------------------------
# World configurations
# ---------------------------------------------------------------------------
WORLDS = [
    {
        "name":             "world_1",
        "label":            "LOW density",
        "density":          0.008,   # ~6 rocks  across ~704 m²
        "collidable_ratio": 0.50,    # 50 % solid
        "heightmap_resolution": 0.25,
        "gradient_scale":   150.0,
        "stability_scale":  90.0,
    },
    {
        "name":             "world_2",
        "label":            "MODERATE density",
        "density":          0.08,    # ~56 rocks  across ~704 m²
        "collidable_ratio": 0.60,    # 60 % solid
        "heightmap_resolution": 0.25,
        "gradient_scale":   150.0,
        "stability_scale":  90.0,
    },
    {
        "name":             "world_3",
        "label":            "HIGH density",
        "density":          0.15,    # ~106 rocks across ~704 m²
        "collidable_ratio": 0.70,    # 70 % solid
        "heightmap_resolution": 0.25,
        "gradient_scale":   150.0,
        "stability_scale":  90.0,
    },
]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
SCRIPT_DIR  = Path(__file__).resolve().parent
ADD_WORLD   = SCRIPT_DIR / "add_world.py"
OUTPUTS_DIR = SCRIPT_DIR / "outputs"
BASE_WORLD = SCRIPT_DIR / "world_setup" / "initial_inputs" / "i_world" / "marsyard.world"
BASE_HEIGHTMAP = SCRIPT_DIR / "world_setup" / "initial_inputs" / "i_heightmap" / "heightmap.npz"
BASE_HEIGHTMAP_PREVIEW = BASE_HEIGHTMAP.with_suffix(".png")
HEIGHTMAP_GENERATOR = SCRIPT_DIR / "world_setup" / "heightMap_gen" / "heightmap_generator.py"
MARSYARD_MODELS = SCRIPT_DIR.parent / "marsyards" / "marsyard" / "models"
NEW_MARSYARD_URI = "model://erc_marsyard_2026"


def refresh_base_heightmap() -> None:
    """Bake placement heights from the exact 2026 collision model Gazebo loads."""
    root = ET.parse(BASE_WORLD).getroot()
    world = root.find("world") if root.tag != "world" else root
    uris = [(node.findtext("uri") or "").strip() for node in world.findall("include")]
    if uris.count(NEW_MARSYARD_URI) != 1:
        raise RuntimeError(
            f"{BASE_WORLD} must include exactly one {NEW_MARSYARD_URI}; found {uris}"
        )

    command = [
        sys.executable,
        str(HEIGHTMAP_GENERATOR),
        str(BASE_WORLD),
        "--output", str(BASE_HEIGHTMAP),
        "--preview", str(BASE_HEIGHTMAP_PREVIEW),
        "--resolution", "0.1",
        "--model-path", str(MARSYARD_MODELS),
        "--model", "erc_marsyard_2026",
        "--fill-nan",
    ]
    print("  Refreshing placement heightmap from the new Mars Yard collision mesh")
    subprocess.run(command, cwd=SCRIPT_DIR, check=True)


def separator(title: str) -> None:
    width = 78
    print()
    print("=" * width)
    print(f"  {title}")
    print("=" * width)


def run_world(cfg: dict) -> bool:
    """Run add_world.py for a single world config. Returns True on success."""
    out_dir = OUTPUTS_DIR / cfg["name"]
    if out_dir.exists():
        print(f"  ⚠️  '{cfg['name']}' already exists — skipping.")
        print(f"      Delete {out_dir} to regenerate.")
        return False

    cmd = [
        sys.executable,
        str(ADD_WORLD),
        "--name",                   cfg["name"],
        "--density",                str(cfg["density"]),
        "--collidable-ratio",       str(cfg["collidable_ratio"]),
        "--heightmap-resolution",   str(cfg["heightmap_resolution"]),
        "--gradient-scale",         str(cfg["gradient_scale"]),
        "--stability-scale",        str(cfg["stability_scale"]),
    ]

    print(f"  Command: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=SCRIPT_DIR)
    return result.returncode == 0


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    separator("ERC Mars Yard 2026 — Batch World Generator")
    print(f"  Pipeline root : {SCRIPT_DIR}")
    print(f"  Outputs dir   : {OUTPUTS_DIR}")
    print(f"  Terrain model : {NEW_MARSYARD_URI}")
    refresh_base_heightmap()
    print()
    print(f"  Generating {len(WORLDS)} worlds:")
    for w in WORLDS:
        rocks_est = int(w["density"] * 704)
        print(f"    {w['name']:10s}  {w['label']:20s}  density={w['density']:.3f}  "
              f"~{rocks_est} rocks  collidable={int(w['collidable_ratio']*100)}%")

    results = {}
    for cfg in WORLDS:
        separator(f"{cfg['name'].upper()} — {cfg['label']}")
        success = run_world(cfg)
        results[cfg["name"]] = "✅ OK" if success else "⚠️  Skipped / Failed"

    failed = [name for name, status in results.items() if status != "✅ OK"]

    # Final report
    separator("BATCH COMPLETE — Summary")
    for name, status in results.items():
        print(f"  {name:10s}  {status}")
    print()
    print(f"  Output folder: {OUTPUTS_DIR}")
    print("=" * 78)
    if failed:
        raise SystemExit(f"World generation failed for: {', '.join(failed)}")


if __name__ == "__main__":
    main()
