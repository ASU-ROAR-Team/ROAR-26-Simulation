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

    # Final report
    separator("BATCH COMPLETE — Summary")
    for name, status in results.items():
        print(f"  {name:10s}  {status}")
    print()
    print(f"  Output folder: {OUTPUTS_DIR}")
    print("=" * 78)


if __name__ == "__main__":
    main()
