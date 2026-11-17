#!/usr/bin/env python3
"""
================================================================================
ASU ROAR 2026 Monorepo Sequential Builder & Audit (One-by-One, Crash-Safe)
================================================================================
Builds all 29 ROS 2 Humble packages sequentially with memory throttling
to prevent compiler OOM crashes on template-heavy packages like MPPI controller.
================================================================================
"""

import os
import sys
import time
import shutil
import subprocess
from pathlib import Path

TARGET_WS = Path(__file__).resolve().parent

ORDERED_PACKAGES = [
    # 1. Custom ROS 2 IDL Messages / Interfaces
    "active_mapping_msgs",
    "roar_msgs",

    # 2. Independent C++ Algorithms & Controllers
    "dstar_navigation",
    "nav2_mppi_controller",
    "adaptive_pure_pursuit",

    # 3. Robot & World Simulation Models
    "coordinate_bridge",
    "roar_rover_clean",
    "roar_rover_noise",
    "roar_simulation_full",

    # 4. ERC Maintenance Panel Description & Simulator
    "aruco_description",
    "electromagnet_description",
    "erc_panel_sim",
    "led_indicator_description",
    "main_switch_description",
    "panel_body_description",
    "plate_description",
    "rotary_description",
    "row_description",
    "slab_description",
    "socket_description",
    "switch_description",

    # 5. Benchmarking & Referee Evaluation
    "referee_engine",
    "roar_perception_testing",
    "roar_slam_benchmark",

    # 6. Downstream Navigation, SLAM & Perception
    "navstack_nav2",
    "perception_module",
    "roar_localization",
    "roar_slam_testing",
    "terrain_mapping",
]

def log_header(title):
    print("\n" + "=" * 80)
    print(f"  {title}")
    print("=" * 80)

def log_step(name, status="INFO", details=""):
    badge = f"[{status}]"
    print(f"  {badge:<8} {name:<40} {details}")

def clean_stale_build_dirs():
    """Clean stale or interrupted build directories from previous crashes."""
    build_dir = TARGET_WS / "build"
    install_dir = TARGET_WS / "install"
    if not build_dir.exists():
        return
    for pkg_dir in build_dir.iterdir():
        if pkg_dir.is_dir():
            pkg_install = install_dir / pkg_dir.name
            if not pkg_install.exists() or pkg_dir.name == "nav2_mppi_controller":
                print(f"  [CLEAN] Removing interrupted build cache for {pkg_dir.name}...")
                shutil.rmtree(pkg_dir, ignore_errors=True)

def build_package(pkg_name, index, total):
    """Build a single package sequentially with memory throttling."""
    print(f"\n  [{index:02d}/{total:02d}] Building package: {pkg_name:<30}", end="", flush=True)
    t0 = time.time()

    cmake_args = "-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"[ -f {TARGET_WS}/install/setup.bash ] && source {TARGET_WS}/install/setup.bash; "
        f"export MAKEFLAGS='-j2'; "
        f"colcon build --symlink-install --parallel-workers 1 --packages-select {pkg_name} --cmake-args {cmake_args}"
    )

    proc = subprocess.run(
        ["bash", "-c", cmd],
        cwd=str(TARGET_WS),
        capture_output=True,
        text=True,
    )
    dt = time.time() - t0

    if proc.returncode == 0:
        print(f" -> ✅ DONE in {dt:.1f}s")
        return True, ""
    else:
        print(f" -> ❌ FAILED in {dt:.1f}s")
        err_lines = [l for l in (proc.stdout + "\n" + proc.stderr).splitlines() if l.strip()]
        tail = "\n".join(err_lines[-35:])
        return False, tail

def run_verification():
    log_header("AUTOMATED POST-BUILD VERIFICATION SUITE")
    failures = []

    # 1. Verify all 29 packages are installed in the ROS 2 index
    pkg_list_cmd = f"source /opt/ros/humble/setup.bash && source {TARGET_WS}/install/setup.bash && ros2 pkg list"
    pkg_res = subprocess.run(["bash", "-c", pkg_list_cmd], capture_output=True, text=True)
    active_built_pkgs = set(pkg_res.stdout.splitlines())

    missing = [p for p in ORDERED_PACKAGES if p not in active_built_pkgs]
    if missing:
        failures.append(f"Missing from ros2 pkg list: {missing}")
        log_step("ROS 2 Package Registration", "FAIL", f"{len(missing)} missing: {missing}")
    else:
        log_step("ROS 2 Package Registration", "PASS", f"All 29 packages indexed in ros2 pkg list")

    # 2. Check for files >= 50 MB
    large_files = []
    total_mb = 0
    for root, dirs, files in os.walk(TARGET_WS):
        if any(ignored in root for ignored in ["/build", "/log", "/.git"]):
            continue
        for f in files:
            fp = Path(root) / f
            try:
                sz = fp.stat().st_size
                total_mb += sz / (1024 * 1024)
                if sz >= 50 * 1024 * 1024:
                    large_files.append((str(fp.relative_to(TARGET_WS)), sz / (1024 * 1024)))
            except Exception:
                pass

    if large_files:
        failures.append(f"Files >= 50MB found: {large_files}")
        log_step("GitHub File Size Audit", "FAIL", f"{len(large_files)} file(s) >= 50 MB")
    else:
        log_step("GitHub File Size Audit", "PASS", f"0 files >= 50 MB! Source tree size: {total_mb:.1f} MB")

    # 3. Hardcoded Path Audit in src/
    forbidden_tokens = [
        "/home/draaven/ROAR-26",
        "/home/draaven/Navigation_Mission_Workspace",
        "/home/draaven/navStack_2026_Workspace",
        "/home/saif",
        "/home/amrtamer",
        "/home/misara",
        "/home/rana",
        "wsl.localhost",
    ]
    code_extensions = {".py", ".sh", ".yaml", ".yml", ".xml", ".launch.py", ".urdf", ".xacro", ".sdf"}
    offenses = []
    for root, dirs, files in os.walk(TARGET_WS / "src"):
        for f in files:
            p = Path(root) / f
            if any(p.name.endswith(ext) for ext in code_extensions) or p.name in {"CMakeLists.txt", "package.xml"}:
                try:
                    txt = p.read_text(errors="ignore")
                    for tok in forbidden_tokens:
                        if tok in txt:
                            for idx, line in enumerate(txt.splitlines(), start=1):
                                if tok in line:
                                    offenses.append((str(p.relative_to(TARGET_WS)), idx, tok, line.strip()[:60]))
                except Exception:
                    pass

    if offenses:
        failures.append(f"Hardcoded paths found: {len(offenses)}")
        log_step("Hardcoded Path Audit", "FAIL", f"{len(offenses)} occurrence(s):")
        for fpath, lnum, tok, line in offenses:
            print(f"       -> {fpath}:{lnum} [{tok}]: {line}")
    else:
        log_step("Hardcoded Path Audit", "PASS", "Zero forbidden hardcoded machine paths found in src/")

    # 4. Xacro Expansion Smoke Test
    xacro_tests = ["roar_rover_clean", "roar_rover_noise", "roar_simulation_full"]
    xacro_fails = []
    for xt in xacro_tests:
        xcmd = f"source /opt/ros/humble/setup.bash && source {TARGET_WS}/install/setup.bash && xacro $(ros2 pkg prefix {xt})/share/{xt}/urdf/base/rover_simulation.urdf.xacro"
        xres = subprocess.run(["bash", "-c", xcmd], capture_output=True, text=True)
        if xres.returncode != 0:
            xacro_fails.append((xt, xres.stderr.strip()[:100]))

    if xacro_fails:
        failures.append(f"Xacro failures: {xacro_fails}")
        log_step("Rover Xacro Expansion", "FAIL", f"{len(xacro_fails)} failed")
    else:
        log_step("Rover Xacro Expansion", "PASS", "clean, noise, and full rover xacro expansion verified")

    # 5. preview_mission.py Smoke Test
    test_img = Path("/tmp/monorepo_preview_test.png")
    if test_img.exists():
        test_img.unlink()
    preview_script = TARGET_WS / "src" / "navigation" / "scripts" / "preview_mission.py"
    if preview_script.exists():
        prev_cmd = f"source /opt/ros/humble/setup.bash && source {TARGET_WS}/install/setup.bash && python3 {preview_script} --world 1 --wp 0 --save {test_img}"
        pres = subprocess.run(["bash", "-c", prev_cmd], capture_output=True, text=True)

        if pres.returncode == 0 and test_img.exists() and test_img.stat().st_size > 1000:
            log_step("preview_mission.py Smoke Test", "PASS", f"Generated preview image ({test_img.stat().st_size} bytes)")
        else:
            failures.append("preview_mission.py failed!")
            log_step("preview_mission.py Smoke Test", "FAIL", pres.stderr.strip()[:100])
    else:
        failures.append(f"preview_mission.py not found at {preview_script}")
        log_step("preview_mission.py Smoke Test", "FAIL", "Script missing")

    log_header("VERIFICATION RESULT SUMMARY")
    if not failures:
        print("  🎉 ALL 29 PACKAGES BUILT AND VERIFIED WITH 100% SUCCESS!")
        print(f"  Unified Monorepo Ready at: {TARGET_WS}")
        print("=" * 80)
        return True
    else:
        print(f"  ❌ {len(failures)} VERIFICATION CHECK(S) FAILED:")
        for f in failures:
            print(f"     - {f}")
        print("=" * 80)
        return False

def main():
    print("""
================================================================================
  ASU ROAR 2026 MONOREPO - SEQUENTIAL (ONE-BY-ONE) BUILDER
================================================================================
  Target: ~/ROAR_2026
  Strategy:
    - Build strictly 1 package at a time (zero parallel crash risk)
    - Memory throttled with MAKEFLAGS="-j2" & -DBUILD_TESTING=OFF
    - Live time and progress reporting per package
================================================================================
""")
    if not TARGET_WS.exists():
        print(f"Error: {TARGET_WS} does not exist!")
        sys.exit(1)

    clean_stale_build_dirs()

    total = len(ORDERED_PACKAGES)
    log_header(f"BUILDING {total} PACKAGES ONE BY ONE")

    total_start = time.time()
    for idx, pkg in enumerate(ORDERED_PACKAGES, start=1):
        success, err = build_package(pkg, idx, total)
        if not success:
            print("\n" + "=" * 80)
            print(f"  ❌ BUILD HALTED ON PACKAGE: {pkg}")
            print("=" * 80)
            print(err)
            print("=" * 80)
            sys.exit(1)

    total_time = time.time() - total_start
    print(f"\n  -> All {total} packages compiled successfully in {total_time:.1f}s!")

    verified = run_verification()
    sys.exit(0 if verified else 1)

if __name__ == "__main__":
    main()
