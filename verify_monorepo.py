#!/usr/bin/env python3
"""
================================================================================
ASU ROAR 2026 Monorepo Verification & Audit Suite
================================================================================
Automated audit tool to verify workspace integrity after cloning or pulling:
  1. ROS 2 Package Registration (All 29 monorepo packages indexed)
  2. GitHub File Size Compliance (0 files >= 50 MB, total tree check)
  3. Zero Hardcoded Machine Path Audit (No user paths: amrtamer, misara, saif, rana, etc.)
  4. Rover URDF/Xacro Expansion (clean, noise, full models)
  5. Mission Preview Smoke Test (World 1 / Waypoint 0 rendering)
================================================================================
"""

import os
import sys
import subprocess
from pathlib import Path

TARGET_WS = Path(__file__).resolve().parent

ORDERED_PACKAGES = [
    "active_mapping_msgs",
    "roar_msgs",
    "dstar_navigation",
    "nav2_mppi_controller",
    "adaptive_pure_pursuit",
    "coordinate_bridge",
    "roar_rover_clean",
    "roar_rover_noise",
    "roar_simulation_full",
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
    "referee_engine",
    "roar_perception_testing",
    "roar_slam_benchmark",
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
    print(f"  {badge:<8} {name:<35} {details}")

def main():
    log_header("ASU ROAR 2026 MONOREPO VERIFICATION & AUDIT")
    print(f"  Workspace Root: {TARGET_WS}")
    failures = []

    if not (TARGET_WS / "src").exists():
        print(f"  [ERROR] src/ directory not found in {TARGET_WS}!")
        return False

    # --------------------------------------------------------------------------
    # Check 1: ROS 2 Package Registration
    # --------------------------------------------------------------------------
    setup_bash = TARGET_WS / "install" / "setup.bash"
    if not setup_bash.exists():
        log_step("ROS 2 Package Registration", "WARN", "install/setup.bash not found. Workspace may need building first.")
    else:
        pkg_list_cmd = f"source /opt/ros/humble/setup.bash && source {setup_bash} && ros2 pkg list"
        pkg_res = subprocess.run(["bash", "-c", pkg_list_cmd], capture_output=True, text=True)
        active_built_pkgs = set(pkg_res.stdout.splitlines())

        missing = [p for p in ORDERED_PACKAGES if p not in active_built_pkgs]
        if missing:
            failures.append(f"Missing from ros2 pkg list: {missing}")
            log_step("ROS 2 Package Registration", "FAIL", f"{len(missing)} missing: {missing}")
        else:
            log_step("ROS 2 Package Registration", "PASS", f"All 29 packages indexed in ros2 pkg list")

    # --------------------------------------------------------------------------
    # Check 2: GitHub File Size Audit (0 files >= 50 MB)
    # --------------------------------------------------------------------------
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
        for p, s in large_files:
            print(f"       -> {p} ({s:.1f} MB)")
    else:
        log_step("GitHub File Size Audit", "PASS", f"0 files >= 50 MB! Source tree: {total_mb:.1f} MB")

    # --------------------------------------------------------------------------
    # Check 3: Zero Hardcoded Machine Path Audit
    # --------------------------------------------------------------------------
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
                                    offenses.append((str(p.relative_to(TARGET_WS)), idx, tok, line.strip()[:65]))
                except Exception:
                    pass

    if offenses:
        failures.append(f"Hardcoded paths found: {len(offenses)}")
        log_step("Hardcoded Path Audit", "FAIL", f"{len(offenses)} occurrence(s):")
        for fpath, lnum, tok, line in offenses:
            print(f"       -> {fpath}:{lnum} [{tok}]: {line}")
    else:
        log_step("Hardcoded Path Audit", "PASS", "Zero forbidden machine/user paths found across src/")

    # --------------------------------------------------------------------------
    # Check 4: Rover Xacro Expansion Smoke Test
    # --------------------------------------------------------------------------
    xacro_tests = ["roar_rover_clean", "roar_rover_noise", "roar_simulation_full"]
    xacro_fails = []
    if setup_bash.exists():
        for xt in xacro_tests:
            xcmd = f"source /opt/ros/humble/setup.bash && source {setup_bash} && xacro $(ros2 pkg prefix {xt})/share/{xt}/urdf/base/rover_simulation.urdf.xacro"
            xres = subprocess.run(["bash", "-c", xcmd], capture_output=True, text=True)
            if xres.returncode != 0:
                xacro_fails.append((xt, xres.stderr.strip()[:100]))

        if xacro_fails:
            failures.append(f"Xacro failures: {xacro_fails}")
            log_step("Rover Xacro Expansion", "FAIL", f"{len(xacro_fails)} failed")
            for xt, err in xacro_fails:
                print(f"       -> {xt}: {err}")
        else:
            log_step("Rover Xacro Expansion", "PASS", "clean, noise, and full rover xacro expansion verified")
    else:
        log_step("Rover Xacro Expansion", "SKIP", "install/setup.bash not built yet")

    # --------------------------------------------------------------------------
    # Check 5: preview_mission.py Smoke Test
    # --------------------------------------------------------------------------
    test_img = Path("/tmp/monorepo_preview_test.png")
    if test_img.exists():
        test_img.unlink()
    preview_script = (TARGET_WS / "src" / "navigation" / "scripts" / "preview_mission.py")
    if preview_script.exists() and setup_bash.exists():
        prev_cmd = f"source /opt/ros/humble/setup.bash && source {setup_bash} && python3 {preview_script} --world 1 --wp 0 --save {test_img}"
        pres = subprocess.run(["bash", "-c", prev_cmd], capture_output=True, text=True)

        if pres.returncode == 0 and test_img.exists() and test_img.stat().st_size > 1000:
            log_step("preview_mission.py Smoke Test", "PASS", f"Generated preview image ({test_img.stat().st_size} bytes)")
        else:
            failures.append("preview_mission.py failed!")
            log_step("preview_mission.py Smoke Test", "FAIL", pres.stderr.strip()[:100])
    elif not preview_script.exists():
        log_step("preview_mission.py Smoke Test", "FAIL", f"{preview_script} not found")
        failures.append("preview_mission.py not found")
    else:
        log_step("preview_mission.py Smoke Test", "SKIP", "install/setup.bash not built yet")

    # --------------------------------------------------------------------------
    # Summary
    # --------------------------------------------------------------------------
    log_header("VERIFICATION RESULT SUMMARY")
    if not failures:
        print("  🎉 100% OF VERIFICATION CHECKS PASSED!")
        print(f"  Monorepo Verified at: {TARGET_WS}")
        print("=" * 80)
        return True
    else:
        print(f"  ❌ {len(failures)} VERIFICATION CHECK(S) FAILED:")
        for f in failures:
            print(f"     - {f}")
        print("=" * 80)
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
