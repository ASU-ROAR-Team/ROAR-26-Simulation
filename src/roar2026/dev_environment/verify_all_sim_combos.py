#!/usr/bin/env python3
"""
================================================================================
ROAR Dev Environment — Full 3x3 Simulation, Base & Arm Controller Test Suite
================================================================================
Tests all 9 combinations of:
  - Rover configs : clean, noise, full
  - Worlds        : world1, world2, world3

For each combination, launches `launch_test.sh <config> <world> --headless`
and verifies:
  1. World & model resources load without missing URI errors.
  2. ros2_control controllers activate (joint_state_broadcaster,
     diff_drive_controller, and arm_controller for 'full').
  3. Base drive responds to /cmd_vel (verifies /diff_drive_controller/odom
     and wheel joint motion).
  4. For 'full': 6-DOF arm & gripper respond to /arm_controller/joint_trajectory
     (verifies joint_states motion on arm_joint_0 and grippers).
================================================================================
"""

import argparse
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

DEV_ENV_DIR = Path(__file__).resolve().parent
LAUNCH_SCRIPT = DEV_ENV_DIR / "launch_test.sh"

DEFAULT_CONFIGS = ["clean", "noise", "full"]
DEFAULT_WORLDS = ["world1", "world2", "world3"]


def run_ros_cmd(cmd: str, timeout: float = 10.0) -> subprocess.CompletedProcess:
    ws_root = DEV_ENV_DIR.parents[2]
    setup_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"[ -f '{ws_root}/install/setup.bash' ] && source '{ws_root}/install/setup.bash'; "
        f"{cmd}"
    )
    return subprocess.run(
        ["bash", "-c", setup_cmd],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def kill_process_group(proc: subprocess.Popen) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
    subprocess.run(
        ["bash", "-c",
         "pkill -9 -f 'ign gazebo|ign_gazebo|gz sim|ruby.*ignition|robot_state_publisher|ros_gz_bridge|ros_gz_sim|cmd_vel_relay|zed_degradation|encoder_sim|clean_camera|collision_alarm' 2>/dev/null || true"],
        capture_output=True,
    )
    time.sleep(1.5)


def _strip_ansi(text: str) -> str:
    return re.sub(r"\x1b\[[0-9;]*m", "", text)


def wait_for_controllers(expected: list[str], timeout: float = 90.0) -> tuple[bool, str]:
    deadline = time.time() + timeout
    last_out = ""
    while time.time() < deadline:
        try:
            res = run_ros_cmd("ros2 control list_controllers", timeout=8.0)
            last_out = _strip_ansi(res.stdout).strip()
            active = set()
            for line in last_out.splitlines():
                parts = line.split()
                if len(parts) >= 2 and parts[-1] == "active":
                    active.add(parts[0])
            if all(ctrl in active for ctrl in expected):
                return True, last_out
        except subprocess.TimeoutExpired:
            pass
        time.sleep(2.5)
    return False, last_out


def get_odom_xy() -> tuple[float, float] | None:
    try:
        res = run_ros_cmd(
            "ros2 topic echo --once /diff_drive_controller/odom --field pose.pose.position",
            timeout=8.0,
        )
        x_m = re.search(r"x:\s*([-+0-9.eE]+)", res.stdout)
        y_m = re.search(r"y:\s*([-+0-9.eE]+)", res.stdout)
        if x_m and y_m:
            return float(x_m.group(1)), float(y_m.group(1))
    except Exception:
        pass
    return None


def get_joint_positions() -> dict[str, float]:
    try:
        res = run_ros_cmd("ros2 topic echo --once /joint_states", timeout=8.0)
        names = []
        positions = []
        mode = None
        for line in res.stdout.splitlines():
            s = line.strip()
            if s.startswith("name:"):
                mode = "name"
                continue
            elif s.startswith("position:"):
                mode = "pos"
                continue
            elif s.startswith(("velocity:", "effort:", "---")):
                mode = None
                continue
            if s.startswith("- "):
                val = s[2:].strip().strip("'\"")
                if mode == "name":
                    names.append(val)
                elif mode == "pos":
                    try:
                        positions.append(float(val))
                    except ValueError:
                        pass
        return dict(zip(names, positions))
    except Exception:
        return {}


def test_base_drive() -> tuple[bool, str]:
    p0 = get_odom_xy()
    j0 = get_joint_positions()
    if p0 is None:
        return False, "No /diff_drive_controller/odom message received"

    # Publish /cmd_vel for ~3 seconds
    pub_cmd = (
        "ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "
        "\"{linear: {x: 0.6, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" "
        "--times 30"
    )
    run_ros_cmd(pub_cmd, timeout=12.0)
    # Stop
    run_ros_cmd("ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \"{}\"", timeout=6.0)

    p1 = get_odom_xy()
    j1 = get_joint_positions()
    if p1 is None:
        return False, "Lost /diff_drive_controller/odom after cmd_vel"

    dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    wheel_delta = abs(j1.get("wheel_lhs_front_joint", 0.0) - j0.get("wheel_lhs_front_joint", 0.0))
    if dist < 0.02 and wheel_delta < 0.1:
        return False, f"Rover did not move (odom_delta={dist:.4f}m, wheel_delta={wheel_delta:.4f}rad)"
    return True, f"odom_delta={dist:.3f}m, wheel_delta={wheel_delta:.2f}rad"


def test_arm_control() -> tuple[bool, str]:
    j0 = get_joint_positions()
    if "arm_joint_0" not in j0:
        return False, f"arm_joint_0 missing from /joint_states (keys={list(j0.keys())})"

    target_j0 = 0.45 if abs(j0["arm_joint_0"]) < 0.2 else 0.0
    traj_cmd = (
        "ros2 topic pub -1 /arm_controller/joint_trajectory "
        "trajectory_msgs/msg/JointTrajectory \"{"
        "joint_names: ['arm_joint_0', 'arm_joint_1', 'arm_joint_2', "
        "'arm_joint_3', 'arm_joint_4', 'arm_joint_5', "
        "'arm_left_gripper', 'arm_right_gripper'], "
        "points: [{positions: ["
        f"{target_j0}, -0.6, 0.9, 0.0, 0.4, 0.0, 0.03, 0.03], "
        "time_from_start: {sec: 1, nanosec: 500000000}}]}\""
    )
    run_ros_cmd(traj_cmd, timeout=10.0)

    deadline = time.time() + 12.0
    dj0 = 0.0
    dgrip = 0.0
    while time.time() < deadline:
        time.sleep(1.5)
        j1 = get_joint_positions()
        dj0 = abs(j1.get("arm_joint_0", 0.0) - j0["arm_joint_0"])
        dgrip = abs(j1.get("arm_left_gripper", 0.0) - j0.get("arm_left_gripper", 0.0))
        if dj0 >= 0.1:
            return True, f"arm_joint_0_delta={dj0:.3f}rad, gripper_delta={dgrip:.3f}m"

    return False, f"arm_joint_0 did not reach target (delta={dj0:.4f}rad)"


def test_combination(config: str, world: str, timeout: float) -> tuple[bool, list[str]]:
    print(f"\n{'=' * 72}")
    print(f"  TESTING: config={config} | world={world}")
    print(f"{'=' * 72}")

    log_path = Path(f"/tmp/roar_sim_test_{config}_{world}.log")
    log_file = open(log_path, "w")
    proc = subprocess.Popen(
        ["bash", str(LAUNCH_SCRIPT), config, world, "--headless"],
        cwd=str(DEV_ENV_DIR),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    notes = []
    passed = True
    try:
        expected_ctrls = ["joint_state_broadcaster", "diff_drive_controller"]
        if config == "full":
            expected_ctrls.append("arm_controller")

        ok_ctrl, ctrl_out = wait_for_controllers(expected_ctrls, timeout=timeout)
        if not ok_ctrl:
            passed = False
            notes.append(f"Controllers not active ({expected_ctrls}). Got:\n{ctrl_out}")
            return False, notes
        notes.append(f"Controllers active: {', '.join(expected_ctrls)}")

        ok_base, base_msg = test_base_drive()
        if not ok_base:
            passed = False
            notes.append(f"Base drive FAILED: {base_msg}")
        else:
            notes.append(f"Base drive PASS: {base_msg}")

        if config == "full":
            ok_arm, arm_msg = test_arm_control()
            if not ok_arm:
                passed = False
                notes.append(f"Arm control FAILED: {arm_msg}")
            else:
                notes.append(f"Arm control PASS: {arm_msg}")

        log_file.flush()
        log_txt = log_path.read_text(errors="ignore")
        uri_errs = [l for l in log_txt.splitlines() if "Unable to find uri" in l]
        if uri_errs:
            passed = False
            notes.append(f"Missing model URIs in Gazebo log: {uri_errs[:3]}")
        else:
            notes.append("Zero missing Gazebo model URIs")

        return passed, notes
    finally:
        kill_process_group(proc)
        log_file.close()


def main():
    parser = argparse.ArgumentParser(description="Verify all ROAR rover configs across all worlds.")
    parser.add_argument("--configs", nargs="+", default=DEFAULT_CONFIGS)
    parser.add_argument("--worlds", nargs="+", default=DEFAULT_WORLDS)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()

    results = []
    for cfg in args.configs:
        for w in args.worlds:
            ok, notes = test_combination(cfg, w, args.timeout)
            results.append((cfg, w, ok, notes))
            for n in notes:
                print(f"  -> {n}")
            print(f"  RESULT: {'PASS' if ok else 'FAIL'}")

    print(f"\n{'=' * 72}")
    print("  SUMMARY MATRIX")
    print(f"{'=' * 72}")
    passed_cnt = sum(1 for _, _, ok, _ in results if ok)
    for cfg, w, ok, notes in results:
        badge = "[PASS]" if ok else "[FAIL]"
        print(f"  {badge} {cfg:<6} x {w:<8} | {notes[-1] if notes else ''}")
    print(f"{'=' * 72}")
    print(f"  Total: {passed_cnt}/{len(results)} combinations passed.")
    sys.exit(0 if passed_cnt == len(results) else 1)


if __name__ == "__main__":
    main()
