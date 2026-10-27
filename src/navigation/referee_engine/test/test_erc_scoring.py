#!/usr/bin/env python3
"""
Unit tests for the ERC Navigation Task Scoring System.
Verifies all scoring rules match the official ERC table.
"""
import sys
import os
import numpy as np
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from referee_engine.score_calculator import ScoreCalculator, ScoreCalculatorConfig
from referee_engine.waypoint_tracker import Waypoint, WaypointTracker, load_waypoints_from_yaml

def test_nav010_waypoint_reached():
    """O/NAV-010: MAX(0, round(30 - 6*d)) where d is edge distance"""
    calc = ScoreCalculator(ScoreCalculatorConfig(rover_radius=0.5))

    # Perfect hit: center distance = 0 -> edge = 0 -> score = 30
    assert calc.score_waypoint_reached(0.0) == 30.0, "Perfect hit should score 30"

    # d_edge = 1.0m -> round(30 - 6) = 24
    assert calc.score_waypoint_reached(1.0) == 24.0, "1m edge distance should score 24"

    # d_edge = 2.0m -> round(30 - 12) = 18
    assert calc.score_waypoint_reached(2.0) == 18.0, "2m edge distance should score 18"

    # d_edge = 5.0m -> round(30 - 30) = 0
    assert calc.score_waypoint_reached(5.0) == 0.0, "5m edge distance should score 0"

    # d_edge = 6.0m -> round(30 - 36) = -6 -> clamped to 0
    assert calc.score_waypoint_reached(6.0) == 0.0, "6m edge distance should score 0 (clamped)"

    # d_edge = 0.5m -> round(30 - 3) = 27
    assert calc.score_waypoint_reached(0.5) == 27.0, "0.5m edge distance should score 27"

    print("  [PASS] O/NAV-010: Waypoint reached scoring")


def test_edge_distance_calculation():
    """Edge distance = max(0, d_center - rover_radius)"""
    calc = ScoreCalculator(ScoreCalculatorConfig(rover_radius=0.5))

    assert calc.calculate_edge_distance(0.3) == 0.0, "Inside rover body -> 0"
    assert calc.calculate_edge_distance(0.5) == 0.0, "On rover edge -> 0"
    assert abs(calc.calculate_edge_distance(1.5) - 1.0) < 1e-9, "1.5m center -> 1.0m edge"
    assert abs(calc.calculate_edge_distance(3.0) - 2.5) < 1e-9, "3.0m center -> 2.5m edge"

    print("  [PASS] Edge distance calculation")


def test_fatal_crash_zero_score():
    """Crash/collision/out-of-bounds -> score = 0 immediately"""
    calc = ScoreCalculator()
    wp_details = [{"id": "W1", "d_center": 0.1, "timestamp": 10.0}]

    result = calc.final_score(
        waypoint_details=wp_details,
        is_fatal=True,
        elapsed_time=100.0
    )
    assert result["final_score"] == 0.0, "Fatal crash must produce 0 score"
    assert result["is_fatal"] == True
    print("  [PASS] Fatal crash -> zero score")


def test_timeout_preserves_score():
    """Timeout should NOT zero the score; it shows accumulated score"""
    calc = ScoreCalculator()
    wp_details = [
        {"id": "W1", "d_center": 0.1, "timestamp": 50.0, "is_autonomous": True},
        {"id": "W2", "d_center": 0.2, "timestamp": 100.0, "is_autonomous": True},
    ]

    result = calc.final_score(
        waypoint_details=wp_details,
        is_fatal=False,
        elapsed_time=1200.0  # at timeout
    )
    assert result["final_score"] > 0.0, f"Timeout should preserve score, got {result['final_score']}"
    assert result["is_fatal"] == False
    print("  [PASS] Timeout preserves accumulated score")


def test_nav020_autonomy_bonus():
    """O/NAV-020: 25 pts per waypoint in autonomous mode, max 100"""
    calc = ScoreCalculator()

    # 4 waypoints all autonomous = 100 pts
    wp_details = [
        {"id": f"W{i+1}", "d_center": 0.0, "timestamp": i*10.0, "is_autonomous": True}
        for i in range(4)
    ]
    result = calc.final_score(waypoint_details=wp_details, is_fatal=False)
    assert result["nav_020_autonomy_bonus"] == 100.0, f"4 auto waypoints should give 100, got {result['nav_020_autonomy_bonus']}"

    # 2 autonomous, 2 manual
    wp_details_mixed = [
        {"id": "W1", "d_center": 0.0, "timestamp": 10.0, "is_autonomous": True},
        {"id": "W2", "d_center": 0.0, "timestamp": 20.0, "is_autonomous": False},
        {"id": "W3", "d_center": 0.0, "timestamp": 30.0, "is_autonomous": True},
        {"id": "W4", "d_center": 0.0, "timestamp": 40.0, "is_autonomous": False},
    ]
    result2 = calc.final_score(waypoint_details=wp_details_mixed, is_fatal=False)
    assert result2["nav_020_autonomy_bonus"] == 50.0, f"2 auto waypoints should give 50, got {result2['nav_020_autonomy_bonus']}"
    print("  [PASS] O/NAV-020: Autonomy bonus")


def test_nav030_back_to_start():
    """O/NAV-030: 25 pts if rover center within 1m of start, requires >=1 wp reached"""
    calc = ScoreCalculator()

    # With 1 waypoint reached and within 1m of start
    wp_details = [{"id": "W1", "d_center": 0.1, "timestamp": 10.0}]
    back = {"d_center": 0.8, "is_autonomous": True}
    result = calc.final_score(waypoint_details=wp_details, back_to_start_detail=back, is_fatal=False)
    assert result["nav_030_back_to_start"] == 25.0, f"Expected 25, got {result['nav_030_back_to_start']}"

    # With 0 waypoints reached -> no back-to-start score
    result_no_wp = calc.final_score(waypoint_details=[], back_to_start_detail=back, is_fatal=False)
    assert result_no_wp["nav_030_back_to_start"] == 0.0, "No waypoints -> no back to start score"

    # With 1 waypoint but > 1m from start
    back_far = {"d_center": 1.5, "is_autonomous": True}
    result_far = calc.final_score(waypoint_details=wp_details, back_to_start_detail=back_far, is_fatal=False)
    assert result_far["nav_030_back_to_start"] == 0.0, ">1m from start -> no score"
    print("  [PASS] O/NAV-030: Back to start")


def test_nav040_back_to_start_autonomy():
    """O/NAV-040: 25 pts if back to start is autonomous"""
    calc = ScoreCalculator()
    wp_details = [{"id": "W1", "d_center": 0.1, "timestamp": 10.0}]

    back_auto = {"d_center": 0.5, "is_autonomous": True}
    result = calc.final_score(waypoint_details=wp_details, back_to_start_detail=back_auto, is_fatal=False)
    assert result["nav_040_back_to_start_autonomy"] == 25.0

    back_manual = {"d_center": 0.5, "is_autonomous": False}
    result2 = calc.final_score(waypoint_details=wp_details, back_to_start_detail=back_manual, is_fatal=False)
    assert result2["nav_040_back_to_start_autonomy"] == 0.0
    print("  [PASS] O/NAV-040: Back to start autonomy bonus")


def test_nav050_traversal_plan():
    """O/NAV-050: 6 pts per leg completed, max 30 for 5 legs"""
    calc = ScoreCalculator()

    # 4 waypoints + return to start = 5 legs -> 30 pts max
    wp_details = [{"id": f"W{i+1}", "d_center": 0.0, "timestamp": i*10.0} for i in range(4)]
    back = {"d_center": 0.5}
    result = calc.final_score(waypoint_details=wp_details, back_to_start_detail=back, is_fatal=False)
    assert result["nav_050_traversal_plan"] == 30.0, f"5 legs -> 30 pts, got {result['nav_050_traversal_plan']}"

    # Only 2 waypoints, no return -> 2 legs -> 12 pts
    wp_2 = [{"id": f"W{i+1}", "d_center": 0.0, "timestamp": i*10.0} for i in range(2)]
    result2 = calc.final_score(waypoint_details=wp_2, is_fatal=False)
    assert result2["nav_050_traversal_plan"] == 12.0, f"2 legs -> 12 pts, got {result2['nav_050_traversal_plan']}"
    print("  [PASS] O/NAV-050: Traversal plan execution")


def test_nav060_return_penalty():
    """O/NAV-060: -5 per return request"""
    calc = ScoreCalculator()
    wp_details = [{"id": "W1", "d_center": 0.0, "timestamp": 10.0}]

    result = calc.final_score(waypoint_details=wp_details, num_return_requests=3, is_fatal=False)
    assert result["nav_060_return_penalty"] == -15.0, f"3 returns -> -15, got {result['nav_060_return_penalty']}"
    print("  [PASS] O/NAV-060: Return penalty")


def test_nav110_gnss_penalty():
    """O/NAV-110: -20% deduction on total traverse score if only GNSS used"""
    calc = ScoreCalculator()
    wp_details = [{"id": f"W{i+1}", "d_center": 0.0, "timestamp": i*10.0, "is_autonomous": True} for i in range(4)]
    back = {"d_center": 0.5, "is_autonomous": True}

    result_no_gnss = calc.final_score(
        waypoint_details=wp_details, back_to_start_detail=back,
        only_gnss_used=False, is_fatal=False
    )
    result_gnss = calc.final_score(
        waypoint_details=wp_details, back_to_start_detail=back,
        only_gnss_used=True, is_fatal=False
    )
    expected = result_no_gnss["final_score"] * 0.80
    assert abs(result_gnss["final_score"] - expected) < 0.01, \
        f"GNSS penalty: expected {expected}, got {result_gnss['final_score']}"
    print("  [PASS] O/NAV-110: GNSS-only penalty (-20%)")


def test_max_possible_score():
    """Perfect run: 4 waypoints at 0 distance, autonomous, return to start -> 300 pts"""
    calc = ScoreCalculator()
    wp_details = [
        {"id": f"W{i+1}", "d_center": 0.0, "timestamp": i*60.0, "is_autonomous": True}
        for i in range(4)
    ]
    back = {"d_center": 0.0, "is_autonomous": True}

    result = calc.final_score(
        waypoint_details=wp_details,
        back_to_start_detail=back,
        num_return_requests=0,
        only_gnss_used=False,
        is_fatal=False
    )

    # 4*30=120 (O/NAV-010) + 4*25=100 (O/NAV-020) + 25 (O/NAV-030) + 25 (O/NAV-040) + 30 (O/NAV-050) = 300
    assert result["final_score"] == 300.0, f"Perfect run should score 300, got {result['final_score']}"
    print("  [PASS] Perfect run = 300 pts (max)")


def test_numpy_output_format():
    """Verify that the output data is numpy-compatible"""
    calc = ScoreCalculator()
    wp_details = [{"id": "W1", "d_center": 0.5, "timestamp": 30.0, "is_autonomous": True}]

    result = calc.final_score(waypoint_details=wp_details, is_fatal=False, elapsed_time=120.0)

    # Verify NumPy array creation works
    arr = np.array(result, dtype=object)
    assert arr is not None
    loaded = arr.item()
    assert loaded["final_score"] == result["final_score"]
    print("  [PASS] NumPy output format verified")


def test_yaml_waypoint_loading():
    """Test loading waypoints from the navstack YAML parameter file"""
    ws_root = os.environ.get('ROAR_WS', str(Path(__file__).resolve().parents[4]))
    yaml_path = os.path.join(ws_root, 'src', 'navstack', 'navstack_nav2', 'config', 'roar_nav2_sim.yaml')
    if not os.path.exists(yaml_path):
        print("  [SKIP] YAML file not found, skipping YAML loading test")
        return

    start_point, waypoints = load_waypoints_from_yaml(yaml_path)
    assert start_point == (0.0, 0.0), f"Start point should be (0,0), got {start_point}"
    assert len(waypoints) == 4, f"Expected 4 waypoints, got {len(waypoints)}"
    assert waypoints[0] == (2.0, 2.0), f"First waypoint should be (2,2), got {waypoints[0]}"
    assert waypoints[1] == (-5.0, -5.0), f"Second waypoint should be (-5,-5), got {waypoints[1]}"
    print("  [PASS] YAML waypoint loading from roar_nav2_sim.yaml")


if __name__ == '__main__':
    print("=" * 60)
    print("ERC Navigation Task Scoring System - Unit Tests")
    print("=" * 60)
    print()

    tests = [
        test_nav010_waypoint_reached,
        test_edge_distance_calculation,
        test_fatal_crash_zero_score,
        test_timeout_preserves_score,
        test_nav020_autonomy_bonus,
        test_nav030_back_to_start,
        test_nav040_back_to_start_autonomy,
        test_nav050_traversal_plan,
        test_nav060_return_penalty,
        test_nav110_gnss_penalty,
        test_max_possible_score,
        test_numpy_output_format,
        test_yaml_waypoint_loading,
    ]

    passed = 0
    failed = 0
    for t in tests:
        try:
            t()
            passed += 1
        except Exception as e:
            failed += 1
            print(f"  [FAIL] {t.__name__}: {e}")

    print()
    print(f"Results: {passed} passed, {failed} failed out of {len(tests)} tests")
    if failed > 0:
        sys.exit(1)
    else:
        print("All tests passed!")
