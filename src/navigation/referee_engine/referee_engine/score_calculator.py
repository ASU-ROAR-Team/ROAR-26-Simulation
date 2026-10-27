from dataclasses import dataclass
from typing import List, Dict, Any, Optional

@dataclass
class ScoreCalculatorConfig:
    # Radius of rover body in meters (used for edge distance calculation d = max(0, d_center - rover_radius))
    rover_radius: float = 0.5
    # Maximum score awarded per waypoint for reaching it (O/NAV-010)
    max_pts_per_waypoint: float = 30.0
    # Autonomy bonus per waypoint reached (O/NAV-020)
    autonomy_bonus_per_waypoint: float = 25.0
    # Back to start reward (O/NAV-030)
    back_to_start_reward: float = 25.0
    # Back to start autonomy bonus (O/NAV-040)
    back_to_start_autonomy_reward: float = 25.0
    # Traversal plan execution score per leg (O/NAV-050)
    traversal_plan_score_per_leg: float = 6.0
    # Return penalty per request (O/NAV-060)
    return_penalty_per_req: float = 5.0
    # Maximum allowed mission duration in simulation time
    max_time_limit: float = 1200.0  # 20 minutes

class ScoreCalculator:
    """
    ERC Navigation Task Score Calculator (TRAVERSE SUB-TASK: MAX 300 POINTS)

    Rules Implemented:
    1. O/NAV-010: Waypoint reached -> MAX(0, round(30 - 6 * d)) where d is edge distance (Max 120 pts total, 30 per wp)
    2. O/NAV-020: Autonomy bonus -> 25 pts per waypoint reached in autonomous mode (Max 100 pts)
    3. O/NAV-030: Back to start -> 25 pts if rover center is within 1m of start at finish (requires >=1 wp reached)
    4. O/NAV-040: Back to start autonomy bonus -> 25 pts if back to start done in autonomous mode
    5. O/NAV-050: Execution of Traversal Plan -> 6 pts per completed leg following plan (Max 30 pts for 5 legs)
    6. O/NAV-060: Penalty points return -> -5 per return request
    7. O/NAV-070 to O/NAV-100: Ignored
    8. O/NAV-110: Penalty points - only GNSS feed used -> -20% deduction on traverse points
    """
    def __init__(self, config: ScoreCalculatorConfig = ScoreCalculatorConfig()):
        self.config = config

    def calculate_edge_distance(self, center_distance: float) -> float:
        """Calculate distance in meters from the edge of the rover's body to the target waypoint."""
        return max(0.0, center_distance - self.config.rover_radius)

    def score_waypoint_reached(self, d_edge: float) -> float:
        """
        O/NAV-010: Waypoint reached score.
        MAX(0, round(30 - (6 * d))) where d is edge distance in meters.
        """
        raw_score = round(30.0 - (6.0 * d_edge))
        return max(0.0, min(self.config.max_pts_per_waypoint, float(raw_score)))

    def final_score(
        self,
        waypoint_details: List[Dict[str, Any]],
        back_to_start_detail: Optional[Dict[str, Any]] = None,
        num_return_requests: int = 0,
        only_gnss_used: bool = False,
        elapsed_time: float = 0.0,
        is_fatal: bool = False,
    ) -> Dict[str, Any]:
        """
        Compute the official ERC Navigation Task score.
        If is_fatal is True (crash, collision, out of boundaries), score is immediately 0.
        If mission is timed out, the accumulated score up to timeout is preserved and returned.
        """
        if is_fatal:
            return {
                "traverse_score": 0.0,
                "droning_score": 0.0,
                "final_score": 0.0,
                "max_possible_score": 300.0,
                "nav_010_waypoint_reached": 0.0,
                "nav_020_autonomy_bonus": 0.0,
                "nav_030_back_to_start": 0.0,
                "nav_040_back_to_start_autonomy": 0.0,
                "nav_050_traversal_plan": 0.0,
                "nav_060_return_penalty": 0.0,
                "nav_110_gnss_penalty_applied": False,
                "waypoint_details": waypoint_details,
                "back_to_start_detail": back_to_start_detail,
                "is_fatal": True,
                "fatal_reason": "Collision or Out of Boundaries",
                "elapsed_time_sec": elapsed_time,
            }

        # 1. O/NAV-010 & O/NAV-020: Waypoint Scores
        nav_010_total = 0.0
        nav_020_total = 0.0
        successful_waypoints_count = 0

        processed_wp_details = []
        for wp in waypoint_details:
            d_center = wp.get("d_center", wp.get("dmin", 0.0))
            d_edge = self.calculate_edge_distance(d_center)
            is_auto = wp.get("is_autonomous", True)
            is_reached = wp.get("reached", True) and wp.get("status", "REACHED") != "FAILED"

            if is_reached:
                wp_score_010 = self.score_waypoint_reached(d_edge)
                wp_score_020 = self.config.autonomy_bonus_per_waypoint if is_auto else 0.0
                successful_waypoints_count += 1
            else:
                wp_score_010 = 0.0
                wp_score_020 = 0.0

            nav_010_total += wp_score_010
            nav_020_total += wp_score_020

            processed_wp_details.append({
                "id": wp.get("id", "WP"),
                "timestamp": wp.get("timestamp", 0.0),
                "d_center": d_center,
                "d_edge": d_edge,
                "positional_error": d_center,
                "score_010": wp_score_010,
                "score_020": wp_score_020,
                "total_score": wp_score_010 + wp_score_020,
                "is_autonomous": is_auto,
                "reached": is_reached,
                "status": wp.get("status", "REACHED" if is_reached else "FAILED"),
                "stop_pose": wp.get("stop_pose", (0.0, 0.0)),
            })

        # Cap sub-totals according to ERC rules
        nav_010_total = min(120.0, nav_010_total)
        nav_020_total = min(100.0, nav_020_total)

        # 2. O/NAV-030 & O/NAV-040: Back to Start Scores
        nav_030_total = 0.0
        nav_040_total = 0.0

        if back_to_start_detail is not None and successful_waypoints_count >= 1:
            dist_start = back_to_start_detail.get("d_center", back_to_start_detail.get("dist_to_start", 999.0))
            is_auto_start = back_to_start_detail.get("is_autonomous", True)

            if dist_start <= 1.0:
                nav_030_total = self.config.back_to_start_reward
                if is_auto_start:
                    nav_040_total = self.config.back_to_start_autonomy_reward

        # 3. O/NAV-050: Execution of Traversal Plan (6 pts per completed leg, max 30 pts for 5 legs)
        legs_completed = successful_waypoints_count
        if back_to_start_detail is not None and back_to_start_detail.get("d_center", 999.0) <= 1.0:
            legs_completed += 1
        nav_050_total = min(30.0, legs_completed * self.config.traversal_plan_score_per_leg)

        # 4. O/NAV-060: Return Penalties (-5 per return request)
        nav_060_total = -1.0 * (num_return_requests * self.config.return_penalty_per_req)

        # Raw traverse score sum
        raw_traverse_score = (
            nav_010_total
            + nav_020_total
            + nav_030_total
            + nav_040_total
            + nav_050_total
            + nav_060_total
        )

        # 5. O/NAV-110: Only GNSS feed used -> 20% penalty on total traverse score
        traverse_score = max(0.0, raw_traverse_score)
        if only_gnss_used:
            traverse_score *= 0.80

        traverse_score = min(300.0, max(0.0, traverse_score))

        return {
            "traverse_score": float(traverse_score),
            "droning_score": 0.0,
            "final_score": float(traverse_score),
            "max_possible_score": 300.0,
            "nav_010_waypoint_reached": float(nav_010_total),
            "nav_020_autonomy_bonus": float(nav_020_total),
            "nav_030_back_to_start": float(nav_030_total),
            "nav_040_back_to_start_autonomy": float(nav_040_total),
            "nav_050_traversal_plan": float(nav_050_total),
            "nav_060_return_penalty": float(nav_060_total),
            "nav_110_gnss_penalty_applied": only_gnss_used,
            "waypoint_details": processed_wp_details,
            "back_to_start_detail": back_to_start_detail,
            "is_fatal": False,
            "elapsed_time_sec": elapsed_time,
        }
