from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple, Dict, Any
import math
import os
import yaml
import numpy as np

@dataclass
class Waypoint:
    id: str
    x: float
    y: float
    acceptance_radius: float = 1.0


class MissionState(Enum):
    IN_PROGRESS = "in_progress"
    WAITING_FOR_COMMAND = "waiting_for_command"
    COMPLETED = "completed"
    TIMED_OUT = "timed_out"
    FAILED = "failed"


def load_waypoints_from_yaml(file_path: str) -> Tuple[Tuple[float, float], List[Tuple[float, float]]]:
    """
    Parses a YAML parameter file (matching erc_waypoint_follower.py format)
    to extract start_end coordinates and waypoints list.
    """
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)

    start_end = None
    waypoints_raw = None

    def search_dict(d):
        nonlocal start_end, waypoints_raw
        if not isinstance(d, dict):
            return
        if 'ros__parameters' in d:
            params = d['ros__parameters']
            for k, v in params.items():
                if isinstance(v, dict):
                    if 'start_end' in v:
                        start_end = v['start_end']
                    if 'waypoints' in v:
                        waypoints_raw = v['waypoints']
            if 'start_end' in params:
                start_end = params['start_end']
            if 'waypoints' in params:
                waypoints_raw = params['waypoints']

        for val in d.values():
            if isinstance(val, dict):
                search_dict(val)

    search_dict(data)

    if start_end is None or waypoints_raw is None:
        raise ValueError(f"Could not find 'start_end' and 'waypoints' parameters in {file_path}")

    waypoints = []
    for i in range(0, len(waypoints_raw), 2):
        waypoints.append((float(waypoints_raw[i]), float(waypoints_raw[i+1])))

    start_point = (float(start_end[0]), float(start_end[1]))
    return start_point, waypoints


@dataclass
class WaypointTracker:
    # Waypoints sequence
    waypoints: List[Waypoint]
    # Starting pose (x, y)
    start_pose: Tuple[float, float]
    # Rover body radius for edge distance calculations
    rover_radius: float = 0.5
    # Auto-advance to next waypoint upon arrival (default True for seamless navstack integration)
    auto_advance: bool = True
    # Optional optimal path segments
    optimal_path_segments: List[List[Tuple[float, float]]] = field(default_factory=list)

    # Internal tracking fields
    _current_index: int = field(default=0, init=False)
    _last_pose: Optional[Tuple[float, float]] = field(default=None, init=False)
    _actual_distance: float = field(default=0.0, init=False)
    _state: MissionState = field(default=MissionState.IN_PROGRESS, init=False)
    _waypoint_records: List[Dict[str, Any]] = field(default_factory=list, init=False)
    _back_to_start_record: Optional[Dict[str, Any]] = field(default=None, init=False)
    _num_returns: int = field(default=0, init=False)
    _pending_overrides: Dict[str, Tuple[float, float, float]] = field(default_factory=dict, init=False)

    def __post_init__(self):
        self._last_pose = self.start_pose
        if not self.optimal_path_segments:
            points = [self.start_pose] + [(wp.x, wp.y) for wp in self.waypoints] + [self.start_pose]
            self.optimal_path_segments = [
                [points[i], points[i+1]] for i in range(len(points) - 1)
            ]

    @staticmethod
    def euclidean(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1])

    @property
    def total_ideal_distance(self) -> float:
        total = 0.0
        for seg in self.optimal_path_segments:
            for i in range(len(seg) - 1):
                total += self.euclidean(seg[i], seg[i+1])
        return total

    def update_position(
        self,
        current_pose: Tuple[float, float],
        current_sim_time: float,
        current_speed: float,
        is_autonomous: bool = True
    ) -> None:
        """
        Updates rover tracking, calculates distance traveled, and checks if waypoints
        or return-to-start are reached. Updates settled stop pose while stationary.
        """
        if self._state not in [MissionState.IN_PROGRESS, MissionState.WAITING_FOR_COMMAND]:
            return

        # While waiting for command (stationary at waypoint waiting for user to press Enter)
        if self._state == MissionState.WAITING_FOR_COMMAND:
            if current_speed > 0.05:
                # Rover started moving toward the next target! Resume tracking for next leg.
                self._state = MissionState.IN_PROGRESS
            self._last_pose = current_pose
            return

        # Traveled distance accumulate
        if self._last_pose is not None:
            self._actual_distance += self.euclidean(self._last_pose, current_pose)
        self._last_pose = current_pose

        # 1. Waypoint checking
        if self._current_index < len(self.waypoints):
            target = self.waypoints[self._current_index]
            dist_to_target = self.euclidean(current_pose, (target.x, target.y))

            # Accept waypoint if within acceptance radius AND stopped (< 0.05 m/s)
            if dist_to_target <= target.acceptance_radius and current_speed < 0.05:
                stop_pos = current_pose
                if target.id in self._pending_overrides:
                    rx, ry, dist_err = self._pending_overrides.pop(target.id)
                    stop_pos = (rx, ry)
                    dist_to_target = dist_err

                edge_dist = max(0.0, dist_to_target - self.rover_radius)
                record = {
                    "id": target.id,
                    "timestamp": current_sim_time,
                    "d_center": dist_to_target,
                    "d_edge": edge_dist,
                    "positional_error": dist_to_target,
                    "stop_pose": stop_pos,
                    "is_autonomous": is_autonomous,
                    "reached": True,
                    "status": "REACHED",
                }
                self._waypoint_records.append(record)
                self._current_index += 1

                # Enter WAITING_FOR_COMMAND to pause timer while rover is parked
                if self._current_index < len(self.waypoints):
                    self._state = MissionState.WAITING_FOR_COMMAND
                else:
                    # Finished all waypoints, targeting back to start
                    self._state = MissionState.IN_PROGRESS

        # 2. Back-to-start checking (after all waypoints reached)
        elif self._current_index == len(self.waypoints):
            dist_to_start = self.euclidean(current_pose, self.start_pose)
            if dist_to_start <= 1.0 and current_speed < 0.05:
                stop_pos = current_pose
                if "START" in self._pending_overrides or "RETURN" in self._pending_overrides:
                    ov_key = "START" if "START" in self._pending_overrides else "RETURN"
                    rx, ry, dist_err = self._pending_overrides.pop(ov_key)
                    stop_pos = (rx, ry)
                    dist_to_start = dist_err

                self._back_to_start_record = {
                    "timestamp": current_sim_time,
                    "d_center": dist_to_start,
                    "positional_error": dist_to_start,
                    "stop_pose": stop_pos,
                    "is_autonomous": is_autonomous,
                }
                self._state = MissionState.COMPLETED

    def skip_current_waypoint(self, current_sim_time: float, current_pose: Tuple[float, float]) -> bool:
        """Records current waypoint as skipped/failed and advances index to next waypoint."""
        if self._current_index < len(self.waypoints):
            target = self.waypoints[self._current_index]
            dist_to_target = self.euclidean(current_pose, (target.x, target.y))
            edge_dist = max(0.0, dist_to_target - self.rover_radius)
            record = {
                "id": target.id,
                "timestamp": current_sim_time,
                "d_center": dist_to_target,
                "d_edge": edge_dist,
                "positional_error": dist_to_target,
                "stop_pose": current_pose,
                "is_autonomous": False,
                "reached": False,
                "status": "FAILED",
            }
            self._waypoint_records.append(record)
            self._current_index += 1
            self._state = MissionState.IN_PROGRESS
            return True
        return False

    def override_waypoint_result(self, wp_id: str, rx: float, ry: float, dist_error: float) -> bool:
        """Overrides a recorded waypoint by ID (e.g. W1, W2, W3, W4, START) with exact follower result."""
        edge_dist = max(0.0, dist_error - self.rover_radius)
        wp_id_upper = wp_id.upper()

        if wp_id_upper in ("START", "RETURN"):
            if self._back_to_start_record is not None:
                self._back_to_start_record["stop_pose"] = (rx, ry)
                self._back_to_start_record["d_center"] = dist_error
                self._back_to_start_record["positional_error"] = dist_error
            else:
                self._pending_overrides["START"] = (rx, ry, dist_error)
            return True

        for rec in self._waypoint_records:
            if rec.get("id", "").upper() == wp_id_upper:
                rec["stop_pose"] = (rx, ry)
                rec["d_center"] = dist_error
                rec["d_edge"] = edge_dist
                rec["positional_error"] = dist_error
                return True

        self._pending_overrides[wp_id_upper] = (rx, ry, dist_error)
        return True

    def resume_mission(self) -> bool:
        """Resumes mission after judge / referee command (e.g. NEXT)."""
        if self._state == MissionState.WAITING_FOR_COMMAND:
            self._state = MissionState.IN_PROGRESS
            return True
        return False

    def record_return_request(self) -> None:
        self._num_returns += 1

    @property
    def actual_distance(self) -> float:
        return self._actual_distance

    @property
    def waypoints_completed(self) -> int:
        return len(self._waypoint_records)

    @property
    def waypoint_records(self) -> List[Dict[str, Any]]:
        return list(self._waypoint_records)

    @property
    def back_to_start_record(self) -> Optional[Dict[str, Any]]:
        return self._back_to_start_record

    @property
    def num_returns(self) -> int:
        return self._num_returns

    @property
    def state(self) -> MissionState:
        return self._state