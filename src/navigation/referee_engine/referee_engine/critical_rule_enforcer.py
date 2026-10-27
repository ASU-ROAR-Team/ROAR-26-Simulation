import time

class CriticalRuleEnforcer:
    """
    Monitors critical mission state:
    - Collision -> Immediate KILL (is_fatal = True -> score = 0)
    - Out of Boundary -> Immediate KILL (is_fatal = True -> score = 0)
    - Timeout -> TIMEOUT at 1200s sim time (is_fatal = False -> score is calculated and displayed up to timeout)
    """

    def __init__(self, bounds=[-1000.0, 1000.0, -1000.0, 1000.0], timeout_secs=1200.0):
        self.bounds = bounds
        self.timeout_secs = timeout_secs
        self.collision_detected = False
        self.halt_triggered = False

    def trigger_collision(self):
        self.collision_detected = True
        self.halt_triggered = True

    def check_boundary(self, pose):
        xmin, xmax, ymin, ymax = self.bounds
        x = pose.position.x
        y = pose.position.y
        if not (xmin <= x <= xmax and ymin <= y <= ymax):
            self.halt_triggered = True

    def evaluate_terminal_state(self, current_x, current_y, elapsed_time, waypoints_remaining, is_completed=False):
        xmin, xmax, ymin, ymax = self.bounds

        # Fatal collisions or out-of-boundary -> score becomes 0 immediately
        if self.collision_detected:
            return "KILL", "Terminal: Collision Detected (Score -> 0).", True

        if not (xmin <= current_x <= xmax and ymin <= current_y <= ymax) or self.halt_triggered:
            return "KILL", "Terminal: Out of Boundary (Score -> 0).", True

        # Timeout at 20 minutes (1200s) -> score is displayed up to timeout
        if elapsed_time > self.timeout_secs:
            return "TIMEOUT", "Terminal: Mission Time Limit Exceeded (Score displayed).", False

        if is_completed:
            return "SUCCESS", "Terminal: Mission Completed.", False

        return "RUNNING", "State: Nominal", False