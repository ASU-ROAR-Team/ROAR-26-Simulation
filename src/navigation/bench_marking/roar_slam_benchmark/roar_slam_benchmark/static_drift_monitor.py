import math
from enum import Enum

class MonitorState(Enum):
    DRIVING = 1
    SETTLING = 2
    STATIONARY_WINDOW_OPEN = 3

class StaticDriftMonitor:
    def __init__(self, static_zero_threshold=1e-3, static_min_duration=3.0, static_grace_period=1.0):
        self.static_zero_threshold = static_zero_threshold
        self.static_min_duration = static_min_duration
        self.static_grace_period = static_grace_period
        
        self.state = MonitorState.DRIVING
        
        self.zero_cmd_start_time = -1.0
        self.window_open_time = -1.0
        
        self.entry_pose = None
        self.max_drift_in_current_window = 0.0
        
        self.phantom_drift_events = []
        
        self.last_pose = None
        
    def update_cmd_vel(self, t, linear_x, angular_z):
        is_zero_cmd = abs(linear_x) < self.static_zero_threshold and abs(angular_z) < self.static_zero_threshold
        
        if self.state == MonitorState.DRIVING:
            if is_zero_cmd:
                self.zero_cmd_start_time = t
                self.state = MonitorState.SETTLING
        
        elif self.state == MonitorState.SETTLING:
            if not is_zero_cmd:
                self.state = MonitorState.DRIVING
            else:
                if (t - self.zero_cmd_start_time) >= self.static_grace_period:
                    self.state = MonitorState.STATIONARY_WINDOW_OPEN
                    self.window_open_time = t
                    self.entry_pose = self.last_pose
                    self.max_drift_in_current_window = 0.0
                    
        elif self.state == MonitorState.STATIONARY_WINDOW_OPEN:
            if not is_zero_cmd:
                # Window closed! Record the event if it was open longer than minimum duration
                window_duration = t - self.window_open_time
                if window_duration >= self.static_min_duration:
                    phantom_drift_final = 0.0
                    if self.last_pose and self.entry_pose:
                        phantom_drift_final = math.sqrt((self.last_pose[0] - self.entry_pose[0])**2 + (self.last_pose[1] - self.entry_pose[1])**2)
                    
                    self.phantom_drift_events.append({
                        "start_time": self.window_open_time,
                        "duration": window_duration,
                        "max_distance": self.max_drift_in_current_window,
                        "final_distance": phantom_drift_final
                    })
                
                self.state = MonitorState.DRIVING
                
    def update_pose(self, t, x, y, z):
        self.last_pose = (x, y, z)
        
        if self.state == MonitorState.STATIONARY_WINDOW_OPEN and self.entry_pose:
            dist = math.sqrt((x - self.entry_pose[0])**2 + (y - self.entry_pose[1])**2)
            if dist > self.max_drift_in_current_window:
                self.max_drift_in_current_window = dist

    def finalize(self, t):
        # Close any open window on shutdown
        if self.state == MonitorState.STATIONARY_WINDOW_OPEN:
            window_duration = t - self.window_open_time
            if window_duration >= self.static_min_duration:
                phantom_drift_final = 0.0
                if self.last_pose and self.entry_pose:
                    phantom_drift_final = math.sqrt((self.last_pose[0] - self.entry_pose[0])**2 + (self.last_pose[1] - self.entry_pose[1])**2)
                
                self.phantom_drift_events.append({
                    "start_time": self.window_open_time,
                    "duration": window_duration,
                    "max_distance": self.max_drift_in_current_window,
                    "final_distance": phantom_drift_final
                })
        
    def get_phantom_drift_score(self):
        """
        Returns max(phantom_drift_distance / window_duration) across all valid windows. [m/s]
        """
        max_rate = 0.0
        for ev in self.phantom_drift_events:
            rate = ev["max_distance"] / ev["duration"]
            if rate > max_rate:
                max_rate = rate
        return max_rate
