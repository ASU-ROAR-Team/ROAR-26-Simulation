import math

class SlamScorer:
    def __init__(self, w_traj=0.35, w_drift=0.15, w_static=0.30, w_topo=0.20):
        self.w_traj = w_traj
        self.w_drift = w_drift
        self.w_static = w_static
        self.w_topo = w_topo
        
        # Reference calibration constants (value at which score is ~37%)
        self.ref_rmse_traj = 0.5        # [m]
        self.ref_drift_rate = 0.05      # [m/s]
        self.ref_phantom_drift = 0.01   # [m/s]
        self.ref_elevation_rmse = 0.15  # [m]
        
        self.static_hard_fail_threshold = 0.10 # [m/s]
        
    def compute(self, traj_rmse, max_windowed_drift_rate, phantom_drift_score, elevation_rmse, obstacle_f1_score):
        """
        Computes final 0-100 score and returns breakdown.
        """
        # Soft-clamped inverse-exponential components (0 to 1)
        traj_comp = math.exp(-traj_rmse / self.ref_rmse_traj)
        drift_comp = math.exp(-max_windowed_drift_rate / self.ref_drift_rate)
        static_comp = math.exp(-phantom_drift_score / self.ref_phantom_drift)
        
        # Topography blends elevation accuracy and discrete obstacle detection
        topo_comp = 0.6 * math.exp(-elevation_rmse / self.ref_elevation_rmse) + 0.4 * obstacle_f1_score
        
        # Weighted sum out of 100
        raw_score = 100 * (
            self.w_traj * traj_comp +
            self.w_drift * drift_comp +
            self.w_static * static_comp +
            self.w_topo * topo_comp
        )
        
        final_score = raw_score
        hard_fail_triggered = False
        
        # Hard fail cap for egregious phantom drift
        if phantom_drift_score > self.static_hard_fail_threshold:
            hard_fail_triggered = True
            final_score = min(raw_score, 25.0)
            
        return {
            "final_score": final_score,
            "raw_weighted_score": raw_score,
            "hard_fail_triggered": hard_fail_triggered,
            "components": {
                "trajectory": {
                    "raw_value": traj_rmse,
                    "normalized_score": traj_comp,
                    "weight": self.w_traj
                },
                "drift_rate": {
                    "raw_value": max_windowed_drift_rate,
                    "normalized_score": drift_comp,
                    "weight": self.w_drift
                },
                "static_drift": {
                    "raw_value": phantom_drift_score,
                    "normalized_score": static_comp,
                    "weight": self.w_static
                },
                "topography": {
                    "raw_value": {"elevation_rmse": elevation_rmse, "obstacle_f1": obstacle_f1_score},
                    "normalized_score": topo_comp,
                    "weight": self.w_topo
                }
            }
        }
