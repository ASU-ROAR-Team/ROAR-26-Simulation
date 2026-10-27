import numpy as np
import math

class TrajectoryErrorTracker:
    def __init__(self, use_3d=False):
        self.use_3d = use_3d
        
        # Welford's algorithm states
        self.count = 0
        self.mean_error = 0.0
        self.m2 = 0.0  # Sum of squared differences from the current mean
        
        self.max_error = 0.0
        self.min_error = float('inf')
        
        # Sum of squared errors for RMSE calculation
        self.sum_sq_error = 0.0
        
        # History for plotting/reporting
        self.times = []
        self.errors = []
        
        # For CSV writing later
        self.raw_data = []

    def update(self, t, p_gt, p_est):
        """
        p_gt: Ground truth position (x, y, z)
        p_est: Estimated position (x, y, z)
        """
        if self.use_3d:
            error = math.sqrt((p_gt[0] - p_est[0])**2 + 
                              (p_gt[1] - p_est[1])**2 + 
                              (p_gt[2] - p_est[2])**2)
        else:
            error = math.sqrt((p_gt[0] - p_est[0])**2 + 
                              (p_gt[1] - p_est[1])**2)

        self.times.append(t)
        self.errors.append(error)
        self.raw_data.append((t, p_gt[0], p_gt[1], p_gt[2], p_est[0], p_est[1], p_est[2], error))

        self.count += 1
        
        # Welford's algorithm for online variance/mean
        delta = error - self.mean_error
        self.mean_error += delta / self.count
        delta2 = error - self.mean_error
        self.m2 += delta * delta2

        self.sum_sq_error += error * error

        if error > self.max_error:
            self.max_error = error
        if error < self.min_error:
            self.min_error = error

    def get_rmse(self):
        if self.count == 0:
            return 0.0
        return math.sqrt(self.sum_sq_error / self.count)

    def get_variance(self):
        if self.count < 2:
            return 0.0
        return self.m2 / (self.count - 1)

    def get_stats(self):
        return {
            "count": self.count,
            "mean_error": self.mean_error,
            "max_error": self.max_error,
            "min_error": self.min_error if self.count > 0 else 0.0,
            "rmse": self.get_rmse(),
            "variance": self.get_variance()
        }
