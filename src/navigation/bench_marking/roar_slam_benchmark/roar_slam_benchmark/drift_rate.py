import numpy as np

class DriftRateCalculator:
    def __init__(self, window_size_sec=30.0):
        self.window_size_sec = window_size_sec

    def compute_global_drift(self, times, errors):
        """
        Calculates global drift rate [m/s] using a linear least-squares fit over the entire trajectory.
        """
        if len(times) < 2:
            return 0.0
            
        t_arr = np.array(times)
        e_arr = np.array(errors)
        
        # A * x = b -> [t 1] * [slope, intercept]^T = e
        A = np.vstack([t_arr, np.ones(len(t_arr))]).T
        m, _ = np.linalg.lstsq(A, e_arr, rcond=None)[0]
        
        # We only care about positive drift (error increasing). Negative slope means self-correcting.
        return max(0.0, m)

    def compute_windowed_drift(self, times, errors):
        """
        Calculates maximum windowed drift rate over a sliding window.
        Returns the max slope found over any window of size window_size_sec.
        """
        if len(times) < 2:
            return 0.0
            
        t_arr = np.array(times)
        e_arr = np.array(errors)
        
        max_drift_rate = 0.0
        n = len(t_arr)
        
        # Sliding window approach
        start_idx = 0
        for end_idx in range(1, n):
            # Advance start index until window size is right
            while start_idx < end_idx and (t_arr[end_idx] - t_arr[start_idx]) > self.window_size_sec:
                start_idx += 1
                
            # Need at least a few points and a minimum time delta to do a meaningful fit
            dt = t_arr[end_idx] - t_arr[start_idx]
            if end_idx - start_idx >= 5 and dt >= (self.window_size_sec * 0.5):
                t_window = t_arr[start_idx:end_idx+1]
                e_window = e_arr[start_idx:end_idx+1]
                
                A = np.vstack([t_window, np.ones(len(t_window))]).T
                m, _ = np.linalg.lstsq(A, e_window, rcond=None)[0]
                
                if m > max_drift_rate:
                    max_drift_rate = m
                    
        return max_drift_rate

    def get_drift_metrics(self, times, errors):
        return {
            "global_drift_rate": self.compute_global_drift(times, errors),
            "max_windowed_drift_rate": self.compute_windowed_drift(times, errors)
        }
