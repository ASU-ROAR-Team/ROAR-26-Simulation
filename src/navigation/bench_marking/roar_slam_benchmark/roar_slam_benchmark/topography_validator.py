import numpy as np

class TopographyValidator:
    def __init__(self, truth_heightmap_path, map_resolution=0.1, origin_x=-10.0, origin_y=-10.0):
        self.map_resolution = map_resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        
        self.truth_map = None
        
        if truth_heightmap_path:
            try:
                data = np.load(truth_heightmap_path)
                if isinstance(data, np.lib.npyio.NpzFile):
                    self.truth_map = data[data.files[0]]
                else:
                    self.truth_map = data
            except Exception as e:
                print(f"TopographyValidator Warning: Failed to load truth heightmap: {e}")

    def evaluate_live_map(self, live_grid, live_res, live_origin_x, live_origin_y, live_width, live_height):
        """
        Compares live_grid to truth_map. live_grid is typically a 1D array from OccupancyGrid.
        """
        if self.truth_map is None:
            return None
            
        # Reconstruct 2D live map (assume row-major)
        live_2d = np.array(live_grid).reshape((live_height, live_width))
        
        truth_rows, truth_cols = self.truth_map.shape
        
        squared_error_sum = 0.0
        valid_cells = 0
        
        true_positives = 0
        false_positives = 0
        false_negatives = 0
        true_lethal_count = 0
        
        LETHAL_THRESHOLD = 50 # costmap 0-100 scale, assume >50 is lethal rock
        
        for r in range(truth_rows):
            for c in range(truth_cols):
                # World coordinates of truth cell
                x = c * self.map_resolution + self.origin_x
                y = r * self.map_resolution + self.origin_y
                
                # Corresponding cell in live map
                live_c = int(round((x - live_origin_x) / live_res))
                live_r = int(round((y - live_origin_y) / live_res))
                
                if 0 <= live_r < live_height and 0 <= live_c < live_width:
                    live_val = live_2d[live_r, live_c]
                    
                    if live_val >= 0: # Not unknown (-1)
                        # We have coverage here
                        valid_cells += 1
                        
                        # Elevation RMSE approximation (costmap vs raw elevation isn't 1:1,
                        # this assumes if elevation is exposed we use it, else we skip elevation RMSE)
                        # We'll calculate classification accuracy for now using the costmap
                        is_truth_lethal = self.truth_map[r, c] > 0.1 # Example heuristic: relative height > 10cm is lethal
                        if is_truth_lethal:
                            true_lethal_count += 1
                            
                        is_live_lethal = live_val >= LETHAL_THRESHOLD
                        
                        if is_truth_lethal and is_live_lethal:
                            true_positives += 1
                        elif not is_truth_lethal and is_live_lethal:
                            false_positives += 1
                        elif is_truth_lethal and not is_live_lethal:
                            false_negatives += 1

        coverage_fraction = valid_cells / (truth_rows * truth_cols) if (truth_rows * truth_cols) > 0 else 0
        
        precision = true_positives / (true_positives + false_positives) if (true_positives + false_positives) > 0 else 0
        recall = true_positives / (true_positives + false_negatives) if (true_positives + false_negatives) > 0 else 0
        f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
        
        return {
            "elevation_rmse": 0.0, # Placeholder unless raw elevation topic is confirmed
            "coverage_fraction": coverage_fraction,
            "obstacle_precision": precision,
            "obstacle_recall": recall,
            "obstacle_f1_score": f1_score
        }
