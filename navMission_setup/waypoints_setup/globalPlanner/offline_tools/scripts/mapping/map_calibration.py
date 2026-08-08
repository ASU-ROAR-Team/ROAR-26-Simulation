#!/usr/bin/env python3
"""
Module for calibrating map images using horizontally-aligned red spots to
determine the resolution (meters per pixel) and updating the config parameters.
The origin is the leftmost marker.
"""

import ast
import os
import sys
import re
from collections import namedtuple
from itertools import combinations
from typing import List, Tuple, Optional
import cv2
import numpy as np
import yaml
import matplotlib.pyplot as plt
from matplotlib.offsetbox import AnchoredText
import matplotlib.patheffects as path_effects

# Data structures for grouped attributes
Origin = namedtuple("Origin", ["index", "pixel", "real"])
PlotObjects = namedtuple("PlotObjects", ["figure", "axes"])


class MapCalibration:
    """Calibrates map images by detecting markers for coordinate transformations."""

    def __init__(self, config_path: str) -> None:
        """Initialize and run calibration."""
        self.config_path = config_path
        self.image: Optional[np.ndarray] = None
        self.realCoords: List[Tuple[float, float]] = []
        self.pixelCoords: List[Tuple[int, int]] = []
        self.origin: Optional[Origin] = None
        self.scale: float = 1.0  # pixels per real unit
        self.resolution: float = 0.1  # real units per pixel
        self.plot: Optional[PlotObjects] = None

        self._load_parameters()
        self._process_image()
        self._calculate_calibration()
        self._setup_visualization()

    def _load_parameters(self) -> None:
        """Load and validate parameters from config.yaml."""
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Config file not found: {self.config_path}")

        with open(self.config_path, "r") as f:
            cfg = yaml.safe_load(f)

        calibration_cfg = cfg.get("calibration", {})
        image_rel_path = calibration_cfg.get("image_path", "")
        real_coords_str = calibration_cfg.get("real_coords", "")

        # Resolve paths relative to config file directory
        config_dir = os.path.dirname(self.config_path)
        image_path = os.path.abspath(os.path.join(config_dir, image_rel_path))

        print(f"Loading calibration image from: {image_path}")
        self.image = cv2.imread(image_path)
        if self.image is None:
            raise IOError(f"Failed to load image from {image_path}")

        if not real_coords_str:
            raise ValueError("Missing 'calibration.real_coords' parameter in config.yaml")

        self.realCoords = ast.literal_eval(real_coords_str)
        print(f"Loaded known real-world coordinates: {self.realCoords}")

    def _process_image(self) -> None:
        """Process the image to find the best horizontally-aligned red markers."""
        if self.image is None:
            raise ValueError("Image not loaded")

        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        
        # Define HSV ranges for the color red
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        
        mask = mask1 + mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        centroids = []
        min_area, max_area = 10, 1000  # Filter out noise and large blobs
        for contour in contours:
            if min_area < cv2.contourArea(contour) < max_area:
                moments = cv2.moments(contour)
                if moments["m00"] > 0:
                    cx = int(moments["m10"] / moments["m00"])
                    cy = int(moments["m01"] / moments["m00"])
                    centroids.append((cx, cy))
        
        num_markers = len(self.realCoords)
        if len(centroids) < num_markers:
            raise RuntimeError(f"Could not find enough candidate markers. Found {len(centroids)}, expected {num_markers}.")

        # Find the combination of N markers that is most horizontal
        best_combo = None
        min_y_std_dev = float('inf')
        for combo in combinations(centroids, num_markers):
            y_coords = [p[1] for p in combo]
            std_dev = np.std(y_coords)
            if std_dev < min_y_std_dev:
                min_y_std_dev = std_dev
                best_combo = combo

        # Sort the final list of markers by their x-coordinate
        self.pixelCoords = sorted(list(best_combo), key=lambda p: p[0])
        print(f"Detected marker pixel coordinates (sorted): {self.pixelCoords}")

    def _calculate_calibration(self) -> None:
        """Calculate origin, scaling factor, and resolution."""
        if not self.pixelCoords or not self.realCoords:
            raise ValueError("No coordinates available for calibration.")

        # The origin is the leftmost marker (minimum x pixel coordinate)
        origin_idx = 0
        self.origin = Origin(
            index=origin_idx,
            pixel=self.pixelCoords[origin_idx],
            real=self.realCoords[origin_idx],
        )

        # Calculate scale based on the X-axis distances
        scales: List[float] = []
        for idx, (pixelPoint, realPoint) in enumerate(zip(self.pixelCoords, self.realCoords)):
            if idx == self.origin.index:
                continue
            
            real_x_delta = realPoint[0] - self.origin.real[0]
            pixel_x_delta = pixelPoint[0] - self.origin.pixel[0]
            
            if real_x_delta != 0:
                scales.append(pixel_x_delta / real_x_delta)

        if not scales:
            raise RuntimeError("Could not calculate scale. Check real_coords for non-zero distances.")
             
        self.scale = np.mean(scales)
        self.resolution = 1.0 / self.scale
        print(f"Calculated scale: {self.scale:.4f} pixels/meter")
        print(f"Calculated resolution: {self.resolution:.7f} meters/pixel")

    def write_resolution_to_config(self) -> None:
        """Update heightmap.resolution and costmap.resolution in config.yaml preserving formatting."""
        print(f"Writing calculated resolution ({self.resolution:.7f}) back to {self.config_path}...")
        
        with open(self.config_path, "r") as f:
            content = f.read()

        lines = content.splitlines()
        in_heightmap = False
        in_costmap = False
        
        for i, line in enumerate(lines):
            striped = line.strip()
            if striped.startswith('heightmap:'):
                in_heightmap = True
                in_costmap = False
            elif striped.startswith('costmap:'):
                in_costmap = True
                in_heightmap = False
            elif striped == '' or striped.startswith('#') or (':' in striped and not striped.startswith('  ') and not striped.startswith('resolution')):
                if not striped.startswith('#') and not striped.startswith('  ') and ':' in striped:
                    in_heightmap = False
                    in_costmap = False
            
            if (in_heightmap or in_costmap) and 'resolution:' in line:
                indent = line[:line.find('resolution:')]
                comment_idx = line.find('#')
                comment = line[comment_idx:] if comment_idx != -1 else ""
                lines[i] = f"{indent}resolution:  {self.resolution:.7f}  {comment}".rstrip()

        with open(self.config_path, "w") as f:
            f.write('\n'.join(lines) + '\n')
            
        print("config.yaml successfully updated!")

    def _setup_visualization(self) -> None:
        """Initialize the calibration visualization display."""
        if self.image is None or self.origin is None:
            raise ValueError("Calibration not complete for visualization")

        plt.ion()
        figure, axes = plt.subplots(figsize=(12, 8))
        self.plot = PlotObjects(figure, axes)

        rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        self.plot.axes.imshow(rgb_image)
        
        self.plot.axes.grid(True, color="blue", linestyle="--", linewidth=0.5, alpha=0.7)
        self.plot.axes.set_title("Calibration Visualization")

        # Plot origin marker
        self.plot.axes.plot(self.origin.pixel[0], self.origin.pixel[1], "ro", markersize=12, markeredgecolor='white')
        origin_text = self.plot.axes.text(self.origin.pixel[0] + 10, self.origin.pixel[1],
                           f"Origin ({self.origin.real[0]}, {self.origin.real[1]})", color="red",
                           fontsize=10, weight='bold')
        origin_text.set_path_effects([path_effects.Stroke(linewidth=2, foreground='white'), path_effects.Normal()])

        # Plot other calibration markers
        for idx, (pixelPoint, realPoint) in enumerate(zip(self.pixelCoords, self.realCoords)):
            if idx == self.origin.index:
                continue
            self.plot.axes.plot(pixelPoint[0], pixelPoint[1], "o", color='yellow', markersize=12, markeredgecolor='black')
            marker_text = self.plot.axes.text(
                pixelPoint[0] + 10, pixelPoint[1], f"({realPoint[0]}, {realPoint[1]})", color="yellow",
                fontsize=10, weight='bold'
            )
            marker_text.set_path_effects([path_effects.Stroke(linewidth=2, foreground='black'), path_effects.Normal()])

        # Add resolution text box
        res_info = f"Scale: {self.scale:.4f} px/m\nRes: {self.resolution:.6f} m/px"
        help_text = self.plot.axes.add_artist(
            AnchoredText(res_info, loc="upper right")
        )
        help_text.patch.set_boxstyle("round,pad=0.5")
        help_text.patch.set_facecolor("wheat")
        help_text.patch.set_alpha(0.8)

        plt.tight_layout()
        plt.draw()

    def run_interactive(self) -> None:
        """Interactive loop to display result and keep open."""
        plt.ioff()
        print("\nCalibration Visualizer active. Close the window to exit.")
        plt.show()


def main():
    # Resolve config path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.abspath(os.path.join(script_dir, "../../config.yaml"))

    print(f"Using config file: {config_path}")
    try:
        calibrator = MapCalibration(config_path)
        calibrator.write_resolution_to_config()
        calibrator.run_interactive()
    except Exception as e:
        print(f"Calibration failed: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
